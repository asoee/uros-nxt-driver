/* Motor controller using micro_ros serial transport for topic_based_ros2_control */
#include <pico/stdlib.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <hardware/pwm.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/logging.h>
#include "pico_uart_transports.h"

// pin declaration
// Left wheel
const int L_FORW = 4;
const int L_BACK = 3;
const int L_enablePin = 2;
const int L_encoderPin1 = 6;
const int L_encoderPin2 = 7;
// right wheel
const int R_FORW = 33;
const int R_BACK = 32;
const int R_enablePin = 5;
const int R_encoderPin1 = 20;
const int R_encoderPin2 = 21;

// Motor driver standby pin
const int MOTOR_STANDBY_PIN = 1;

// encoder value per revolution of left wheel and right wheel
// Hardware specs: 12 slits, 32:10 gear ratio = 720 ticks/rev with quadrature
const int tickPerRevolution_LW = 720;
const int tickPerRevolution_RW = 720;
const int threshold = 0;

// PID tuning parameters - start conservative to prevent oscillation
// Tuning guide: Start with Kp only, then add Ki, finally Kd
const float PID_KP = 10.0;    // Proportional gain (start small, increase until stable response)
const float PID_KI = 0.5;     // Integral gain (start at 0, add small amount if steady-state error)
const float PID_KD = 0.0;     // Derivative gain (start at 0, add small amount to reduce overshoot)
const bool ENABLE_PID = true; // Set to false to use direct PWM control for comparison

// pwm parameters setup
const int freq = 30000;
const int resolution = 8;

// ROS 2 variables
rcl_subscription_t joint_commands_subscriber;
rcl_subscription_t test_subscriber;
rcl_publisher_t joint_states_publisher;
rcl_publisher_t debug_publisher;
sensor_msgs__msg__JointState joint_commands_msg;
sensor_msgs__msg__JointState joint_states_msg;
std_msgs__msg__String debug_msg;
std_msgs__msg__String test_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t joint_states_timer;
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;

// PID Controller class
class PIDController
{
public:
    float kp;                // Proportional gain
    float ki;                // Integral gain
    float kd;                // Derivative gain
    float setpoint;          // Target value (rad/s)
    float integral;          // Integral accumulator
    float prev_error;        // Previous error for derivative
    float output_min;        // Minimum output limit
    float output_max;        // Maximum output limit
    unsigned long last_time; // Last calculation time

    PIDController(float p = 1.0, float i = 0.0, float d = 0.0,
                  float min_out = -200.0, float max_out = 200.0)
        : kp(p), ki(i), kd(d), setpoint(0.0), integral(0.0),
          prev_error(0.0), output_min(min_out), output_max(max_out),
          last_time(0)
    {
    }

    // Calculate PID output
    float calculate(float measured_value, float dt_seconds)
    {
        if (dt_seconds <= 0.0)
            return 0.0;

        // Immediately return 0 if target speed is 0.0
        if (fabs(setpoint) < 0.001f)
        {
            reset(); // Clear integral term to prevent windup
            return 0.0;
        }

        float error = setpoint - measured_value;

        // Proportional term
        float p_term = kp * error;

        // Integral term with windup protection
        integral += error * dt_seconds;
        // Clamp integral to prevent windup
        float max_integral = (output_max - output_min) / (2.0 * ki);
        if (ki > 0.0 && fabs(integral) > max_integral)
        {
            integral = (integral > 0) ? max_integral : -max_integral;
        }
        float i_term = ki * integral;

        // Derivative term
        float derivative = (error - prev_error) / dt_seconds;
        float d_term = kd * derivative;

        // Calculate total output
        float output = p_term + i_term + d_term;

        // Clamp output to limits
        if (output > output_max)
            output = output_max;
        if (output < output_min)
            output = output_min;

        // Store for next calculation
        prev_error = error;

        return output;
    }

    // Set new setpoint
    void setTarget(float target)
    {
        setpoint = target;
    }

    // Reset integral term (useful when changing setpoints)
    void reset()
    {
        integral = 0.0;
        prev_error = 0.0;
    }

    // Update PID parameters
    void setParameters(float p, float i, float d)
    {
        kp = p;
        ki = i;
        kd = d;
    }
};

// Motor controller class
class MotorController
{
public:
    int Forward;
    int Backward;
    int Enable;
    int EncoderPinA;
    int EncoderPinB;
    volatile long CurrentPosition;
    volatile long PreviousPosition;
    int tick;
    uint pwm_slice_num;

    // Speed and direction tracking
    volatile float CurrentSpeed; // in rad/s
    volatile int Direction;      // 1 for forward, -1 for backward, 0 for stopped
    volatile unsigned long LastUpdateTime;
    volatile long LastPosition;
    volatile bool EncoderAState;
    volatile bool EncoderBState;
    volatile bool LastEncoderAState;
    volatile bool LastEncoderBState;

    // Speed averaging buffer
    static const int SPEED_BUFFER_SIZE = 4; // Average over 4 readings (80ms at 50Hz)
    float speed_buffer[SPEED_BUFFER_SIZE];
    int speed_buffer_index;
    bool speed_buffer_full;

    // PID Control
    PIDController pid_controller;
    float target_speed;          // Target speed in rad/s
    unsigned long last_pid_time; // Last PID calculation time
    bool pid_enabled;

    // Current PWM output tracking
    volatile float current_pwm_output;

    MotorController(int ForwardPin, int BackwardPin, int EnablePin, int EncoderA, int EncoderB, int tickPerRevolution)
        : Forward(ForwardPin), Backward(BackwardPin), Enable(EnablePin),
          EncoderPinA(EncoderA), EncoderPinB(EncoderB), tick(tickPerRevolution),
          CurrentPosition(0), PreviousPosition(0), CurrentSpeed(0.0), Direction(0),
          LastUpdateTime(0), LastPosition(0), EncoderAState(false), EncoderBState(false),
          LastEncoderAState(false), LastEncoderBState(false), speed_buffer_index(0), speed_buffer_full(false),
          pid_controller(PID_KP, PID_KI, PID_KD), target_speed(0.0), last_pid_time(0), pid_enabled(ENABLE_PID),
          current_pwm_output(0.0)
    {
        gpio_init(Forward);
        gpio_set_dir(Forward, GPIO_OUT);
        gpio_init(Backward);
        gpio_set_dir(Backward, GPIO_OUT);
        gpio_init(EnablePin);
        gpio_set_dir(EnablePin, GPIO_OUT);
        gpio_init(EncoderPinA);
        gpio_set_dir(EncoderPinA, GPIO_IN);
        gpio_pull_up(EncoderPinA);
        gpio_init(EncoderPinB);
        gpio_set_dir(EncoderPinB, GPIO_IN);
        gpio_pull_up(EncoderPinB);

        // Initialize encoder states
        EncoderAState = gpio_get(EncoderPinA);
        EncoderBState = gpio_get(EncoderPinB);
        LastEncoderAState = EncoderAState;
        LastEncoderBState = EncoderBState;
        LastUpdateTime = to_ms_since_boot(get_absolute_time());

        // Initialize speed buffer
        resetSpeedBuffer();

        // Debug: Print initial encoder states (will be visible during startup)
        printf("Motor init: EncoderA=%d EncoderB=%d (pins %d,%d)\n",
               EncoderAState, EncoderBState, EncoderPinA, EncoderPinB);

        // Initialize PWM for this motor
        gpio_set_function(Enable, GPIO_FUNC_PWM);
        pwm_slice_num = pwm_gpio_to_slice_num(Enable);
        pwm_set_wrap(pwm_slice_num, 65535);
        pwm_set_enabled(pwm_slice_num, true);
    }

    // get current encoder position as radians
    float getPosition()
    {
        return ((float)CurrentPosition / tick) * 2.0 * 3.14159;
    }

    // Add speed reading to averaging buffer
    void addSpeedReading(float speed)
    {
        speed_buffer[speed_buffer_index] = speed;
        speed_buffer_index = (speed_buffer_index + 1) % SPEED_BUFFER_SIZE;
        if (!speed_buffer_full && speed_buffer_index == 0)
        {
            speed_buffer_full = true;
        }
    }

    // get current averaged speed in rad/s
    float getSpeed()
    {
        if (!speed_buffer_full && speed_buffer_index == 0)
        {
            return 0.0f; // No readings yet
        }

        float sum = 0.0f;
        int count = speed_buffer_full ? SPEED_BUFFER_SIZE : speed_buffer_index;

        for (int i = 0; i < count; i++)
        {
            sum += speed_buffer[i];
        }

        return sum / count;
    }

    // get instantaneous speed (non-averaged) for debugging
    float getInstantSpeed()
    {
        return CurrentSpeed;
    }

    // Reset speed averaging buffer
    void resetSpeedBuffer()
    {
        for (int i = 0; i < SPEED_BUFFER_SIZE; i++)
        {
            speed_buffer[i] = 0.0f;
        }
        speed_buffer_index = 0;
        speed_buffer_full = false;
    }

    // get current direction (-1, 0, 1)
    int getDirection()
    {
        return Direction;
    }

    // get current PWM output value
    float getCurrentPWM()
    {
        return current_pwm_output;
    }

    // PID control methods
    void setTargetSpeed(float speed_rad_s)
    {
        target_speed = speed_rad_s;
        pid_controller.setTarget(speed_rad_s);
        last_pid_time = to_ms_since_boot(get_absolute_time());

        // Immediately stop motor if target speed is 0
        if (fabs(speed_rad_s) < 0.001f)
        {
            stop(); // This will also reset the speed buffer
        }
    }

    void setPIDParameters(float kp, float ki, float kd)
    {
        pid_controller.setParameters(kp, ki, kd);
    }

    void enablePID(bool enable)
    {
        if (enable && !pid_enabled)
        {
            // Reset PID when enabling
            pid_controller.reset();
            last_pid_time = to_ms_since_boot(get_absolute_time());
        }
        pid_enabled = enable;
    }

    // Update motor with PID control
    void updatePIDControl()
    {
        if (!pid_enabled)
            return;

        unsigned long current_time = to_ms_since_boot(get_absolute_time());
        float dt_seconds = (current_time - last_pid_time) / 1000.0f;

        if (dt_seconds >= 0.01f) // Update at least every 10ms
        {
            // Safety check: limit target speed to reasonable range
            if (fabs(target_speed) > 20.0f) // 20 rad/s max
            {
                target_speed = (target_speed > 0) ? 20.0f : -20.0f;
                pid_controller.setTarget(target_speed);
            }

            float pid_output = pid_controller.calculate(CurrentSpeed, dt_seconds);

            // Add minimum PWM to overcome static friction
            if (fabs(target_speed) > 0.1f && fabs(pid_output) < 20.0f && fabs(pid_output) > 0.0f)
            {
                pid_output = (pid_output > 0) ? 20.0f : -20.0f;
            }

            moveMotor(pid_output);
            last_pid_time = current_time;
        }
    }

    // Get PID error for tuning purposes
    float getPIDError()
    {
        return target_speed - CurrentSpeed;
    }

    // Get target speed
    float getTargetSpeed()
    {
        return target_speed;
    }

    // update speed and direction based on encoder changes
    void updateSpeedAndDirection()
    {
        unsigned long currentTime = to_ms_since_boot(get_absolute_time());
        long positionDelta = CurrentPosition - LastPosition;
        unsigned long timeDelta = currentTime - LastUpdateTime;

        if (timeDelta > 10) // Update every 10ms minimum to avoid noise
        {
            if (timeDelta > 0)
            {
                // Calculate speed in rad/s
                float positionRadians = ((float)positionDelta / tick) * 2.0 * 3.14159;
                float timeSeconds = (float)timeDelta / 1000.0;
                CurrentSpeed = positionRadians / timeSeconds;

                // Add speed reading to averaging buffer
                addSpeedReading(CurrentSpeed);

                // Determine direction
                if (positionDelta > 0)
                    Direction = 1; // Forward
                else if (positionDelta < 0)
                    Direction = -1; // Backward
                else
                    Direction = 0; // Stopped
            }

            LastPosition = CurrentPosition;
            LastUpdateTime = currentTime;
        }

        // If no movement for a while, consider stopped
        if (timeDelta > 100) // 100ms without movement (faster stop detection)
        {
            CurrentSpeed = 0.0;
            Direction = 0;
            // Add zero speed reading to buffer
            addSpeedReading(0.0f);
        }
    }

    // Unified encoder interrupt handler
    void handleEncoderInterrupt(uint gpio, bool isRightWheel = false)
    {
        // Read current states
        bool currentA = gpio_get(EncoderPinA);
        bool currentB = gpio_get(EncoderPinB);

        // Update encoder states
        if (gpio == EncoderPinA)
        {
            LastEncoderAState = EncoderAState;
            EncoderAState = currentA;
        }
        else if (gpio == EncoderPinB)
        {
            LastEncoderBState = EncoderBState;
            EncoderBState = currentB;
        }

        // Quadrature decoding logic
        if (gpio == EncoderPinA)
        {
            if (EncoderAState != LastEncoderAState)
            {
                if (isRightWheel)
                {
                    // Right wheel logic (opposite direction for differential drive)
                    if (EncoderAState == EncoderBState)
                        CurrentPosition++; // Forward
                    else
                        CurrentPosition--; // Reverse
                }
                else
                {
                    // Left wheel logic
                    if (EncoderAState == EncoderBState)
                        CurrentPosition--; // Reverse
                    else
                        CurrentPosition++; // Forward
                }
            }
        }
        else if (gpio == EncoderPinB)
        {
            if (EncoderBState != LastEncoderBState)
            {
                if (isRightWheel)
                {
                    // Right wheel logic
                    if (EncoderBState == EncoderAState)
                        CurrentPosition--; // Reverse
                    else
                        CurrentPosition++; // Forward
                }
                else
                {
                    // Left wheel logic
                    if (EncoderBState == EncoderAState)
                        CurrentPosition++; // Forward
                    else
                        CurrentPosition--; // Reverse
                }
            }
        }

        // Update speed and direction
        updateSpeedAndDirection();
    }

    // move the motor with direct PWM control (-255 to +255)
    void moveMotor(float pwm_value)
    {
        if (pwm_value > 0)
        {
            gpio_put(Forward, 1);
            gpio_put(Backward, 0);
        }
        else if (pwm_value < 0)
        {
            gpio_put(Forward, 0);
            gpio_put(Backward, 1);
        }
        else
        {
            gpio_put(Forward, 0);
            gpio_put(Backward, 0);
        }
        int pwm = (int)fabs(pwm_value);
        if (pwm > 255)
            pwm = 255;
        // Convert to duty cycle (0-65535 for 16-bit PWM)
        pwm_set_gpio_level(Enable, (pwm * 65535) / 255);

        // Track current PWM output (with sign for direction)
        current_pwm_output = (pwm_value > 0) ? pwm : -pwm;
    }

    void stop()
    {
        gpio_put(Forward, 0);
        gpio_put(Backward, 0);
        pwm_set_gpio_level(Enable, 0);
        // Reset speed buffer when motor stops
        resetSpeedBuffer();
        // Reset PWM output tracking
        current_pwm_output = 0.0;
    }
};

// Motor driver control functions
void enableMotorDriver()
{
    gpio_put(MOTOR_STANDBY_PIN, 1); // HIGH disables standby (enables motors)
}

void disableMotorDriver()
{
    gpio_put(MOTOR_STANDBY_PIN, 0); // LOW enables standby (disables motors)
}

// creating objects for right wheel and left wheel
MotorController leftWheel(L_FORW, L_BACK, L_enablePin, L_encoderPin1, L_encoderPin2, tickPerRevolution_LW);
MotorController rightWheel(R_FORW, R_BACK, R_enablePin, R_encoderPin1, R_encoderPin2, tickPerRevolution_RW);

#define LED_PIN 25 // Built-in LED on Pico
#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            return false;            \
        }                            \
    }
#define RCSOFTCHECK(fn)                  \
    {                                    \
        rcl_ret_t temp_rc = fn;          \
        if ((temp_rc != RCL_RET_OK))     \
        {                                \
            /* Log error but continue */ \
        }                                \
    }

// Global interrupt handler for all encoder pins
void encoder_interrupt_handler(uint gpio, uint32_t events)
{
    (void)events;
    static volatile int total_interrupt_count = 0;
    static volatile int left_interrupt_count = 0;
    total_interrupt_count++;

    // Determine which motor's encoder triggered the interrupt
    if (gpio == leftWheel.EncoderPinA || gpio == leftWheel.EncoderPinB)
    {
        left_interrupt_count++;
        leftWheel.handleEncoderInterrupt(gpio, false); // false = left wheel
    }
    else if (gpio == rightWheel.EncoderPinA || gpio == rightWheel.EncoderPinB)
    {
        rightWheel.handleEncoderInterrupt(gpio, true); // true = right wheel
    }
}

// State machine for agent connection
enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// Debug message publishing function
void publish_debug(const char *message)
{
    if (state == AGENT_CONNECTED)
    {
        // Ensure we have enough capacity
        size_t msg_len = strlen(message);
        if (debug_msg.data.capacity < msg_len + 1)
        {
            debug_msg.data.data = (char *)realloc(debug_msg.data.data, msg_len + 1);
            debug_msg.data.capacity = msg_len + 1;
        }

        strcpy(debug_msg.data.data, message);
        debug_msg.data.size = msg_len;

        rcl_publish(&debug_publisher, &debug_msg, NULL);
    }
}

// Joint commands callback function
void joint_commands_callback(const void *msgin)
{
    static int callback_count = 0;
    callback_count++;

    // First, let's see if this callback is even being called
    publish_debug("JOINT CALLBACK: Function entry detected!");

    // Basic safety check - don't process if msgin is null
    if (msgin == NULL)
    {
        publish_debug("ERROR: Null message pointer in callback!");
        return;
    }

    char debug_buffer[100];
    snprintf(debug_buffer, sizeof(debug_buffer), "JOINT CALLBACK #%d: Message received", callback_count);
    publish_debug(debug_buffer);

    const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;

    // Safety checks before accessing message data
    if (msg->velocity.data == NULL)
    {
        publish_debug("WARNING: Velocity data is NULL");
        return;
    }

    if (msg->velocity.size == 0)
    {
        publish_debug("WARNING: Velocity size is 0");
        return;
    }

    prev_cmd_time = to_ms_since_boot(get_absolute_time());

    // Apply joint commands directly to motors
    if (msg->velocity.size >= 2)
    {
        // Use velocity commands (rad/s) converted to PWM
        float left_vel = msg->velocity.data[0];
        float right_vel = msg->velocity.data[1];

        // Log control mode and speeds
        char debug_buffer[200]; // Increased buffer size for longer message
        if (ENABLE_PID)
        {
            snprintf(debug_buffer, sizeof(debug_buffer), "PID: Target L=%.2f R=%.2f | Avg L=%.2f R=%.2f | PWM L=%.0f R=%.0f | Instant L=%.2f R=%.2f",
                     left_vel, right_vel, leftWheel.getSpeed(), rightWheel.getSpeed(),
                     leftWheel.getCurrentPWM(), rightWheel.getCurrentPWM(),
                     leftWheel.getInstantSpeed(), rightWheel.getInstantSpeed());
        }
        else
        {
            snprintf(debug_buffer, sizeof(debug_buffer), "PWM: Cmd L=%.2f R=%.2f | Speed L=%.2f R=%.2f",
                     left_vel, right_vel, leftWheel.getSpeed(), rightWheel.getSpeed());
        }
        publish_debug(debug_buffer);

        // Log PID performance information occasionally
        static int pid_log_counter = 0;
        pid_log_counter++;
        if (pid_log_counter % 25 == 0) // Every ~1.25 seconds at 20Hz
        {
            snprintf(debug_buffer, sizeof(debug_buffer), "PID Errors: L=%.2f R=%.2f | Targets: L=%.2f R=%.2f",
                     leftWheel.getPIDError(), rightWheel.getPIDError(),
                     leftWheel.getTargetSpeed(), rightWheel.getTargetSpeed());
            publish_debug(debug_buffer);
        }

        // Enable motor driver when commands are received
        enableMotorDriver();

        if (ENABLE_PID)
        {
            // Use PID control for accurate speed control
            leftWheel.setTargetSpeed(left_vel);
            rightWheel.setTargetSpeed(right_vel);
            // PID controllers will be updated in the joint states callback for consistent timing
        }
        else
        {
            // Fallback to direct PWM control for testing/comparison
            float left_pwm = left_vel * 30.0; // Conservative scale factor
            float right_pwm = right_vel * 30.0;
            leftWheel.moveMotor(left_pwm);
            rightWheel.moveMotor(right_pwm);
        }
    }
    else
    {
        publish_debug("Velocity size < 2, stopping motors");
        leftWheel.stop();
        rightWheel.stop();
        disableMotorDriver(); // Put motor driver in standby for safety
    }
}

// Forward declarations
struct timespec getTime();
bool syncTime();
bool create_entities();
void destroy_entities();
void test_callback(const void *msgin);

// Test callback function for simple string messages
void test_callback(const void *msgin)
{
    static int test_count = 0;
    test_count++;
    char debug_buffer[100];
    snprintf(debug_buffer, sizeof(debug_buffer), "TEST CALLBACK #%d TRIGGERED!", test_count);
    publish_debug(debug_buffer);

    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    if (msg && msg->data.data)
    {
        snprintf(debug_buffer, sizeof(debug_buffer), "Test message: %s", msg->data.data);
        publish_debug(debug_buffer);
    }
}

// function which publishes joint states
void joint_states_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    static int counter = 0;
    counter++;
    if (counter % 50 == 0) // Every 1 second (50Hz * 50 = 1s)
    {
        publish_debug("Joint states timer callback running - executor is working");

        // Debug encoder positions, raw tick counts, and speeds
        char debug_buffer[200];
        snprintf(debug_buffer, sizeof(debug_buffer),
                 "Ticks: L=%ld R=%ld | Pos: L=%.2f R=%.2f",
                 leftWheel.CurrentPosition, rightWheel.CurrentPosition,
                 leftWheel.getPosition(), rightWheel.getPosition());
        publish_debug(debug_buffer);
        
        snprintf(debug_buffer, sizeof(debug_buffer),
                 "Speed Avg: L=%.2f R=%.2f | Instant: L=%.2f R=%.2f | PWM: L=%.0f R=%.0f",
                 leftWheel.getSpeed(), rightWheel.getSpeed(),
                 leftWheel.getInstantSpeed(), rightWheel.getInstantSpeed(),
                 leftWheel.getCurrentPWM(), rightWheel.getCurrentPWM());
        publish_debug(debug_buffer);

        // PID debugging for both motors
        snprintf(debug_buffer, sizeof(debug_buffer), "PID Left: Target=%.2f Speed=%.2f Error=%.2f",
                 leftWheel.getTargetSpeed(), leftWheel.getSpeed(), leftWheel.getPIDError());
        publish_debug(debug_buffer);
        
        snprintf(debug_buffer, sizeof(debug_buffer), "PID Right: Target=%.2f Speed=%.2f Error=%.2f",
                 rightWheel.getTargetSpeed(), rightWheel.getSpeed(), rightWheel.getPIDError());
        publish_debug(debug_buffer);
    }

    // Update speed calculations for both wheels (handles stopped motor detection)
    leftWheel.updateSpeedAndDirection();
    rightWheel.updateSpeedAndDirection();

    // Update PID controllers for both motors
    leftWheel.updatePIDControl();
    rightWheel.updatePIDControl();

    // Update joint positions from encoders
    joint_states_msg.position.data[0] = leftWheel.getPosition();
    joint_states_msg.position.data[1] = rightWheel.getPosition();

    // Update velocities from encoder measurements
    joint_states_msg.velocity.data[0] = leftWheel.getSpeed();
    joint_states_msg.velocity.data[1] = rightWheel.getSpeed();

    // Set efforts to 0 (no torque sensing)
    joint_states_msg.effort.data[0] = 0.0;
    joint_states_msg.effort.data[1] = 0.0;

    // Set timestamp
    struct timespec time_stamp = getTime();
    joint_states_msg.header.stamp.sec = time_stamp.tv_sec;
    joint_states_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&joint_states_publisher, &joint_states_msg, NULL));
}

void setup()
{
    // Set up micro-ROS transport (USB CDC will be initialized by stdio_init_all)
    set_microros_transports();

    // Set up global interrupt handler for all encoder pins
    // All pins need the callback explicitly set
    gpio_set_irq_enabled_with_callback(leftWheel.EncoderPinA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_interrupt_handler);
    gpio_set_irq_enabled_with_callback(leftWheel.EncoderPinB, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_interrupt_handler);
    gpio_set_irq_enabled_with_callback(rightWheel.EncoderPinA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_interrupt_handler);
    gpio_set_irq_enabled_with_callback(rightWheel.EncoderPinB, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_interrupt_handler);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0); // Start with LED off

    // Initialize motor standby pin - HIGH disables standby
    gpio_init(MOTOR_STANDBY_PIN);
    gpio_set_dir(MOTOR_STANDBY_PIN, GPIO_OUT);
    gpio_put(MOTOR_STANDBY_PIN, 1); // Set HIGH to disable standby

    // Initialize state machine
    state = WAITING_AGENT;
}

bool syncTime()
{
    // get the current time from the agent
    unsigned long now = to_ms_since_boot(get_absolute_time());
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
    return true;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = to_ms_since_boot(get_absolute_time()) + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

bool create_entities()
{
    allocator = rcl_get_default_allocator();

    // create init_options
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    {
        return false;
    }

    // Initialize logging system (keeping minimal RCUTILS for error handling)
    if (rcutils_logging_initialize() != RCUTILS_RET_OK)
    {
        printf("Failed to initialize logging\n");
        return false;
    }

    // create node
    if (rclc_node_init_default(&node, "pico_joint_controller", "", &support) != RCL_RET_OK)
    {
        return false;
    }

    // create subscriber for joint commands with default QoS
    if (rclc_subscription_init_default(
            &joint_commands_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "/robot_joint_commands") != RCL_RET_OK)
    {
        printf("ERROR: Failed to create joint commands subscriber!\n");
        return false;
    }
    else
    {
        printf("Joint commands subscriber created successfully\n");
    }

    // create test subscriber for simple string messages
    if (rclc_subscription_init_default(
            &test_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/test_topic") != RCL_RET_OK)
    {
        printf("ERROR: Failed to create test subscriber!\n");
        return false;
    }
    else
    {
        printf("Test subscriber created successfully\n");
    }

    // create joint states publisher
    if (rclc_publisher_init_default(
            &joint_states_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "/robot_joint_states") != RCL_RET_OK)
    {
        return false;
    }

    // create debug publisher
    if (rclc_publisher_init_default(
            &debug_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/pico_debug") != RCL_RET_OK)
    {
        return false;
    }

    // initialize joint state message
    joint_states_msg.name.capacity = 2;
    joint_states_msg.name.size = 2;
    joint_states_msg.name.data = (rosidl_runtime_c__String *)malloc(2 * sizeof(rosidl_runtime_c__String));

    joint_states_msg.name.data[0].data = (char *)malloc(20 * sizeof(char));
    joint_states_msg.name.data[0].capacity = 20;
    strcpy(joint_states_msg.name.data[0].data, "left_wheel_joint");
    joint_states_msg.name.data[0].size = strlen(joint_states_msg.name.data[0].data);

    joint_states_msg.name.data[1].data = (char *)malloc(20 * sizeof(char));
    joint_states_msg.name.data[1].capacity = 20;
    strcpy(joint_states_msg.name.data[1].data, "right_wheel_joint");
    joint_states_msg.name.data[1].size = strlen(joint_states_msg.name.data[1].data);

    joint_states_msg.position.capacity = 2;
    joint_states_msg.position.size = 2;
    joint_states_msg.position.data = (double *)malloc(2 * sizeof(double));

    joint_states_msg.velocity.capacity = 2;
    joint_states_msg.velocity.size = 2;
    joint_states_msg.velocity.data = (double *)malloc(2 * sizeof(double));

    joint_states_msg.effort.capacity = 2;
    joint_states_msg.effort.size = 2;
    joint_states_msg.effort.data = (double *)malloc(2 * sizeof(double));

    // Initialize header frame_id
    joint_states_msg.header.frame_id.data = (char *)malloc(20 * sizeof(char));
    joint_states_msg.header.frame_id.capacity = 20;
    strcpy(joint_states_msg.header.frame_id.data, "base_link");
    joint_states_msg.header.frame_id.size = strlen("base_link");

    // Initialize debug message
    debug_msg.data.data = (char *)malloc(200 * sizeof(char));
    debug_msg.data.capacity = 200;
    debug_msg.data.size = 0;

    // Initialize test message
    test_msg.data.data = (char *)malloc(100 * sizeof(char));
    test_msg.data.capacity = 100;
    test_msg.data.size = 0;

    // Initialize joint commands message with proper string initialization
    joint_commands_msg.name.capacity = 2;
    joint_commands_msg.name.size = 0;
    joint_commands_msg.name.data = (rosidl_runtime_c__String *)malloc(2 * sizeof(rosidl_runtime_c__String));
    // Initialize the string entries with actual memory allocation
    for (int i = 0; i < 2; i++)
    {
        joint_commands_msg.name.data[i].data = (char *)malloc(50 * sizeof(char));
        joint_commands_msg.name.data[i].capacity = 50;
        joint_commands_msg.name.data[i].size = 0;
    }

    joint_commands_msg.position.capacity = 2;
    joint_commands_msg.position.size = 0;
    joint_commands_msg.position.data = (double *)malloc(2 * sizeof(double));

    joint_commands_msg.velocity.capacity = 2;
    joint_commands_msg.velocity.size = 0;
    joint_commands_msg.velocity.data = (double *)malloc(2 * sizeof(double));

    joint_commands_msg.effort.capacity = 2;
    joint_commands_msg.effort.size = 0;
    joint_commands_msg.effort.data = (double *)malloc(2 * sizeof(double));

    // Initialize header for joint commands message
    joint_commands_msg.header.frame_id.data = (char *)malloc(20 * sizeof(char));
    joint_commands_msg.header.frame_id.capacity = 20;
    joint_commands_msg.header.frame_id.size = 0;

    // timer for publishing joint states
    const unsigned int publish_rate = 20; // 50ms = 20Hz
    if (rclc_timer_init_default2(
            &joint_states_timer,
            &support,
            RCL_MS_TO_NS(publish_rate),
            joint_states_callback,
            true) != RCL_RET_OK)
    {
        return false;
    }

    // create executor with capacity for 3 handles: timer + joint subscription + test subscription
    if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK)
    {
        return false;
    }

    if (rclc_executor_add_subscription(&executor, &joint_commands_subscriber, &joint_commands_msg, &joint_commands_callback, ON_NEW_DATA) != RCL_RET_OK)
    {
        printf("ERROR: Failed to add subscription to executor!\n");
        return false;
    }
    else
    {
        printf("Subscription added to executor successfully\n");
    }

    if (rclc_executor_add_subscription(&executor, &test_subscriber, &test_msg, &test_callback, ON_NEW_DATA) != RCL_RET_OK)
    {
        printf("ERROR: Failed to add test subscription to executor!\n");
        return false;
    }
    else
    {
        printf("Test subscription added to executor successfully\n");
    }

    if (rclc_executor_add_timer(&executor, &joint_states_timer) != RCL_RET_OK)
    {
        printf("ERROR: Failed to add timer to executor!\n");
        return false;
    }
    else
    {
        printf("Timer added to executor successfully\n");
    }

    return true;
}

void destroy_entities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // Stop motors and disable driver when connection is lost
    leftWheel.stop();
    rightWheel.stop();
    disableMotorDriver();

    // Clean up allocated memory for joint_states_msg
    if (joint_states_msg.name.data)
    {
        if (joint_states_msg.name.data[0].data)
            free(joint_states_msg.name.data[0].data);
        if (joint_states_msg.name.data[1].data)
            free(joint_states_msg.name.data[1].data);
        free(joint_states_msg.name.data);
        joint_states_msg.name.data = NULL;
    }
    if (joint_states_msg.position.data)
    {
        free(joint_states_msg.position.data);
        joint_states_msg.position.data = NULL;
    }
    if (joint_states_msg.velocity.data)
    {
        free(joint_states_msg.velocity.data);
        joint_states_msg.velocity.data = NULL;
    }
    if (joint_states_msg.effort.data)
    {
        free(joint_states_msg.effort.data);
        joint_states_msg.effort.data = NULL;
    }
    if (joint_states_msg.header.frame_id.data)
    {
        free(joint_states_msg.header.frame_id.data);
        joint_states_msg.header.frame_id.data = NULL;
    }
    if (debug_msg.data.data)
    {
        free(debug_msg.data.data);
        debug_msg.data.data = NULL;
    }
    if (test_msg.data.data)
    {
        free(test_msg.data.data);
        test_msg.data.data = NULL;
    }

    // Clean up allocated memory for joint_commands_msg
    if (joint_commands_msg.name.data)
    {
        for (int i = 0; i < 2; i++)
        {
            if (joint_commands_msg.name.data[i].data)
            {
                free(joint_commands_msg.name.data[i].data);
            }
        }
        free(joint_commands_msg.name.data);
        joint_commands_msg.name.data = NULL;
    }
    if (joint_commands_msg.position.data)
    {
        free(joint_commands_msg.position.data);
        joint_commands_msg.position.data = NULL;
    }
    if (joint_commands_msg.velocity.data)
    {
        free(joint_commands_msg.velocity.data);
        joint_commands_msg.velocity.data = NULL;
    }
    if (joint_commands_msg.effort.data)
    {
        free(joint_commands_msg.effort.data);
        joint_commands_msg.effort.data = NULL;
    }
    if (joint_commands_msg.header.frame_id.data)
    {
        free(joint_commands_msg.header.frame_id.data);
        joint_commands_msg.header.frame_id.data = NULL;
    }

    (void)rcl_publisher_fini(&debug_publisher, &node);
    (void)rcl_publisher_fini(&joint_states_publisher, &node);
    (void)rcl_subscription_fini(&joint_commands_subscriber, &node);
    (void)rcl_subscription_fini(&test_subscriber, &node);
    (void)rcl_timer_fini(&joint_states_timer);
    (void)rclc_executor_fini(&executor);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);

    // Finalize logging system
    rcutils_logging_shutdown();
}

int main()
{
    // Timeout for each ping attempt
    const int timeout_ms = 1000;

    // Number of ping attempts
    const uint8_t attempts = 1;

    // Spin period
    const unsigned int spin_timeout = RCL_MS_TO_NS(100);

    setup();

    while (true)
    {
        switch (state)
        {
        case WAITING_AGENT:
            // Check for agent connection
            state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_AVAILABLE : WAITING_AGENT;
            if (state == WAITING_AGENT)
            {
                // Blink LED to indicate waiting for agent
                gpio_put(LED_PIN, !gpio_get(LED_PIN));
                sleep_ms(200); // Faster blinking for more responsive feedback
            }
            break;

        case AGENT_AVAILABLE:
            // Create micro-ROS entities
            gpio_put(LED_PIN, 1); // Solid LED when creating entities
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;

            if (state == WAITING_AGENT)
            {
                // Creation failed, release allocated resources
                destroy_entities();
            }
            break;

        case AGENT_CONNECTED:
            // First time connected - send debug messages
            static bool first_connection = true;
            if (first_connection)
            {
                first_connection = false;
                publish_debug("Micro-ROS agent connected successfully!");
                publish_debug("Debug topic is working - waiting for joint commands...");
                publish_debug("Subscription topic: /robot_joint_commands");
                publish_debug("Publishing to: /robot_joint_states and /pico_debug");
            }

            // Check connection and spin on success
            state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            if (state == AGENT_CONNECTED)
            {
                static int spin_counter = 0;
                spin_counter++;
                if (spin_counter % 200 == 0) // Every 10 seconds roughly
                {
                    char debug_buffer[100];
                    snprintf(debug_buffer, sizeof(debug_buffer), "Executor spins: %d - checking for messages...", spin_counter);
                    publish_debug(debug_buffer);
                }

                rcl_ret_t ret = rclc_executor_spin_some(&executor, spin_timeout);
                if (ret != RCL_RET_OK && spin_counter % 100 == 0)
                {
                    char debug_buffer[100];
                    snprintf(debug_buffer, sizeof(debug_buffer), "Executor spin returned error: %d", ret);
                    publish_debug(debug_buffer);
                }

                // Periodically check if executor has pending work
                if (spin_counter % 500 == 0) // Every 25 seconds roughly
                {
                    publish_debug("Executor spinning - subscription should be active");
                }
            }
            else
            {
                // Connection lost, reset the first_connection flag
                first_connection = true;
            }
            break;

        case AGENT_DISCONNECTED:
            // Connection is lost, destroy entities and go back to first step
            gpio_put(LED_PIN, 0); // Turn off LED when disconnected
            destroy_entities();
            state = WAITING_AGENT;
            break;

        default:
            break;
        }
    }

    return 0;
}