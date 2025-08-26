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
const int L_FORW = 3;
const int L_BACK = 4;
const int L_enablePin = 2;
const int L_encoderPin1 = 6;
const int L_encoderPin2 = 7;
// right wheel
const int R_FORW = 33;
const int R_BACK = 32;
const int R_enablePin = 5;
const int R_encoderPin1 = 20;
const int R_encoderPin2 = 21;

// encoder value per revolution of left wheel and right wheel
// Hardware specs: 12 slits, 32:10 gear ratio = 720 ticks/rev with quadrature
const int tickPerRevolution_LW = 720;
const int tickPerRevolution_RW = 720;
const int threshold = 0;

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

    MotorController(int ForwardPin, int BackwardPin, int EnablePin, int EncoderA, int EncoderB, int tickPerRevolution)
        : Forward(ForwardPin), Backward(BackwardPin), Enable(EnablePin),
          EncoderPinA(EncoderA), EncoderPinB(EncoderB), tick(tickPerRevolution),
          CurrentPosition(0), PreviousPosition(0), CurrentSpeed(0.0), Direction(0),
          LastUpdateTime(0), LastPosition(0), EncoderAState(false), EncoderBState(false),
          LastEncoderAState(false), LastEncoderBState(false)
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

    // get current speed in rad/s
    float getSpeed()
    {
        return CurrentSpeed;
    }

    // get current direction (-1, 0, 1)
    int getDirection()
    {
        return Direction;
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
    }

    void stop()
    {
        gpio_put(Forward, 0);
        gpio_put(Backward, 0);
        pwm_set_gpio_level(Enable, 0);
    }
};

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
    total_interrupt_count++;
    
    // Determine which motor's encoder triggered the interrupt
    if (gpio == leftWheel.EncoderPinA || gpio == leftWheel.EncoderPinB)
    {
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

        // Log velocity changes and encoder feedback
        snprintf(debug_buffer, sizeof(debug_buffer), "Cmd: L=%.2f R=%.2f | Enc: L=%.2f R=%.2f",
                 left_vel, right_vel, leftWheel.getSpeed(), rightWheel.getSpeed());
        publish_debug(debug_buffer);

        // Log direction information occasionally
        static int dir_log_counter = 0;
        dir_log_counter++;
        if (dir_log_counter % 50 == 0) // Every ~2.5 seconds at 20Hz
        {
            snprintf(debug_buffer, sizeof(debug_buffer), "Dir: L=%d R=%d | Pos: L=%.2f R=%.2f",
                     leftWheel.getDirection(), rightWheel.getDirection(),
                     leftWheel.getPosition(), rightWheel.getPosition());
            publish_debug(debug_buffer);
        }

        // Convert rad/s to PWM (-255 to +255)
        // NOTE: Scale factor may need adjustment due to encoder tick change from 1055 to 720
        float left_pwm = left_vel * 50.0; // Scale factor - tune as needed
        float right_pwm = right_vel * 50.0;

        leftWheel.moveMotor(left_pwm);
        rightWheel.moveMotor(right_pwm);
    }
    else
    {
        publish_debug("Velocity size < 2, stopping motors");
        leftWheel.stop();
        rightWheel.stop();
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
    if (counter % 100 == 0) // Every 5 seconds (20Hz * 100 = 5s)
    {
        publish_debug("Joint states timer callback running - executor is working");
        
        // Debug encoder positions and raw tick counts
        char debug_buffer[200];
        snprintf(debug_buffer, sizeof(debug_buffer), "Raw ticks: L=%ld R=%ld | Positions: L=%.3f R=%.3f", 
                leftWheel.CurrentPosition, rightWheel.CurrentPosition,
                leftWheel.getPosition(), rightWheel.getPosition());
        publish_debug(debug_buffer);
    }

    // Update speed calculations for both wheels (handles stopped motor detection)
    leftWheel.updateSpeedAndDirection();
    rightWheel.updateSpeedAndDirection();
    
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
    gpio_set_irq_enabled_with_callback(leftWheel.EncoderPinA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_interrupt_handler);
    gpio_set_irq_enabled(leftWheel.EncoderPinB, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(rightWheel.EncoderPinA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(rightWheel.EncoderPinB, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0); // Start with LED off

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
    const unsigned int publish_rate = 50; // 50ms = 20Hz
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

    // Stop motors when connection is lost
    leftWheel.stop();
    rightWheel.stop();

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