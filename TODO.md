Before switching to UART, try:

  1. Flash the new UF2 (has longer USB init delay + test message)
  2. Check for the test message:
  # Open serial monitor to see "Pico starting..." message
  cat /dev/ttyACM0
  3. Run agent with verbose output:
  ros2 run micro_ros_agent micro_ros_agent serial --dev
  /dev/ttyACM0 -v6

  If you see "Pico starting..." in the serial output, USB
  communication is working and the issue is just the micro-ROS
  protocol.

  If USB Still Fails → UART Option

  For UART connection to Raspberry Pi:

  Physical connections:
  Pico 2 Pin 1 (GP0/TX) → RPi Pin 10 (GPIO15/RX)
  Pico 2 Pin 2 (GP1/RX) → RPi Pin 8  (GPIO14/TX)
  Pico 2 Pin 3 (GND)    → RPi Pin 6  (GND)

  Enable UART on Raspberry Pi:
  sudo raspi-config
  # → Interface Options → Serial Port → No (login shell) → Yes 
  (hardware)
  # OR edit /boot/config.txt: 
  echo "enable_uart=1" | sudo tee -a /boot/config.txt
  sudo reboot

  Use UART:
  ros2 run micro_ros_agent micro_ros_agent serial --dev
  /dev/ttyS0 -b 115200

building setup:
```
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir uros_ws && cd uros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
```

building agent;
```
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
running agent
```
cd uros_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash 
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v4
```