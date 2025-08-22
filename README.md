# roscan ROS Package

This package provides a ROS node to bridge communication between a ROS environment and a CAN bus via a UART serial connection. It is designed to handle sensor data (IMU, GPS, encoders) and control commands (keyboard, robot arm).

## Features

- **ROS-to-UART Bridge:** The main node, `roscan_node.py`, listens for ROS messages, packetizes them into a custom UART protocol, and sends them to a serial device. It also reads UART packets, de-packetizes them, and publishes the data to ROS topics.
- **Modular Parsers:** The package includes a collection of parsers for various CAN message formats, including IMU, GPS, and encoders.
- **Data Handlers:** For complex sensors like GPS and IMU that send data across multiple CAN frames, the package includes data handlers to pair and process the frames before publishing.
- **Extensible:** The package is designed to be extensible. New parsers and handlers can be easily added to support new sensors or control messages.
- **Parameterized:** The node is fully parameterized, allowing for easy configuration through a launch file.

## Node

### `roscan_node.py`

This is the main node of the package. It performs the following functions:

- Initializes a ROS node named `ros_uart_bridge`.
- Connects to a serial port specified by the `~port` and `~baudrate` ROS parameters.
- Spawns a thread to read and process incoming data from the serial port.
- Subscribes to ROS topics to receive control commands and sends them to the serial port.
- Publishes sensor data received from the serial port to ROS topics.

### Parameters

The node can be configured using the following ROS parameters:

- `~port` (string, default: `/dev/ttyTHS0`): The serial port to connect to.
- `~baudrate` (int, default: `115200`): The baud rate of the serial connection.
- `~rate` (int, default: `10`): The rate at which to publish the heartbeat message.
- `~gps_latitude_frame_id` (int, default: `0x120`): The CAN frame ID for GPS latitude messages.
- `~gps_longitude_frame_id` (int, default: `0x121`): The CAN frame ID for GPS longitude messages.
- `~imu_orientation_frame_id` (int, default: `0x200`): The CAN frame ID for IMU orientation messages.
- `~imu_linear_accel_frame_id` (int, default: `0x201`): The CAN frame ID for IMU linear acceleration messages.
- `~imu_linear_vel_frame_id` (int, default: `0x202`): The CAN frame ID for IMU linear velocity messages.
- `~encoder_frame_id` (int, default: `0x001`): The CAN frame ID for encoder messages.
- `~test_frame_id` (int, default: `0x123`): The CAN frame ID for test messages.
- `~load_cell_frame_id` (int, default: `0xFAD`): The CAN frame ID for load cell messages.
- `~gps_topic` (string, default: `gpsData`): The topic to publish GPS data to.
- `~imu_topic` (string, default: `imuData`): The topic to publish IMU data to.
- `~encoder_topic` (string, default: `encoderData`): The topic to publish encoder data to.
- `~test_topic` (string, default: `testCanData`): The topic to publish test data to.
- `~load_cells_topic` (string, default: `/load_cells`): The topic to publish load cell data to.
- `~heartbeat_topic` (string, default: `/system/heartbeat`): The topic to publish the heartbeat message to.
- `~cmd_vel_topic` (string, default: `/cmd_vel`): The topic to subscribe to for keyboard control commands.
- `~arm_joint_velocities_topic` (string, default: `/arm_joint_velocities`): The topic to subscribe to for robot arm control commands.
- `~heartbeat_message` (string, default: `ros_uart_bridge:running`): The content of the heartbeat message.

### Published Topics

- `imuData` (`sensor_msgs/Imu`): IMU sensor data.
- `gpsData` (`sensor_msgs/NavSatFix`): GPS data.
- `encoderData` (`roar_msgs/EncoderStamped`): Encoder data.
- `testCanData` (`std_msgs/String`): Test data.
- `/load_cells` (`std_msgs/Float32MultiArray`): Load cell data.
- `/system/heartbeat` (`std_msgs/String`): Heartbeat message.

### Subscribed Topics

- `/cmd_vel` (`geometry_msgs/Twist`): Keyboard control commands.
- `/arm_joint_velocities` (`std_msgs/Float32MultiArray`): Robot arm control commands.

## Usage

To run the node, it is recommended to use the provided launch file:

```bash
roslaunch roscan roscan.launch
```

You can also run the node directly using `rosrun` and set the parameters on the command line:

```bash
rosrun roscan roscan_node.py _port:=/dev/ttyTHS0 _baudrate:=115200
```

## Testing

A testing script, `test_parsers.py`, is provided to test the functionality of the CAN frame parsers and data handlers. To run the tests, use the following command:

```bash
python -m src.roscan.test_parsers
```