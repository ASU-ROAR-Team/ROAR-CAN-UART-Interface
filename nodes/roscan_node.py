#!/usr/bin/env python3
"""
Main ROS node for the ROSCAN bridge.

This module provides the main ROS node that bridges ROS messages
with CAN bus communication.
"""

import rospy
import threading
import time
from typing import Dict, Any

# ROS message imports
from can_msgs.msg import Frame
from std_msgs.msg import String, Float32MultiArray, Float32
from roar_msgs.msg import EncoderStamped, DrillingStatus, DrillingCommand # Added DrillingStatus and DrillingCommand
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix, Imu

# Import our restructured components
from roscan.dummy_communication_manager import CommunicationManager # Changed from dummy_communication_manager
from roscan.messages.registry import MessageRegistry
from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import RoscanError, ParsingError
from roscan.messages.imu_frame_pairer import ImuFramePairer
from roscan.messages.gps_coordinate_pairer import GpsCoordinatePairer

# Import incoming messages
from roscan.messages.incoming.encoder_message import EncoderMessage
from roscan.messages.incoming.gps_message import GpsLatitudeMessage, GpsLongitudeMessage
from roscan.messages.incoming.imu_message import ImuOrientationMessage, ImuLinearAccelMessage
from roscan.messages.incoming.load_cell_message import LoadCellMessage
from roscan.messages.incoming.test_message import TestMessage
from roscan.messages.incoming.drilling_message import DrillingStatusMessage 

# Import outgoing messages
from roscan.messages.outgoing.keyboard_control_message import KeyboardControlMessage
from roscan.messages.outgoing.robot_arm_control_message import RobotArmControlMessage
from roscan.messages.outgoing.motor_control_message import OutgoingMotorControlMessage
from roscan.messages.outgoing.drilling_control_message import OutgoingDrillingCommandMessage 


class RoscanNode:
    """
    Main class for the ROSCAN node.
    """
    
    def __init__(self):
        """Initialize the ROSCAN node."""
        rospy.init_node("roscan_node")
        
        # Get ROS parameters
        self._load_parameters()
        
        # Initialize managers
        self.communication_manager = CommunicationManager(self.port, self.baudrate)
        self.message_registry = MessageRegistry()
        self.gps_pairer = GpsCoordinatePairer()
        self.imu_pairer = ImuFramePairer()

        # ROS Publishers
        self.gps_pub = rospy.Publisher(self.gps_topic, NavSatFix, queue_size=10)
        self.imu_pub = rospy.Publisher(self.imu_topic, Imu, queue_size=10)
        self.encoder_pub = rospy.Publisher(self.encoder_topic, EncoderStamped, queue_size=10)
        self.test_pub = rospy.Publisher(self.test_topic, String, queue_size=10)
        self.load_cell_pub = rospy.Publisher(self.load_cells_topic, Float32, queue_size=10)
        self.drilling_status_pub = rospy.Publisher(self.drilling_status_topic, DrillingStatus, queue_size=10)
        self.motor_control_pub = rospy.Publisher("/motor_control", Float32MultiArray, queue_size=10)

        # ROS Subscribers
        # rospy.Subscriber(self.cmd_vel_topic, Float32MultiArray, self._keyboard_control_callback)
        rospy.Subscriber(self.arm_joint_velocities_topic, PoseStamped, self._robot_arm_control_callback)
        rospy.Subscriber(self.motor_control_cmd_topic, Float32MultiArray, self._motor_control_callback)
        rospy.Subscriber(self.drilling_command_topic, DrillingCommand, self._drilling_command_callback)
        
        # Initialize messages
        self._initialize_messages()
        
        # Set up communication callback
        self.communication_manager.set_frame_callback(self._process_frame)
        
        rospy.loginfo("ROSCAN Node initialized successfully")
    
    def _load_parameters(self) -> None:
        """Load ROS parameters."""
        self.port = rospy.get_param("~port", "/dev/ttyTHS0")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.rate = rospy.Rate(rospy.get_param("~rate",10))
        
        # Frame IDs
        self.GPS_LATITUDE_FRAME_ID = rospy.get_param("~gps_latitude_frame_id", 0x120)
        self.GPS_LONGITUDE_FRAME_ID = rospy.get_param("~gps_longitude_frame_id", 0x121)
        self.IMU_ORIENTATION_FRAME_ID = rospy.get_param("~imu_orientation_frame_id", 0x200)
        self.IMU_LINEAR_ACCEL_FRAME_ID = rospy.get_param("~imu_linear_accel_frame_id", 0x201)
        self.ENCODER_FRAME_ID = rospy.get_param("~encoder_frame_id", 0x222)
        self.TEST_FRAME_ID = rospy.get_param("~test_frame_id", 0x123)
        self.LOAD_CELL_FRAME_ID = rospy.get_param("~load_cell_frame_id", 0xFAD)
        self.MOTOR_CONTROL_FRAME_ID = rospy.get_param("~motor_control_frame_id", 0x6A5)
        self.DRILLING_STATUS_FRAME_ID = rospy.get_param("~drilling_status_frame_id", 0x400) 
        self.DRILLING_COMMAND_FRAME_ID = rospy.get_param("~drilling_command_frame_id", 0x333)
        
        # Topic names
        self.gps_topic = rospy.get_param("~gps_topic", "gpsData")
        self.load_cells_topic = rospy.get_param("~load_cells_topic", "/load_cells")
        self.imu_topic = rospy.get_param("~imu_topic", "imuData")
        self.encoder_topic = rospy.get_param("~encoder_topic", "encoderData")
        self.test_topic = rospy.get_param("~test_topic", "testCanData")
        self.heartbeat_topic = rospy.get_param("~heartbeat_topic", "/system/heartbeat")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.arm_joint_velocities_topic = "/robot/joint_command"
        self.motor_control_cmd_topic = rospy.get_param("~motor_control_cmd_topic", "/motor_control_cmd")
        self.drilling_status_topic = rospy.get_param("~drilling_status_topic", "/drilling_status")
        self.drilling_command_topic = rospy.get_param("~drilling_command_topic", "/drilling_command")
        self.heartbeat_message = rospy.get_param("~heartbeat_message", "roscan_bridge:running")
    
    def _initialize_messages(self) -> None:
        """Initialize all messages and register them with the message registry."""
        self.message_registry.register(EncoderMessage(self.ENCODER_FRAME_ID, self.encoder_pub))
        self.message_registry.register(TestMessage(self.TEST_FRAME_ID, self.test_pub))
        self.message_registry.register(GpsLatitudeMessage(self.GPS_LATITUDE_FRAME_ID, self.gps_pub))
        self.message_registry.register(GpsLongitudeMessage(self.GPS_LONGITUDE_FRAME_ID, self.gps_pub))
        self.message_registry.register(ImuOrientationMessage(self.IMU_ORIENTATION_FRAME_ID, self.imu_pub))
        self.message_registry.register(ImuLinearAccelMessage(self.IMU_LINEAR_ACCEL_FRAME_ID, self.imu_pub))
        self.message_registry.register(LoadCellMessage(self.LOAD_CELL_FRAME_ID, self.load_cell_pub))
        self.message_registry.register(DrillingStatusMessage(0x400, self.drilling_status_pub))

        # self.keyboard_control_parser = KeyboardControlMessage(0x100) # TODO: Get from params
        self.robot_arm_control_message = RobotArmControlMessage(0x101)  # TODO: Get from params
        self.motor_control_message = OutgoingMotorControlMessage(self.MOTOR_CONTROL_FRAME_ID)  # TODO: Get from params
        self.drilling_command_message = OutgoingDrillingCommandMessage(0x333)  # TODO: Get from params


    def _process_frame(self, frame_id: int, data: list) -> None:
        """Process a received CAN frame."""
        try:
            rospy.loginfo(f"Received Frame 0x{frame_id:03X}")
            if frame_id in [self.GPS_LATITUDE_FRAME_ID, self.GPS_LONGITUDE_FRAME_ID]:
                self._handle_gps_frame(frame_id, data)
            elif frame_id in [self.IMU_ORIENTATION_FRAME_ID, self.IMU_LINEAR_ACCEL_FRAME_ID]:
                self._handle_imu_frame(frame_id, data)
            else:
                message = self.message_registry.get_message(frame_id)
                if message:
                    frame = CanFrame(can_id=frame_id, dlc=len(data), data=data)
                    parsed_data = message.parse(frame)
                    if parsed_data:
                        message.handle(parsed_data)
                    else:
                        rospy.logwarn(f"Failed to parse data for frame ID 0x{frame_id:03X}")
                else:
                    rospy.logwarn(f"No message handler found for frame ID 0x{frame_id:03X}")
        except Exception as e:
            rospy.logerr(f"Error processing frame 0x{frame_id:03X}: {e}")

    def _handle_gps_frame(self, frame_id: int, data: list) -> None:
        """Handle GPS frames (latitude and longitude)."""
        message = self.message_registry.get_message(frame_id)
        if message:
            frame = CanFrame(can_id=frame_id, dlc=len(data), data=data)
            parsed_data = message.parse(frame)
            if parsed_data:
                frame_type = 'latitude' if frame_id == self.GPS_LATITUDE_FRAME_ID else 'longitude'
                complete_coords = self.gps_pairer.add_frame(parsed_data, frame_type)
                if complete_coords:
                    self._publish_gps_data(complete_coords)
                    rospy.logdebug(f"Published complete GPS coordinates: seq={complete_coords['sequence']}")
            else:
                rospy.logwarn(f"Failed to parse data for frame ID 0x{frame_id:03X}")

    def _handle_imu_frame(self, frame_id: int, data: list) -> None:
        """Handle IMU frames (orientation and linear acceleration)."""
        message = self.message_registry.get_message(frame_id)
        if message:
            frame = CanFrame(can_id=frame_id, dlc=len(data), data=data)
            parsed_data = message.parse(frame)
            if parsed_data:
                frame_type = 'orientation' if frame_id == self.IMU_ORIENTATION_FRAME_ID else 'linear_accel'
                complete_imu = self.imu_pairer.add_frame(parsed_data, frame_type)
                if complete_imu:
                    self._publish_imu_data(complete_imu)
                    rospy.logdebug(f"Published complete IMU data: seq={complete_imu['sequence']}")
            else:
                rospy.logwarn(f"Failed to parse data for frame ID 0x{frame_id:03X}")

    def _ddmm_to_decimal_degrees(self, ddmm_value: float) -> float:
        """Convert DDMM format to decimal degrees."""
        degrees = int(ddmm_value // 100)
        minutes = ddmm_value - (degrees * 100)
        decimal_degrees = degrees + (minutes / 60.0)
        return decimal_degrees

    def _publish_gps_data(self, gps_data: dict) -> None:
        """Publish GPS data as a NavSatFix message."""
        try:
            lat_decimal = self._ddmm_to_decimal_degrees(gps_data['latitude'])
            lon_decimal = self._ddmm_to_decimal_degrees(gps_data['longitude'])

            if not gps_data['is_north']:
                lat_decimal = -lat_decimal
            if not gps_data['is_east']:
                lon_decimal = -lon_decimal

            gps_msg = NavSatFix()
            gps_msg.header.stamp = rospy.Time.now()
            gps_msg.header.frame_id = "gps"
            gps_msg.latitude = lat_decimal
            gps_msg.longitude = lon_decimal
            gps_msg.altitude = 0.0
            gps_msg.status.status = NavSatFix.STATUS_FIX
            gps_msg.status.service = NavSatFix.SERVICE_GPS
            self.gps_pub.publish(gps_msg)
            rospy.logdebug(f"Published GPS: lat={lat_decimal:.8f}, lon={lon_decimal:.8f}, seq={gps_data['sequence']}")
        except Exception as e:
            rospy.logerr(f"Error publishing GPS data: {e}")

    def _publish_imu_data(self, imu_data: dict) -> None:
        """Publish IMU data as an Imu message."""
        try:
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu"

            imu_msg.orientation.x = imu_data['orientation_x']
            imu_msg.orientation.y = imu_data['orientation_y']
            imu_msg.orientation.z = imu_data['orientation_z']
            imu_msg.orientation.w = imu_data['orientation_w']

            imu_msg.linear_acceleration.x = imu_data['linear_accel_x']
            imu_msg.linear_acceleration.y = imu_data['linear_accel_y']
            imu_msg.linear_acceleration.z = imu_data['linear_accel_z']

            self.imu_pub.publish(imu_msg)
            rospy.logdebug(f"Published IMU data: seq={imu_data['sequence']}")
        except Exception as e:
            rospy.logerr(f"Error publishing IMU data: {e}")
    
    def _handle_drilling_status_frame(self, frame_id: int, data: list) -> None:
        """Handle DrillingStatus frames."""
        message = self.message_registry.get_message(frame_id)
        if message:
            frame = CanFrame(can_id=frame_id, dlc=len(data), data=data)
            parsed_data = message.parse(frame)
            if parsed_data:
                # The DrillingStatusMessage's handle method directly publishes the ROS message
                message.handle(parsed_data)
                rospy.logdebug(
                    f"Published DrillingStatus data: Height={parsed_data['current_height']:.2f} cm, "
                    f"Weight={parsed_data['current_weight']:.2f} g"
                )
            else:
                rospy.logwarn(f"Failed to parse data for DrillingStatus frame ID 0x{frame_id:03X}")
        else:
            rospy.logwarn(f"No DrillingStatus message handler found for frame ID 0x{frame_id:03X}")

    # def _keyboard_control_callback(self, msg: Float32MultiArray) -> None:
    #     """Handle keyboard control messages from ROS."""
    #     rospy.loginfo("Float32MultiArray message received")
    #     try:
    #         can_frame = self.keyboard_control_parser.parse(msg)
    #         if can_frame:
    #             self.communication_manager.send_frame(can_frame.can_id, can_frame.data)
    #         else:
    #             rospy.logerr("Failed to parse Twist message into a CAN frame.")
    #     except Exception as e:
    #         rospy.logerr(f"Error in keyboard control callback: {e}")
    
    def _robot_arm_control_callback(self, msg: PoseStamped) -> None:
        """Handle robot arm control messages from ROS."""
        try:
            rospy.loginfo("Robot arm control callback triggered")
            can_frame = self.robot_arm_control_message.build(msg)
            if can_frame:
                self.communication_manager.send_frame(can_frame.can_id, can_frame.data)
            else:
                rospy.logerr("Failed to parse PoseStamped into a CAN frame.")
        except Exception as e:
            rospy.logerr(f"Error in robot arm control callback: {e}")
    
    def _motor_control_callback(self, msg: Float32MultiArray) -> None:
        """Handle motor control messages from ROS."""
        try:
            rospy.loginfo("Motor control callback triggered")
            can_frame = self.motor_control_message.build(msg)
            if can_frame:
                self.communication_manager.send_frame(can_frame.can_id, can_frame.data)
            else:
                rospy.logerr("Failed to parse Float32MultiArray into a CAN frame.")
        except Exception as e:
            rospy.logerr(f"Error in motor control callback: {e}")

    def _drilling_command_callback(self, msg: DrillingCommand) -> None:
        """Handle drilling command messages from ROS and send them as CAN frames."""
        try:
            rospy.loginfo("DrillingCommand message received")
            can_frame = self.drilling_command_message.build(msg)
            if can_frame:
                self.communication_manager.send_frame(can_frame.can_id, can_frame.data)
                rospy.logdebug(
                    f"Sent DrillingCommand CAN frame 0x{can_frame.can_id:03X}: "
                    f"Height={msg.target_height_cm:.2f}cm, Gate={msg.gate_open}, "
                    f"Auger={msg.auger_on}, Up={msg.manual_up}, Down={msg.manual_down}"
                )
            else:
                rospy.logerr("Failed to build DrillingCommand into a CAN frame.")
        except Exception as e:
            rospy.logerr(f"Error in drilling command callback: {e}")
    
    def run(self) -> None:
        """Run the main loop of the ROSCAN node."""
        try:
            self.communication_manager.connect()
            rospy.loginfo("Communication manager connected.")
            self.communication_manager.start_receiving()
            rospy.loginfo("Communication manager started receiving.")
            rospy.loginfo("ROSCAN Node started successfully")
            rospy.spin()
        except RoscanError as e:
            rospy.logfatal(f"ROSCAN Error: {e}")
        except Exception as e:
            rospy.logfatal(f"Unhandled exception: {e}")
        finally:
            self.shutdown()
    
    def shutdown(self) -> None:
        """Shutdown the ROSCAN node."""
        rospy.loginfo("Shutting down ROSCAN Node...")
        self.communication_manager.stop_receiving()
        self.communication_manager.disconnect()
        rospy.loginfo("ROSCAN Node shutdown complete")


def main():
    """Main entry point for the ROSCAN node."""
    try:
        node = RoscanNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except Exception as e:
        rospy.logfatal(f"Failed to start ROSCAN Node: {e}")



if __name__ == "__main__":
    main()