
from typing import Any, Dict, Optional

import rospy
from sensor_msgs.msg import Imu

from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import ParsingError
from roscan.messages.base_message import BaseMessage


class ImuOrientationMessage(BaseMessage):
    """
    Message class for IMU orientation data.
    """

    def __init__(self, can_id: int, publisher: rospy.Publisher):
        super().__init__(can_id)
        self.publisher = publisher

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        try:
            if len(frame.data) != 8:
                raise ParsingError(f"Invalid data length for IMU orientation frame: {len(frame.data)}")

            orientation_x = int.from_bytes(frame.data[:2], 'little', signed=True) / 1000.0
            orientation_y = int.from_bytes(frame.data[2:4], 'little', signed=True) / 1000.0
            orientation_z = int.from_bytes(frame.data[4:6], 'little', signed=True) / 1000.0
            orientation_w = int.from_bytes(frame.data[6:8], 'little', signed=True) / 1000.0

            result = {
                "orientation_x": orientation_x,
                "orientation_y": orientation_y,
                "orientation_z": orientation_z,
                "orientation_w": orientation_w,
            }
            rospy.logdebug(f"Parsed IMU orientation frame: {result}")
            return result

        except Exception as e:
            rospy.logerr(f"Error parsing IMU orientation frame: {e}")
            raise ParsingError(f"IMU orientation parsing failed: {e}") from e

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        # The pairing logic will be handled in the RoscanNode for now
        pass


class ImuLinearAccelMessage(BaseMessage):
    """
    Message class for IMU linear acceleration data.
    """

    def __init__(self, can_id: int, publisher: rospy.Publisher):
        super().__init__(can_id)
        self.publisher = publisher

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        try:
            if len(frame.data) != 6:
                raise ParsingError(f"Invalid data length for IMU linear acceleration frame: {len(frame.data)}")

            linear_accel_x = int.from_bytes(frame.data[:2], 'little', signed=True) / 100.0
            linear_accel_y = int.from_bytes(frame.data[2:4], 'little', signed=True) / 100.0
            linear_accel_z = int.from_bytes(frame.data[4:6], 'little', signed=True) / 100.0

            result = {
                "linear_accel_x": linear_accel_x,
                "linear_accel_y": linear_accel_y,
                "linear_accel_z": linear_accel_z,
            }
            rospy.logdebug(f"Parsed IMU linear acceleration frame: {result}")
            return result

        except Exception as e:
            rospy.logerr(f"Error parsing IMU linear acceleration frame: {e}")
            raise ParsingError(f"IMU linear acceleration parsing failed: {e}") from e

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        # The pairing logic will be handled in the RoscanNode for now
        pass
