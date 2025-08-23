

from typing import Any, Dict, Optional

import rospy
from geometry_msgs.msg import Twist

from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import BuildingError
from roscan.messages.base_message import BaseMessage


class KeyboardControlMessage(BaseMessage):
    """
    Message class for keyboard control data.
    """

    def __init__(self, can_id: int):
        super().__init__(can_id)

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        # This is an outgoing message, so we don't need to parse anything
        return None

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        # This is an outgoing message, so we don't need to handle anything
        pass

    def build(self, msg: Twist) -> Optional[CanFrame]:
        """
        Builds a CAN frame from a Twist message.

        Args:
            msg: The Twist message to build the CAN frame from.

        Returns:
            The built CAN frame, or None if building fails.
        """
        try:
            # Assuming the linear x and angular z are scaled to fit in 2 bytes each
            linear_velocity = msg.linear.x
            angular_velocity = msg.angular.z
            max_motor_rpm = 15
            
            # Convert velocities to motor RPMs using differential drive kinematics
            left_motor_rpm = (linear_velocity - angular_velocity) * 10
            right_motor_rpm = (linear_velocity + angular_velocity) * 10
            
            # Clamp RPMs to valid range
            left_motor_rpm = max(-max_motor_rpm, min(max_motor_rpm, left_motor_rpm))
            right_motor_rpm = max(-max_motor_rpm, min(max_motor_rpm, right_motor_rpm))
            
            # Map RPMs to single byte values (0-127) where 0 RPM = 64
            # Negative RPMs: 0-63, Positive RPMs: 65-127, Zero RPM: 64
            left_motor_signal = int(left_motor_rpm * (63 / max_motor_rpm) + 64)
            right_motor_signal = int(right_motor_rpm * (63 / max_motor_rpm) + 64)
            
            # Ensure values are within valid range
            left_motor_signal = max(0, min(127, left_motor_signal))
            right_motor_signal = max(0, min(127, right_motor_signal))
            
            # Create the CAN frame data with one byte per wheel (6 wheels total)
            # Bytes 0-2 are right motors (top to bottom), bytes 3-5 are left motors (top to bottom)
            frame_data = [
                right_motor_signal,  # Right top motor
                right_motor_signal,  # Right middle motor
                right_motor_signal,  # Right bottom motor
                left_motor_signal,   # Left top motor
                left_motor_signal,   # Left middle motor
                left_motor_signal,   # Left bottom motor
                0,                   # Padding to maintain 8-byte frame
                0                    # Padding to maintain 8-byte frame
            ]

            return CanFrame(can_id=self.can_id, dlc=len(frame_data), data=frame_data)

        except Exception as e:
            rospy.logerr(f"Error building keyboard control frame: {e}")
            raise BuildingError(f"Keyboard control building failed: {e}") from e

