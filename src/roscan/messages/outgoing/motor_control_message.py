from typing import Any, Dict, Optional

import rospy
from std_msgs.msg import Float32MultiArray

from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import BuildingError
from roscan.messages.base_message import BaseMessage


class OutgoingMotorControlMessage(BaseMessage):
    """
    Message class for outgoing motor control data.
    
    Converts Float32MultiArray messages to CAN frames for transmission.
    """

    def __init__(self, can_id: int):
        super().__init__(can_id)

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        # This is an outgoing message, so we don't need to parse anything
        return None

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        # This is an outgoing message, so we don't need to handle anything
        pass

    def build(self, msg: Float32MultiArray) -> Optional[CanFrame]:
        """
        Builds a CAN frame from a Float32MultiArray message for 6 motors.

        Args:
            msg: The Float32MultiArray message to build the CAN frame from.

        Returns:
            The built CAN frame, or None if building fails.
        """
        try:
            max_motor_rpm = 15.0  # Maximum absolute RPM value

            # Expecting exactly 6 motor values: [right_front, right_middle, right_rear, left_front, left_middle, left_rear]
            if len(msg.data) != 6:
                raise BuildingError(f"Expected 6 motor values, got {len(msg.data)}")

            # Clamp and map each motor RPM to a single byte (0-127, 64 is zero)
            def map_rpm_to_signal(rpm: float) -> int:
                rpm = max(-max_motor_rpm, min(max_motor_rpm, rpm))
                signal = int(rpm * (63 / max_motor_rpm) + 64)
                return max(0, min(127, signal))

            right_signals = [map_rpm_to_signal(msg.data[i]) for i in range(3)]  # right_front, right_middle, right_rear
            left_signals = [map_rpm_to_signal(msg.data[i]) for i in range(3, 6)]  # left_front, left_middle, left_rear

            # Compose the 8-byte CAN frame: [right_front, right_middle, right_rear, left_front, left_middle, left_rear, 0, 0]
            frame_data = right_signals + left_signals + [0, 0]

            return CanFrame(can_id=self.can_id, dlc=len(frame_data), data=frame_data)

        except Exception as e:
            rospy.logerr(f"Error building motor control frame: {e}")
            raise BuildingError(f"Motor control building failed: {e}") from e