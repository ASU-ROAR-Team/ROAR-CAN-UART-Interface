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
        Builds a CAN frame from a Float32MultiArray message.

        Args:
            msg: The Float32MultiArray message to build the CAN frame from.

        Returns:
            The built CAN frame, or None if building fails.
        """
        try:
            # Expecting exactly 2 motor values (left and right)
            if len(msg.data) != 2:
                raise BuildingError(f"Expected 2 motor values, got {len(msg.data)}")
            
            # Scale the motor values to fit in 4 bytes each (signed integer)
            left_motor = int(msg.data[0] * 1000)  # Scale by 1000
            right_motor = int(msg.data[1] * 1000)  # Scale by 1000

            # Convert to bytes (4 bytes each, little endian, signed)
            data = list(left_motor.to_bytes(4, 'little', signed=True)) + \
                   list(right_motor.to_bytes(4, 'little', signed=True))

            return CanFrame(can_id=self.can_id, dlc=len(data), data=data)

        except Exception as e:
            rospy.logerr(f"Error building motor control frame: {e}")
            raise BuildingError(f"Motor control building failed: {e}") from e