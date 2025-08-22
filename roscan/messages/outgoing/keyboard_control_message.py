

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
            linear_x = int(msg.linear.x * 1000)
            angular_z = int(msg.angular.z * 1000)

            data = list(linear_x.to_bytes(2, 'little', signed=True)) + \
                   list(angular_z.to_bytes(2, 'little', signed=True))

            return CanFrame(can_id=self.can_id, dlc=len(data), data=data)

        except Exception as e:
            rospy.logerr(f"Error building keyboard control frame: {e}")
            raise BuildingError(f"Keyboard control building failed: {e}") from e

