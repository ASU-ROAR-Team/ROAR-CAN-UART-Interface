

from typing import Any, Dict, Optional

import rospy
from geometry_msgs.msg import PoseStamped

from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import BuildingError
from roscan.messages.base_message import BaseMessage


class RobotArmControlMessage(BaseMessage):
    """
    Message class for robot arm control data.
    """

    def __init__(self, can_id: int):
        super().__init__(can_id)

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        # This is an outgoing message, so we don't need to parse anything
        return None

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        # This is an outgoing message, so we don't need to handle anything
        pass

    def build(self, msg: PoseStamped) -> Optional[CanFrame]:
        """
        Builds a CAN frame from a PoseStamped message.

        Args:
            msg: The PoseStamped message to build the CAN frame from.

        Returns:
            The built CAN frame, or None if building fails.
        """
        try:
            # Assuming the position x, y, z and orientation x, y, z, w are scaled to fit in 2 bytes each
            pos_x = int(msg.pose.position.x * 1000)
            pos_y = int(msg.pose.position.y * 1000)
            pos_z = int(msg.pose.position.z * 1000)

            data = list(pos_x.to_bytes(2, 'little', signed=True)) + \
                   list(pos_y.to_bytes(2, 'little', signed=True)) + \
                   list(pos_z.to_bytes(2, 'little', signed=True))

            return CanFrame(can_id=self.can_id, dlc=len(data), data=data)

        except Exception as e:
            rospy.logerr(f"Error building robot arm control frame: {e}")
            raise BuildingError(f"Robot arm control building failed: {e}") from e

