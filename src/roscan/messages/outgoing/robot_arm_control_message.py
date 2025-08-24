from typing import Any, Dict, Optional
import math
import rospy
from std_msgs.msg import Float64MultiArray

from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import BuildingError
from roscan.messages.base_message import BaseMessage


class RobotArmControlMessage(BaseMessage):
    """
    Message class for robot arm control data.
    Converts Float64MultiArray messages to CAN frames for transmission.
    """

    def __init__(self, can_id: int):
        super().__init__(can_id)

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        # This is an outgoing message, so we don't need to parse anything
        return None

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        # This is an outgoing message, so we don't need to handle anything
        pass

    def build(self, msg: Float64MultiArray) -> Optional[CanFrame]:
        """
        Builds an 8-byte CAN frame from a Float64MultiArray message containing 6 joint values.

        The 6 joint values are packed into a 64-bit integer with the following layout:
        - joint1: bits 0-9
        - joint2: bits 10-19
        - joint3: bits 20-29
        - joint4: bits 30-39
        - joint5: bits 40-49
        - end_effector: bits 50-59
        - unused: bits 60-63

        Args:
            msg: The Float64MultiArray message to build the CAN frame from.

        Returns:
            The built CAN frame, or None if building fails.
        """
        try:
            if len(msg.data) != 6:
                raise BuildingError(f"Expected 6 joint values, got {len(msg.data)}")

            # Scale and pack the 6 joint values into a 64-bit integer
            packed_data = 0
            
            # Helper to scale a value from [-pi, pi] to [0, 1023]
            def scale_joint_value(value):
                scaled = ((value + math.pi) / (2 * math.pi)) * 1023
                return int(max(0, min(1023, scaled)))

            # Pack each joint value into its 10-bit slot
            packed_data |= (scale_joint_value(msg.data[0]) & 0x3FF)
            packed_data |= (scale_joint_value(msg.data[1]) & 0x3FF) << 10
            packed_data |= (scale_joint_value(msg.data[2]) & 0x3FF) << 20
            packed_data |= (scale_joint_value(msg.data[3]) & 0x3FF) << 30
            packed_data |= (scale_joint_value(msg.data[4]) & 0x3FF) << 40
            packed_data |= (scale_joint_value(msg.data[5]) & 0x3FF) << 50

            # Convert the 64-bit integer to an 8-byte list (little-endian)
            can_data = list(packed_data.to_bytes(8, 'little'))

            return CanFrame(can_id=self.can_id, dlc=8, data=can_data)

        except Exception as e:
            rospy.logerr(f"Error building robot arm control frame: {e}")
            raise BuildingError(f"Robot arm control building failed: {e}") from e