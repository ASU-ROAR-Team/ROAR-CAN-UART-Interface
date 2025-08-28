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

    def map_value(self, value: float, from_min: float, from_max: float, to_min: float, to_max: float) -> int:
        """
        Maps a value from one range to another.
        Ensures the returned value is an integer and clamped to the target range.
        """
        # Clamp the value to the source range
        clamped_value = max(from_min, min(from_max, value))
        # Perform the mapping
        mapped_value = (clamped_value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min
        return int(round(mapped_value))

    def build(self, msg: Float64MultiArray) -> Optional[CanFrame]:
        """
        Builds an 8-byte CAN frame from a Float64MultiArray message containing 6 joint values.

        Each joint value is converted from its specific range to degrees,
        then mapped to a 10-bit integer in the range [0, 1023].

        Args:
            msg: The Float64MultiArray message to build the CAN frame from.

        Returns:
            The built CAN frame, or None if building fails.
        """
        try:
            if len(msg.data) != 6:
                raise BuildingError(f"Expected 6 joint values, got {len(msg.data)}")
            
            # A list to store the final 10-bit integer values
            mapped_values = []
            
            # Joint 1 (index 0): (-pi, pi) rad -> (-180, 180) deg -> (0, 1023)
            # The input range is slightly different than (-pi, pi) for conversion so we will convert it from (-180,180)
            joint1_deg = math.degrees(msg.data[0])
            mapped_values.append(self.map_value(joint1_deg, -180.0, 180.0, 0.0, 1023.0))

            # Joint 2 (index 1): (0, 1.5*pi) rad -> (0, 270) deg -> (0, 1023)
            joint2_deg = math.degrees(msg.data[1])
            mapped_values.append(self.map_value(joint2_deg, 0.0, 270.0, 0.0, 1023.0))

            # Joint 3 (index 2): (0, 2*pi) rad -> (0, 360) deg -> (0, 1023)
            joint3_deg = math.degrees(msg.data[2])
            mapped_values.append(self.map_value(joint3_deg, 0.0, 360.0, 0.0, 1023.0))

            # Joint 4 (index 3): (0, 2*pi) rad -> (0, 360) deg -> (0, 1023)
            joint4_deg = math.degrees(msg.data[3])
            mapped_values.append(self.map_value(joint4_deg, 0.0, 360.0, 0.0, 1023.0))
            
            # Joint 5 (index 4): (-pi, pi) rad -> (-180, 180) deg -> (0, 1023)
            joint5_deg = math.degrees(msg.data[4])
            mapped_values.append(self.map_value(joint5_deg, -180.0, 180.0, 0.0, 1023.0))
            
            # End Effector (index 5): (0, 360) deg -> (0, 1023)
            # This value is already in degrees, so we only need to map it
            mapped_values.append(self.map_value(msg.data[5], 0.0, 360.0, 0.0, 1023.0))

            # Log the joint values after conversion to degrees
            rospy.loginfo(f"J1: {joint1_deg:.2f}, J2: {joint2_deg:.2f}, J3: {joint3_deg:.2f}, J4: {joint4_deg:.2f}, J5: {joint5_deg:.2f}, EE: {msg.data[5]:.2f}")

            # Scale and pack the 6 joint values into a 64-bit integer
            packed_data = 0
            packed_data |= (mapped_values[0] & 0x3FF)
            packed_data |= (mapped_values[1] & 0x3FF) << 10
            packed_data |= (mapped_values[2] & 0x3FF) << 20
            packed_data |= (mapped_values[3] & 0x3FF) << 30
            packed_data |= (mapped_values[4] & 0x3FF) << 40
            packed_data |= (mapped_values[5] & 0x3FF) << 50

            # Convert the 64-bit integer to an 8-byte list (little-endian)
            can_data = list(packed_data.to_bytes(8, 'little'))

            return CanFrame(can_id=self.can_id, dlc=8, data=can_data)

        except Exception as e:
            rospy.logerr(f"Error building robot arm control frame: {e}")
            raise BuildingError(f"Robot arm control building failed: {e}") from e
