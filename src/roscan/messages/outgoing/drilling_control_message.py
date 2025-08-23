from typing import Any, Dict, Optional

import rospy
from std_msgs.msg import Float32MultiArray
from roar_msgs.msg import DrillingCommand # Import your custom DrillingCommand message

from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import BuildingError
from roscan.messages.base_message import BaseMessage


class OutgoingDrillingCommandMessage(BaseMessage):
    """
    Message class for outgoing drilling command data.
    
    Converts roar_msgs/DrillingCommand messages to CAN frames for transmission.
    
    CAN Frame Structure:
    - Bytes 0-1 (2 bytes): Scaled target_height_cm (unsigned integer, little-endian)
    - Byte 2 (1 byte): Command byte with individual bits representing:
                      Bit 0: gate_open
                      Bit 1: auger_on
                      Bit 2: manual_up
                      Bit 3: manual_down
                      Bits 4-7: Unused (set to 0)
    """

    def __init__(self, can_id: int):
        """
        Initializes the OutgoingDrillingCommandMessage.

        Args:
            can_id (int): The CAN ID associated with this outgoing message.
        """
        super().__init__(can_id)
        # rospy.loginfo(f"OutgoingDrillingCommandMessage initialized for CAN ID: 0x{can_id:03X}")

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        """
        This is an outgoing message, so it does not parse incoming CAN frames.
        """
        return None

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        """
        This is an outgoing message, so it does not handle parsed data.
        """
        pass

    def build(self, msg: DrillingCommand) -> Optional[CanFrame]:
        """
        Builds a CAN frame from a DrillingCommand message.

        Args:
            msg: The DrillingCommand message to build the CAN frame from.

        Returns:
            Optional[CanFrame]: The built CAN frame, or None if building fails.
        """
        try:
            # Add a robust type check and conversion for target_height_cm
            if not isinstance(msg.target_height_cm, (int, float)):
                rospy.logerr(
                    f"Invalid type for msg.target_height_cm. "
                    f"Expected number, got {type(msg.target_height_cm)} with value: {msg.target_height_cm}"
                )
                # Raise a BuildingError if the type is incorrect
                raise BuildingError(f"Invalid type for target_height_cm: {type(msg.target_height_cm)}")

            # 1. Pack Target Height (2 bytes)
            # Ensure target_height_cm is definitively a float before multiplication
            target_height_as_float = float(msg.target_height_cm)
            scaled_height:int = int(target_height_as_float * 10) 
            print(type(scaled_height))
            print("AAAAAAAAAAAAAAAAAAAAAAAAAA")
            # Check for overflow if height exceeds 2-byte unsigned limit
            if not (0 <= scaled_height <= 65535):
                rospy.logwarn(
                    f"Target height {msg.target_height_cm} cm (scaled to {scaled_height}) "
                    f"exceeds 2-byte unsigned integer limit (0-65535). Clamping value."
                )
                scaled_height = max(0, min(scaled_height, 65535)) # Clamp to valid range
            print("CCCCCCCCCCC ")
            height_bytes = scaled_height.to_bytes(2, 'little')
            print("BBBBBBBBB")
            # 2. Pack Boolean Commands (1 byte)
            command_byte = 0
            if msg.gate_open:
                command_byte |= (1 << 0)  # Set Bit 0
            if msg.auger_on:
                command_byte |= (1 << 1)  # Set Bit 1
            if msg.manual_up:
                command_byte |= (1 << 2)  # Set Bit 2
            if msg.manual_down:
                command_byte |= (1 << 3)  # Set Bit 3
            # Bits 4-7 remain 0 as they are unused

            # Combine all bytes
            # Convert bytes objects to lists of integers for the CanFrame data field
            data = list(height_bytes) + [command_byte]
            print("DDDDDDDDDD")
            print(type(data))
            print(type(self.can_id))

            # The total data length will be 2 (height) + 1 (command_byte) = 3 bytes.
            return CanFrame(can_id=int(self.can_id), dlc=len(data), data=data)

        except Exception as e:
            # rospy.logerr(f"Error building drilling command frame for CAN ID 0x{self.can_id:03X}: {e}")
            raise BuildingError(f"Drilling command building failed: {e}") from e
