from typing import Any, Dict, Optional

import rospy
from roar_msgs.msg import DrillingCommand, DrillingStatus # Import both command and status messages
from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import BuildingError
from roscan.messages.base_message import BaseMessage


class OutgoingDrillingCommandMessage(BaseMessage):
    """
    Message class for outgoing drilling command data.
    
    Converts roar_msgs/DrillingCommand messages to CAN frames for transmission.
    
    CAN Frame Structure:
    - Bytes 0-1 (2 bytes): Scaled target_height_cm (signed integer, little-endian)
    - Byte 2 (1 byte): Command byte with individual bits representing:
                      Bit 0: gate_open
                      Bit 1: auger_on
                      Bit 2: manual_up
                      Bit 3: manual_down
                      Bits 4-7: Unused (set to 0)
    """

    def __init__(self, can_id: int):
        """
        Initializes the OutgoingDrillingCommandMessage and sets up a subscriber
        for drilling feedback.

        Args:
            can_id (int): The CAN ID associated with this outgoing message.
        """
        super().__init__(can_id)
        
        # Initialize the last known height from feedback
        self._last_known_height = 0.0
        
        # Create a subscriber for the drilling status feedback topic
        # This callback updates the last known height
        rospy.Subscriber("/drilling/feedback", DrillingStatus, self._feedback_callback)
        rospy.loginfo(f"OutgoingDrillingCommandMessage initialized for CAN ID: 0x{can_id:03X}. "
                      "Subscribing to /drilling/feedback.")

    def _feedback_callback(self, msg: DrillingStatus) -> None:
        """
        Callback to update the last known height from the drilling feedback topic.
        """
        self._last_known_height = msg.current_height
        rospy.logdebug(f"Updated last known height to: {self._last_known_height:.2f} cm")

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
            # 1. Determine the target height
            target_height = msg.target_height_cm
            if msg.manual_up or msg.manual_down:
                # Override the target height with the last known height from feedback
                target_height = self._last_known_height
                rospy.logdebug(f"Manual command activated. Target height set to feedback height: {target_height:.2f} cm")
            
            # 2. Pack Target Height (2 bytes as a SIGNED integer)
            # We scale the height by 10 to preserve one decimal place.
            scaled_height: int = int(target_height * 10)

            # Check for overflow if height exceeds 2-byte signed integer limits
            # A 2-byte signed integer ranges from -32768 to 32767.
            if not (-32768 <= scaled_height <= 32767):
                rospy.logwarn(
                    f"Target height {target_height} cm (scaled to {scaled_height}) "
                    f"exceeds 2-byte signed integer limit (-32768 to 32767). Clamping value."
                )
                scaled_height = max(-32768, min(scaled_height, 32767)) # Clamp to valid range

            # Convert to bytes using a signed integer
            height_bytes = scaled_height.to_bytes(2, 'little', signed=True)

            # 3. Pack Boolean Commands (1 byte)
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
            data = list(height_bytes) + [command_byte]
            
            # The total data length will be 2 (height) + 1 (command_byte) = 3 bytes.
            return CanFrame(can_id=int(self.can_id), dlc=len(data), data=data)

        except Exception as e:
            # Log the specific error and re-raise it as a BuildingError
            rospy.logerr(f"Error building drilling command frame for CAN ID 0x{self.can_id:03X}: {e}")
            raise BuildingError(f"Drilling command building failed: {e}") from e

if __name__ == '__main__':
    try:
        rospy.init_node('drilling_command_node', anonymous=True)
        # Assuming a CAN ID is defined elsewhere, e.g., from a config file or rosparam
        can_id = 0x123  # Example CAN ID
        command_message = OutgoingDrillingCommandMessage(can_id)
        
        # The main loop of your program goes here.
        # This example assumes another part of the system is publishing to /drilling/command
        # and this node simply handles the conversions in its build method when needed.
        
        # Keep the node running and process callbacks
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
