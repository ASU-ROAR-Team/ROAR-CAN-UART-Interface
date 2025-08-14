#!/usr/bin/env python3
"""
Drilling mechanism parser module.
"""
from typing import Dict, Any, List, Optional
from .base_parser import BaseParser
from .message import CanMessage
# Removed `from enum import IntEnum` as CommandMode enum is replaced by bit flags

class DrillingParser(BaseParser):
    """
    Parser for drilling mechanism CAN messages.

    This parser handles the conversion of 3-byte CAN frames
    into structured data for the drilling system's high-level
    control, supporting both autonomous height commands and manual
    directional controls within a single frame.
    """
    SCALING_FACTOR = 10.0  # Represents 10 mm per unit for 1mm precision
    DRIVING_CAN_ID = 0x01  # Placeholder CAN ID. Please set a unique ID for your system.

    def parse(self, message: CanMessage) -> Optional[Dict[str, Any]]:
        """
        Parse a CAN message from the drilling mechanism (status feedback).

        Parameters
        ----------
        message : CanMessage
            The received CAN message. Expected DLC is 3.

        Returns
        -------
        Optional[Dict[str, Any]]
            A dictionary of parsed drilling status data, or None if parsing fails.
        """
        if message.dlc != 3:
            rospy.logwarn(f"DrillingParser: Received message with unexpected DLC: {message.dlc}. Expected 3.")
            return None

        try:
            # Bytes 0-1: Height Data (Unsigned 16-bit)
            # The simulator or low-level controller sends its current height here.
            height_raw = self.extract_16bit_values(message.data[0:2], 1)[0]
            height_cm = float(height_raw) / self.SCALING_FACTOR

            # Byte 2: Combined Control/Status Byte
            control_status_byte = message.data[2]
            
            # Extract individual bit flags
            gate_status = (control_status_byte >> 0) & 0x01  # Bit 0 for Gate
            auger_status = (control_status_byte >> 1) & 0x01 # Bit 1 for Auger
            manual_up_status = (control_status_byte >> 2) & 0x01 # Bit 2 for Manual Up
            manual_down_status = (control_status_byte >> 3) & 0x01 # Bit 3 for Manual Down

            return {
                "platform_height_cm": height_cm,
                "auger_status": "ON" if auger_status == 1 else "OFF",
                "servo_status": "OPEN" if gate_status == 1 else "CLOSED",
                "manual_up_active": True if manual_up_status == 1 else False,
                "manual_down_active": True if manual_down_status == 1 else False,
            }

        except (IndexError, ValueError) as e:
            rospy.logerr(f"Error parsing drilling status message: {e}")
            return None

    def create_command_message(self, target_height_cm: float, gate_open: bool, auger_on: bool, manual_up: bool, manual_down: bool) -> CanMessage:
        """
        Create a CAN message to command the drilling mechanism.

        This method generates a 3-byte payload for combined autonomous and manual control.

        Parameters
        ----------
        target_height_cm : float
            The target platform height in centimeters for autonomous mode.
            The low-level controller should use this if manual_up/down are False.
        gate_open : bool
            True to open the servo gate, False to close it.
        auger_on : bool
            True to turn the auger on, False to turn it off.
        manual_up : bool
            True to command platform movement UP (manual mode). Overrides target_height_cm.
        manual_down : bool
            True to command platform movement DOWN (manual mode). Overrides target_height_cm.

        Returns
        -------
        CanMessage
            A fully formed CAN message with a 3-byte payload ready for transmission.
        """
        
        # Byte 0-1: Height Data (16-bit Unsigned)
        # Even in manual modes, we send the last known target height or current height.
        # The low-level controller decides whether to use it based on manual_up/down bits.
        height_raw = int(target_height_cm * self.SCALING_FACTOR)
        height_bytes = height_raw.to_bytes(2, 'big', signed=False) # Unsigned 16-bit

        # Byte 2: Combined Control Byte
        control_byte = 0x00
        if gate_open:
            control_byte |= (1 << 0) # Set Bit 0 for Gate Open
        if auger_on:
            control_byte |= (1 << 1) # Set Bit 1 for Auger On
        if manual_up:
            control_byte |= (1 << 2) # Set Bit 2 for Manual Up
        if manual_down:
            control_byte |= (1 << 3) # Set Bit 3 for Manual Down
        
        # Ensure only one manual direction is active
        if manual_up and manual_down:
            rospy.logwarn("Both manual_up and manual_down commanded. Defaulting to STOP.")
            control_byte &= ~((1 << 2) | (1 << 3)) # Clear both bits
        
        payload = [
            height_bytes[0],
            height_bytes[1],
            control_byte
        ]

        return CanMessage(can_id=self.DRIVING_CAN_ID, dlc=3, data=payload)