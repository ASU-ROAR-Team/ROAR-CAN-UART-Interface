#!/usr/bin/env python3
"""
Drilling mechanism parser module.


Important CAN DATA FORMAT:
1. Platform height: 2 bytes
2. Auger status: 1 byte
3. Servo status: 1 byte
4. Manual up active: 1 byte
5. Manual down active: 1 byte

In this exact order.
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

    def __init__(self):
        """
        Initialize the drilling parser.
        """
        
        super().__init__()
        self.SCALING_FACTOR = 10.0  # Represents 10 mm per unit for 1mm precision
        self.DRIVING_CAN_ID = 0x01  # Placeholder CAN ID. Please set a unique ID for your system.
        self.CAN_MESSAGE_FORMAT = {
            "platform_height_cm": 0, # 2 bytes
            "auger_status": 0, # 1 byte
            "servo_status": 0, # 1 byte
            "manual_up_active": 0, # 1 byte
            "manual_down_active": 0, # 1 byte
        }

    def parse(self, message: CanMessage) -> Optional[Dict[str, Any]]:
        """
        Parse a CAN message from the drilling mechanism (status feedback).

        Parameters
        ----------
        message : CanMessage
            The received CAN message. Expected DLC is 6.

        Returns
        -------
        Optional[Dict[str, Any]]
            A dictionary of parsed drilling status data, or None if parsing fails.
        """
        if message.dlc != 6:
            rospy.logwarn(f"DrillingParser: Received message with unexpected DLC: {message.dlc}. Expected 3.")
            return None

        try:
            data = message.data

            # Bytes 0-1: Height Data (Unsigned 16-bit)
            # The simulator or low-level controller sends its current height here.
            self.CAN_MESSAGE_FORMAT["platform_height_cm"] = (data[0] << 8 | data[1]) / self.SCALING_FACTOR

            # Byte 3-5: Control/Status Bytes
            self.CAN_MESSAGE_FORMAT["auger_status"] = True if (data[2] & 0x01) == 1 else False
            self.CAN_MESSAGE_FORMAT["servo_status"] = True if (data[3] & 0x01) == 1 else False
            self.CAN_MESSAGE_FORMAT["manual_up_active"] = True if (data[4] & 0x01) == 1 else False
            self.CAN_MESSAGE_FORMAT["manual_down_active"] = True if (data[5] & 0x01) == 1 else False

            return self.CAN_MESSAGE_FORMAT

        except (IndexError, ValueError) as e:
            rospy.logerr(f"Error parsing drilling status message: {e}")
            return None

    def create_command_message(self, CAN_PARAMETERS: Dict[str, Any]) -> CanMessage:
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
        height_raw = int(CAN_PARAMETERS["platform_height_cm"] * self.SCALING_FACTOR)
        height_bytes = height_raw.to_bytes(2, 'big', signed=False) # Unsigned 16-bit

        payload = [
            int(height_bytes[1]),
            int(height_bytes[0]),
            int(CAN_PARAMETERS["auger_status"]),
            int(CAN_PARAMETERS["servo_status"]),
        ]



        if "manual_status_flags" in CAN_PARAMETERS:
            payload.append(int(CAN_PARAMETERS["manual_status_flags"]))
            payload.append(int(CAN_PARAMETERS["manual_up_active"]))
            payload.append(int(CAN_PARAMETERS["manual_down_active"]))
        else:
            payload.extend([0, 0, 0])
        

        return CanMessage(can_id=self.DRIVING_CAN_ID, dlc=6, data=payload)