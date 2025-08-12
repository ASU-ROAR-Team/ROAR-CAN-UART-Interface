#!/usr/bin/env python3
"""
CAN message data structure and formatting.

Provides CanMessage class for representing CAN bus messages with
framing, checksum calculation, and validation.
"""
from dataclasses import dataclass
from typing import List, Optional
import struct

@dataclass
class CanMessage:
    """
    CAN message data structure with validation and serialization.
    
    Represents complete CAN message with ID, data length, payload,
    and methods for byte conversion with framing and checksums.
    
    Attributes
    ----------
    can_id : int
        CAN message identifier (16-bit value)
    dlc : int
        Data Length Code - number of data bytes (0-8)
    data : List[int]
        List of data bytes (0-8 bytes)
    
    Raises
    ------
    ValueError
        If data length exceeds 8 bytes or DLC doesn't match data length
    """
    can_id: int
    dlc: int
    data: List[int]
    
    def __post_init__(self):
        """
        Validate message after initialization.
        
        Ensures CAN protocol constraints:
        - Data payload cannot exceed 8 bytes
        - DLC must match actual data length
        - CAN ID must be between 0 and 0xFFFF
        """
        
        if len(self.data) > 8:
            raise ValueError("CAN data payload cannot exceed 8 bytes")
        if self.dlc != len(self.data):
            raise ValueError("DLC must match data length")
        if not 0 <= self.can_id <= 0xFFFF:
            raise ValueError("CAN ID must be between 0 and 0xFFFF")
    
    def to_bytes(self) -> bytes:
        """
        Convert message to byte array for transmission.
        
        Creates complete CAN message frame with start byte, ID, DLC,
        data payload, checksum, and end byte.
        
        Returns
        -------
        bytes
            Complete message frame ready for transmission
        """

        # Start byte, ID (2 bytes), DLC, data, checksum, end byte
        message = bytearray([0xAA])
        message.extend(self.can_id.to_bytes(2, 'big'))
        message.append(self.dlc)
        message.extend(self.data)
        message.append(self._calculate_checksum())
        message.append(0x55)
        return bytes(message)
    
    def _calculate_checksum(self) -> int:
        """
        Calculate XOR checksum of ID, DLC, and data.
        
        Computes XOR of high/low CAN ID bytes, DLC, and each data byte.
        
        Returns
        -------
        int
            Single byte checksum value (0-255)
        """

        checksum = (self.can_id >> 8) ^ (self.can_id & 0xFF) ^ self.dlc
        for byte in self.data:
            checksum ^= byte
        return checksum
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'CanMessage':
        """
        Create message from received byte array.
        
        Class method that parses complete CAN message frame and
        validates checksum. Can be called directly on class.
        
        Parameters
        ----------
        data : bytes
            Raw byte array containing complete message frame
            
        Returns
        -------
        CanMessage
            New message instance with parsed data
            
        Raises
        ------
        ValueError
            If message too short, invalid framing, or checksum mismatch
        """
        
        if len(data) < 6:  # Minimum: start + id + dlc + checksum + end
            raise ValueError("Message too short")
        
        if data[0] != 0xAA or data[-1] != 0x55:
            raise ValueError("Invalid message framing")
        
        can_id = int.from_bytes(data[1:3], 'big')
        dlc = data[3]
        message_data = list(data[4:4+dlc])
        checksum = data[4+dlc]
        
        # Create temporary message to use its checksum calculation
        temp_message = cls(can_id, dlc, message_data)
        expected_checksum = temp_message._calculate_checksum()
        
        if checksum != expected_checksum:
            raise ValueError(f"Checksum mismatch: expected {expected_checksum:02X}, got {checksum:02X}")
        
        return temp_message 