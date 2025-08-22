#!/usr/bin/env python3
"""
CAN frame data structure and formatting.

This module provides a CanFrame class for representing CAN bus messages
with proper validation, serialization, and deserialization.
"""

from dataclasses import dataclass
from typing import List, Optional
import struct
import logging

# Set up module logger
logger = logging.getLogger(__name__)


@dataclass
class CanFrame:
    """
    CAN message data structure with validation and serialization.
    
    Represents complete CAN message with ID, data length, payload,
    and methods for byte conversion with framing and checksums.
    
    Attributes:
        can_id: CAN message identifier (16-bit value)
        dlc: Data Length Code - number of data bytes (0-8)
        data: List of data bytes (0-8 bytes)
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
        if not isinstance(self.data, list):
            raise ValueError("Data must be a list of integers")
        
        if len(self.data) > 8:
            raise ValueError("CAN data payload cannot exceed 8 bytes")
        if self.dlc != len(self.data):
            raise ValueError("DLC must match data length")
        if not 0 <= self.can_id <= 0xFFFF:
            raise ValueError("CAN ID must be between 0 and 0xFFFF")
        if not all(0 <= byte <= 255 for byte in self.data):
            raise ValueError("All data bytes must be between 0 and 255")
    
    def to_bytes(self) -> bytes:
        """
        Convert message to byte array for transmission.
        
        Creates complete CAN message frame with start byte, ID, DLC,
        data payload, checksum, and end byte.
        
        Returns:
            Complete message frame ready for transmission
        """
        try:
            # Start byte, ID (2 bytes), DLC, data, checksum, end byte
            message = bytearray([0xAA])
            message.extend(self.can_id.to_bytes(2, 'big'))
            message.append(self.dlc)
            message.extend(self.data)
            message.append(self._calculate_checksum())
            message.append(0x55)
            return bytes(message)
        except Exception as e:
            logger.error(f"Error serializing CAN frame: {e}")
            raise
    
    def _calculate_checksum(self) -> int:
        """
        Calculate XOR checksum of ID, DLC, and data.
        
        Computes XOR of high/low CAN ID bytes, DLC, and each data byte.
        
        Returns:
            Single byte checksum value (0-255)
        """
        try:
            checksum = (self.can_id >> 8) ^ (self.can_id & 0xFF) ^ self.dlc
            for byte in self.data:
                checksum ^= byte
            return checksum & 0xFF
        except Exception as e:
            logger.error(f"Error calculating checksum: {e}")
            raise
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'CanFrame':
        """
        Create message from received byte array.
        
        Class method that parses complete CAN message frame and
        validates checksum. Can be called directly on class.
        
        Args:
            data: Raw byte array containing complete message frame
            
        Returns:
            New message instance with parsed data
            
        Raises:
            ValueError: If message too short, invalid framing, or checksum mismatch
        """
        try:
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
        except Exception as e:
            logger.error(f"Error parsing CAN frame from bytes: {e}")
            raise
    
    def __str__(self) -> str:
        """String representation of the CAN frame."""
        return f"CanFrame(ID=0x{self.can_id:03X}, DLC={self.dlc}, Data={self.data})"
    
    def __repr__(self) -> str:
        """Detailed string representation of the CAN frame."""
        return (f"CanFrame(can_id=0x{self.can_id:03X}, dlc={self.dlc}, "
                f"data={self.data})")