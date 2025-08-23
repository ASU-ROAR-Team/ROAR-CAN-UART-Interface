#!/usr/bin/env python3
"""
Abstract base class for all CAN message parsers.

This module defines the base interface and common functionality
for all CAN message parsers in the system.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, List, Optional
import logging

from .can_frame import CanFrame
from .exceptions import ParsingError

# Set up module logger
logger = logging.getLogger(__name__)


class BaseParser(ABC):
    """
    Abstract base class for CAN message parsers.
    
    This class defines the common interface and basic functionality
    that all CAN message parsers should implement. It provides
    error handling, logging, and utility methods that can be used
    by concrete parser implementations.
    
    Attributes:
        logger: Logger instance for this parser
    """
    
    def __init__(self, name: str = None):
        """
        Initialize the base parser.
        
        Args:
            name: Optional name for the parser, used in logging
        """
        self.name = name or self.__class__.__name__
        self.logger = logging.getLogger(f"{__name__}.{self.name}")
    
    @abstractmethod
    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        """
        Parse a CAN frame into structured data.
        
        This method must be implemented by all concrete parser classes.
        It should convert the raw CAN frame data into a dictionary
        containing the parsed information.
        
        Args:
            frame: The CAN frame to parse
            
        Returns:
            Parsed data dictionary or None if parsing fails
            
        Raises:
            ParsingError: If there's an error during parsing
        """
        pass
    
    def _validate_frame_length(self, frame: CanFrame, expected_length: int) -> bool:
        """
        Validate that a frame has the expected length.
        
        Args:
            frame: The CAN frame to validate
            expected_length: The expected length in bytes
            
        Returns:
            True if the frame length matches expected length
            
        Raises:
            ParsingError: If the frame length doesn't match expected length
        """
        if len(frame.data) != expected_length:
            error_msg = (f"Invalid frame length for {self.name}: "
                        f"expected {expected_length} bytes, got {len(frame.data)}")
            self.logger.error(error_msg)
            raise ParsingError(error_msg)
        return True
    
    def _extract_bits(self, byte_value: int, start_bit: int, bit_count: int) -> int:
        """
        Extract a sequence of bits from a byte value.
        
        Args:
            byte_value: The byte value to extract bits from
            start_bit: The starting bit position (0-7, where 0 is LSB)
            bit_count: The number of bits to extract
            
        Returns:
            The extracted bits as an integer
            
        Example:
            _extract_bits(0b11010110, 2, 3) -> 0b101 (5)
        """
        if not (0 <= start_bit <= 7):
            raise ValueError("Start bit must be between 0 and 7")
        if not (1 <= bit_count <= 8):
            raise ValueError("Bit count must be between 1 and 8")
        if start_bit + bit_count > 8:
            raise ValueError("Bit range exceeds byte boundaries")
            
        mask = (1 << bit_count) - 1
        shift = 7 - (start_bit + bit_count - 1)
        return (byte_value >> shift) & mask
    
    def _extract_16bit_value(self, data: List[int], index: int) -> int:
        """
        Extract a 16-bit value from byte array in big-endian format.
        
        Args:
            data: List of bytes
            index: Starting index for the 16-bit value
            
        Returns:
            16-bit integer value
            
        Raises:
            IndexError: If there aren't enough bytes available
        """
        if index + 1 >= len(data):
            raise IndexError("Not enough bytes to extract 16-bit value")
        return (data[index] << 8) | data[index + 1]
    
    def _extract_32bit_value(self, data: List[int], index: int) -> int:
        """
        Extract a 32-bit value from byte array in big-endian format.
        
        Args:
            data: List of bytes
            index: Starting index for the 32-bit value
            
        Returns:
            32-bit integer value
            
        Raises:
            IndexError: If there aren't enough bytes available
        """
        if index + 3 >= len(data):
            raise IndexError("Not enough bytes to extract 32-bit value")
        return (data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3]