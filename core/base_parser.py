#!/usr/bin/env python3
"""
Abstract base class for all CAN message parsers.
"""
from abc import ABC, abstractmethod
from typing import Dict, Any, List, Optional
from .message import CanMessage

class BaseParser(ABC):
    """Abstract base class for CAN message parsers."""
    
    @abstractmethod
    def parse(self, message: CanMessage) -> Optional[Dict[str, Any]]:
        """
        Parse a CAN message into structured data.
        
        Args:
            message: The CAN message to parse
            
        Returns:
            Parsed data dictionary or None if parsing fails
        """
        pass
    
    def extract_10bit_segments(self, data: List[int], num_segments: int) -> List[int]:
        """
        Extract consecutive 10-bit segments from byte array.
        
        Extracts consecutive 10-bit segments starting from bit 0 of the first byte.
        Each segment represents a complete 10-bit value (e.g., motor RPM).
        Bits are extracted MSB-first (big-endian) from each byte.
        
        Args:
            data: List of bytes to extract from
            num_segments: Number of 10-bit segments to extract
            
        Returns:
            List of 10-bit integer values
            
        Example:
            For bytes [0b10101100, 0b01100110, 0b11001100, 0b10101010]:
            - Segment 1: 1010110001 (bits 0-9 from bytes 0-1)
            - Segment 2: 1001101100 (bits 10-19 from bytes 1-2)  
            - Segment 3: 1010101010 (bits 20-29 from bytes 2-3)
        """
        segments = []
        
        for i in range(num_segments):
            # Calculate starting position for this segment
            start_bit = i * 10
            start_byte = start_bit // 8
            bit_offset = start_bit % 8
            
            if start_byte + 1 >= len(data):
                break
                
            # Extract 10 bits starting from the calculated position
            if bit_offset <= 6:  # Fits within 2 bytes
                # From first byte: take (8 - bit_offset) bits starting from bit_offset
                bits_from_first = 8 - bit_offset
                mask_first = (1 << bits_from_first) - 1
                
                # From second byte: take remaining bits (10 - bits_from_first) starting from MSB
                bits_from_second = 10 - bits_from_first
                mask_second = (1 << bits_from_second) - 1
                
                # Extract bits from first byte starting from bit_offset
                first_part = data[start_byte] & mask_first
                
                # Extract MSBs from second byte
                second_part = (data[start_byte + 1] >> (8 - bits_from_second)) & mask_second
                
                # Combine: first_part (MSBs) + second_part (LSBs)
                value = (first_part << bits_from_second) | second_part
                
            else:  # bit_offset == 7, spans 3 bytes
                # From first byte: take 1 bit (LSB)
                first_part = data[start_byte] & 0x01
                
                # From second byte: take all 8 bits
                second_part = data[start_byte + 1]
                
                # From third byte: take 1 bit (MSB)
                if start_byte + 2 < len(data):
                    third_part = (data[start_byte + 2] >> 7) & 0x01
                else:
                    third_part = 0
                
                # Combine: first_part (1 bit) + second_part (8 bits) + third_part (1 bit)
                value = (first_part << 9) | (second_part << 1) | third_part
            
            segments.append(value)
        
        return segments
    
    def extract_16bit_values(self, data: List[int], num_values: int) -> List[int]:
        """Extract 16-bit values from byte array."""
        values = []
        for i in range(num_values):
            if i * 2 + 1 < len(data):
                value = (data[i * 2] << 8) | data[i * 2 + 1]
                values.append(value)
        return values
    
    def extract_32bit_values(self, data: List[int], num_values: int) -> List[int]:
        """Extract 32-bit values from byte array."""
        values = []
        for i in range(num_values):
            if i * 4 + 3 < len(data):
                value = (data[i * 4] << 24) | (data[i * 4 + 1] << 16) | \
                       (data[i * 4 + 2] << 8) | data[i * 4 + 3]
                values.append(value)
        return values 
