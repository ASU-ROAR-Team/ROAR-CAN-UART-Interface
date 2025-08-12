#!/usr/bin/env python3
"""
Debug test for 10-bit extraction - testing 3 consecutive segments.
"""
from core.base_parser import BaseParser

class TestParser(BaseParser):
    def parse(self, data):
        pass

def debug_all_segments():
    """Debug all 3 consecutive 10-bit segments step by step."""
    parser = TestParser()
    
    # Test data: [0b10101100, 0b01100110, 0b11001100, 0b10101010]
    data = [0b10101100, 0b01100110, 0b11001100, 0b10101010]
    
    print("=== Debug All 3 Consecutive Segments ===")
    print(f"Input bytes: {[bin(b)[2:].zfill(8) for b in data]}")
    print(f"           : {[hex(b) for b in data]}")
    print()
    
    # Manual calculation of what should happen
    print("Manual calculation of expected results:")
    print("Segment 1 (bits 0-9):  1010110001 (from bytes 0-1)")
    print("Segment 2 (bits 10-19): 1001101100 (from bytes 1-2)")
    print("Segment 3 (bits 20-29): 1010101010 (from bytes 2-3)")
    print()
    
    # Extract segments using the method
    segments = parser.extract_10bit_segments(data, 3)
    print("Actual results from method:")
    for i, seg in enumerate(segments):
        print(f"Segment {i+1}: {bin(seg)[2:].zfill(10)} (decimal: {seg})")
    
    print()
    
    # Now trace through each segment step by step
    for i in range(3):
        print(f"=== Tracing Segment {i+1} ===")
        start_bit = i * 10
        start_byte = start_bit // 8
        bit_offset = start_bit % 8
        
        print(f"i = {i}")
        print(f"start_bit = {start_bit}")
        print(f"start_byte = {start_byte}")
        print(f"bit_offset = {bit_offset}")
        
        if bit_offset <= 6:  # Fits within 2 bytes
            bits_from_first = 8 - bit_offset
            mask_first = (1 << bits_from_first) - 1
            bits_from_second = 10 - bits_from_first
            mask_second = (1 << bits_from_second) - 1
            
            print(f"bits_from_first = {bits_from_first}")
            print(f"mask_first = {bin(mask_first)}")
            print(f"bits_from_second = {bits_from_second}")
            print(f"mask_second = {bin(mask_second)}")
            
            first_part = (data[start_byte] >> bit_offset) & mask_first
            second_part = (data[start_byte + 1] >> (8 - bits_from_second)) & mask_second
            
            print(f"first_part = {bin(first_part)} (from byte {start_byte})")
            print(f"second_part = {bin(second_part)} (from byte {start_byte + 1})")
            
            value = (first_part << bits_from_second) | second_part
            print(f"Final value = {bin(value)} (decimal: {value})")
            
        else:  # bit_offset == 7, spans 3 bytes
            print("Spans 3 bytes case")
            first_part = data[start_byte] & 0x01
            second_part = data[start_byte + 1]
            if start_byte + 2 < len(data):
                third_part = (data[start_byte + 2] >> 7) & 0x01
            else:
                third_part = 0
            
            print(f"first_part = {bin(first_part)} (from byte {start_byte})")
            print(f"second_part = {bin(second_part)} (from byte {start_byte + 1})")
            print(f"third_part = {bin(third_part)} (from byte {start_byte + 2})")
            
            value = (first_part << 9) | (second_part << 1) | third_part
            print(f"Final value = {bin(value)} (decimal: {value})")
        
        print()

if __name__ == "__main__":
    debug_all_segments() 