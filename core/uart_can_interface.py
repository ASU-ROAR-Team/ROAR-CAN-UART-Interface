#!/usr/bin/env python3
"""
UART interface for CAN communication.

Provides UartCanInterface class for handling UART-based CAN message
communication with state machine parsing and error recovery.
"""

import serial
import time
from typing import Optional, Generator
from .message import CanMessage

class UartCanInterface:
    """
    Handles UART communication for CAN messages.
    
    Provides interface for sending/receiving CAN messages over serial
    with robust state machine parsing and error recovery.
    
    Attributes
    ----------
    port : str
        Serial port device path (e.g., '/dev/ttyUSB0')
    baudrate : int
        Baud rate for serial communication (default: 115200)
    ser : Optional[serial.Serial]
        Serial connection object (None when disconnected)
    """
    
    def __init__(self, port: str, baudrate: int = 115200):
        """
        Initialize the UART CAN interface.
        
        Parameters
        ----------
        port : str
            Serial port device path (e.g., '/dev/ttyUSB0', 'COM3')
        baudrate : int, optional
            Baud rate for serial communication (default: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
    
    def connect(self) -> None:
        """
        Open the serial connection.
        
        Establishes connection with 1-second timeout and flushes buffers.
        
        Raises
        ------
        ConnectionError
            If serial port cannot be opened or configured
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.ser.flushInput()
            self.ser.flushOutput()
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to connect to {self.port}: {e}")
    
    def disconnect(self) -> None:
        """
        Close the serial connection.
        
        Safely closes connection if open. Safe to call multiple times.
        """
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
    
    def send_message(self, message: CanMessage) -> None:
        """
        Send a CAN message over UART.
        
        Converts message to bytes and sends over serial connection.
        
        Parameters
        ----------
        message : CanMessage
            The CAN message to send
            
        Raises
        ------
        ConnectionError
            If serial port is not connected
        """
        if not self.ser or not self.ser.is_open:
            raise ConnectionError("Serial port not connected")
        
        data = message.to_bytes()
        self.ser.write(data)
    
    def receive_messages(self) -> Generator[CanMessage, None, None]:
        """
        Generator that yields received CAN messages.
        
        Implements state machine for parsing incoming CAN message frames.
        Handles framing, checksums, and error recovery. Runs indefinitely
        until connection closes or unrecoverable error occurs.
        
        Yields
        ------
        CanMessage
            Complete, validated CAN message
            
        Raises
        ------
        ConnectionError
            If serial port is not connected
        """
        if not self.ser or not self.ser.is_open:
            raise ConnectionError("Serial port not connected")
        
        # State machine variables
        state = "WAITING_FOR_START"
        buffer = bytearray()
        calculated_checksum = 0
        data_index = 0  # Initialize data_index
        
        while True:
            try:
                if self.ser.in_waiting > 0:
                    byte = self.ser.read(1)[0]
                    
                    if state == "WAITING_FOR_START":
                        if byte == 0xAA:
                            state = "READING_ID"
                            buffer = bytearray()
                            calculated_checksum = 0
                    
                    elif state == "READING_ID":
                        buffer.append(byte)
                        calculated_checksum ^= byte
                        if len(buffer) == 2:
                            state = "READING_DLC"
                    
                    elif state == "READING_DLC":
                        buffer.append(byte)
                        calculated_checksum ^= byte
                        dlc = buffer[-1]
                        if dlc <= 8:
                            state = "READING_DATA" if dlc > 0 else "READING_CHECKSUM"
                            data_index = 0  # Reset data_index for new message
                        else:
                            # Invalid DLC - reset to start
                            state = "WAITING_FOR_START"
                            buffer.clear()
                    
                    elif state == "READING_DATA":
                        buffer.append(byte)
                        calculated_checksum ^= byte
                        data_index += 1
                        if data_index == dlc:
                            state = "READING_CHECKSUM"
                    
                    elif state == "READING_CHECKSUM":
                        if byte == calculated_checksum:
                            state = "READING_END"
                        else:
                            # Checksum mismatch - reset to start
                            state = "WAITING_FOR_START"
                            buffer.clear()
                    
                    elif state == "READING_END":
                        if byte == 0x55:
                            try:
                                # Create complete message frame and parse
                                message_data = bytes([0xAA] + buffer + [calculated_checksum, 0x55])
                                message = CanMessage.from_bytes(message_data)
                                yield message
                            except ValueError as e:
                                print(f"Invalid message format: {e}")
                        # Always reset state after end byte (success or failure)
                        state = "WAITING_FOR_START"
                        buffer.clear()
                        
            except serial.SerialException as e:
                print(f"Serial port error: {e}")
                break
            except Exception as e:
                print(f"Unexpected error in message parsing: {e}")
                # Reset state machine on any unexpected error
                state = "WAITING_FOR_START"
                buffer.clear()
                calculated_checksum = 0
                data_index = 0
    
    def __enter__(self):
        """Context manager entry - connects to serial port."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - disconnects from serial port."""
        self.disconnect()
    
    def is_connected(self) -> bool:
        """Check if serial connection is active."""
        return self.ser is not None and self.ser.is_open 