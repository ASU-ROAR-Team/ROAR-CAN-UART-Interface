import serial
import threading
import time
import struct
from roscan.core.can_frame import CanFrame

class CommunicationManager:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.receiving_thread = None
        self.running = False
        self.frame_callback = None

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")
            raise

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected from serial port.")

    def start_receiving(self):
        if self.serial and self.serial.is_open:
            self.running = True
            self.receiving_thread = threading.Thread(target=self._receive_loop)
            self.receiving_thread.daemon = True
            self.receiving_thread.start()
            print("Started receiving frames.")

    def stop_receiving(self):
        self.running = False
        if self.receiving_thread:
            self.receiving_thread.join()
            print("Stopped receiving frames.")

    def _receive_loop(self):
        buffer = bytearray()
        while self.running:
            try:
                # Read available data
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    buffer.extend(data)
                
                # Process complete frames from the buffer
                while len(buffer) >= 6:  # Minimum frame size: start + id(2) + dlc + checksum + end
                    # Look for start byte
                    start_idx = buffer.find(0xAA)
                    if start_idx == -1:
                        # No start byte found, clear buffer if it's getting too large
                        if len(buffer) > 100:
                            buffer = buffer[-10:]  # Keep last 10 bytes
                        break
                    
                    # Remove data before start byte
                    if start_idx > 0:
                        buffer = buffer[start_idx:]
                    
                    # Check if we have enough data for a minimum frame
                    if len(buffer) < 6:
                        break
                    
                    # Extract frame components
                    start_byte = buffer[0]
                    if start_byte != 0xAA:
                        # This shouldn't happen, but just in case
                        buffer = buffer[1:]
                        continue
                    
                    # Extract ID (2 bytes)
                    if len(buffer) < 3:
                        break
                    can_id = int.from_bytes(buffer[1:3], 'big')
                    
                    # Extract DLC
                    if len(buffer) < 4:
                        break
                    dlc = buffer[3]
                    
                    # Check if we have enough data for the complete frame
                    # start(1) + id(2) + dlc(1) + data(dlc) + checksum(1) + end(1)
                    frame_length = 5 + dlc
                    if len(buffer) < frame_length:
                        break
                    
                    # Extract data, checksum, and end byte
                    data_bytes = buffer[4:4+dlc]
                    checksum = buffer[4+dlc]
                    end_byte = buffer[4+dlc+1]
                    
                    # Validate end byte
                    if end_byte != 0x55:
                        # Invalid frame, remove the start byte and continue
                        buffer = buffer[1:]
                        continue
                    
                    # Validate checksum
                    expected_checksum = (can_id >> 8) ^ (can_id & 0xFF) ^ dlc
                    for byte in data_bytes:
                        expected_checksum ^= byte
                    expected_checksum &= 0xFF
                    
                    if checksum != expected_checksum:
                        # Invalid checksum, remove the start byte and continue
                        buffer = buffer[1:]
                        continue
                    
                    # Valid frame, process it
                    if self.frame_callback:
                        self.frame_callback(can_id, list(data_bytes))
                    
                    # Remove processed frame from buffer
                    buffer = buffer[frame_length:]
                
                time.sleep(0.001)  # Small delay to prevent busy waiting
                
            except Exception as e:
                print(f"Error in receive loop: {e}")
                time.sleep(0.01)

    def send_frame(self, frame_id, data):
        if self.serial and self.serial.is_open:
            try:
                print(f"Sending frame with ID {hex(frame_id)}")
                # Create a CanFrame and send it
                frame = CanFrame(can_id=frame_id, dlc=len(data), data=data)
                self.serial.write(frame.to_bytes())
            except Exception as e:
                print(f"Error sending frame: {e}")

    def set_frame_callback(self, callback):
        self.frame_callback = callback