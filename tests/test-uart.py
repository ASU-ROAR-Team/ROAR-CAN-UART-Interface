import serial
import time
import struct

# --- MESSAGE STRUCTURE DEFINITION ---
# Start Byte: 0xAA
# CAN ID:     2 bytes
# DLC:        1 byte
# Data:       0-8 bytes (variable length)
# Checksum:   1 byte (XOR sum of ID, DLC, and Data)
# End Byte:   0x55

# Define the start and end bytes for message framing
START_BYTE = 0xAA
END_BYTE = 0x55

class CanMessage:
    def __init__(self, can_id, dlc, data):
        self.can_id = can_id
        self.dlc = dlc
        self.data = data
        if len(self.data) > 8:
            raise ValueError("CAN data payload cannot exceed 8 bytes.")

# --- Function to send a CAN message over UART ---
def send_can_message(ser, msg):
    """Sends a formatted CAN message over the serial port."""
    # Calculate checksum (XOR sum of ID, DLC, and Data)
    checksum = (msg.can_id >> 8) ^ (msg.can_id & 0xFF) ^ msg.dlc
    for byte in msg.data:
        checksum ^= byte

    # Create the full message buffer
    message_buffer = bytearray([START_BYTE])
    message_buffer.extend(msg.can_id.to_bytes(2, 'big'))
    message_buffer.append(msg.dlc)
    message_buffer.extend(msg.data)
    message_buffer.append(checksum)
    message_buffer.append(END_BYTE)

    ser.write(message_buffer)
    print(f"Sent: {message_buffer.hex()}")

# --- State machine for receiving UART messages ---
def receive_can_message(ser):
    """Receives and parses a formatted CAN message from the serial port."""
    state = "WAITING_FOR_START"
    buffer = bytearray()
    
    while True:
        try:
            if ser.in_waiting > 0:
                byte = ord(ser.read(1))
                
                if state == "WAITING_FOR_START":
                    if byte == START_BYTE:
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
                        data_index = 0
                    else: # Invalid DLC
                        state = "WAITING_FOR_START"
                        
                elif state == "READING_DATA":
                    buffer.append(byte)
                    calculated_checksum ^= byte
                    data_index += 1
                    if data_index == dlc:
                        state = "READING_CHECKSUM"
                
                elif state == "READING_CHECKSUM":
                    state = "READING_END"
                    if byte == calculated_checksum:
                        checksum_ok = True
                    else:
                        checksum_ok = False
                
                elif state == "READING_END":
                    state = "WAITING_FOR_START"
                    if byte == END_BYTE and checksum_ok:
                        can_id = int.from_bytes(buffer[0:2], 'big')
                        dlc = buffer[2]
                        data = buffer[3:3+dlc]
                        print("Received message:")
                        print(f"  ID: 0x{can_id:X}")
                        print(f"  DLC: {dlc}")
                        print(f"  Data: {[hex(b) for b in data]}")
                        return can_id, dlc, data

        except serial.SerialException as e:
            print(f"Serial port error: {e}")
            break
        except Exception as e:
            print(f"An error occurred: {e}")
            state = "WAITING_FOR_START"

# --- Main test logic ---
if __name__ == '__main__':
    # You may need to change this port to match your setup
    port = '/dev/ttyTHS0' 
    baudrate = 115200

    print(f"Opening serial port {port} at {baudrate}...")
    ser = serial.Serial(port, baudrate, timeout=1)
    try:
        ser.flushInput()
        ser.flushOutput()
        print("Serial port opened successfully.")
        
        # Test 1: Send a message to the STM32
        test_data = [0xDE, 0xAD, 0xBE, 0xEF]
        test_msg = CanMessage(0x123, 4, test_data)
        send_can_message(ser, test_msg)
        
        print("\nWaiting for a message from the STM32...")
        # Test 2: Listen for a message from the STM32
        received = receive_can_message(ser)
        if received:
            print("Test passed: received a message!")
        else:
            print("Test failed: no message received.")

        ser.close()
    except:
        print(f"Error opening serial port:")
        print("Check the port name and permissions.")

