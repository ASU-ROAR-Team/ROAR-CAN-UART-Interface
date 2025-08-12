# CAN Message Parsers

A Python library for parsing CAN bus messages with support for various data formats, UART communication, and specialized parsers for different sensor types.

## Project Structure

```
parsers/
├── README.md                 # This file
├── requirements.txt          # Python dependencies
├── core/                    # Core infrastructure
│   ├── __init__.py         # Core package initialization
│   ├── message.py          # CAN message data structure and formatting
│   ├── uart_can_interface.py # UART communication interface
│   └── base_parser.py      # Abstract base parser class with utilities
├── parsers/                 # Specialized message parsers
│   ├── __init__.py         # Parsers package initialization
│   ├── encoder_parser.py   # Encoder data parser (10-bit segments)
│   ├── gps_parser.py       # GPS coordinate parsers
│   └── robot_arm_parser.py # Robot arm control parser
└── tests/                   # Test files
    ├── __init__.py         # Test package initialization
    ├── test_10bit_extraction.py # 10-bit extraction tests
    └── debug_test.py       # Debug utilities
```

## Features

- **CAN Message Handling**: Complete message structure with framing, checksums, and validation
- **UART Communication**: Robust UART interface with state machine parsing
- **Flexible Parsing**: Abstract base class with utility methods for bit manipulation
- **Specialized Parsers**: Ready-to-use parsers for encoders, GPS, and robot control
- **10-bit Segment Support**: Built-in support for extracting 10-bit segments from byte arrays

## Installation

1. Clone the repository:
```bash
git clone <your-repo-url>
cd parsers
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Quick Start

### Basic CAN Message Usage

```python
from core.message import CanMessage

# Create a CAN message
msg = CanMessage(0x123, 4, [0xDE, 0xAD, 0xBE, 0xEF])

# Convert to bytes for transmission
data = msg.to_bytes()

# Parse from received bytes
received_msg = CanMessage.from_bytes(data)
```

### UART Communication

```python
from core.uart_can_interface import UartCanInterface
from core.message import CanMessage

# Initialize interface
interface = UartCanInterface('/dev/ttyUSB0', 115200)

# Connect and send message
interface.connect()
msg = CanMessage(0x123, 2, [0xDE, 0xAD])
interface.send_message(msg)

# Receive messages
for received_msg in interface.receive_messages():
    print(f"Received: {received_msg}")
    
interface.disconnect()
```

### Using Parsers

```python
from parsers.encoder_parser import EncoderParser
from core.message import CanMessage

# Create parser
parser = EncoderParser()

# Parse encoder data
msg = CanMessage(0x456, 5, [0x12, 0x34, 0x56, 0x78, 0x9A])
result = parser.parse(msg)

if result:
    print(f"Encoder values: {result['encoder_values']}")
```

## Architecture

### Core Components

- **CanMessage**: Data structure for CAN messages with validation and serialization
- **UartCanInterface**: Handles UART communication and message framing
- **BaseParser**: Abstract base class providing common parsing utilities

### Parser System

All parsers inherit from `BaseParser` and implement the `parse()` method. The base class provides:

- 10-bit segment extraction
- 16-bit and 32-bit value extraction
- Bit manipulation utilities

### Communication Layer

The UART interface implements a robust state machine for parsing incoming CAN message frames, handling:

- Message framing (start/end bytes)
- Checksum validation
- Error recovery and state reset

## Data Formats

### CAN Message Frame

```
[0xAA] [ID:2bytes] [DLC:1byte] [Data:0-8bytes] [Checksum:1byte] [0x55]
```

### 10-bit Segment Extraction

For data like encoder readings that use 10-bit segments:

- **Segment 1**: Bits 0-9 from bytes 0-1
- **Segment 2**: Bits 10-19 from bytes 1-2
- **Segment 3**: Bits 20-29 from bytes 2-3

## Testing

Run the test suite:

```bash
# Test 10-bit extraction
python3 tests/test_10bit_extraction.py

# Debug utilities
python3 tests/debug_test.py
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

[Add your license here]

## TODO

- [ ] Add more specialized parsers
- [ ] Implement CAN bus interface (not just UART)
- [ ] Add configuration file support
- [ ] Improve error handling and logging
- [ ] Add performance benchmarks 