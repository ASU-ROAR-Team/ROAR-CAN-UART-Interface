# ROSCAN Package - Restructured Architecture

## Overview

This is the restructured version of the ROSCAN package, organized following modern OOP principles and Python packaging best practices.

## Directory Structure

```
src/roscan/
├── communication/          # Serial communication handling
├── config/                 # Configuration management
├── core/                   # Base classes and shared utilities
├── handlers/               # Data processing handlers
│   ├── incoming/           # Handlers for incoming CAN data
│   │   ├── control/        # Control-related handlers
│   │   └── sensors/        # Sensor data handlers
│   └── outgoing/           # Handlers for outgoing ROS messages
├── managers/               # Component coordination
├── node/                   # Main node implementation
├── parsers/                # Message parsing components
│   ├── incoming/           # Parsers for incoming CAN data
│   │   ├── control/        # Control message parsers
│   │   ├── sensors/        # Sensor data parsers
│   │   └── utility/        # Utility parsers
│   └── outgoing/           # Parsers for outgoing ROS messages
│       ├── control/        # Control message parsers
│       └── utility/        # Utility parsers
├── tests/                  # Test modules
└── utils/                  # Utility functions
```

## Key Components

### Communication
Handles UART serial communication with the CAN bus.

### Configuration
Centralized management of ROS parameters, frame IDs, and topic names.

### Core
Base classes and shared utilities used throughout the package.

### Handlers
Process parsed data and publish to ROS topics or handle outgoing messages.

### Managers
Coordinate components and provide centralized access.

### Node
Main ROS node implementation that ties everything together.

### Parsers
Convert between CAN frames and structured data.

## Development Guidelines

1. **Adding New Parsers**: Place in appropriate subdirectory under `parsers/` and register in `RoscanBridgeNode`
2. **Adding New Handlers**: Place in appropriate subdirectory under `handlers/` and register in `RoscanBridgeNode`
3. **Configuration Changes**: Modify `config/configuration.py` and update parameter registration
4. **New Functionality**: Follow existing patterns and maintain consistency with current architecture

## Import Examples

```python
# Import core components
from roscan.core.base_parser import BaseParser
from roscan.core.can_frame import CanFrame

# Import configuration
from roscan.config.configuration import RoscanConfig

# Import managers
from roscan.managers.component_managers import ParserManager, HandlerManager

# Import parsers
from roscan.parsers.incoming.sensors import EncoderParser
from roscan.parsers.outgoing.control import KeyboardControlParser

# Import handlers
from roscan.handlers.incoming.sensors import GpsCoordinatePairer
```