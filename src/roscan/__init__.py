#!/usr/bin/env python3
"""
ROSCAN package.

This module provides a collection of CAN frame parsers and handlers
for various sensors and control messages in a ROS environment.
"""

# Core components
from .core.base_parser import BaseParser
from .core.can_frame import CanFrame
from .core.exceptions import (
    RoscanError,
    ParsingError,
    CommunicationError,
    ValidationError,
    ConfigurationError
)

__all__ = [
    # Core components
    "BaseParser",
    "CanFrame",
    "RoscanError",
    "ParsingError",
    "CommunicationError",
    "ValidationError",
    "ConfigurationError",
]