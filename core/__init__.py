#!/usr/bin/env python3
"""
Core package for CAN message parsing infrastructure.
"""

from .message import CanMessage
from .uart_can_interface import UartCanInterface
from .base_parser import BaseParser

__all__ = ['CanMessage', 'UartCanInterface', 'BaseParser'] 