#!/usr/bin/env python3
"""
Core module for the ROSCAN package.

This module contains the fundamental classes and message definitions
that are used throughout the package.
"""

from .can_frame import CanFrame

__all__ = [
    "CanFrame"
]