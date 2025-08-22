#!/usr/bin/env python3
"""
Custom exceptions for the ROSCAN package.

This module defines custom exception classes for various error conditions
that can occur in the ROSCAN package.
"""

class RoscanError(Exception):
    """
    Base exception class for all ROSCAN package errors.
    
    This is the parent class for all custom exceptions in the ROSCAN package.
    It provides a common base that can be caught to handle any ROSCAN-related error.
    """
    
    def __init__(self, message: str):
        """
        Initialize the exception.
        
        Args:
            message: Error message describing the exception
        """
        self.message = message
        super().__init__(self.message)


class ParsingError(RoscanError):
    """
    Exception raised when there's an error parsing CAN frames.
    
    This exception is raised when a parser encounters an issue while
    trying to parse a CAN frame, such as invalid data format, incorrect
    frame length, or checksum errors.
    """
    
    def __init__(self, message: str):
        """
        Initialize the parsing error.
        
        Args:
            message: Error message describing the parsing issue
        """
        super().__init__(f"Parsing error: {message}")


class CommunicationError(RoscanError):
    """
    Exception raised when there's a communication error.
    
    This exception is raised when there are issues with serial communication,
    such as timeouts, connection failures, or data transmission errors.
    """
    
    def __init__(self, message: str):
        """
        Initialize the communication error.
        
        Args:
            message: Error message describing the communication issue
        """
        super().__init__(f"Communication error: {message}")


class ValidationError(RoscanError):
    """
    Exception raised when data validation fails.
    
    This exception is raised when data doesn't meet expected criteria,
    such as invalid parameter values or malformed messages.
    """
    
    def __init__(self, message: str):
        """
        Initialize the validation error.
        
        Args:
            message: Error message describing the validation issue
        """
        super().__init__(f"Validation error: {message}")


class ConfigurationError(RoscanError):
    """
    Exception raised when there's a configuration error.
    
    This exception is raised when configuration parameters are invalid
    or missing, such as incorrect port settings or missing ROS parameters.
    """
    
    def __init__(self, message: str):
        """
        Initialize the configuration error.
        
        Args:
            message: Error message describing the configuration issue
        """
        super().__init__(f"Configuration error: {message}")


class BuildingError(RoscanError):
    """
    Exception raised when there's an error building CAN frames.
    
    This exception is raised when a builder encounters an issue while
    trying to build a CAN frame from ROS messages.
    """
    
    def __init__(self, message: str):
        """
        Initialize the building error.
        
        Args:
            message: Error message describing the building issue
        """
        super().__init__(f"Building error: {message}")