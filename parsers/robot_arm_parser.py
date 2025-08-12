#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
"""
Control action parser module for encoding ROS keyboard twist teleop commands to CAN frames.
This module provides a parser for encoding keyboard control commands into CAN frames.
It is used for sending control commands to test the rover via CAN bus.
"""
from typing import Dict, List, Optional
from std_msgs.msg import Float64MultiArray
from can_msgs.msg import Frame
from math import pi, ceil
import rospy


class RobotArmControlParser:  # pylint: disable=too-few-public-methods
    """Parser for encoding keyboard control commands into CAN frames.

    This module provides a parser for encoding keyboard control commands into CAN frames.
    It is used for sending control commands to test the rover via CAN bus.

    Methods
    -------
    parse(msg: Twist) -> Frame:
        Parse a ROS keyboard twist teleop command into a CAN frame.
    """

    def parse(self, msg: Float64MultiArray) -> Frame:
        """
        Parse a ROS keyboard twist teleop command into a CAN frame.
        Parameters
        ----------
        msg : Twist
            ROS keyboard twist teleop command message.
        Returns
        -------
        List[int]
            Encoded CAN frame data.
        Raises
        ------
        Exception
            If an error occurs during encoding.

        """

        def rpmToSignal(rpm: float, max_rpm: float = 15.0) -> int:
            """Convert RPM to motor signal."""
            return int(ceil((rpm + max_rpm) * (127 / (2 * max_rpm))))

        try:

            # Create the CAN frame data
            frame_data = [
                rpmToSignal(msg.data[0], 30),  # Arm joint 1 RPM
                rpmToSignal(msg.data[1], 30),  # Arm joint 2 RPM
                rpmToSignal(msg.data[2], 30),  # Arm joint 3 RPM
                rpmToSignal(msg.data[3], 30),  # Arm joint 4 RPM
                0,  # Placeholder for future use
                0,  # Placeholder for future use
                0,  # Placeholder for future use
                0,  # Placeholder for future use
            ]

            # Create the CAN frame
            can_frame = Frame()
            can_frame.header.stamp = rospy.Time.now()
            can_frame.header.frame_id = "keyboard_control"
            can_frame.is_rtr = False
            can_frame.is_extended = False
            can_frame.is_error = False
            can_frame.id = 0xA7A
            can_frame.dlc = len(frame_data)
            can_frame.data = frame_data
            return can_frame

        except IndexError as e:
            print(f"Error parsing Testing frame: {e}")
            return None
