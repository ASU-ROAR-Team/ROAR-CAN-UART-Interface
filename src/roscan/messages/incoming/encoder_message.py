
from typing import Any, Dict, Optional

import rospy
from roar_msgs.msg import EncoderStamped

from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import ParsingError
from roscan.messages.base_message import BaseMessage


class EncoderMessage(BaseMessage):
    """
    Message class for encoder data.

    This class parses and handles CAN frames containing encoder data.
    """

    EXPECTED_DATA_LENGTH = 6

    def __init__(self, can_id: int, publisher: rospy.Publisher):
        """Initialize the encoder message."""
        super().__init__(can_id)
        self.publisher = publisher
        self.encoders_map = -16  # Adjustment value for encoder readings

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        """
        Parse a CAN frame into a dictionary of encoder values.

        Args:
            frame: CAN frame containing encoder data

        Returns:
            Parsed encoder values, or None if parsing fails

        Raises:
            ParsingError: If frame data is invalid
        """
        try:
            # Validate frame length
            if len(frame.data) != self.EXPECTED_DATA_LENGTH:
                raise ParsingError(f"Invalid data length for encoder frame: {len(frame.data)}")

            # Extract encoder readings (first 6 bytes)
            encoder_readings = [reading + self.encoders_map for reading in frame.data[:6]]

            result = {
                "encoder1": encoder_readings[0],
                "encoder2": encoder_readings[1],
                "encoder3": encoder_readings[2],
                "encoder4": encoder_readings[3],
                "encoder5": encoder_readings[4],
                "encoder6": encoder_readings[5],
            }

            rospy.logdebug(f"Parsed encoder frame: {result}")
            return result

        except Exception as e:
            rospy.logerr(f"Error parsing encoder frame: {e}")
            raise ParsingError(f"Encoder parsing failed: {e}") from e

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        """
        Handles the parsed encoder data by publishing it as a ROS message.

        Args:
            parsed_data: The parsed encoder data.
        """
        try:
            encoder_msg = EncoderStamped()
            encoder_msg.header.stamp = rospy.Time.now()
            encoder_msg.data = [
                parsed_data.get("encoder1", 0.0),
                parsed_data.get("encoder2", 0.0),
                parsed_data.get("encoder3", 0.0),
                parsed_data.get("encoder4", 0.0),
                parsed_data.get("encoder5", 0.0),
                parsed_data.get("encoder6", 0.0),
            ]
            self.publisher.publish(encoder_msg)
        except Exception as e:
            rospy.logerr(f"Error handling encoder data: {e}")
