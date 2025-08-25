from typing import Any, Dict, Optional

import rospy
from roar_msgs.msg import DrillingStatus # Import the custom ROS message for DrillingStatus

from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import ParsingError
from roscan.messages.base_message import BaseMessage


class DrillingStatusMessage(BaseMessage):
    """
    Message class for drilling status data.
    
    Parses incoming CAN frames containing current height/depth and load cell
    (weight) data, and publishes it as a roar_msgs/DrillingStatus ROS message.
    
    CAN Frame Structure:
    - Bytes 0-1 (2 bytes): Height/depth value (signed integer, little-endian)
    - Bytes 2-3 (2 bytes): Load cell value (unsigned integer, little-endian)
    """

    def __init__(self, can_id: int, publisher: rospy.Publisher):
        """
        Initializes the DrillingStatusMessage.

        Args:
            can_id (int): The CAN ID associated with this message.
            publisher (rospy.Publisher): The ROS publisher for DrillingStatus messages.
        """
        super().__init__(can_id)
        self.publisher = publisher
        # rospy.loginfo(f"DrillingStatusMessage initialized for CAN ID: 0x{can_id:03X}")

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        """
        Parses a raw CAN frame into a dictionary containing drilling status data.

        Args:
            frame (CanFrame): The incoming CAN frame to parse.

        Returns:
            Optional[Dict[str, Any]]: A dictionary with "current_height" and 
                                     "current_weight" if parsing is successful, 
                                     otherwise None.

        Raises:
            ParsingError: If the frame data length is invalid or other parsing issues occur.
        """
        try:
            # The DrillingStatus message specifies 2 bytes for height and 2 for weight.
            # Total expected data length is 4 bytes.
            expected_data_len = 4
            if len(frame.data) != expected_data_len:
                raise ParsingError(
                    f"Invalid data length for DrillingStatus frame 0x{frame.can_id:03X}. "
                    f"Expected {expected_data_len} bytes, got {len(frame.data)}"
                )

            # Extract height/depth (first 2 bytes) as a SIGNED integer.
            # Convert to centimeters (cm) by dividing by 10.0.
            height_raw_mm = int.from_bytes(frame.data[0:2], 'little', signed=True)
            current_height_cm = height_raw_mm / 10.0  # Convert mm to cm

            # Extract load cell (next 2 bytes) as an UNSIGNED integer.
            current_weight_g = int.from_bytes(frame.data[2:4], 'little', signed=False)
            current_weight_g = float(current_weight_g) # Ensure it's a float64 for the ROS message

            result = {
                "current_height": current_height_cm,
                "current_weight": current_weight_g,
            }
            # rospy.logdebug(f"Parsed DrillingStatus frame 0x{frame.can_id:03X}: {result}")
            return result

        except Exception as e:
            # rospy.logerr(f"Error parsing DrillingStatus frame 0x{frame.can_id:03X}: {e}")
            # Re-raise as ParsingError for consistency with other message parsers
            raise ParsingError(f"DrillingStatus parsing failed: {e}") from e

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        """
        Publishes the parsed drilling status data as a roar_msgs/DrillingStatus message.

        Args:
            parsed_data (Dict[str, Any]): A dictionary containing "current_height"
                                          and "current_weight".
        """
        try:
            drilling_status_msg = DrillingStatus()
            drilling_status_msg.current_height = parsed_data["current_height"]
            drilling_status_msg.current_weight = parsed_data["current_weight"]
            
            self.publisher.publish(drilling_status_msg)
            rospy.logdebug(
                f"Published DrillingStatus: Height={drilling_status_msg.current_height:.2f} cm, "
                f"Weight={drilling_status_msg.current_weight:.2f} g"
            )
        except Exception as e:
            rospy.logerr(f"Error publishing DrillingStatus message: {e}")
