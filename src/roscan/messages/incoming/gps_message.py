
from typing import Any, Dict, Optional

import rospy
from geometry_msgs.msg import PoseStamped

from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import ParsingError
from roscan.messages.base_message import BaseMessage


class GpsLatitudeMessage(BaseMessage):
    """
    Message class for GPS latitude data.
    """

    def __init__(self, can_id: int, publisher: rospy.Publisher):
        super().__init__(can_id)
        self.publisher = publisher

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        try:
            if len(frame.data) != 8:
                raise ParsingError(f"Invalid data length for GPS latitude frame: {len(frame.data)}")

            latitude = int.from_bytes(frame.data[:4], 'little') / 1e7
            sequence = int.from_bytes(frame.data[4:], 'little')

            result = {
                "latitude": latitude,
                "sequence": sequence,
            }
            rospy.logdebug(f"Parsed GPS latitude frame: {result}")
            return result

        except Exception as e:
            rospy.logerr(f"Error parsing GPS latitude frame: {e}")
            raise ParsingError(f"GPS latitude parsing failed: {e}") from e

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        # The pairing logic will be handled in the RoscanBridgeNode for now
        pass


class GpsLongitudeMessage(BaseMessage):
    """
    Message class for GPS longitude data.
    """

    def __init__(self, can_id: int, publisher: rospy.Publisher):
        super().__init__(can_id)
        self.publisher = publisher

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        try:
            if len(frame.data) != 8:
                raise ParsingError(f"Invalid data length for GPS longitude frame: {len(frame.data)}")

            longitude = int.from_bytes(frame.data[:4], 'little') / 1e7
            sequence = int.from_bytes(frame.data[4:], 'little')

            result = {
                "longitude": longitude,
                "sequence": sequence,
            }
            rospy.logdebug(f"Parsed GPS longitude frame: {result}")
            return result

        except Exception as e:
            rospy.logerr(f"Error parsing GPS longitude frame: {e}")
            raise ParsingError(f"GPS longitude parsing failed: {e}") from e

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        # The pairing logic will be handled in the RoscanBridgeNode for now
        pass
