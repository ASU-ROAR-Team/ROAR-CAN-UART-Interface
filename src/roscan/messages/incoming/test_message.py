
from typing import Any, Dict, Optional

import rospy
from std_msgs.msg import String

from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import ParsingError
from roscan.messages.base_message import BaseMessage


class TestMessage(BaseMessage):
    """
    Message class for test data.
    """

    def __init__(self, can_id: int, publisher: rospy.Publisher):
        super().__init__(can_id)
        self.publisher = publisher

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        try:
            # Assuming the data is a utf-8 encoded string
            test_string = frame.data.decode('utf-8')

            result = {
                "test_data": test_string,
            }
            rospy.logdebug(f"Parsed test frame: {result}")
            return result

        except Exception as e:
            rospy.logerr(f"Error parsing test frame: {e}")
            raise ParsingError(f"Test parsing failed: {e}") from e

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        try:
            test_msg = String()
            test_msg.data = parsed_data.get("test_data", "")
            self.publisher.publish(test_msg)
        except Exception as e:
            rospy.logerr(f"Error handling test data: {e}")
