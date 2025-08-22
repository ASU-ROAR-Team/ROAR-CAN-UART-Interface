
from typing import Any, Dict, Optional

import rospy
from std_msgs.msg import Float32

from roscan.core.can_frame import CanFrame
from roscan.core.exceptions import ParsingError
from roscan.messages.base_message import BaseMessage


class LoadCellMessage(BaseMessage):
    """
    Message class for load cell data.
    """

    def __init__(self, can_id: int, publisher: rospy.Publisher):
        super().__init__(can_id)
        self.publisher = publisher

    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        try:
            if len(frame.data) != 4:
                raise ParsingError(f"Invalid data length for load cell frame: {len(frame.data)}")

            load_cell_value = int.from_bytes(frame.data, 'little', signed=True) / 1000.0

            result = {
                "load_cell": load_cell_value,
            }
            rospy.logdebug(f"Parsed load cell frame: {result}")
            return result

        except Exception as e:
            rospy.logerr(f"Error parsing load cell frame: {e}")
            raise ParsingError(f"Load cell parsing failed: {e}") from e

    def handle(self, parsed_data: Dict[str, Any]) -> None:
        try:
            load_cell_msg = Float32()
            load_cell_msg.data = parsed_data.get("load_cell", 0.0)
            self.publisher.publish(load_cell_msg)
        except Exception as e:
            rospy.logerr(f"Error handling load cell data: {e}")
