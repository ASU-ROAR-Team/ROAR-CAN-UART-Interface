
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

from roscan.core.can_frame import CanFrame


class BaseMessage(ABC):
    """
    Abstract base class for all message types.

    This class defines the standard interface for all messages, including
    parsing, handling, and building CAN frames.
    """

    def __init__(self, can_id: int):
        self.can_id = can_id

    @abstractmethod
    def parse(self, frame: CanFrame) -> Optional[Dict[str, Any]]:
        """
        Parses a CAN frame and returns a dictionary of parsed data.

        Args:
            frame: The CAN frame to parse.

        Returns:
            A dictionary of parsed data, or None if parsing fails.
        """
        pass

    @abstractmethod
    def handle(self, parsed_data: Dict[str, Any]) -> None:
        """
        Handles the parsed data (e.g., publishes a ROS message).

        Args:
            parsed_data: The parsed data to handle.
        """
        pass

    def build(self, *args, **kwargs) -> Optional[CanFrame]:
        """
        Builds a CAN frame from the given data.

        Returns:
            The built CAN frame, or None if building fails.
        """
        return None
