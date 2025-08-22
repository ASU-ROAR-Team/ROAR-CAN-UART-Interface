
from typing import Dict, Optional, Type

from roscan.messages.base_message import BaseMessage


class MessageRegistry:
    """
    A registry for all message types.

    This class maps CAN IDs to their corresponding Message objects.
    """

    def __init__(self):
        self._registry: Dict[int, BaseMessage] = {}

    def register(self, message: BaseMessage) -> None:
        """
        Registers a message with the registry.

        Args:
            message: The message to register.
        """
        self._registry[message.can_id] = message

    def get_message(self, can_id: int) -> Optional[BaseMessage]:
        """
        Gets a message from the registry.

        Args:
            can_id: The CAN ID of the message to get.

        Returns:
            The message object, or None if the message is not found.
        """
        return self._registry.get(can_id)
