#!/usr/bin/env python3
"""
Encoder handler module.
"""
from typing import Dict, List, Optional
from .can_frame_parser import CanFrameParser


class EncoderParser(CanFrameParser):  # pylint: disable=too-few-public-methods
    """Parser for encoder data.

    Attributes
    ----------


    Methods
    -------
    parse(frameData: List[int]) -> Optional[Dict[str, float]]:
        Parse CAN frame data into encoder values.
    """

    def __init__(self) -> None:
        # TODO: What is this value?
        self.encodersMap = -16  # Adjustment value for encoder readings

    def parse(self, frameData: List[int]) -> Optional[Dict[str, float]]:
        """
        Parse a CAN frame into a dictionary of encoder values.

        Parameters
        ----------
        frameData : List[int]
            Raw data bytes from the CAN frame.

        Returns
        -------
        Optional[Dict[str, float]]
            Parsed encoder values, or None if parsing fails.

        Raises
        ------
        Exception
            If an error occurs during parsing.
        """

        try:
            # Extract encoder readings (first 5 bytes)
            # TODO: Check if we will use bytes or 10 bit segments
            return [reading + self.encodersMap for reading in frameData[:5]] 
        except IndexError as e:
            print(f"Error parsing encoder frame: {e}")
            return None
