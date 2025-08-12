#!/usr/bin/env python3
"""
GPS CAN message parsers for latitude and longitude frames.
"""
from typing import Dict, List, Optional
from .can_frame_parser import CanFrameParser
import rospy
from sensor_msgs.msg import NavSatFix

class GpsLatitudeParser(CanFrameParser):
    """
    Parser for GPS latitude CAN frames.
    
    Frame format:
    - Byte 0: [sequence (7 bits)] + [N/S flag (1 bit)]
    - Bytes 1-7: scaled latitude as 56-bit signed integer
    
    Attributes
    ----------
    SCALE_FACTOR : int
        Scale factor used to convert from raw coordinate (10^10)
    
    Methods
    -------
    parse(frameData: List[int]) -> Optional[Dict[str, float]]
        Parse a latitude CAN frame into GPS coordinate data.
    """
    
    SCALE_FACTOR = 10000000000  # 10^10 for sub-millimeter precision
    
    def parse(self, frameData: List[int]) -> Optional[Dict[str, float]]:
        """
        Parse a CAN frame containing GPS latitude data.
        
        Parameters
        ----------
        frameData : List[int]
            8-byte CAN frame data
            
        Returns
        -------
        Optional[Dict[str, float]]
            Dictionary containing:
            - 'latitude_raw': Raw DDDMM.MMMMM coordinate value
            - 'is_north': Boolean indicating North (True) or South (False)
            - 'sequence': Frame sequence number (0-127)
            Returns None if parsing fails.
            
        Raises
        ------
        Exception
            Error parsing GPS latitude frame
        """
        try:
            if len(frameData) != 8:
                print(f"Invalid frame length: expected 8 bytes, got {len(frameData)}")
                return None
            
            # Extract sequence number and direction flag from first byte
            control_byte = frameData[0]
            sequence = control_byte & 0x7F  # Lower 7 bits (0-127)
            is_north = bool(control_byte & 0x80)  # MSB: 1=North, 0=South
            
            # Extract scaled latitude from bytes 1-7
            lat_bytes = bytes(frameData[1:])
            lat_scaled = int.from_bytes(lat_bytes, byteorder='little', signed=True)
            
            # Convert back to original DDDMM.MMMMM format
            latitude_raw = lat_scaled / self.SCALE_FACTOR
            
            return {
                "latitude_raw": latitude_raw,
                "is_north": is_north,
                "sequence": sequence
            }
            
        except (ValueError, OverflowError) as e:
            print(f"Error parsing GPS latitude frame: {e}")
            return None


class GpsLongitudeParser(CanFrameParser):
    """
    Parser for GPS longitude CAN frames.
    
    Frame format:
    - Byte 0: [sequence (7 bits)] + [E/W flag (1 bit)]
    - Bytes 1-7: scaled longitude as 56-bit signed integer
    
    Attributes
    ----------
    SCALE_FACTOR : int
        Scale factor used to convert from raw coordinate (10^10)
    
    Methods
    -------
    parse(frameData: List[int]) -> Optional[Dict[str, float]]
        Parse a longitude CAN frame into GPS coordinate data.
    """
    
    SCALE_FACTOR = 10000000000  # 10^10 for sub-millimeter precision
    
    def parse(self, frameData: List[int]) -> Optional[Dict[str, float]]:
        """
        Parse a CAN frame containing GPS longitude data.
        
        Parameters
        ----------
        frameData : List[int]
            8-byte CAN frame data
            
        Returns
        -------
        Optional[Dict[str, float]]
            Dictionary containing:
            - 'longitude_raw': Raw DDDMM.MMMMM coordinate value  
            - 'is_east': Boolean indicating East (True) or West (False)
            - 'sequence': Frame sequence number (0-127)
            Returns None if parsing fails.
            
        Raises
        ------
        Exception
            Error parsing GPS longitude frame
        """
        try:
            if len(frameData) != 8:
                print(f"Invalid frame length: expected 8 bytes, got {len(frameData)}")
                return None
            
            # Extract sequence number and direction flag from first byte
            control_byte = frameData[0]
            sequence = control_byte & 0x7F  # Lower 7 bits (0-127)
            is_east = bool(control_byte & 0x80)  # MSB: 1=East, 0=West
            
            # Extract scaled longitude from bytes 1-7
            lon_bytes = bytes(frameData[1:])
            lon_scaled = int.from_bytes(lon_bytes, byteorder='little', signed=True)
            
            # Convert back to original DDDMM.MMMMM format
            longitude_raw = lon_scaled / self.SCALE_FACTOR
            
            return {
                "longitude_raw": longitude_raw,
                "is_east": is_east,
                "sequence": sequence
            }
            
        except (ValueError, OverflowError) as e:
            print(f"Error parsing GPS longitude frame: {e}")
            return None


class GpsCoordinatePairer:
    """
    Utility class to pair latitude and longitude frames using sequence numbers.
    
    Attributes
    ----------
    _pending_frames : Dict[int, Dict]
        Buffer storing unpaired frames indexed by sequence number
    
    Methods
    -------
    add_frame(frame_data: Dict, frame_type: str) -> Optional[Dict]
        Add a GPS frame and attempt to create a coordinate pair.
    clear_old_frames(max_age_sequences: int = 5) -> None
        Remove old unpaired frames to prevent memory buildup.
    """
    
    def __init__(self):
        self._pending_frames = {}
    
    def add_frame(self, frame_data: Dict, frame_type: str) -> Optional[Dict]:
        """
        Add a GPS frame and return complete coordinate pair if available.
        
        Parameters
        ----------
        frame_data : Dict
            Parsed frame data from GpsLatitudeParser or GpsLongitudeParser
        frame_type : str
            Either 'latitude' or 'longitude'
            
        Returns
        -------
        Optional[Dict]
            Complete GPS coordinate pair if both lat/lon available, None otherwise.
            Contains: latitude_raw, longitude_raw, is_north, is_east, sequence
        """
        if not frame_data or 'sequence' not in frame_data:
            return None
            
        sequence = frame_data['sequence']
        
        # Initialize sequence entry if needed
        if sequence not in self._pending_frames:
            self._pending_frames[sequence] = {}
        
        # Add this frame
        self._pending_frames[sequence][frame_type] = frame_data
        
        # Check if we have both latitude and longitude
        if 'latitude' in self._pending_frames[sequence] and 'longitude' in self._pending_frames[sequence]:
            lat_data = self._pending_frames[sequence]['latitude']
            lon_data = self._pending_frames[sequence]['longitude']
            
            # Create complete coordinate pair
            coordinate_pair = {
                'latitude_raw': lat_data['latitude_raw'],
                'longitude_raw': lon_data['longitude_raw'],
                'is_north': lat_data['is_north'],
                'is_east': lon_data['is_east'],
                'sequence': sequence
            }
            
            # Clean up completed pair
            del self._pending_frames[sequence]
            
            return coordinate_pair
        
        return None
    
    def clear_old_frames(self, max_age_sequences: int = 5) -> None:
        """
        Remove old unpaired frames to prevent memory buildup.
        
        Parameters
        ----------
        max_age_sequences : int
            Maximum number of sequence numbers to keep in buffer
        """
        if len(self._pending_frames) > max_age_sequences:
            # Remove oldest sequences (assuming they increment)
            sorted_sequences = sorted(self._pending_frames.keys())
            sequences_to_remove = sorted_sequences[:-max_age_sequences]
            
            for seq in sequences_to_remove:
                del self._pending_frames[seq]

def publishGpsData(self, gps_data: Dict) -> None:
    """
    Publishes complete GPS coordinate data as NavSatFix message.
    
    Parameters
    ----------
    gps_data : Dict
        Complete GPS data containing latitude_raw, longitude_raw, 
        is_north, is_east, and sequence
    """
    try:
        # Convert from DDDMM.MMMMM to decimal degrees
        lat_decimal = self.ddmm_to_decimal_degrees(gps_data['latitude_raw'])
        lon_decimal = self.ddmm_to_decimal_degrees(gps_data['longitude_raw'])
        
        # Apply direction signs
        if not gps_data['is_north']:
            lat_decimal = -lat_decimal
        if not gps_data['is_east']:
            lon_decimal = -lon_decimal
        
        # Create and populate NavSatFix message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = "gps"
        
        # Set position
        gps_msg.latitude = lat_decimal
        gps_msg.longitude = lon_decimal
        gps_msg.altitude = 0.0  # Not provided by our GPS setup
        
        # Set status (assuming GPS is working if we receive data)
        gps_msg.status.status = NavSatFix.STATUS_FIX
        gps_msg.status.service = NavSatFix.SERVICE_GPS
        
        # Publish the message
        self.gpsPub.publish(gps_msg)
        
        rospy.logdebug(f"Published GPS: lat={lat_decimal:.8f}, lon={lon_decimal:.8f}, seq={gps_data['sequence']}")
        
    except Exception as e:
        rospy.logerr(f"Error publishing GPS data: {e}")

# Usage example:
"""
# Initialize parsers and pairer
lat_parser = GpsLatitudeParser()
lon_parser = GpsLongitudeParser()
pairer = GpsCoordinatePairer()

# Process incoming CAN frames
def process_gps_can_frame(can_id: int, frame_data: List[int]):
    if can_id == GPS_LATITUDE_FRAME_ID:
        lat_data = lat_parser.parse(frame_data)
        if lat_data:
            complete_coords = pairer.add_frame(lat_data, 'latitude')
            if complete_coords:
                print(f"Complete GPS coordinates: {complete_coords}")
                
    elif can_id == GPS_LONGITUDE_FRAME_ID:
        lon_data = lon_parser.parse(frame_data)
        if lon_data:
            complete_coords = pairer.add_frame(lon_data, 'longitude')
            if complete_coords:
                print(f"Complete GPS coordinates: {complete_coords}")
    
    # Periodically clean up old frames
    pairer.clear_old_frames()
"""
