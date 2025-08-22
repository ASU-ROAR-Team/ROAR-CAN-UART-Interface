
from typing import Dict, Optional

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
                'latitude': lat_data['latitude'],
                'longitude': lon_data['longitude'],
                'is_north': True, # TODO: Get from message
                'is_east': True, # TODO: Get from message
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
