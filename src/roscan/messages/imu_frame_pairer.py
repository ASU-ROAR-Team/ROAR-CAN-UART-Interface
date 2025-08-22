
from typing import Dict, Optional

class ImuFramePairer:
    """
    Utility class to pair IMU orientation and linear acceleration frames.
    """
    
    def __init__(self):
        self._pending_frames = {}
    
    def add_frame(self, frame_data: Dict, frame_type: str) -> Optional[Dict]:
        """
        Add an IMU frame and return a complete IMU message if available.
        """
        if not frame_data or 'sequence' not in frame_data:
            # For now, we assume no sequence number
            sequence = 0
        else:
            sequence = frame_data['sequence']
        
        if sequence not in self._pending_frames:
            self._pending_frames[sequence] = {}
        
        self._pending_frames[sequence][frame_type] = frame_data
        
        if 'orientation' in self._pending_frames[sequence] and 'linear_accel' in self._pending_frames[sequence]:
            orientation_data = self._pending_frames[sequence]['orientation']
            linear_accel_data = self._pending_frames[sequence]['linear_accel']
            
            complete_imu_data = {
                "orientation_x": orientation_data['orientation_x'],
                "orientation_y": orientation_data['orientation_y'],
                "orientation_z": orientation_data['orientation_z'],
                "orientation_w": orientation_data['orientation_w'],
                "linear_accel_x": linear_accel_data['linear_accel_x'],
                "linear_accel_y": linear_accel_data['linear_accel_y'],
                "linear_accel_z": linear_accel_data['linear_accel_z'],
                "sequence": sequence
            }
            
            del self._pending_frames[sequence]
            
            return complete_imu_data
        
        return None
    
    def clear_old_frames(self, max_age_sequences: int = 5) -> None:
        """
        Remove old unpaired frames to prevent memory buildup.
        """
        if len(self._pending_frames) > max_age_sequences:
            sorted_sequences = sorted(self._pending_frames.keys())
            sequences_to_remove = sorted_sequences[:-max_age_sequences]
            
            for seq in sequences_to_remove:
                del self._pending_frames[seq]
