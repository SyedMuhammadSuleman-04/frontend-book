"""
PerceptionData Model
Represents sensor data processed during task execution, including object detection results and environmental mapping
"""

from dataclasses import dataclass
from typing import Dict, Any, Optional
from datetime import datetime


@dataclass
class PerceptionData:
    """
    Represents perception data from sensors during task execution.
    """
    id: str
    task_execution_id: str
    sensor_type: str  # camera, lidar, etc.
    data: bytes
    timestamp: datetime
    processed_results: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        """Validate the PerceptionData after initialization."""
        if self.processed_results is None:
            self.processed_results = {}

        # Validate sensor type
        allowed_sensor_types = {"camera", "lidar", "imu", "depth", "rgbd", "sonar"}
        if self.sensor_type not in allowed_sensor_types:
            raise ValueError(f"Sensor type must be one of {allowed_sensor_types}")