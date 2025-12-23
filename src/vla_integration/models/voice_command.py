"""
VoiceCommand Model
Represents a spoken instruction from the user, containing raw audio, transcribed text, parsed intent, and extracted parameters
"""

from dataclasses import dataclass
from typing import Dict, Any, Optional
from datetime import datetime


@dataclass
class VoiceCommand:
    """
    Represents a voice command with all its components.
    """
    id: str
    raw_audio: bytes
    transcribed_text: str
    parsed_intent: str
    entities: Dict[str, Any]
    timestamp: datetime
    confidence: float

    def __post_init__(self):
        """Validate the VoiceCommand after initialization."""
        if not self.transcribed_text.strip():
            raise ValueError("transcribed_text must not be empty")

        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0.0 and 1.0")

        if self.timestamp > datetime.now():
            # Allow some tolerance for processing time
            import time
            current_time = datetime.fromtimestamp(time.time())
            if (self.timestamp - current_time).total_seconds() > 1.0:
                raise ValueError("timestamp must be within reasonable time window")