"""
Speech Recognition Service
Implements the speech recognition service with confidence scoring
"""

import uuid
from datetime import datetime
from typing import Dict, Any
from .whisper_interface import WhisperInterface
from .audio_processing import AudioProcessor
from ..models.voice_command import VoiceCommand


class SpeechRecognitionService:
    """
    Service class for handling speech recognition tasks.
    """

    def __init__(self, model_name: str = "base", language: str = "en", confidence_threshold: float = 0.7, device: str = None, use_cache: bool = True):
        """
        Initialize the speech recognition service.

        Args:
            model_name: Whisper model size
            language: Default language for speech recognition
            confidence_threshold: Minimum confidence for valid recognition
            device: Device to run the model on (cpu, cuda, auto)
            use_cache: Whether to use model caching for faster initialization
        """
        self.whisper_interface = WhisperInterface(model_name=model_name, device=device, use_cache=use_cache)
        self.audio_processor = AudioProcessor()
        self.language = language
        self.confidence_threshold = confidence_threshold

    def process_audio(self, audio_data: bytes) -> VoiceCommand:
        """
        Process audio data and return a VoiceCommand object.

        Args:
            audio_data: Raw audio data as bytes

        Returns:
            VoiceCommand object with transcribed text and metadata
        """
        # Preprocess the audio
        processed_audio = self.audio_processor.preprocess_audio(audio_data)

        # Transcribe the audio
        transcription_result = self.whisper_interface.transcribe_audio(
            processed_audio,
            language=self.language
        )

        # Create a unique ID for this command
        command_id = str(uuid.uuid4())

        # Create VoiceCommand object
        voice_command = VoiceCommand(
            id=command_id,
            raw_audio=audio_data,
            transcribed_text=transcription_result["transcribed_text"],
            parsed_intent="",  # Will be filled by NLU service
            entities={},  # Will be filled by NLU service
            timestamp=datetime.now(),
            confidence=transcription_result["confidence"]
        )

        return voice_command

    def is_valid_transcription(self, voice_command: VoiceCommand) -> bool:
        """
        Check if the transcription meets the confidence threshold.

        Args:
            voice_command: VoiceCommand object to validate

        Returns:
            True if transcription is valid, False otherwise
        """
        return voice_command.confidence >= self.confidence_threshold

    def capture_and_process(self, duration: float = 5.0) -> VoiceCommand:
        """
        Capture audio for a specific duration and process it.

        Args:
            duration: Duration in seconds to capture audio

        Returns:
            VoiceCommand object with transcribed text and metadata
        """
        # Capture audio
        audio_data = self.audio_processor.capture_audio(duration=duration)

        # Process the audio
        voice_command = self.process_audio(audio_data)

        return voice_command

    def get_confidence_threshold(self) -> float:
        """
        Get the current confidence threshold.

        Returns:
            Current confidence threshold value
        """
        return self.confidence_threshold

    def set_confidence_threshold(self, threshold: float):
        """
        Set a new confidence threshold.

        Args:
            threshold: New confidence threshold value (0.0 to 1.0)
        """
        if not 0.0 <= threshold <= 1.0:
            raise ValueError("Confidence threshold must be between 0.0 and 1.0")
        self.confidence_threshold = threshold


# Example usage function
def example_usage():
    """Example of how to use the SpeechRecognitionService."""
    service = SpeechRecognitionService(model_name="base", confidence_threshold=0.7, use_cache=True)

    print(f"Service initialized with confidence threshold: {service.get_confidence_threshold()}")
    print(f"Whisper model cached: {hasattr(service.whisper_interface, '_model_cache')}")

    # Note: To test with real audio, you would:
    # voice_command = service.capture_and_process(duration=3.0)
    # print(f"Transcribed: {voice_command.transcribed_text}")
    # print(f"Confidence: {voice_command.confidence}")
    # print(f"Valid: {service.is_valid_transcription(voice_command)}")

    # For this example, we'll just show the service is set up
    print("Speech recognition service ready.")


if __name__ == "__main__":
    example_usage()