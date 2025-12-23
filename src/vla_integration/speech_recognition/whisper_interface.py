"""
Whisper Interface Module
Integrates OpenAI Whisper for speech-to-text conversion with confidence scoring
"""

import whisper
import torch
import numpy as np
from typing import Dict, Any, Optional
import io
from datetime import datetime
import threading
from scipy.io import wavfile


class WhisperInterface:
    """
    Interface for OpenAI Whisper model for speech-to-text conversion.
    """

    # Class-level model cache to avoid reloading across instances
    _model_cache = {}
    _model_lock = threading.Lock()

    def __init__(self, model_name: str = "base", device: str = None, use_cache: bool = True):
        """
        Initialize the Whisper interface.

        Args:
            model_name: Whisper model size (tiny, base, small, medium, large)
            device: Device to run the model on (cpu, cuda, auto)
            use_cache: Whether to use model caching for faster initialization
        """
        self.model_name = model_name
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.use_cache = use_cache

        # Load or retrieve the Whisper model from cache
        self.model = self._load_model_cached(model_name, self.device)
        self.get_logger().info(f"Whisper model {model_name} loaded on {self.device}")

    def _load_model_cached(self, model_name: str, device: str):
        """
        Load model with caching to avoid reloading.

        Args:
            model_name: Whisper model size
            device: Device to run the model on

        Returns:
            Loaded Whisper model
        """
        cache_key = f"{model_name}_{device}"

        with self._model_lock:
            if self.use_cache and cache_key in self._model_cache:
                self.get_logger().info(f"Using cached Whisper model: {cache_key}")
                return self._model_cache[cache_key]

            self.get_logger().info(f"Loading Whisper model: {model_name} on {device}")
            model = whisper.load_model(model_name, device=device)

            if self.use_cache:
                self._model_cache[cache_key] = model

            return model

    def transcribe_audio(self, audio_data: bytes, language: str = "en") -> Dict[str, Any]:
        """
        Transcribe audio data to text using Whisper.

        Args:
            audio_data: Raw audio data as bytes
            language: Language code for the audio (default: "en")

        Returns:
            Dictionary containing transcribed text and confidence score
        """
        try:
            # Convert audio bytes to numpy array
            audio_array = self._bytes_to_numpy(audio_data)

            # Transcribe the audio
            result = self.model.transcribe(
                audio_array,
                language=language,
                temperature=0.0  # For consistent results
            )

            # Calculate confidence based on the model's internal probability
            confidence = self._calculate_confidence(result)

            return {
                "transcribed_text": result["text"].strip(),
                "confidence": confidence,
                "language": result.get("language", language),
                "duration": result.get("duration", 0.0),
                "segments": result.get("segments", [])
            }

        except Exception as e:
            self.get_logger().error(f"Error in Whisper transcription: {e}")
            return {
                "transcribed_text": "",
                "confidence": 0.0,
                "language": language,
                "duration": 0.0,
                "segments": [],
                "error": str(e)
            }

    def _bytes_to_numpy(self, audio_data: bytes) -> np.ndarray:
        """
        Convert raw audio bytes to numpy array for Whisper processing.

        Args:
            audio_data: Raw audio data as bytes

        Returns:
            Numpy array of audio samples
        """
        # Convert raw bytes to numpy array directly (assuming 16-bit PCM)
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)

        # Normalize audio to [-1, 1] range (Whisper expects this)
        audio_array /= 32768.0

        # Ensure audio is in the right format for Whisper (mono, 16kHz)
        # If stereo, convert to mono by taking the mean of channels
        if len(audio_array.shape) > 1:
            audio_array = np.mean(audio_array, axis=1)

        return audio_array

    def _calculate_confidence(self, result: Dict[str, Any]) -> float:
        """
        Calculate confidence score from Whisper result.

        Args:
            result: Whisper transcription result

        Returns:
            Confidence score between 0.0 and 1.0
        """
        # If the result has tokens with probabilities, calculate average
        if "segments" in result and result["segments"]:
            total_prob = 0.0
            token_count = 0

            for segment in result["segments"]:
                if "tokens" in segment:
                    for token in segment["tokens"]:
                        # Whisper tokens have probability information
                        if hasattr(token, 'probability'):
                            total_prob += token.probability
                            token_count += 1
                        elif isinstance(token, dict) and "probability" in token:
                            total_prob += token["probability"]
                            token_count += 1

            if token_count > 0:
                avg_prob = total_prob / token_count
                # Normalize to 0-1 range
                return max(0.0, min(1.0, avg_prob))

        # If no token probabilities available, use a simpler heuristic
        # based on the length and content of the transcription
        text = result.get("text", "")
        if text.strip():
            # Simple heuristic: longer, more complex text might be more reliable
            # This is a fallback since Whisper doesn't always provide probabilities
            return min(1.0, len(text.strip()) / 100.0 + 0.5)
        else:
            return 0.0

    def get_logger(self):
        """Simple logger for this class."""
        import logging
        return logging.getLogger(self.__class__.__name__)

    def is_available(self) -> bool:
        """
        Check if Whisper is properly configured and available.

        Returns:
            True if Whisper is available, False otherwise
        """
        return self.model is not None


# Example usage function
def example_usage():
    """Example of how to use the WhisperInterface."""
    # Initialize the interface with caching enabled
    whisper_interface = WhisperInterface(model_name="base", use_cache=True)

    # Example: transcribe some audio (in practice, you'd load actual audio)
    # For this example, we'll just show the interface
    print(f"Whisper model loaded: {whisper_interface.is_available()}")
    print(f"Model cached: {'_model_cache' in WhisperInterface.__dict__}")

    # Note: To test with real audio, you would:
    # audio_data = load_audio_from_file("path/to/audio.wav")
    # result = whisper_interface.transcribe_audio(audio_data)
    # print(f"Transcription: {result['transcribed_text']}")
    # print(f"Confidence: {result['confidence']}")


if __name__ == "__main__":
    example_usage()