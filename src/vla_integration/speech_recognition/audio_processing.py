"""
Audio Processing Module
Handles audio input capture and preprocessing for the VLA system
"""

import pyaudio
import numpy as np
import wave
import threading
from typing import Optional, Tuple, Callable
import time
import queue


class AudioProcessor:
    """
    Handles audio input, capture, and preprocessing for speech recognition.
    """

    def __init__(self, sample_rate: int = 16000, chunk_size: int = 1024, channels: int = 1):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.channels = channels
        self.format = pyaudio.paInt16

        self.audio = pyaudio.PyAudio()
        self.is_recording = False
        self.recording_thread = None
        self.audio_queue = queue.Queue()

        # Noise reduction parameters
        self.enable_noise_reduction = True
        self.noise_threshold = -30.0  # dB

    def start_recording(self, callback: Optional[Callable] = None) -> threading.Thread:
        """
        Start audio recording in a separate thread.

        Args:
            callback: Optional callback function to process audio chunks

        Returns:
            Thread object for the recording process
        """
        self.is_recording = True
        self.recording_thread = threading.Thread(
            target=self._record_audio,
            args=(callback,)
        )
        self.recording_thread.start()
        return self.recording_thread

    def stop_recording(self):
        """Stop audio recording."""
        self.is_recording = False
        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join()

    def _record_audio(self, callback: Optional[Callable] = None):
        """
        Internal method to record audio from the microphone.

        Args:
            callback: Optional callback function to process audio chunks
        """
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        try:
            while self.is_recording:
                data = stream.read(self.chunk_size, exception_on_overflow=False)

                # Add to queue for other processes
                self.audio_queue.put(data)

                # Call callback if provided
                if callback:
                    callback(data)

        except Exception as e:
            print(f"Error during audio recording: {e}")
        finally:
            stream.stop_stream()
            stream.close()

    def capture_audio(self, duration: float = 5.0) -> bytes:
        """
        Capture audio for a specific duration.

        Args:
            duration: Duration in seconds to capture audio

        Returns:
            Raw audio data as bytes
        """
        frames = []
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        try:
            # Calculate number of chunks needed
            chunks = int(duration * self.sample_rate / self.chunk_size)

            for _ in range(chunks):
                if not self.is_recording:  # Allow early termination
                    break
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                frames.append(data)

        finally:
            stream.stop_stream()
            stream.close()

        # Combine all frames into a single bytes object
        audio_data = b''.join(frames)
        return audio_data

    def preprocess_audio(self, audio_data: bytes) -> bytes:
        """
        Apply preprocessing to audio data (noise reduction, normalization, etc.).

        Args:
            audio_data: Raw audio data as bytes

        Returns:
            Preprocessed audio data as bytes
        """
        if not self.enable_noise_reduction:
            return audio_data

        # Convert bytes to numpy array for processing
        audio_array = np.frombuffer(audio_data, dtype=np.int16)

        # Apply noise reduction (simplified approach)
        # In a real implementation, this would use more sophisticated techniques
        if len(audio_array) > 0:
            # Calculate RMS amplitude
            rms = np.sqrt(np.mean(audio_array.astype(np.float32) ** 2))

            # Convert to dB
            if rms > 0:
                db = 20 * np.log10(rms)

                # If signal is below noise threshold, reduce amplitude
                if db < self.noise_threshold:
                    audio_array = audio_array * 0.1  # Reduce by 90%

        # Convert back to bytes
        processed_audio = audio_array.astype(np.int16).tobytes()
        return processed_audio

    def save_audio_to_wav(self, audio_data: bytes, filename: str):
        """
        Save audio data to a WAV file.

        Args:
            audio_data: Raw audio data as bytes
            filename: Path to save the WAV file
        """
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.audio.get_sample_size(self.format))
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_data)

    def __del__(self):
        """Cleanup audio resources."""
        if hasattr(self, 'audio'):
            self.audio.terminate()


# Example usage function
def example_usage():
    """Example of how to use the AudioProcessor."""
    processor = AudioProcessor()

    print("Starting audio capture for 3 seconds...")
    audio_data = processor.capture_audio(duration=3.0)
    print(f"Captured {len(audio_data)} bytes of audio data")

    # Preprocess the audio
    processed_audio = processor.preprocess_audio(audio_data)
    print(f"Processed audio data: {len(processed_audio)} bytes")

    # Save to file
    processor.save_audio_to_wav(audio_data, "test_audio.wav")
    print("Audio saved to test_audio.wav")

    # Cleanup
    del processor


if __name__ == "__main__":
    example_usage()