---
sidebar_position: 1
title: "Voice-to-Action Pipeline"
description: "Module 4, Chapter 1: Converting voice input to robot actions using OpenAI Whisper"
---

# Voice-to-Action Pipeline

## Introduction

In this chapter, we'll explore the Vision-Language-Action (VLA) integration system that enables humanoid robots to understand and execute voice commands. This chapter focuses on the foundational component: converting voice input to actionable commands using OpenAI Whisper technology.

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up and configure OpenAI Whisper for real-time speech recognition
- Process voice commands and convert them to text with confidence scoring
- Integrate speech recognition with ROS 2 for robotic applications
- Implement basic command parsing and validation

## Prerequisites

Before starting this chapter, you should have:
- Basic knowledge of ROS 2 (covered in Module 1)
- Understanding of Python programming
- Experience with simulation environments (covered in Module 2)
- Basic knowledge of humanoid robot control (covered in Module 3)

## OpenAI Whisper Integration

### Installation and Setup

First, let's install the required dependencies for OpenAI Whisper:

```bash
pip install openai-whisper torch torchaudio
pip install pyaudio numpy scipy
```

### Basic Whisper Usage

Here's a simple example of using Whisper for speech-to-text conversion:

```python
import whisper

# Load the model (this may take time on first run)
model = whisper.load_model("base")

# Transcribe audio
result = model.transcribe("path/to/audio.wav")
print(result["text"])
```

### Real-time Audio Processing

For real-time applications, we need to process audio streams. Here's how to capture and process audio:

```python
import pyaudio
import numpy as np
import whisper

class AudioProcessor:
    def __init__(self, sample_rate=16000, chunk_size=1024):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.audio = pyaudio.PyAudio()

    def capture_audio(self, duration=5.0):
        """Capture audio for a specific duration."""
        frames = []

        stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        try:
            chunks = int(duration * self.sample_rate / self.chunk_size)
            for _ in range(chunks):
                data = stream.read(self.chunk_size)
                frames.append(data)
        finally:
            stream.stop_stream()
            stream.close()

        return b''.join(frames)

    def audio_to_numpy(self, audio_data):
        """Convert audio bytes to numpy array for Whisper processing."""
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
        audio_array /= 32768.0  # Normalize to [-1, 1]
        return audio_array
```

## Voice Command Processing Pipeline

### Whisper Interface

Let's create a robust interface for Whisper that handles confidence scoring:

```python
import whisper
import torch
import numpy as np
from typing import Dict, Any

class WhisperInterface:
    def __init__(self, model_name: str = "base"):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model(model_name, device=self.device)

    def transcribe_audio(self, audio_data: bytes, language: str = "en") -> Dict[str, Any]:
        """Transcribe audio data with confidence scoring."""
        try:
            # Convert audio bytes to numpy array
            audio_array = self._bytes_to_numpy(audio_data)

            # Transcribe
            result = self.model.transcribe(audio_array, language=language)

            # Calculate confidence
            confidence = self._calculate_confidence(result)

            return {
                "text": result["text"].strip(),
                "confidence": confidence,
                "language": result.get("language", language),
                "duration": result.get("duration", 0.0)
            }
        except Exception as e:
            return {
                "text": "",
                "confidence": 0.0,
                "language": language,
                "error": str(e)
            }

    def _bytes_to_numpy(self, audio_data: bytes) -> np.ndarray:
        """Convert audio bytes to numpy array."""
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
        audio_array /= 32768.0
        return audio_array

    def _calculate_confidence(self, result: Dict) -> float:
        """Calculate confidence score from transcription result."""
        # Simplified confidence calculation
        text = result.get("text", "")
        if text.strip():
            return min(1.0, len(text.strip()) / 100.0 + 0.5)
        return 0.0
```

## Integration with ROS 2

### Creating a Speech Recognition Node

Let's create a ROS 2 node that integrates our Whisper interface:

```python
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String, Float32

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')

        # Initialize Whisper interface
        self.whisper_interface = WhisperInterface(model_name="base")

        # Publishers
        self.text_pub = self.create_publisher(String, 'recognized_text', 10)
        self.confidence_pub = self.create_publisher(Float32, 'recognition_confidence', 10)

        # Timer for processing
        self.timer = self.create_timer(0.1, self.process_audio)

        self.get_logger().info('Speech Recognition Node initialized')

    def process_audio(self):
        """Process audio and publish results."""
        # In a real implementation, this would capture audio
        # For this example, we'll simulate with dummy data
        dummy_audio = b'\x00' * 1024  # Placeholder for actual audio data

        result = self.whisper_interface.transcribe_audio(dummy_audio)

        if result["confidence"] > 0.5:  # Confidence threshold
            text_msg = String()
            text_msg.data = result["text"]
            self.text_pub.publish(text_msg)

            confidence_msg = Float32()
            confidence_msg.data = result["confidence"]
            self.confidence_pub.publish(confidence_msg)

            self.get_logger().info(f'Recognized: "{result["text"]}" (confidence: {result["confidence"]:.2f})')
```

## Command Parsing and Validation

### Basic Command Parser

Now let's implement a basic command parser to extract intent and entities:

```python
import re
from typing import Dict, List, Tuple

class CommandParser:
    def __init__(self):
        self.navigation_patterns = [
            (r'go to the (\w+)', 'navigate_to_location'),
            (r'move to the (\w+)', 'navigate_to_location'),
            (r'go to (\w+)', 'navigate_to_location'),
        ]

        self.object_patterns = [
            (r'pick up the (\w+ \w+|\w+)', 'pick_object'),
            (r'take the (\w+ \w+|\w+)', 'pick_object'),
            (r'grab the (\w+ \w+|\w+)', 'pick_object'),
        ]

    def parse_command(self, text: str) -> Tuple[str, Dict[str, str]]:
        """Parse a command and extract intent and entities."""
        text_lower = text.lower().strip()

        # Check navigation patterns
        for pattern, intent in self.navigation_patterns:
            match = re.search(pattern, text_lower)
            if match:
                return intent, {"location": match.group(1)}

        # Check object patterns
        for pattern, intent in self.object_patterns:
            match = re.search(pattern, text_lower)
            if match:
                return intent, {"object": match.group(1)}

        # Default to unknown intent
        return "unknown", {"raw_command": text}
```

## Practical Exercise

### Exercise 1: Voice Command Recognition

Create a simple application that captures voice commands and processes them:

1. Create a Python script that uses the AudioProcessor to capture audio
2. Process the audio with Whisper to get text
3. Parse the command to extract intent
4. Print the results

```python
def main():
    # Initialize components
    audio_processor = AudioProcessor()
    whisper_interface = WhisperInterface()
    command_parser = CommandParser()

    print("Press Enter to capture audio for 3 seconds...")
    input()

    # Capture audio
    audio_data = audio_processor.capture_audio(duration=3.0)
    print(f"Captured {len(audio_data)} bytes of audio data")

    # Process with Whisper
    result = whisper_interface.transcribe_audio(audio_data)
    print(f"Transcribed: '{result['text']}'")
    print(f"Confidence: {result['confidence']:.2f}")

    if result['confidence'] > 0.6:
        # Parse the command
        intent, entities = command_parser.parse_command(result['text'])
        print(f"Intent: {intent}")
        print(f"Entities: {entities}")
    else:
        print("Confidence too low, command not processed")

if __name__ == "__main__":
    main()
```

## Summary

In this chapter, we've built the foundation for voice-to-action processing using OpenAI Whisper. We've learned how to:
- Set up and configure Whisper for speech recognition
- Process real-time audio streams
- Create a robust interface with confidence scoring
- Integrate with ROS 2 for robotic applications
- Parse commands to extract intent and entities

## Next Steps

In the next chapter, we'll explore how to translate these natural language commands into cognitive planning and action sequences for humanoid robots.