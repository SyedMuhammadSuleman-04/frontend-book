# Vision-Language-Action (VLA) Integration - Quick Start Guide

## Overview

This guide will help you quickly set up and start using the Vision-Language-Action (VLA) Integration system for humanoid robots. The system enables voice command processing, natural language understanding, and robot action execution.

## Prerequisites

Before starting, ensure you have:

- Python 3.8 or higher
- ROS 2 (Humble Hawksbill or later) installed and sourced
- OpenAI Whisper installed (`pip install openai-whisper`)
- PyTorch installed (`pip install torch`)
- Appropriate ROS packages for your robot

## Installation

1. Clone or download the VLA Integration package
2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Build the ROS 2 package:
   ```bash
   colcon build
   source install/setup.bash
   ```

## Basic Setup

### 1. Import the VLA System

```python
from vla_integration import VLAPipeline, SpeechRecognitionService
```

### 2. Initialize the Pipeline

```python
# Create a basic pipeline
pipeline = VLAPipeline()

# Or create with custom configuration
config = {
    "speech_recognition": {
        "model": "base",  # Use "tiny" for faster processing, "large" for better accuracy
        "language": "en",
        "device": "cpu",  # Use "cuda" if you have a GPU
        "use_cache": True
    },
    "nlu": {
        "confidence_threshold": 0.7
    },
    "safety": {
        "enable_safety_validation": True
    }
}

pipeline = VLAPipeline(config=config)
```

## Usage Examples

### Example 1: Process Voice Command

```python
# Assuming you have audio data as bytes
audio_data = b"..."  # Your audio data here

# Process the voice command
task_execution = pipeline.process_voice_command(audio_data)

# Check the result
if task_execution.status == "completed":
    print("Task completed successfully!")
else:
    print(f"Task status: {task_execution.status}")
    for feedback in task_execution.feedback:
        print(f"- {feedback.level}: {feedback.message}")
```

### Example 2: Capture and Process Audio

```python
# Capture audio for 5 seconds and process
task_execution = pipeline.capture_and_process(duration=5.0)

print(f"Execution status: {task_execution.status}")
print(f"Progress: {task_execution.progress * 100:.1f}%")
```

### Example 3: Using Individual Components

```python
from vla_integration import SpeechRecognitionService

# Create a speech recognition service
speech_service = SpeechRecognitionService(
    model_name="base",
    language="en",
    confidence_threshold=0.7
)

# Process audio data
voice_command = speech_service.process_audio(audio_data)

# Check if transcription is valid
if speech_service.is_valid_transcription(voice_command):
    print(f"Recognized: {voice_command.transcribed_text}")
    print(f"Confidence: {voice_command.confidence}")
else:
    print(f"Low confidence: {voice_command.confidence}")
```

## Configuration Options

### Speech Recognition
- `model`: Whisper model size ("tiny", "base", "small", "medium", "large")
- `language`: Language code (e.g., "en", "es", "fr")
- `device`: "cpu", "cuda", or None (auto-detect)
- `use_cache`: True/False to enable model caching
- `timeout`: Maximum time to wait for speech recognition

### NLU (Natural Language Understanding)
- `confidence_threshold`: Minimum confidence for valid recognition (0.0-1.0)
- `max_command_length`: Maximum length of command text

### Safety
- `enable_safety_validation`: Enable/disable safety checks
- `max_velocity`: Maximum robot movement speed
- `enable_obstacle_detection`: Enable/disable obstacle detection

## Troubleshooting

### Common Issues

1. **Slow Performance**:
   - Use smaller Whisper models ("tiny" or "base")
   - Ensure GPU is properly configured if available
   - Enable model caching

2. **Low Recognition Accuracy**:
   - Use larger Whisper models ("medium" or "large")
   - Improve audio quality and reduce background noise
   - Adjust confidence threshold

3. **Security Validation Blocking Commands**:
   - Review command patterns in security validator
   - Ensure commands match safe patterns
   - Adjust security settings carefully

### Performance Tips

- Use caching for multiple instances
- Select appropriate model size for your performance needs
- Ensure proper audio preprocessing
- Monitor system resources during operation

## Next Steps

1. **Customize Configuration**: Adjust parameters based on your specific use case
2. **Integrate with Robot**: Connect to your specific robot platform
3. **Test with Real Audio**: Use actual microphone input for testing
4. **Extend Functionality**: Add custom actions and capabilities
5. **Performance Tuning**: Optimize for your specific hardware and requirements

## Example Complete Application

```python
import rclpy
from vla_integration import VLAPipeline

def main():
    # Initialize ROS 2
    rclpy.init()

    # Create pipeline with optimized settings
    pipeline = VLAPipeline(config={
        "speech_recognition": {
            "model": "base",
            "language": "en",
            "device": "cpu",
            "use_cache": True
        },
        "nlu": {
            "confidence_threshold": 0.7
        },
        "safety": {
            "enable_safety_validation": True
        }
    })

    print("VLA System ready. Speak a command!")

    # Process audio (you would capture real audio here)
    # task_execution = pipeline.capture_and_process(duration=5.0)

    # Cleanup
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

## Support

For additional help:
- Check the full documentation in `docs/vla_system_documentation.md`
- Review the example usage in each module
- Look at the test files for additional usage patterns
- Contact the development team for specific issues