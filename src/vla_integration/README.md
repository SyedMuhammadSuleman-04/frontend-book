# Vision-Language-Action (VLA) Integration Package

This package provides integration between voice input, natural language processing, and robotic action execution for humanoid robots.

## Overview

The VLA Integration system creates a complete pipeline from voice commands to robot action execution, featuring:

- **Speech Recognition**: Using OpenAI Whisper for accurate speech-to-text conversion
- **Natural Language Understanding**: Advanced NLU for intent extraction and entity recognition
- **Cognitive Planning**: Conversion of natural language commands to ROS 2 action sequences
- **Robot Control**: Integration with ROS 2 for reliable action execution
- **Security Validation**: Protection against dangerous commands and parameter injection

## Installation

```bash
pip install -r requirements.txt
```

## Usage

### Basic Pipeline Usage

```python
from vla_integration import VLAPipeline

# Create a pipeline with default configuration
pipeline = VLAPipeline()

# Process voice command (audio_data as bytes)
task_execution = pipeline.process_voice_command(audio_data)

# Check results
if task_execution.status == "completed":
    print("Task completed successfully!")
else:
    print(f"Task failed: {task_execution.feedback[-1].message}")
```

### With Custom Configuration

```python
from vla_integration import VLAPipeline

config = {
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
}

pipeline = VLAPipeline(config=config)
```

### Using Individual Components

```python
from vla_integration import SpeechRecognitionService

# Initialize speech recognition service
speech_service = SpeechRecognitionService(
    model_name="base",
    language="en",
    confidence_threshold=0.7,
    use_cache=True
)

# Process audio data
voice_command = speech_service.process_audio(audio_data)

if speech_service.is_valid_transcription(voice_command):
    print(f"Recognized: {voice_command.transcribed_text}")
    print(f"Confidence: {voice_command.confidence}")
```

## Components

### Core Modules
- `vla_node`: Main ROS 2 node for the VLA system
- `speech_recognition`: Speech recognition with Whisper integration
- `nlu`: Natural language understanding components
- `cognitive_planning`: Action generation and task planning
- `ros_interfaces`: ROS 2 interfaces for robot control
- `pipeline`: Complete pipeline orchestration
- `utils`: Utility modules (logging, caching, validation, security)

### Data Models
- `VoiceCommand`: Represents a processed voice command
- `ActionSequence`: Represents a sequence of robot actions
- `TaskExecution`: Represents the execution of a task
- `TaskFeedback`: Represents feedback during task execution
- `PerceptionData`: Represents perception data for task execution

## Configuration

The system supports extensive configuration through a dictionary. Key parameters include:

- **Speech Recognition**: Model type, language, device, caching
- **NLU**: Confidence thresholds, command length limits
- **Task Planning**: Action sequence limits, timeouts, retry attempts
- **Safety**: Validation settings, velocity limits, obstacle detection
- **Performance**: Response time limits, monitoring settings

## Performance Optimizations

- **Model Caching**: Whisper models are cached to avoid reloading
- **Efficient Audio Processing**: Direct conversion without temporary files
- **Thread Safety**: Proper synchronization for concurrent access
- **Configurable Settings**: Device selection and caching options

## Security Features

- **Command Validation**: Pattern-based validation for dangerous commands
- **Parameter Validation**: Validation of action parameters
- **Entity Validation**: Checking of extracted entities for safety
- **Whitelist Approach**: Only safe command patterns are allowed

## Architecture

```
Voice Input → Speech Recognition → NLU → Cognitive Planning → ROS Execution → Robot Actions
```

The system follows a modular architecture with clear separation of concerns, allowing for easy extension and maintenance.

## Testing

The package includes comprehensive testing at multiple levels:

- **Unit Tests**: Individual component testing
- **Integration Tests**: Pipeline integration testing
- **Acceptance Tests**: End-to-end functionality testing

## Requirements

- Python 3.8+
- ROS 2 (Humble Hawksbill or later)
- OpenAI Whisper
- PyTorch
- Required ROS packages for robot control

## License

[Specify license type here]

## Contributing

Please see our contributing guidelines in the main repository.

## Support

For support, please open an issue in the repository or contact the development team.