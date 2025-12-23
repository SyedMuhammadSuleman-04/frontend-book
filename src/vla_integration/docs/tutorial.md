# Vision-Language-Action (VLA) System - Tutorial

## Overview

This tutorial will guide you through building and using the Vision-Language-Action (VLA) system for humanoid robots. You'll learn how to set up the system, process voice commands, and execute robot actions.

## Prerequisites

Before starting this tutorial, ensure you have:
- Python 3.8+ installed
- ROS 2 (Humble Hawksbill or later) installed and sourced
- Basic understanding of Python and ROS concepts
- A humanoid robot or simulation environment (optional for initial testing)

## Step 1: Environment Setup

### Install Dependencies

First, let's set up the environment and install required packages:

```bash
# Create a virtual environment (recommended)
python -m venv vla_env
source vla_env/bin/activate  # On Windows: vla_env\Scripts\activate

# Install required packages
pip install openai-whisper torch torchaudio
pip install pyaudio numpy scipy
pip install rclpy  # ROS 2 Python client library
```

### Verify Installation

Create a simple test script to verify everything works:

```python
# test_installation.py
try:
    import whisper
    import torch
    import pyaudio
    print("✓ All required packages imported successfully!")

    # Check if CUDA is available
    if torch.cuda.is_available():
        print("✓ CUDA is available")
    else:
        print("⚠ CUDA not available, will use CPU")

except ImportError as e:
    print(f"✗ Import error: {e}")
```

Run the test:
```bash
python test_installation.py
```

## Step 2: Understanding the VLA Architecture

The VLA system consists of several key components:

```
Voice Input → Speech Recognition → NLU → Cognitive Planning → ROS Execution → Robot Actions
```

Let's explore each component:

### 2.1 Speech Recognition

The speech recognition component uses OpenAI Whisper to convert voice commands to text:

```python
# speech_recognition_example.py
from vla_integration.speech_recognition.whisper_interface import WhisperInterface

# Initialize Whisper interface
whisper_interface = WhisperInterface(model_name="base", use_cache=True)

# Example: Process audio data (in practice, you'd capture real audio)
# For this example, we'll just demonstrate the interface
print("Whisper model loaded:", whisper_interface.is_available())

# You would use: result = whisper_interface.transcribe_audio(audio_data)
```

### 2.2 Natural Language Understanding (NLU)

The NLU component parses the transcribed text to extract intent and entities:

```python
# nlu_example.py
from vla_integration.nlu.intent_extractor import IntentExtractor

# Initialize intent extractor
intent_extractor = IntentExtractor()

# Extract intent from text
text = "Go to the kitchen and pick up the red cup"
intent, entities, confidence = intent_extractor.extract_intent(text)

print(f"Intent: {intent}")
print(f"Entities: {entities}")
print(f"Confidence: {confidence}")
```

### 2.3 Cognitive Planning

The cognitive planning component converts intents to action sequences:

```python
# cognitive_planning_example.py
from vla_integration.cognitive_planning.action_generator import ActionGenerator

# Initialize action generator
action_generator = ActionGenerator()

# Generate actions from intent and entities
intent = "navigate_and_pickup"
entities = {"location": "kitchen", "object": "red cup"}

action_sequence = action_generator.generate_action_sequence(intent, entities)
print(f"Generated {len(action_sequence.actions)} actions")
for i, action in enumerate(action_sequence.actions):
    print(f"  {i+1}. {action.action_type}: {action.parameters}")
```

## Step 3: Building a Simple Voice Command System

Now let's build a simple system that processes voice commands:

```python
# simple_voice_system.py
import time
import pyaudio
import numpy as np
from vla_integration.pipeline.vla_pipeline import VLAPipeline
from vla_integration.speech_recognition.audio_processing import AudioProcessor

class SimpleVoiceSystem:
    def __init__(self):
        # Create VLA pipeline with optimized settings
        self.pipeline = VLAPipeline(config={
            "speech_recognition": {
                "model": "base",
                "language": "en",
                "device": "cpu",  # Use "cuda" if available
                "use_cache": True
            },
            "nlu": {
                "confidence_threshold": 0.7
            },
            "safety": {
                "enable_safety_validation": True
            }
        })

        # Audio processor for capturing voice input
        self.audio_processor = AudioProcessor()

    def capture_and_process_voice(self, duration=5.0):
        """
        Capture voice for specified duration and process it.

        Args:
            duration: Duration in seconds to capture audio

        Returns:
            TaskExecution result
        """
        print(f"Listening for {duration} seconds...")

        # Capture audio
        audio_data = self.audio_processor.capture_audio(duration=duration)
        print(f"Captured {len(audio_data)} bytes of audio data")

        # Process through VLA pipeline
        print("Processing voice command...")
        start_time = time.time()

        task_execution = self.pipeline.process_voice_command(audio_data)

        end_time = time.time()
        processing_time = end_time - start_time

        print(f"Processing completed in {processing_time:.2f} seconds")
        print(f"Task status: {task_execution.status}")

        return task_execution

    def run_demo(self):
        """Run a simple demo of the voice system."""
        print("=== VLA Voice Command System Demo ===")
        print("Speak a command when prompted...")
        print()

        while True:
            input("Press Enter to start listening, or 'q' to quit: ")
            response = input().strip().lower()
            if response == 'q':
                break

            result = self.capture_and_process_voice(duration=5.0)

            print(f"\n--- Results ---")
            print(f"Status: {result.status}")
            print(f"Progress: {result.progress * 100:.1f}%")

            if result.feedback:
                print("Feedback:")
                for feedback in result.feedback:
                    print(f"  - {feedback.level}: {feedback.message}")

            print()

if __name__ == "__main__":
    system = SimpleVoiceSystem()
    system.run_demo()
```

## Step 4: Testing with Sample Audio

For testing purposes, you can create sample audio or use text-to-speech to generate test commands:

```python
# test_with_sample.py
import wave
import numpy as np
from vla_integration.pipeline.vla_pipeline import VLAPipeline

def generate_silent_audio(duration=3.0, sample_rate=16000):
    """
    Generate silent audio for testing purposes.
    In practice, you'd capture real audio.
    """
    # Create silent audio data
    samples = int(duration * sample_rate)
    audio_data = np.zeros(samples, dtype=np.int16)
    return audio_data.tobytes()

def test_pipeline():
    """Test the VLA pipeline with sample data."""
    # Create pipeline
    pipeline = VLAPipeline(config={
        "speech_recognition": {
            "model": "base",
            "language": "en",
            "use_cache": True
        },
        "nlu": {
            "confidence_threshold": 0.7
        }
    })

    # Generate sample audio (silent in this case)
    sample_audio = generate_silent_audio(duration=3.0)

    print("Testing VLA pipeline with sample audio...")

    # Process the sample
    result = pipeline.process_voice_command(sample_audio)

    print(f"Task execution result:")
    print(f"  Status: {result.status}")
    print(f"  Progress: {result.progress}")

    for feedback in result.feedback:
        print(f"  Feedback: {feedback.level} - {feedback.message}")

if __name__ == "__main__":
    test_pipeline()
```

## Step 5: Creating Custom Voice Commands

Let's create a more sophisticated example that handles different types of commands:

```python
# custom_commands.py
from vla_integration.pipeline.vla_pipeline import VLAPipeline
from vla_integration.models.voice_command import VoiceCommand
import uuid
from datetime import datetime

class CustomVoiceController:
    def __init__(self):
        self.pipeline = VLAPipeline(config={
            "speech_recognition": {
                "model": "base",
                "language": "en",
                "use_cache": True
            },
            "nlu": {
                "confidence_threshold": 0.6  # Lower threshold for testing
            },
            "safety": {
                "enable_safety_validation": True
            }
        })

        # Command history
        self.command_history = []

    def process_command_text(self, text):
        """
        Process a text command (for testing without audio).
        In real usage, this would come from speech recognition.
        """
        # Create a mock VoiceCommand object
        voice_cmd = VoiceCommand(
            id=str(uuid.uuid4()),
            raw_audio=b"",  # Empty since we're using text
            transcribed_text=text,
            parsed_intent="",
            entities={},
            timestamp=datetime.now(),
            confidence=0.9  # High confidence for direct text input
        )

        # Process through NLU components
        from vla_integration.nlu.command_parser import CommandParser
        from vla_integration.nlu.intent_extractor import IntentExtractor

        cmd_parser = CommandParser()
        intent_extractor = IntentExtractor()

        # Update voice command with parsed data
        cmd_parser.update_voice_command(voice_cmd)
        intent, entities, confidence = intent_extractor.extract_intent(text)
        voice_cmd.parsed_intent = intent
        voice_cmd.entities.update(entities)
        voice_cmd.confidence = confidence

        print(f"Processed command: '{text}'")
        print(f"  Intent: {intent}")
        print(f"  Entities: {entities}")
        print(f"  Confidence: {confidence}")

        # In a real system, we would convert this to audio bytes and process through the full pipeline
        # For this tutorial, we'll just show the NLU results

        return voice_cmd

    def demo_commands(self):
        """Demonstrate processing of various voice commands."""
        commands = [
            "Go to the kitchen",
            "Pick up the red ball",
            "Move forward slowly",
            "Turn left carefully",
            "Find the blue cube",
            "Navigate to the table and place the object"
        ]

        print("=== VLA Command Processing Demo ===\n")

        for cmd in commands:
            print(f"Processing: '{cmd}'")
            result = self.process_command_text(cmd)
            print()

    def run_interactive_demo(self):
        """Run an interactive demo where user can input commands."""
        print("=== Interactive VLA Demo ===")
        print("Enter voice commands to process (type 'quit' to exit):")

        while True:
            user_input = input("\nEnter command: ").strip()

            if user_input.lower() in ['quit', 'exit', 'q']:
                print("Exiting demo...")
                break

            if user_input:
                print("Processing...")
                result = self.process_command_text(user_input)

                # Store in history
                self.command_history.append({
                    'command': user_input,
                    'result': result,
                    'timestamp': datetime.now()
                })

if __name__ == "__main__":
    controller = CustomVoiceController()

    # Run the demo
    controller.demo_commands()

    # Optionally run interactive demo
    response = input("\nRun interactive demo? (y/n): ")
    if response.lower() == 'y':
        controller.run_interactive_demo()
```

## Step 6: Performance Optimization

Let's explore performance optimization techniques:

```python
# performance_optimization.py
from vla_integration.pipeline.vla_pipeline import VLAPipeline
from vla_integration.utils.performance_monitor import get_performance_monitor
import time

def optimize_pipeline_performance():
    """Demonstrate performance optimization techniques."""

    print("=== Performance Optimization Demo ===\n")

    # 1. Basic pipeline (baseline)
    print("1. Basic pipeline configuration:")
    basic_config = {
        "speech_recognition": {
            "model": "base",
            "language": "en",
            "use_cache": True
        },
        "nlu": {"confidence_threshold": 0.7},
        "safety": {"enable_safety_validation": True}
    }

    basic_pipeline = VLAPipeline(config=basic_config)
    perf_monitor = get_performance_monitor()

    print(f"   Performance monitoring enabled: {perf_monitor.is_enabled()}")

    # 2. Performance metrics
    print("\n2. Performance metrics available:")
    stats = perf_monitor.get_statistics()
    print(f"   - Average response time: {stats.get('avg_response_time', 'N/A')}")
    print(f"   - Success rate: {stats.get('success_rate', 'N/A')}")
    print(f"   - Total requests: {stats.get('total_requests', 'N/A')}")

    # 3. Optimization tips
    print("\n3. Optimization strategies:")
    print("   - Use smaller models ('tiny', 'base') for faster processing")
    print("   - Enable model caching for repeated usage")
    print("   - Use GPU acceleration when available")
    print("   - Adjust confidence thresholds based on requirements")
    print("   - Pre-load models during initialization")

def performance_comparison():
    """Compare different configuration options."""

    configs = {
        "Fast (Tiny model)": {
            "speech_recognition": {"model": "tiny", "use_cache": True}
        },
        "Balanced (Base model)": {
            "speech_recognition": {"model": "base", "use_cache": True}
        },
        "Accurate (Large model)": {
            "speech_recognition": {"model": "large", "use_cache": True}
        }
    }

    print("\n=== Performance Comparison ===")
    for name, config in configs.items():
        print(f"\n{name}:")
        print(f"  Model: {config['speech_recognition']['model']}")
        print(f"  Caching: {config['speech_recognition']['use_cache']}")

        # In practice, you would test with actual audio data
        # This is just to show the configuration differences

if __name__ == "__main__":
    optimize_pipeline_performance()
    performance_comparison()
```

## Step 7: Integration with ROS 2

For full functionality, you'll want to integrate with ROS 2:

```python
# ros_integration.py
import rclpy
from rclpy.node import Node
from vla_integration.vla_node import VLANode

class VLAIntegratedSystem(Node):
    def __init__(self):
        super().__init__('vla_integrated_system')

        # Initialize the VLA node
        self.vla_node = VLANode()
        self.vla_node.initialize_components()

        self.get_logger().info('VLA Integrated System initialized')

        # You can now use the VLA components
        self.speech_service = self.vla_node.speech_recognition_service
        self.nlu_service = self.vla_node.nlu_service
        self.task_planner = self.vla_node.task_planning_service

    def process_voice_command_ros(self, audio_data):
        """Process voice command within ROS 2 context."""
        # Use the pipeline with ROS integration
        from vla_integration.pipeline.vla_pipeline import VLAPipeline

        pipeline = VLAPipeline(node=self.vla_node, config={
            "speech_recognition": {
                "model": "base",
                "language": "en",
                "use_cache": True
            }
        })

        return pipeline.process_voice_command(audio_data)

def main(args=None):
    rclpy.init(args=args)

    system = VLAIntegratedSystem()

    try:
        # Run your application logic here
        system.get_logger().info('System running...')
        rclpy.spin(system)
    except KeyboardInterrupt:
        pass
    finally:
        system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 8: Testing and Validation

Create tests to validate your VLA system:

```python
# test_vla_system.py
import unittest
from vla_integration.pipeline.vla_pipeline import VLAPipeline
from vla_integration.speech_recognition.speech_recognition_service import SpeechRecognitionService

class TestVLASystem(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.pipeline = VLAPipeline(config={
            "speech_recognition": {
                "model": "base",
                "language": "en",
                "use_cache": True
            },
            "nlu": {"confidence_threshold": 0.7}
        })

    def test_pipeline_initialization(self):
        """Test that the pipeline initializes correctly."""
        self.assertIsNotNone(self.pipeline.speech_service)
        self.assertIsNotNone(self.pipeline.command_parser)

    def test_configuration_validation(self):
        """Test configuration validation."""
        # Test with valid configuration
        valid_config = {
            "speech_recognition": {
                "model": "base",
                "language": "en",
                "use_cache": True
            }
        }

        pipeline = VLAPipeline(config=valid_config)
        self.assertIsNotNone(pipeline)

    def test_speech_service_initialization(self):
        """Test speech recognition service initialization."""
        service = SpeechRecognitionService(
            model_name="base",
            language="en",
            confidence_threshold=0.7,
            use_cache=True
        )

        self.assertIsNotNone(service.whisper_interface)
        self.assertIsNotNone(service.audio_processor)

if __name__ == '__main__':
    unittest.main()
```

## Conclusion

This tutorial has covered:

1. **Environment Setup**: Installing and verifying the VLA system
2. **Architecture Understanding**: Learning about the key components
3. **Basic Implementation**: Building a simple voice command system
4. **Custom Commands**: Handling different types of voice commands
5. **Performance Optimization**: Techniques for better performance
6. **ROS Integration**: Connecting with ROS 2
7. **Testing**: Validating the system

### Next Steps

1. **Connect to Hardware**: Integrate with your actual robot platform
2. **Custom Actions**: Add domain-specific robot actions
3. **Enhanced NLU**: Improve intent recognition for your use cases
4. **Security**: Review and customize security validation
5. **Performance Tuning**: Optimize for your specific hardware

### Resources

- Check the full documentation in `docs/vla_system_documentation.md`
- Review the quick start guide in `docs/quick_start_guide.md`
- Look at example implementations in the source code
- Join the community for support and updates

Remember to always test thoroughly in simulation before deploying to physical robots, and ensure all safety measures are in place.