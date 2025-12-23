# Quickstart Guide: Vision-Language-Action (VLA) Integration

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Python 3.11
- NVIDIA GPU with CUDA support (for accelerated processing)
- OpenAI Whisper model files (or access to Whisper API)

## Installation

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install Python dependencies:**
   ```bash
   pip install openai-whisper speechrecognition rclpy transformers torch
   ```

3. **Install ROS 2 dependencies:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-nav2-bringup ros-humble-isaac-ros-* ros-humble-vision-opencv
   ```

4. **Build the ROS 2 workspace:**
   ```bash
   colcon build --packages-select vla_integration
   source install/setup.bash
   ```

## Setup Whisper Model

1. **Download Whisper model:**
   ```bash
   python -c "import whisper; whisper.load_model('base')"
   ```

2. **Or configure API access (if using OpenAI API):**
   ```bash
   export OPENAI_API_KEY="your-api-key"
   ```

## Running the VLA System

1. **Launch the VLA integration system:**
   ```bash
   ros2 launch vla_integration vla_integration.launch.py
   ```

2. **The system will start these components:**
   - Speech recognition node
   - Natural language understanding service
   - Task planning service
   - ROS 2 action execution interface

3. **Speak a command to the microphone:**
   - Example: "Move to the table and pick up the red block"
   - The system will process the command and execute robot actions

## Testing the System

1. **Check active nodes:**
   ```bash
   ros2 node list
   ```

2. **Monitor the system:**
   ```bash
   ros2 topic echo /vla_feedback
   ```

3. **Send a test command programmatically:**
   ```bash
   ros2 service call /process_voice_command vla_interfaces/VoiceCommand "{'audio_data': 'test'}"
   ```

## Configuration

The system can be configured via `config/vla_params.yaml`:

```yaml
speech_recognition:
  model: "base"  # Whisper model size
  language: "en"  # Audio language
  timeout: 5.0    # Audio processing timeout

nlu:
  confidence_threshold: 0.7  # Minimum confidence for command acceptance
  max_command_length: 200    # Maximum command length in characters

task_planning:
  max_action_sequence_length: 10  # Maximum number of actions in sequence
  action_timeout: 30.0            # Timeout for individual actions
```

## Troubleshooting

- **Audio not being detected**: Check microphone permissions and input levels
- **Speech recognition poor quality**: Ensure quiet environment or adjust Whisper model
- **Robot not responding**: Verify ROS 2 network connection and robot bringup
- **Command parsing errors**: Speak clearly and use simple, direct commands