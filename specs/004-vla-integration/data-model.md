# Data Model: Vision-Language-Action (VLA) Integration

## Core Entities

### VoiceCommand
- **id**: string (unique identifier for the command)
- **raw_audio**: bytes (original audio data)
- **transcribed_text**: string (speech-to-text result)
- **parsed_intent**: string (extracted intent from command)
- **entities**: map<string, any> (key-value pairs of extracted entities like objects, locations, actions)
- **timestamp**: datetime (when command was received)
- **confidence**: float (confidence score of speech recognition)

### ActionSequence
- **id**: string (unique identifier for the sequence)
- **command_id**: string (reference to VoiceCommand)
- **actions**: list<ActionStep> (ordered list of action steps)
- **status**: enum (pending, executing, completed, failed)
- **dependencies**: list<string> (action IDs that must complete before this one)
- **created_at**: datetime

### ActionStep
- **id**: string (unique identifier for the step)
- **action_type**: string (type of action: navigate, detect_object, pick_place, etc.)
- **parameters**: map<string, any> (action-specific parameters)
- **timeout**: int (max time to execute in seconds)
- **retry_count**: int (number of retry attempts)
- **status**: enum (pending, executing, completed, failed)

### TaskExecution
- **id**: string (unique identifier for the execution)
- **voice_command_id**: string (reference to original voice command)
- **action_sequence_id**: string (reference to action sequence)
- **status**: enum (pending, executing, completed, failed, paused)
- **current_step**: string (ID of currently executing step)
- **progress**: float (percentage completion)
- **feedback**: list<TaskFeedback> (feedback messages during execution)
- **start_time**: datetime
- **end_time**: datetime (nullable)

### TaskFeedback
- **id**: string (unique identifier)
- **task_execution_id**: string (reference to task execution)
- **message**: string (feedback message)
- **level**: enum (info, warning, error)
- **timestamp**: datetime
- **data**: map<string, any> (additional data related to feedback)

### PerceptionData
- **id**: string (unique identifier)
- **task_execution_id**: string (reference to task execution)
- **sensor_type**: string (camera, lidar, etc.)
- **data**: bytes (sensor data)
- **timestamp**: datetime
- **processed_results**: map<string, any> (processed perception results like detected objects)

## State Transitions

### TaskExecution States
- `pending` → `executing` (when first action step starts)
- `executing` → `completed` (when all action steps succeed)
- `executing` → `failed` (when any action step fails permanently)
- `executing` → `paused` (when execution is temporarily halted)

### ActionStep States
- `pending` → `executing` (when action execution begins)
- `executing` → `completed` (when action completes successfully)
- `executing` → `failed` (when action fails after retries)

## Validation Rules

### VoiceCommand
- transcribed_text must not be empty
- confidence must be between 0.0 and 1.0
- timestamp must be within reasonable time window

### ActionSequence
- Must contain at least one action
- All dependency references must exist within the sequence
- Timeout values must be positive

### TaskExecution
- Cannot transition from completed or failed to other states
- Progress percentage must be between 0.0 and 1.0
- Start time must be before end time when end time is set