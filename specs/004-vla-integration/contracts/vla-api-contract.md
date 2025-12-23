# API Contract: Vision-Language-Action (VLA) Integration

## Overview

This document defines the API contracts for the Vision-Language-Action (VLA) integration system. The system provides services for voice command processing, natural language understanding, and robot action execution.

## Services

### 1. Voice Command Processing Service

#### Process Voice Command
- **Type**: ROS 2 Service
- **Name**: `/process_voice_command`
- **Request**: `VoiceCommandRequest`
- **Response**: `VoiceCommandResponse`

**Request Fields:**
- `audio_data`: byte array - Raw audio data from microphone
- `language`: string - Language code (default: "en")
- `timeout`: float - Maximum processing time in seconds

**Response Fields:**
- `success`: bool - Whether the command was processed successfully
- `command_id`: string - Unique identifier for the processed command
- `transcription`: string - Transcribed text from speech
- `confidence`: float - Confidence score (0.0 to 1.0)
- `error_message`: string - Error details if success is false

### 2. Natural Language Understanding Service

#### Parse Command
- **Type**: ROS 2 Service
- **Name**: `/parse_command`
- **Request**: `CommandParseRequest`
- **Response**: `CommandParseResponse`

**Request Fields:**
- `command_text`: string - Transcribed command text
- `context`: string - Optional context for disambiguation

**Response Fields:**
- `intent`: string - Parsed intent from the command
- `entities`: map<string, string> - Extracted entities (objects, locations, actions)
- `action_sequence`: list<ActionStep> - Planned action sequence
- `confidence`: float - Confidence score (0.0 to 1.0)

### 3. Task Execution Service

#### Execute Task
- **Type**: ROS 2 Service
- **Name**: `/execute_task`
- **Request**: `TaskExecutionRequest`
- **Response**: `TaskExecutionResponse`

**Request Fields:**
- `action_sequence_id`: string - ID of the action sequence to execute
- `task_id`: string - Unique identifier for this task execution
- `parameters`: map<string, any> - Task-specific parameters

**Response Fields:**
- `execution_id`: string - Unique identifier for the task execution
- `status`: string - Initial status of the execution
- `estimated_duration`: float - Estimated time for completion in seconds

#### Get Task Status
- **Type**: ROS 2 Service
- **Name**: `/get_task_status`
- **Request**: `TaskStatusRequest`
- **Response**: `TaskStatusResponse`

**Request Fields:**
- `execution_id`: string - ID of the task execution

**Response Fields:**
- `status`: string - Current status (pending, executing, completed, failed)
- `progress`: float - Completion percentage (0.0 to 1.0)
- `current_action`: string - Current action being executed
- `feedback`: list<string> - Recent feedback messages

### 4. Feedback and Monitoring

#### Feedback Topic
- **Type**: ROS 2 Topic
- **Name**: `/vla_feedback`
- **Message Type**: `VLAFeedback`

**Message Fields:**
- `timestamp`: time - When the feedback was generated
- `execution_id`: string - Associated task execution ID
- `level`: string - Feedback level (info, warning, error)
- `message`: string - Feedback message
- `data`: string - Additional data in JSON format

## Data Types

### ActionStep
- `action_type`: string - Type of action (navigate, detect_object, pick_place, etc.)
- `parameters`: map<string, string> - Action-specific parameters
- `timeout`: float - Maximum time for action completion in seconds

### VLAFeedback
- `timestamp`: time - Timestamp of feedback
- `execution_id`: string - Task execution ID
- `level`: string - Feedback level (info, warning, error)
- `message`: string - Feedback message
- `data`: string - Additional data in JSON format

## Error Handling

All services should return appropriate error codes:

- `0`: Success
- `1`: Invalid input parameters
- `2`: Processing error
- `3`: Timeout
- `4`: Robot unavailable
- `5`: Safety constraint violation

## Validation Rules

1. All timestamps must be in ROS time format
2. Confidence scores must be between 0.0 and 1.0
3. Action timeouts must be positive values
4. All required fields must be present
5. String fields must not exceed 1000 characters unless otherwise specified