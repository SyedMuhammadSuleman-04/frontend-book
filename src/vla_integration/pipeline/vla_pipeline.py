"""
VLA Pipeline Orchestrator
Manages the complete pipeline from voice commands to robot action execution
"""

import uuid
from datetime import datetime
from typing import Dict, Any, List, Optional
from ..models.voice_command import VoiceCommand
from ..models.action_sequence import ActionSequence
from ..models.task_execution import TaskExecution
from ..models.task_feedback import TaskFeedback
from ..models.perception_data import PerceptionData
from ..speech_recognition.speech_recognition_service import SpeechRecognitionService
from ..nlu.command_parser import CommandParser
from ..nlu.intent_extractor import IntentExtractor
from ..cognitive_planning.action_generator import ActionGenerator
from ..cognitive_planning.task_planner import TaskPlanner
from ..ros_interfaces.ros_action_client import ROSActionClient
from ..ros_interfaces.robot_control_interface import RobotControlInterface
import time


class VLAPipeline:
    """
    Orchestrates the complete Vision-Language-Action pipeline from voice input to robot execution.
    """

    def __init__(self, node=None, config: Optional[Dict[str, Any]] = None):
        """
        Initialize the VLA pipeline.

        Args:
            node: ROS 2 node for ROS interfaces (optional)
            config: Configuration dictionary
        """
        self.config = config or {}

        # Initialize services
        self.speech_service = SpeechRecognitionService(
            model_name=self.config.get('speech_recognition', {}).get('model', 'base'),
            language=self.config.get('speech_recognition', {}).get('language', 'en'),
            confidence_threshold=self.config.get('nlu', {}).get('confidence_threshold', 0.7),
            device=self.config.get('speech_recognition', {}).get('device', None),
            use_cache=self.config.get('speech_recognition', {}).get('use_cache', True)
        )

        self.command_parser = CommandParser()
        self.intent_extractor = IntentExtractor()
        self.action_generator = ActionGenerator()
        self.task_planner = TaskPlanner(
            max_sequence_length=self.config.get('task_planning', {}).get('max_action_sequence_length', 10)
        )

        # Initialize ROS interfaces (if node is provided)
        self.ros_action_client = None
        self.robot_control_interface = None
        if node:
            self.ros_action_client = ROSActionClient(node)
            self.robot_control_interface = RobotControlInterface(node)

        # Track active task executions
        self.active_executions = {}

        # Safety settings
        self.enable_safety_validation = self.config.get('safety', {}).get('enable_safety_validation', True)
        self.max_velocity = self.config.get('safety', {}).get('max_velocity', 0.5)

    def process_voice_command(self, audio_data: bytes) -> TaskExecution:
        """
        Process a voice command through the complete pipeline.

        Args:
            audio_data: Raw audio data as bytes

        Returns:
            TaskExecution object representing the result
        """
        # Create a unique execution ID
        execution_id = str(uuid.uuid4())

        # Initialize task execution
        task_execution = TaskExecution(
            id=execution_id,
            voice_command_id="",
            action_sequence_id="",
            status="pending",
            progress=0.0
        )

        # Start performance monitoring for the entire pipeline
        from ..utils.performance_monitor import get_performance_monitor
        perf_monitor = get_performance_monitor()
        perf_monitor.start_timer(f"pipeline_{execution_id}")

        try:
            # Step 1: Speech recognition
            self._add_feedback(task_execution, "Starting speech recognition", "info")
            voice_command = self.speech_service.process_audio(audio_data)

            # Check if transcription meets confidence threshold
            if not self.speech_service.is_valid_transcription(voice_command):
                error_msg = f"Speech recognition confidence too low: {voice_command.confidence}"
                self._add_feedback(task_execution, error_msg, "error")
                task_execution.status = "failed"
                return task_execution

            task_execution.voice_command_id = voice_command.id
            self._add_feedback(task_execution, f"Recognized: '{voice_command.transcribed_text}'", "info")
            task_execution.progress = 0.2

            # Step 2: Natural Language Understanding
            self._add_feedback(task_execution, "Processing natural language understanding", "info")

            # Update voice command with parsed intent and entities
            self.command_parser.update_voice_command(voice_command)

            # Extract intent using the intent extractor
            intent, entities, confidence = self.intent_extractor.extract_intent(voice_command.transcribed_text)
            voice_command.parsed_intent = intent
            voice_command.entities.update(entities)

            self._add_feedback(task_execution, f"Parsed intent: {intent}", "info")
            task_execution.progress = 0.4

            # Step 3: Action Generation
            self._add_feedback(task_execution, "Generating action sequence", "info")
            action_sequence = self.action_generator.generate_action_sequence(intent, entities)
            task_execution.action_sequence_id = action_sequence.id
            task_execution.progress = 0.6

            # Step 4: Task Planning (for complex multi-step commands)
            if intent == "complex_task":  # Placeholder for complex task handling
                # In a real system, this would handle complex multi-step commands
                pass

            # Step 5: Safety Validation
            if self.enable_safety_validation:
                self._add_feedback(task_execution, "Performing safety validation", "info")
                if not self._validate_safety(action_sequence):
                    error_msg = "Action sequence failed safety validation"
                    self._add_feedback(task_execution, error_msg, "error")
                    task_execution.status = "failed"
                    return task_execution

            # Step 6: Execution
            self._add_feedback(task_execution, "Starting action execution", "info")
            task_execution.status = "executing"
            task_execution.start_time = datetime.now()

            success = self._execute_action_sequence(action_sequence, task_execution)

            if success:
                task_execution.status = "completed"
                task_execution.progress = 1.0
                task_execution.end_time = datetime.now()
                self._add_feedback(task_execution, "Task completed successfully", "info")
                perf_monitor.record_success()
            else:
                task_execution.status = "failed"
                task_execution.end_time = datetime.now()
                self._add_feedback(task_execution, "Task execution failed", "error")
                perf_monitor.record_failure()

        except Exception as e:
            task_execution.status = "failed"
            error_msg = f"Error in VLA pipeline: {str(e)}"
            self._add_feedback(task_execution, error_msg, "error")
            print(error_msg)  # Also print to console for debugging
            perf_monitor.record_failure()

        finally:
            # End performance monitoring and record the elapsed time
            elapsed_time = perf_monitor.end_timer(f"pipeline_{execution_id}")
            self._add_feedback(task_execution, f"Pipeline completed in {elapsed_time:.3f}s", "info")

            # Check performance compliance
            if elapsed_time > 3.0:
                self._add_feedback(task_execution, f"Warning: Pipeline took {elapsed_time:.3f}s (>3s)", "warning")

        return task_execution

    def _validate_safety(self, action_sequence: ActionSequence) -> bool:
        """
        Validate an action sequence for safety compliance.

        Args:
            action_sequence: Action sequence to validate

        Returns:
            True if safe, False otherwise
        """
        # In a real system, this would perform comprehensive safety checks
        # For now, just check if the action sequence is valid
        if not action_sequence or not action_sequence.actions:
            return False

        # Check for potentially unsafe action types or parameters
        for action in action_sequence.actions:
            if action.action_type == "unknown":
                return False

        return True

    def _execute_action_sequence(self, action_sequence: ActionSequence, task_execution: TaskExecution) -> bool:
        """
        Execute an action sequence on the robot.

        Args:
            action_sequence: Action sequence to execute
            task_execution: Task execution object for feedback

        Returns:
            True if all actions completed successfully, False otherwise
        """
        if not self.ros_action_client:
            self._add_feedback(task_execution, "ROS Action Client not available", "error")
            return False

        success = True
        total_actions = len(action_sequence.actions)

        for i, action_step in enumerate(action_sequence.actions):
            self._add_feedback(
                task_execution,
                f"Executing action {i+1}/{total_actions}: {action_step.action_type}",
                "info"
            )

            # Update progress
            task_execution.progress = 0.6 + (0.4 * (i + 1) / total_actions)

            # Execute the action
            action_success = self.ros_action_client.execute_action(
                action_step.action_type,
                action_step.parameters,
                timeout=action_step.timeout
            )

            if not action_success:
                self._add_feedback(
                    task_execution,
                    f"Action failed: {action_step.action_type}",
                    "error"
                )
                success = False

                # Check if we should continue or fail fast
                if action_step.retry_count > 0:
                    # Try to retry the action
                    for retry in range(action_step.retry_count):
                        self._add_feedback(
                            task_execution,
                            f"Retrying action {action_step.action_type} (attempt {retry + 2})",
                            "info"
                        )

                        action_success = self.ros_action_client.execute_action(
                            action_step.action_type,
                            action_step.parameters,
                            timeout=action_step.timeout
                        )

                        if action_success:
                            break

                if not action_success:
                    # Action failed after retries, continue to next or stop based on configuration
                    self._add_feedback(
                        task_execution,
                        f"Action permanently failed after retries: {action_step.action_type}",
                        "error"
                    )

        return success

    def _add_feedback(self, task_execution: TaskExecution, message: str, level: str = "info"):
        """
        Add feedback to a task execution.

        Args:
            task_execution: Task execution to add feedback to
            message: Feedback message
            level: Feedback level (info, warning, error)
        """
        feedback = TaskFeedback(
            id=str(uuid.uuid4()),
            task_execution_id=task_execution.id,
            message=message,
            level=level,
            timestamp=datetime.now()
        )

        task_execution.feedback.append(feedback)

    def capture_and_process(self, duration: float = 5.0) -> TaskExecution:
        """
        Capture audio for a specified duration and process it through the pipeline.

        Args:
            duration: Duration in seconds to capture audio

        Returns:
            TaskExecution object representing the result
        """
        # Capture audio
        voice_command = self.speech_service.capture_and_process(duration=duration)

        # Process through pipeline
        return self.process_voice_command(voice_command.raw_audio)

    def get_task_status(self, execution_id: str) -> Optional[TaskExecution]:
        """
        Get the status of a task execution.

        Args:
            execution_id: ID of the task execution

        Returns:
            TaskExecution object or None if not found
        """
        # In a real system, this would look up active or completed executions
        # For this implementation, we don't maintain a registry of past executions
        # This is a placeholder for the concept
        return self.active_executions.get(execution_id)

    def cancel_task(self, execution_id: str) -> bool:
        """
        Cancel a running task execution.

        Args:
            execution_id: ID of the task execution to cancel

        Returns:
            True if cancellation was successful, False otherwise
        """
        # In a real system, this would interrupt a running task
        # For this implementation, it's a placeholder
        if execution_id in self.active_executions:
            task_execution = self.active_executions[execution_id]
            task_execution.status = "failed"
            self._add_feedback(task_execution, "Task cancelled by user", "info")
            return True
        return False


# Example usage function
def example_usage():
    """Example of how to use the VLAPipeline."""
    print("VLA Pipeline example usage:")
    print("-" * 30)

    # Create pipeline with default configuration
    pipeline = VLAPipeline(config={
        "speech_recognition": {"model": "base", "language": "en"},
        "nlu": {"confidence_threshold": 0.7},
        "task_planning": {"max_action_sequence_length": 10},
        "safety": {"enable_safety_validation": True}
    })

    print("VLA Pipeline initialized")
    print("Ready to process voice commands through the complete pipeline")
    print()
    print("Note: To run with real audio, you would:")
    print("1. Capture audio data from a microphone")
    print("2. Call pipeline.process_voice_command(audio_data)")
    print("3. Monitor the returned TaskExecution object for results")


if __name__ == "__main__":
    example_usage()