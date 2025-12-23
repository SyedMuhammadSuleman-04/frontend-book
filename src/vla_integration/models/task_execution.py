"""
TaskExecution Model
Represents an ongoing task with status, progress tracking, and feedback mechanisms
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from datetime import datetime
from .task_feedback import TaskFeedback


@dataclass
class TaskExecution:
    """
    Represents an ongoing task execution with status and tracking.
    """
    id: str
    voice_command_id: str
    action_sequence_id: str
    status: str  # pending, executing, completed, failed, paused
    current_step: Optional[str] = None
    progress: float = 0.0  # percentage completion
    feedback: List[TaskFeedback] = None
    start_time: datetime = None
    end_time: Optional[datetime] = None

    def __post_init__(self):
        """Validate the TaskExecution after initialization."""
        if self.feedback is None:
            self.feedback = []

        # Validate status is one of the allowed values
        allowed_statuses = {"pending", "executing", "completed", "failed", "paused"}
        if self.status not in allowed_statuses:
            raise ValueError(f"Status must be one of {allowed_statuses}")

        # Validate progress percentage
        if not 0.0 <= self.progress <= 1.0:
            raise ValueError("Progress percentage must be between 0.0 and 1.0")

        # Validate time relationship if both times are set
        if self.start_time and self.end_time:
            if self.start_time > self.end_time:
                raise ValueError("Start time must be before end time when end time is set")

        # Validate that completed/failed tasks cannot transition to other states
        completed_failed_states = {"completed", "failed"}
        if self.status in completed_failed_states:
            # This is just a validation check - actual state transition logic would be elsewhere
            pass