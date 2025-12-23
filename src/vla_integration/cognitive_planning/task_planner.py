"""
Task Planner Module
Implements task planning for multi-step commands and dependency resolution
"""

from typing import Dict, Any, List
from ..models.action_sequence import ActionSequence
from ..models.action_step import ActionStep
from datetime import datetime
import uuid


class TaskPlanner:
    """
    Plans complex multi-step tasks with dependency resolution and proper sequencing.
    """

    def __init__(self, max_sequence_length: int = 10):
        """
        Initialize the task planner.

        Args:
            max_sequence_length: Maximum number of actions allowed in a sequence
        """
        self.max_sequence_length = max_sequence_length

        # Define task dependencies and ordering rules
        self.dependency_rules = {
            # Navigation must happen before object manipulation
            ("detect_object", "pick_object"): ["detect_object", "pick_object"],
            ("navigate", "pick_object"): ["navigate", "pick_object"],
            ("navigate", "place_object"): ["navigate", "place_object"],
            ("detect_object", "place_object"): ["detect_object", "place_object"],

            # Object detection should happen before picking
            ("detect_object", "pick_object"): ["detect_object", "pick_object"],

            # For place operations, navigate to destination first
            ("navigate", "place_object"): ["navigate", "place_object"],
        }

    def plan_complex_task(self, commands: List[Dict[str, Any]]) -> ActionSequence:
        """
        Plan a complex multi-step task from a sequence of commands.

        Args:
            commands: List of command dictionaries with intent and entities

        Returns:
            ActionSequence object representing the planned task
        """
        # Generate individual action sequences for each command
        individual_sequences = []
        for cmd in commands:
            intent = cmd.get("intent", "")
            entities = cmd.get("entities", {})

            # This is a simplified approach - in a real system, we'd have access
            # to the ActionGenerator to create these sequences
            action_step = self._create_action_step_for_command(intent, entities)
            sequence = ActionSequence(
                id=str(uuid.uuid4()),
                command_id=cmd.get("id", "temp"),
                actions=[action_step],
                status="pending",
                dependencies=[],
                created_at=datetime.now()
            )
            individual_sequences.append(sequence)

        # Combine and organize sequences with proper dependencies
        combined_sequence = self._organize_sequences(individual_sequences)

        return combined_sequence

    def _create_action_step_for_command(self, intent: str, entities: Dict[str, Any]) -> ActionStep:
        """
        Create an action step for a given command intent and entities.

        Args:
            intent: Command intent
            entities: Command entities

        Returns:
            ActionStep object
        """
        # Simplified action type mapping
        action_type_map = {
            "navigate": "navigate_to_pose",
            "pick_up": "pick_object",
            "place": "place_object",
            "detect": "detect_object",
            "move": "move_base",
            "stop": "stop_robot",
            "follow": "follow_target",
            "wait": "wait"
        }

        action_type = action_type_map.get(intent, "unknown")

        # Set default parameters based on entities
        parameters = entities.copy()
        if intent == "navigate" and "location" in entities:
            parameters["target_pose"] = self._get_pose_for_location(entities["location"])
        elif intent == "move" and "direction" in entities:
            parameters["direction"] = entities["direction"]
            parameters["distance"] = 1.0

        return ActionStep(
            id=str(uuid.uuid4()),
            action_type=action_type,
            parameters=parameters,
            timeout=30,
            retry_count=3,
            status="pending"
        )

    def _organize_sequences(self, sequences: List[ActionSequence]) -> ActionSequence:
        """
        Organize action sequences with proper dependencies.

        Args:
            sequences: List of individual action sequences

        Returns:
            Combined ActionSequence with resolved dependencies
        """
        if len(sequences) == 0:
            return ActionSequence(
                id=str(uuid.uuid4()),
                command_id="empty",
                actions=[],
                status="pending",
                dependencies=[],
                created_at=datetime.now()
            )

        if len(sequences) == 1:
            return sequences[0]

        # Combine all actions from the sequences
        all_actions = []
        for seq in sequences:
            all_actions.extend(seq.actions)

        # Apply dependency rules to determine proper ordering
        ordered_actions = self._apply_dependencies(all_actions)

        # Create the final sequence with dependencies
        final_sequence = ActionSequence(
            id=str(uuid.uuid4()),
            command_id="planned_task",
            actions=ordered_actions,
            status="pending",
            dependencies=[],
            created_at=datetime.now()
        )

        return final_sequence

    def _apply_dependencies(self, actions: List[ActionStep]) -> List[ActionStep]:
        """
        Apply dependency rules to order actions properly.

        Args:
            actions: List of actions to order

        Returns:
            List of actions ordered with dependencies in mind
        """
        # This is a simplified dependency resolution
        # In a real system, this would be much more sophisticated

        # Separate actions by type
        navigation_actions = []
        detection_actions = []
        manipulation_actions = []
        other_actions = []

        for action in actions:
            if "navigate" in action.action_type or "move" in action.action_type:
                navigation_actions.append(action)
            elif "detect" in action.action_type or "find" in action.action_type:
                detection_actions.append(action)
            elif "pick" in action.action_type or "place" in action_type:
                manipulation_actions.append(action)
            else:
                other_actions.append(action)

        # Create ordered list: navigation -> detection -> manipulation -> other
        ordered_actions = navigation_actions + detection_actions + manipulation_actions + other_actions

        # If we exceed the maximum sequence length, we might need to break it down
        if len(ordered_actions) > self.max_sequence_length:
            # For now, just truncate - in a real system we'd handle this differently
            ordered_actions = ordered_actions[:self.max_sequence_length]

        return ordered_actions

    def _get_pose_for_location(self, location: str) -> Dict[str, float]:
        """
        Get a predefined pose for a given location.

        Args:
            location: The location name

        Returns:
            Dictionary with pose coordinates
        """
        # In a real system, this would come from a map or localization system
        predefined_poses = {
            "kitchen": {"x": 1.0, "y": 2.0, "z": 0.0, "orientation": 0.0},
            "living room": {"x": 0.0, "y": 0.0, "z": 0.0, "orientation": 0.0},
            "bedroom": {"x": -1.0, "y": 1.0, "z": 0.0, "orientation": 1.57},
            "table": {"x": 2.0, "y": 0.0, "z": 0.0, "orientation": 0.0},
            "couch": {"x": 0.0, "y": -1.0, "z": 0.0, "orientation": 3.14}
        }

        return predefined_poses.get(location.lower(), {"x": 0.0, "y": 0.0, "z": 0.0, "orientation": 0.0})

    def handle_ambiguous_commands(self, text: str, context: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Handle ambiguous commands by requesting clarification or making assumptions.

        Args:
            text: The ambiguous command text
            context: Context information for disambiguation

        Returns:
            Dictionary with disambiguation results
        """
        # This is a placeholder for handling ambiguous commands
        # In a real system, this would involve more sophisticated NLU
        return {
            "resolved_intent": "unknown",
            "resolved_entities": {},
            "requires_clarification": True,
            "suggestions": ["Please specify location", "Please specify object"]
        }


# Example usage function
def example_usage():
    """Example of how to use the TaskPlanner."""
    planner = TaskPlanner(max_sequence_length=10)

    print("Task planning examples:")
    print("-" * 40)

    # Example: Multi-step command "Go to kitchen, find red block, pick it up"
    commands = [
        {"intent": "navigate", "entities": {"location": "kitchen"}},
        {"intent": "detect", "entities": {"object": "red block"}},
        {"intent": "pick_up", "entities": {"object": "red block"}}
    ]

    planned_task = planner.plan_complex_task(commands)

    print(f"Multi-step task planned:")
    print(f"  Sequence ID: {planned_task.id}")
    print(f"  Number of actions: {len(planned_task.actions)}")
    print(f"  Status: {planned_task.status}")
    print()

    # Print each action in the sequence
    for i, action in enumerate(planned_task.actions):
        print(f"  Action {i+1}: {action.action_type}")
        print(f"    Parameters: {action.parameters}")
        print(f"    Status: {action.status}")

    print()

    # Example: Handling ambiguous command
    ambiguity_result = planner.handle_ambiguous_commands("Go there and do something")
    print(f"Ambiguous command handling:")
    print(f"  Requires clarification: {ambiguity_result['requires_clarification']}")
    print(f"  Suggestions: {ambiguity_result['suggestions']}")


if __name__ == "__main__":
    example_usage()