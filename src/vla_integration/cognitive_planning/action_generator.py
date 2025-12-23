"""
Action Generator Module
Creates ROS 2 action sequences from parsed natural language commands
"""

from typing import Dict, Any, List
from ..models.action_sequence import ActionSequence
from ..models.action_step import ActionStep
from datetime import datetime
import uuid


class ActionGenerator:
    """
    Generates ROS 2 action sequences from parsed natural language commands.
    """

    def __init__(self):
        """
        Initialize the action generator with known action mappings.
        """
        # Define action mappings from intents to ROS 2 actions
        self.action_mappings = {
            "navigate": {
                "action_type": "navigate_to_pose",
                "required_entities": ["location"],
                "parameter_mapping": {
                    "location": "target_pose"
                }
            },
            "pick_up": {
                "action_type": "pick_object",
                "required_entities": ["object"],
                "parameter_mapping": {
                    "object": "target_object"
                }
            },
            "place": {
                "action_type": "place_object",
                "required_entities": ["object", "location"],
                "parameter_mapping": {
                    "object": "target_object",
                    "location": "target_location"
                }
            },
            "detect": {
                "action_type": "detect_object",
                "required_entities": ["object"],
                "parameter_mapping": {
                    "object": "target_object"
                }
            },
            "move": {
                "action_type": "move_base",
                "required_entities": ["direction"],
                "parameter_mapping": {
                    "direction": "direction"
                }
            },
            "stop": {
                "action_type": "stop_robot",
                "required_entities": [],
                "parameter_mapping": {}
            },
            "follow": {
                "action_type": "follow_target",
                "required_entities": ["target"],
                "parameter_mapping": {
                    "target": "target"
                }
            },
            "wait": {
                "action_type": "wait",
                "required_entities": [],
                "parameter_mapping": {}
            }
        }

    def generate_action_sequence(self, intent: str, entities: Dict[str, Any]) -> ActionSequence:
        """
        Generate an action sequence from intent and entities.

        Args:
            intent: Parsed intent from natural language
            entities: Extracted entities from the command

        Returns:
            ActionSequence object with the planned actions
        """
        if intent not in self.action_mappings:
            # If intent is unknown, create a simple error action
            action_step = ActionStep(
                id=str(uuid.uuid4()),
                action_type="error",
                parameters={"message": f"Unknown intent: {intent}"},
                timeout=5,
                retry_count=0,
                status="pending"
            )

            action_sequence = ActionSequence(
                id=str(uuid.uuid4()),
                command_id="unknown",
                actions=[action_step],
                status="pending",
                dependencies=[],
                created_at=datetime.now()
            )

            return action_sequence

        # Get the action mapping for this intent
        mapping = self.action_mappings[intent]

        # Check if required entities are present
        missing_entities = []
        for required_entity in mapping["required_entities"]:
            if required_entity not in entities or not entities[required_entity]:
                missing_entities.append(required_entity)

        if missing_entities:
            # Create an action sequence with an error action
            action_step = ActionStep(
                id=str(uuid.uuid4()),
                action_type="error",
                parameters={
                    "message": f"Missing required entities: {', '.join(missing_entities)}",
                    "intent": intent,
                    "entities": entities
                },
                timeout=5,
                retry_count=0,
                status="pending"
            )

            action_sequence = ActionSequence(
                id=str(uuid.uuid4()),
                command_id="missing_entities",
                actions=[action_step],
                status="pending",
                dependencies=[],
                created_at=datetime.now()
            )

            return action_sequence

        # Generate the appropriate action
        action_step = self._create_action_step(intent, entities, mapping)

        action_sequence = ActionSequence(
            id=str(uuid.uuid4()),
            command_id="generated",
            actions=[action_step],
            status="pending",
            dependencies=[],
            created_at=datetime.now()
        )

        return action_sequence

    def _create_action_step(self, intent: str, entities: Dict[str, Any], mapping: Dict[str, Any]) -> ActionStep:
        """
        Create an ActionStep based on intent, entities, and mapping.

        Args:
            intent: The parsed intent
            entities: Extracted entities
            mapping: Action mapping for this intent

        Returns:
            ActionStep object
        """
        # Map entities to action parameters
        parameters = {}
        for entity_key, param_key in mapping["parameter_mapping"].items():
            if entity_key in entities:
                parameters[param_key] = entities[entity_key]

        # Add additional parameters based on intent
        if intent == "navigate":
            # Convert location to a specific pose (simplified)
            location = entities.get("location", "unknown")
            parameters["target_pose"] = self._get_pose_for_location(location)
        elif intent == "move":
            # Convert direction to movement parameters
            direction = entities.get("direction", "forward")
            parameters["direction"] = direction
            parameters["distance"] = 1.0  # Default distance
        elif intent == "detect":
            # Add detection parameters
            parameters["detection_timeout"] = 10.0

        # Create the action step
        action_step = ActionStep(
            id=str(uuid.uuid4()),
            action_type=mapping["action_type"],
            parameters=parameters,
            timeout=30,  # Default timeout
            retry_count=3,  # Default retry count
            status="pending"
        )

        return action_step

    def _get_pose_for_location(self, location: str) -> Dict[str, float]:
        """
        Get a predefined pose for a given location (simplified implementation).

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

    def extend_for_complex_commands(self, action_sequences: List[ActionSequence]) -> ActionSequence:
        """
        Combine multiple action sequences for complex multi-step commands.

        Args:
            action_sequences: List of individual action sequences

        Returns:
            Combined ActionSequence with proper dependencies
        """
        if not action_sequences:
            # Return empty sequence
            return ActionSequence(
                id=str(uuid.uuid4()),
                command_id="empty",
                actions=[],
                status="pending",
                dependencies=[],
                created_at=datetime.now()
            )

        # Combine all actions from the sequences
        all_actions = []
        all_dependencies = []

        for seq in action_sequences:
            all_actions.extend(seq.actions)
            all_dependencies.extend(seq.dependencies)

        # Create a combined sequence
        combined_sequence = ActionSequence(
            id=str(uuid.uuid4()),
            command_id="combined",
            actions=all_actions,
            status="pending",
            dependencies=list(set(all_dependencies)),  # Remove duplicates
            created_at=datetime.now()
        )

        return combined_sequence


# Example usage function
def example_usage():
    """Example of how to use the ActionGenerator."""
    generator = ActionGenerator()

    print("Action generation examples:")
    print("-" * 40)

    # Example 1: Navigate command
    nav_sequence = generator.generate_action_sequence("navigate", {"location": "kitchen"})
    print(f"Navigate to kitchen:")
    print(f"  Sequence ID: {nav_sequence.id}")
    print(f"  Action type: {nav_sequence.actions[0].action_type}")
    print(f"  Parameters: {nav_sequence.actions[0].parameters}")
    print()

    # Example 2: Pick up command
    pick_sequence = generator.generate_action_sequence("pick_up", {"object": "red block"})
    print(f"Pick up red block:")
    print(f"  Sequence ID: {pick_sequence.id}")
    print(f"  Action type: {pick_sequence.actions[0].action_type}")
    print(f"  Parameters: {pick_sequence.actions[0].parameters}")
    print()

    # Example 3: Place command
    place_sequence = generator.generate_action_sequence("place", {"object": "cup", "location": "table"})
    print(f"Place cup on table:")
    print(f"  Sequence ID: {place_sequence.id}")
    print(f"  Action type: {place_sequence.actions[0].action_type}")
    print(f"  Parameters: {place_sequence.actions[0].parameters}")
    print()

    # Example 4: Command with missing entities
    error_sequence = generator.generate_action_sequence("place", {"object": "cup"})
    print(f"Place command with missing location:")
    print(f"  Sequence ID: {error_sequence.id}")
    print(f"  Action type: {error_sequence.actions[0].action_type}")
    print(f"  Parameters: {error_sequence.actions[0].parameters}")


if __name__ == "__main__":
    example_usage()