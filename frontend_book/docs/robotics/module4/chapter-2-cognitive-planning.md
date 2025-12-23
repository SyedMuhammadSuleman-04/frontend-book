---
sidebar_position: 2
title: "Cognitive Planning for VLA"
description: "Module 4, Chapter 2: Translating natural language to robot action sequences"
---

# Cognitive Planning for Vision-Language-Action

## Introduction

In this chapter, we'll explore how to translate natural language commands into executable action sequences for humanoid robots. This cognitive planning layer is crucial for converting high-level voice commands into specific robot behaviors.

## Learning Objectives

By the end of this chapter, you will be able to:
- Design intent extraction systems for natural language processing
- Create action generators that map language to robot actions
- Implement task planners for multi-step commands
- Integrate perception data into action planning
- Validate action sequences for safety and feasibility

## Prerequisites

Before starting this chapter, you should have:
- Understanding of speech recognition (covered in Chapter 1)
- Basic knowledge of ROS 2 actions and services
- Experience with humanoid robot control (covered in Module 3)
- Familiarity with simulation environments (covered in Module 2)

## Natural Language Understanding (NLU)

### Intent Extraction

The first step in cognitive planning is extracting the intent from the transcribed text. Let's create a more sophisticated intent extractor:

```python
import re
from typing import Dict, Tuple, List
from dataclasses import dataclass

@dataclass
class IntentResult:
    intent: str
    entities: Dict[str, str]
    confidence: float

class AdvancedIntentExtractor:
    def __init__(self):
        # Define complex patterns for different intents
        self.patterns = {
            "navigate": [
                (r'go to the (\w+)', {"location": 1}),
                (r'move to the (\w+)', {"location": 1}),
                (r'navigate to the (\w+)', {"location": 1}),
                (r'walk to the (\w+)', {"location": 1}),
                (r'head to the (\w+)', {"location": 1}),
            ],
            "pickup": [
                (r'pick up the (\w+ \w+|\w+)', {"object": 1}),
                (r'take the (\w+ \w+|\w+)', {"object": 1}),
                (r'grab the (\w+ \w+|\w+)', {"object": 1}),
                (r'get the (\w+ \w+|\w+)', {"object": 1}),
            ],
            "place": [
                (r'place the (\w+ \w+|\w+) on the (\w+)', {"object": 1, "location": 2}),
                (r'put the (\w+ \w+|\w+) on the (\w+)', {"object": 1, "location": 2}),
                (r'put down the (\w+ \w+|\w+)', {"object": 1}),
            ],
            "detect": [
                (r'find the (\w+ \w+|\w+)', {"object": 1}),
                (r'detect the (\w+ \w+|\w+)', {"object": 1}),
                (r'look for the (\w+ \w+|\w+)', {"object": 1}),
                (r'locate the (\w+ \w+|\w+)', {"object": 1}),
            ],
            "move": [
                (r'move (\w+) by? (\d*\.?\d+) meters?', {"direction": 1, "distance": 2}),
                (r'go (\w+) by? (\d*\.?\d+) meters?', {"direction": 1, "distance": 2}),
                (r'step (\w+)', {"direction": 1}),
            ],
            "complex_task": [
                (r'go to the (\w+) and (pick up|take) the (\w+ \w+|\w+)', {"location": 1, "action": 2, "object": 3}),
                (r'(pick up|take) the (\w+ \w+|\w+) and place it in the (\w+)', {"action": 1, "object": 2, "location": 3}),
            ]
        }

    def extract_intent(self, text: str) -> IntentResult:
        """Extract intent and entities from text with confidence scoring."""
        text_lower = text.lower().strip()

        for intent, pattern_list in self.patterns.items():
            for pattern, entity_map in pattern_list:
                match = re.search(pattern, text_lower)
                if match:
                    entities = {}
                    for entity_name, group_idx in entity_map.items():
                        entities[entity_name] = match.group(group_idx)

                    # Calculate confidence based on pattern match quality
                    confidence = self._calculate_confidence(text_lower, match, pattern)
                    return IntentResult(intent=intent, entities=entities, confidence=confidence)

        # If no pattern matches, return unknown with low confidence
        return IntentResult(intent="unknown", entities={"raw_text": text}, confidence=0.1)

    def _calculate_confidence(self, text: str, match, pattern: str) -> float:
        """Calculate confidence based on match quality."""
        # Simple confidence calculation based on text length and match completeness
        match_ratio = len(match.group(0)) / len(text) if text else 0
        return min(1.0, 0.5 + (match_ratio * 0.5))
```

## Action Generation

### Mapping Intents to Robot Actions

Now let's create an action generator that converts intents and entities into executable robot actions:

```python
from dataclasses import dataclass
from typing import List, Dict, Any
import uuid
from datetime import datetime

@dataclass
class ActionStep:
    action_type: str
    parameters: Dict[str, Any]
    timeout: float = 30.0
    retry_count: int = 3

@dataclass
class ActionSequence:
    id: str
    actions: List[ActionStep]
    created_at: datetime
    metadata: Dict[str, Any] = None

class ActionGenerator:
    def __init__(self):
        self.action_mappings = {
            "navigate": self._generate_navigate_actions,
            "pickup": self._generate_pickup_actions,
            "place": self._generate_place_actions,
            "detect": self._generate_detect_actions,
            "move": self._generate_move_actions,
            "complex_task": self._generate_complex_task_actions,
        }

    def generate_action_sequence(self, intent: str, entities: Dict[str, str]) -> ActionSequence:
        """Generate an action sequence based on intent and entities."""
        if intent in self.action_mappings:
            actions = self.action_mappings[intent](entities)
        else:
            # Default to unknown action
            actions = [ActionStep(
                action_type="unknown",
                parameters={"intent": intent, "entities": entities},
                timeout=5.0
            )]

        return ActionSequence(
            id=str(uuid.uuid4()),
            actions=actions,
            created_at=datetime.now(),
            metadata={"intent": intent, "entities": entities}
        )

    def _generate_navigate_actions(self, entities: Dict[str, str]) -> List[ActionStep]:
        """Generate navigation actions."""
        location = entities.get("location", "unknown")

        # In a real system, this would look up coordinates for the location
        # For now, we'll use a placeholder
        return [
            ActionStep(
                action_type="navigate_to_pose",
                parameters={
                    "target_pose": {
                        "x": self._get_location_x(location),
                        "y": self._get_location_y(location),
                        "z": 0.0
                    },
                    "location_name": location
                },
                timeout=60.0
            )
        ]

    def _generate_pickup_actions(self, entities: Dict[str, str]) -> List[ActionStep]:
        """Generate pickup actions."""
        obj = entities.get("object", "unknown")

        return [
            ActionStep(
                action_type="detect_object",
                parameters={"target_object": obj},
                timeout=30.0
            ),
            ActionStep(
                action_type="approach_object",
                parameters={"target_object": obj},
                timeout=30.0
            ),
            ActionStep(
                action_type="pick_object",
                parameters={"target_object": obj},
                timeout=30.0
            )
        ]

    def _generate_place_actions(self, entities: Dict[str, str]) -> List[ActionStep]:
        """Generate place actions."""
        obj = entities.get("object", "unknown")
        location = entities.get("location", "default")

        actions = []
        if location:
            actions.extend([
                ActionStep(
                    action_type="navigate_to_pose",
                    parameters={
                        "target_pose": {
                            "x": self._get_location_x(location),
                            "y": self._get_location_y(location),
                            "z": 0.0
                        },
                        "location_name": location
                    },
                    timeout=60.0
                )
            ])

        actions.append(
            ActionStep(
                action_type="place_object",
                parameters={"target_object": obj, "placement_location": location},
                timeout=30.0
            )
        )

        return actions

    def _generate_detect_actions(self, entities: Dict[str, str]) -> List[ActionStep]:
        """Generate detection actions."""
        obj = entities.get("object", "unknown")

        return [
            ActionStep(
                action_type="detect_object",
                parameters={"target_object": obj},
                timeout=30.0
            )
        ]

    def _generate_move_actions(self, entities: Dict[str, str]) -> List[ActionStep]:
        """Generate movement actions."""
        direction = entities.get("direction", "forward")
        distance = float(entities.get("distance", "1.0"))

        return [
            ActionStep(
                action_type="move_base",
                parameters={
                    "direction": direction,
                    "distance": distance
                },
                timeout=30.0
            )
        ]

    def _generate_complex_task_actions(self, entities: Dict[str, str]) -> List[ActionStep]:
        """Generate actions for complex multi-step tasks."""
        # Complex tasks require combining multiple action sequences
        # This is a simplified example - real systems would have more sophisticated planning

        actions = []

        # Navigate to location
        if "location" in entities:
            location = entities["location"]
            actions.append(
                ActionStep(
                    action_type="navigate_to_pose",
                    parameters={
                        "target_pose": {
                            "x": self._get_location_x(location),
                            "y": self._get_location_y(location),
                            "z": 0.0
                        },
                        "location_name": location
                    },
                    timeout=60.0
                )
            )

        # Pick up object
        if "object" in entities:
            obj = entities["object"]
            actions.extend([
                ActionStep(
                    action_type="detect_object",
                    parameters={"target_object": obj},
                    timeout=30.0
                ),
                ActionStep(
                    action_type="pick_object",
                    parameters={"target_object": obj},
                    timeout=30.0
                )
            ])

        return actions

    def _get_location_x(self, location: str) -> float:
        """Get X coordinate for a location (placeholder implementation)."""
        # In a real system, this would look up coordinates from a map
        location_coords = {
            "kitchen": 2.0,
            "table": 1.5,
            "door": 3.0,
            "couch": 0.5,
            "charger": -1.0
        }
        return location_coords.get(location, 0.0)

    def _get_location_y(self, location: str) -> float:
        """Get Y coordinate for a location (placeholder implementation)."""
        # In a real system, this would look up coordinates from a map
        location_coords = {
            "kitchen": 1.0,
            "table": -0.5,
            "door": 2.0,
            "couch": -1.5,
            "charger": 0.0
        }
        return location_coords.get(location, 0.0)
```

## Task Planning for Multi-Step Commands

### Dependency Resolution and Task Planning

For complex commands, we need a task planner that can handle dependencies and optimize action sequences:

```python
from typing import Set
import networkx as nx  # This would need to be installed: pip install networkx

class TaskPlanner:
    def __init__(self, max_sequence_length: int = 20):
        self.max_sequence_length = max_sequence_length

    def plan_task_sequence(self, initial_actions: List[ActionStep]) -> List[ActionStep]:
        """Plan and optimize a sequence of actions."""
        if len(initial_actions) > self.max_sequence_length:
            raise ValueError(f"Action sequence too long: {len(initial_actions)} > {self.max_sequence_length}")

        # Create a dependency graph
        graph = self._create_dependency_graph(initial_actions)

        # Perform topological sort to get execution order
        try:
            ordered_actions = list(nx.topological_sort(graph))
            return [action for action in initial_actions if action in ordered_actions]
        except nx.NetworkXUnfeasible:
            # If there's a cycle, return original order
            return initial_actions

    def _create_dependency_graph(self, actions: List[ActionStep]) -> nx.DiGraph:
        """Create a dependency graph for the actions."""
        graph = nx.DiGraph()

        # Add all actions as nodes
        for action in actions:
            graph.add_node(action)

        # Add dependencies (simplified example)
        # In a real system, dependencies would be determined by action types and parameters
        for i, action in enumerate(actions):
            if i > 0:
                # Simple sequential dependency
                graph.add_edge(actions[i-1], action)

        return graph

    def optimize_sequence(self, actions: List[ActionStep]) -> List[ActionStep]:
        """Optimize the action sequence for efficiency."""
        # This is a simplified optimization
        # In a real system, this would include:
        # - Action merging where possible
        # - Path optimization
        # - Resource allocation
        # - Parallel execution where safe

        optimized = []
        for action in actions:
            # Skip duplicate actions if they're identical
            if not self._is_duplicate_action(action, optimized):
                optimized.append(action)

        return optimized

    def _is_duplicate_action(self, action: ActionStep, existing_actions: List[ActionStep]) -> bool:
        """Check if an action is a duplicate of an existing one."""
        for existing in existing_actions:
            if (action.action_type == existing.action_type and
                action.parameters == existing.parameters):
                return True
        return False
```

## Integration with Perception Data

### Using Perception Data in Planning

Real-world planning needs to incorporate perception data to adapt to the current environment:

```python
from dataclasses import dataclass
from typing import Dict, Any, Optional

@dataclass
class PerceptionData:
    detected_objects: List[Dict[str, Any]]
    robot_pose: Dict[str, float]
    environment_map: Dict[str, Any]
    timestamp: datetime

class PerceptionIntegratedPlanner:
    def __init__(self):
        self.action_generator = ActionGenerator()
        self.task_planner = TaskPlanner()

    def generate_adaptive_sequence(self, intent: str, entities: Dict[str, str],
                                 perception: PerceptionData) -> ActionSequence:
        """Generate an action sequence adapted to current perception data."""

        # Adjust entities based on perception data
        adjusted_entities = self._adjust_entities_with_perception(entities, perception)

        # Generate base sequence
        base_sequence = self.action_generator.generate_action_sequence(intent, adjusted_entities)

        # Adapt sequence based on perception
        adapted_actions = self._adapt_actions_to_perception(
            base_sequence.actions, perception
        )

        return ActionSequence(
            id=base_sequence.id,
            actions=adapted_actions,
            created_at=base_sequence.created_at,
            metadata={**base_sequence.metadata, "adapted": True}
        )

    def _adjust_entities_with_perception(self, entities: Dict[str, str],
                                       perception: PerceptionData) -> Dict[str, str]:
        """Adjust entities based on current perception."""
        adjusted = entities.copy()

        # If looking for a specific object, check if it's detected
        target_obj = entities.get("object")
        if target_obj:
            detected_obj = self._find_object_in_perception(target_obj, perception)
            if detected_obj:
                adjusted["object_location"] = detected_obj.get("pose")

        # If navigating to a location, check if it's accessible
        location = entities.get("location")
        if location:
            # Check environment map for accessibility
            adjusted["location_accessible"] = self._is_location_accessible(
                location, perception.environment_map
            )

        return adjusted

    def _find_object_in_perception(self, target_obj: str, perception: PerceptionData) -> Optional[Dict[str, Any]]:
        """Find an object in the perception data."""
        for obj in perception.detected_objects:
            if target_obj.lower() in obj.get("name", "").lower():
                return obj
        return None

    def _is_location_accessible(self, location: str, env_map: Dict[str, Any]) -> bool:
        """Check if a location is accessible based on environment map."""
        # In a real system, this would check for obstacles, etc.
        return True  # Simplified for this example

    def _adapt_actions_to_perception(self, actions: List[ActionStep],
                                   perception: PerceptionData) -> List[ActionStep]:
        """Adapt action sequence based on perception data."""
        adapted = []

        for action in actions:
            # Adapt navigation actions based on obstacles
            if action.action_type == "navigate_to_pose":
                adapted_action = self._adapt_navigation_action(action, perception)
                adapted.append(adapted_action)
            # Adapt object detection based on known positions
            elif action.action_type == "detect_object":
                adapted_action = self._adapt_detection_action(action, perception)
                adapted.append(adapted_action)
            else:
                adapted.append(action)

        return adapted

    def _adapt_navigation_action(self, action: ActionStep, perception: PerceptionData) -> ActionStep:
        """Adapt navigation action based on obstacle detection."""
        # In a real system, this would use path planning algorithms
        # to find an obstacle-free path
        return action  # Simplified for this example

    def _adapt_detection_action(self, action: ActionStep, perception: PerceptionData) -> ActionStep:
        """Adapt detection action based on known object locations."""
        target_obj = action.parameters.get("target_object")
        known_obj = self._find_object_in_perception(target_obj, perception)

        if known_obj:
            # If object is already known, we might skip detection or go directly to it
            action.parameters["known_location"] = known_obj.get("pose")

        return action
```

## Safety Validation

### Action Sequence Validation

Before executing any action sequence, we need to validate it for safety:

```python
from enum import Enum
from typing import Tuple

class SafetyLevel(Enum):
    SAFE = 1
    CAUTION = 2
    DANGEROUS = 3

class SafetyValidator:
    def __init__(self):
        self.dangerous_actions = [
            "power_off", "emergency_stop", "disable_safety", "shutdown"
        ]

        self.max_velocities = {
            "linear": 1.0,  # m/s
            "angular": 1.0  # rad/s
        }

    def validate_action_sequence(self, sequence: ActionSequence) -> Tuple[bool, List[str]]:
        """Validate an action sequence for safety."""
        violations = []

        for i, action in enumerate(sequence.actions):
            action_violations = self._validate_action(action, i)
            violations.extend(action_violations)

        is_safe = len(violations) == 0
        return is_safe, violations

    def _validate_action(self, action: ActionStep, index: int) -> List[str]:
        """Validate a single action for safety."""
        violations = []

        # Check for dangerous action types
        if action.action_type.lower() in self.dangerous_actions:
            violations.append(
                f"Action {index}: Dangerous action type '{action.action_type}' not allowed"
            )

        # Validate parameters
        param_violations = self._validate_action_parameters(action)
        violations.extend([f"Action {index}: {v}" for v in param_violations])

        return violations

    def _validate_action_parameters(self, action: ActionStep) -> List[str]:
        """Validate action parameters for safety."""
        violations = []

        # Validate navigation parameters
        if action.action_type == "navigate_to_pose":
            target_pose = action.parameters.get("target_pose", {})
            x = target_pose.get("x", 0)
            y = target_pose.get("y", 0)

            # Check if coordinates are reasonable (not extremely large)
            if abs(x) > 100 or abs(y) > 100:
                violations.append("Navigation coordinates are too large")

        # Validate movement parameters
        elif action.action_type == "move_base":
            direction = action.parameters.get("direction", "").lower()
            valid_directions = ["forward", "backward", "left", "right"]

            if direction not in valid_directions:
                violations.append(f"Invalid movement direction: {direction}")

            distance = action.parameters.get("distance", 1.0)
            if abs(distance) > 10.0:  # Max 10m movement
                violations.append(f"Movement distance too large: {distance}m")

        # Validate object interaction parameters
        elif action.action_type in ["pick_object", "place_object"]:
            target_obj = action.parameters.get("target_object", "")
            if not target_obj:
                violations.append("No target object specified")

            # Check for potentially dangerous object names
            dangerous_keywords = ["system", "admin", "root", "network", "file", "all"]
            for keyword in dangerous_keywords:
                if keyword.lower() in target_obj.lower():
                    violations.append(f"Potentially dangerous object name: {target_obj}")

        return violations
```

## Practical Exercise

### Exercise 1: Cognitive Planning Implementation

Implement a complete cognitive planning pipeline:

1. Create a main pipeline class that integrates all components
2. Process a complex command like "Go to the kitchen and pick up the red cup"
3. Extract intent and entities
4. Generate action sequence
5. Validate for safety
6. Print the resulting action sequence

```python
class CognitivePlanningPipeline:
    def __init__(self):
        self.intent_extractor = AdvancedIntentExtractor()
        self.action_generator = ActionGenerator()
        self.task_planner = TaskPlanner()
        self.safety_validator = SafetyValidator()
        self.perception_planner = PerceptionIntegratedPlanner()

    def process_command(self, text: str, use_perception: bool = False) -> ActionSequence:
        """Process a natural language command end-to-end."""
        print(f"Processing command: '{text}'")

        # Extract intent and entities
        intent_result = self.intent_extractor.extract_intent(text)
        print(f"Extracted intent: {intent_result.intent}")
        print(f"Entities: {intent_result.entities}")
        print(f"Confidence: {intent_result.confidence:.2f}")

        if intent_result.confidence < 0.5:
            print("Confidence too low, skipping action generation")
            return None

        # Generate action sequence
        if use_perception:
            # Create mock perception data for this example
            from datetime import datetime
            mock_perception = PerceptionData(
                detected_objects=[
                    {"name": "red cup", "pose": {"x": 1.0, "y": 0.5, "z": 0.0}},
                    {"name": "blue block", "pose": {"x": 2.0, "y": 1.0, "z": 0.0}}
                ],
                robot_pose={"x": 0.0, "y": 0.0, "z": 0.0, "theta": 0.0},
                environment_map={"accessible_areas": ["kitchen", "living_room"]},
                timestamp=datetime.now()
            )
            sequence = self.perception_planner.generate_adaptive_sequence(
                intent_result.intent, intent_result.entities, mock_perception
            )
        else:
            sequence = self.action_generator.generate_action_sequence(
                intent_result.intent, intent_result.entities
            )

        print(f"Generated {len(sequence.actions)} actions")
        for i, action in enumerate(sequence.actions):
            print(f"  {i+1}. {action.action_type}: {action.parameters}")

        # Validate for safety
        is_safe, violations = self.safety_validator.validate_action_sequence(sequence)
        if is_safe:
            print("✓ Action sequence is safe")
        else:
            print("⚠ Safety violations found:")
            for violation in violations:
                print(f"  - {violation}")

        # Optimize sequence
        optimized_actions = self.task_planner.optimize_sequence(sequence.actions)
        print(f"Optimized to {len(optimized_actions)} actions")

        # Update sequence with optimized actions
        sequence.actions = optimized_actions

        return sequence

def main():
    pipeline = CognitivePlanningPipeline()

    # Test various commands
    test_commands = [
        "Go to the kitchen",
        "Pick up the red cup",
        "Go to the table and pick up the blue block",
        "Move forward by 2 meters",
        "Find the blue cube"
    ]

    for cmd in test_commands:
        print("\n" + "="*50)
        sequence = pipeline.process_command(cmd)
        if sequence:
            print(f"Final sequence has {len(sequence.actions)} actions")
        print("="*50)

if __name__ == "__main__":
    main()
```

## Summary

In this chapter, we've built a sophisticated cognitive planning system that:
- Extracts intent and entities from natural language with high accuracy
- Maps language to executable robot action sequences
- Plans multi-step tasks with dependency resolution
- Integrates perception data for adaptive planning
- Validates actions for safety before execution

## Next Steps

In the final chapter of Module 4, we'll implement the complete autonomous humanoid task execution pipeline that brings together voice recognition, cognitive planning, and real robot execution with feedback and adaptation capabilities.