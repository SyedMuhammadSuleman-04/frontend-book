"""
Security Validation Module
Provides security validation for command execution in the VLA system
"""

import re
from typing import Dict, Any, List, Tuple
from enum import Enum


class SecurityLevel(Enum):
    """
    Security level for commands.
    """
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4


class SecurityValidator:
    """
    Validates commands for security compliance before execution.
    """

    def __init__(self):
        """
        Initialize the security validator with security rules.
        """
        # Dangerous patterns that should be blocked
        self.dangerous_patterns = [
            # System commands
            r"(?i)\bsystem\b",
            r"(?i)\bexec\b",
            r"(?i)\bshell\b",
            r"(?i)\bcommand\b",
            r"(?i)\brun\b",
            # File system access
            r"(?i)\bfile\b",
            r"(?i)\bdelete\b",
            r"(?i)\bremove\b",
            r"(?i)\bformat\b",
            # Network commands
            r"(?i)\bnetwork\b",
            r"(?i)\bport\b",
            r"(?i)\bssh\b",
            r"(?i)\brdp\b",
            # Robot-specific dangerous actions
            r"(?i)\bshutdown\b",
            r"(?i)\bpower off\b",
            r"(?i)\bemergency stop\b",
            r"(?i)\bdisable safety\b",
        ]

        # Safe command patterns (whitelist approach)
        self.safe_patterns = [
            # Navigation commands
            r"(?i)\bgo to\b",
            r"(?i)\bmove to\b",
            r"(?i)\bnavigate to\b",
            r"(?i)\bwalk to\b",
            # Object interaction
            r"(?i)\bpick up\b",
            r"(?i)\btake\b",
            r"(?i)\bgrab\b",
            r"(?i)\bget\b",
            r"(?i)\bplace\b",
            r"(?i)\bput\b",
            # Detection commands
            r"(?i)\bfind\b",
            r"(?i)\bdetect\b",
            r"(?i)\blook for\b",
            r"(?i)\blocate\b",
            # Movement commands
            r"(?i)\bmove forward\b",
            r"(?i)\bmove backward\b",
            r"(?i)\bturn left\b",
            r"(?i)\bturn right\b",
            # Basic commands
            r"(?i)\bstop\b",
            r"(?i)\bwait\b",
            r"(?i)\bpause\b",
            r"(?i)\bfollow\b",
        ]

        # Define security levels for different command types
        self.command_security_levels = {
            "navigate": SecurityLevel.LOW,
            "pick_up": SecurityLevel.LOW,
            "place": SecurityLevel.LOW,
            "detect": SecurityLevel.LOW,
            "move": SecurityLevel.LOW,
            "stop": SecurityLevel.LOW,
            "wait": SecurityLevel.LOW,
            "follow": SecurityLevel.MEDIUM,
            "power_off": SecurityLevel.CRITICAL,
            "emergency_stop": SecurityLevel.HIGH,
            "disable_safety": SecurityLevel.CRITICAL,
        }

    def validate_command(self, command_text: str, parsed_intent: str = None, entities: Dict[str, Any] = None) -> Tuple[bool, str, SecurityLevel]:
        """
        Validate a command for security compliance.

        Args:
            command_text: Original command text
            parsed_intent: Parsed intent from NLU
            entities: Extracted entities from the command

        Returns:
            Tuple of (is_valid, reason, security_level)
        """
        # Check for dangerous patterns
        for pattern in self.dangerous_patterns:
            if re.search(pattern, command_text):
                return False, f"Command contains dangerous pattern: {pattern}", SecurityLevel.CRITICAL

        # Check if command is in safe patterns
        is_safe = False
        for pattern in self.safe_patterns:
            if re.search(pattern, command_text):
                is_safe = True
                break

        if not is_safe:
            return False, "Command does not match any safe pattern", SecurityLevel.HIGH

        # Determine security level based on intent
        security_level = self._get_security_level(parsed_intent or "unknown")

        # Additional checks based on entities
        if entities:
            if self._has_dangerous_entities(entities):
                return False, "Command contains potentially dangerous entities", SecurityLevel.HIGH

        return True, "Command is valid", security_level

    def _get_security_level(self, intent: str) -> SecurityLevel:
        """
        Get the security level for a command intent.

        Args:
            intent: Command intent

        Returns:
            Security level for the intent
        """
        return self.command_security_levels.get(intent, SecurityLevel.MEDIUM)

    def _has_dangerous_entities(self, entities: Dict[str, Any]) -> bool:
        """
        Check if entities contain dangerous values.

        Args:
            entities: Command entities to check

        Returns:
            True if entities contain dangerous values, False otherwise
        """
        dangerous_keywords = [
            "system", "admin", "root", "network", "file", "all", "every", "shutdown"
        ]

        for key, value in entities.items():
            if isinstance(value, str):
                for keyword in dangerous_keywords:
                    if keyword.lower() in value.lower():
                        return True

        return False

    def validate_action_parameters(self, action_type: str, parameters: Dict[str, Any]) -> Tuple[bool, str]:
        """
        Validate action parameters for security compliance.

        Args:
            action_type: Type of action to validate
            parameters: Action parameters to validate

        Returns:
            Tuple of (is_valid, reason)
        """
        # Validate based on action type
        if action_type == "navigate_to_pose":
            return self._validate_navigation_params(parameters)
        elif action_type in ["pick_object", "place_object", "detect_object"]:
            return self._validate_object_params(parameters)
        elif action_type == "move_base":
            return self._validate_movement_params(parameters)
        elif action_type == "stop_robot":
            return True, "Stop command is allowed"
        elif action_type == "follow_target":
            return self._validate_follow_params(parameters)
        else:
            # For unknown action types, apply general validation
            return self._validate_general_params(parameters)

    def _validate_navigation_params(self, parameters: Dict[str, Any]) -> Tuple[bool, str]:
        """
        Validate navigation action parameters.

        Args:
            parameters: Navigation parameters to validate

        Returns:
            Tuple of (is_valid, reason)
        """
        # Check if target pose is reasonable
        target_pose = parameters.get('target_pose', {})

        if not isinstance(target_pose, dict):
            return False, "Target pose must be a dictionary"

        # Check coordinates are reasonable (not extremely large values that could be malicious)
        for coord in ['x', 'y', 'z']:
            value = target_pose.get(coord, 0)
            if not isinstance(value, (int, float)):
                return False, f"Coordinate {coord} must be a number"
            if abs(value) > 1000:  # Reasonable limit for robot workspace
                return False, f"Coordinate {coord} value {value} is too large"

        return True, "Navigation parameters are valid"

    def _validate_object_params(self, parameters: Dict[str, Any]) -> Tuple[bool, str]:
        """
        Validate object interaction action parameters.

        Args:
            parameters: Object interaction parameters to validate

        Returns:
            Tuple of (is_valid, reason)
        """
        # Check object name is reasonable
        target_object = parameters.get('target_object', '')

        if not isinstance(target_object, str):
            return False, "Target object must be a string"

        # Check for potentially malicious object names
        dangerous_patterns = [
            r"[;|&]",  # Command separators
            r"system", r"admin", r"root", r"file", r"network"
        ]

        for pattern in dangerous_patterns:
            if re.search(pattern, target_object, re.IGNORECASE):
                return False, f"Target object name contains dangerous pattern: {pattern}"

        return True, "Object parameters are valid"

    def _validate_movement_params(self, parameters: Dict[str, Any]) -> Tuple[bool, str]:
        """
        Validate movement action parameters.

        Args:
            parameters: Movement parameters to validate

        Returns:
            Tuple of (is_valid, reason)
        """
        # Check direction is valid
        direction = parameters.get('direction', '').lower()
        valid_directions = ['forward', 'backward', 'left', 'right']

        if direction not in valid_directions:
            return False, f"Invalid direction: {direction}. Valid directions: {valid_directions}"

        # Check distance is reasonable
        distance = parameters.get('distance', 1.0)
        if not isinstance(distance, (int, float)):
            return False, "Distance must be a number"
        if abs(distance) > 10.0:  # Max 10m movement
            return False, f"Distance {distance}m is too large"

        return True, "Movement parameters are valid"

    def _validate_follow_params(self, parameters: Dict[str, Any]) -> Tuple[bool, str]:
        """
        Validate follow action parameters.

        Args:
            parameters: Follow parameters to validate

        Returns:
            Tuple of (is_valid, reason)
        """
        # Check target is reasonable
        target = parameters.get('target', '').lower()

        if not isinstance(target, str):
            return False, "Follow target must be a string"

        # Check for potentially malicious target names
        if any(char in target for char in [';', '|', '&']):
            return False, "Follow target contains invalid characters"

        return True, "Follow parameters are valid"

    def _validate_general_params(self, parameters: Dict[str, Any]) -> Tuple[bool, str]:
        """
        Validate general action parameters.

        Args:
            parameters: General parameters to validate

        Returns:
            Tuple of (is_valid, reason)
        """
        # Check for potentially dangerous parameter values
        for key, value in parameters.items():
            if isinstance(value, str):
                # Check for command injection patterns
                if any(char in value for char in [';', '|', '&', '$', '`']):
                    return False, f"Parameter {key} contains potentially dangerous characters"

        return True, "General parameters are valid"

    def get_security_recommendations(self, command_text: str) -> List[str]:
        """
        Get security recommendations for a command.

        Args:
            command_text: Command text to analyze

        Returns:
            List of security recommendations
        """
        recommendations = []

        # Check for potential issues
        if len(command_text) > 200:
            recommendations.append("Command is very long, consider validating length")

        if any(char.isupper() for char in command_text) and any(char.islower() for char in command_text):
            # Mixed case might be normal, but worth noting
            pass

        # Add more recommendations based on analysis
        return recommendations


# Global security validator instance
security_validator = SecurityValidator()


def get_security_validator() -> SecurityValidator:
    """
    Get the global security validator instance.

    Returns:
        SecurityValidator instance
    """
    return security_validator


# Example usage function
def example_usage():
    """Example of how to use the SecurityValidator."""
    print("Security Validator example usage:")
    print("-" * 30)

    validator = get_security_validator()

    # Test various commands
    test_commands = [
        ("Go to the kitchen", "navigate", {"location": "kitchen"}),
        ("Pick up the red block", "pick_up", {"object": "red block"}),
        ("Run system shutdown", "unknown", {"action": "shutdown"}),
        ("Move forward slowly", "move", {"direction": "forward", "distance": 0.5})
    ]

    for cmd_text, intent, entities in test_commands:
        is_valid, reason, security_level = validator.validate_command(cmd_text, intent, entities)
        print(f"Command: '{cmd_text}'")
        print(f"  Valid: {is_valid}")
        print(f"  Reason: {reason}")
        print(f"  Security Level: {security_level.name}")
        print()

    # Test action parameter validation
    print("Testing action parameter validation:")
    is_valid, reason = validator.validate_action_parameters(
        "navigate_to_pose",
        {"target_pose": {"x": 1.0, "y": 2.0, "z": 0.0}}
    )
    print(f"Navigation params valid: {is_valid}, reason: {reason}")

    is_valid, reason = validator.validate_action_parameters(
        "pick_object",
        {"target_object": "normal_object"}
    )
    print(f"Object params valid: {is_valid}, reason: {reason}")


if __name__ == "__main__":
    example_usage()