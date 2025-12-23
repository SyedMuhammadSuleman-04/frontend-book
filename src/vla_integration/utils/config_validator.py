"""
Configuration Validator Module
Validates configuration parameters for the VLA system
"""

from typing import Dict, Any, List, Tuple
import re


class ConfigValidator:
    """
    Validates configuration parameters for the VLA system.
    """

    def __init__(self):
        """
        Initialize the configuration validator with validation rules.
        """
        self.validation_rules = {
            # Speech recognition parameters
            "speech_recognition.model": {
                "type": str,
                "allowed_values": ["tiny", "base", "small", "medium", "large"],
                "required": True
            },
            "speech_recognition.language": {
                "type": str,
                "pattern": r"^[a-z]{2}$",  # Two-letter language code
                "required": True
            },
            "speech_recognition.timeout": {
                "type": float,
                "min_value": 1.0,
                "max_value": 30.0,
                "required": True
            },
            "speech_recognition.sample_rate": {
                "type": int,
                "min_value": 8000,
                "max_value": 48000,
                "required": False,
                "default": 16000
            },
            "speech_recognition.channels": {
                "type": int,
                "min_value": 1,
                "max_value": 2,
                "required": False,
                "default": 1
            },
            "speech_recognition.device": {
                "type": str,
                "allowed_values": ["cpu", "cuda", "auto", None],
                "required": False,
                "default": None
            },
            "speech_recognition.use_cache": {
                "type": bool,
                "required": False,
                "default": True
            },

            # NLU parameters
            "nlu.confidence_threshold": {
                "type": float,
                "min_value": 0.0,
                "max_value": 1.0,
                "required": True
            },
            "nlu.max_command_length": {
                "type": int,
                "min_value": 10,
                "max_value": 1000,
                "required": True
            },

            # Task planning parameters
            "task_planning.max_action_sequence_length": {
                "type": int,
                "min_value": 1,
                "max_value": 100,
                "required": True
            },
            "task_planning.action_timeout": {
                "type": float,
                "min_value": 1.0,
                "max_value": 300.0,  # 5 minutes max
                "required": True
            },
            "task_planning.retry_attempts": {
                "type": int,
                "min_value": 0,
                "max_value": 10,
                "required": True
            },

            # Audio processing parameters
            "audio.chunk_size": {
                "type": int,
                "min_value": 256,
                "max_value": 8192,
                "required": False,
                "default": 1024
            },
            "audio.enable_noise_reduction": {
                "type": bool,
                "required": False,
                "default": True
            },
            "audio.noise_reduction_threshold": {
                "type": float,
                "required": False,
                "default": -30.0
            },

            # Performance parameters
            "performance.max_response_time": {
                "type": float,
                "min_value": 0.1,
                "max_value": 10.0,
                "required": True
            },
            "performance.enable_performance_monitoring": {
                "type": bool,
                "required": False,
                "default": True
            },

            # Safety parameters
            "safety.enable_safety_validation": {
                "type": bool,
                "required": False,
                "default": True
            },
            "safety.max_velocity": {
                "type": float,
                "min_value": 0.01,
                "max_value": 2.0,
                "required": False,
                "default": 0.5
            },
            "safety.enable_obstacle_detection": {
                "type": bool,
                "required": False,
                "default": True
            }
        }

    def validate_config(self, config: Dict[str, Any]) -> Tuple[bool, List[str]]:
        """
        Validate the entire configuration.

        Args:
            config: Configuration dictionary to validate

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []

        # Check required parameters
        for param_path, rules in self.validation_rules.items():
            if rules.get("required", False):
                if not self._get_nested_value(config, param_path):
                    errors.append(f"Required parameter missing: {param_path}")

        # Validate all parameters
        for param_path, value in self._flatten_dict(config).items():
            if param_path in self.validation_rules:
                param_errors = self._validate_parameter(param_path, value, self.validation_rules[param_path])
                errors.extend(param_errors)
            else:
                # Check if it's a valid sub-parameter (e.g., under a category)
                param_errors = self._validate_parameter_path(param_path, value)
                if param_errors:
                    errors.extend(param_errors)

        return len(errors) == 0, errors

    def _validate_parameter(self, param_path: str, value: Any, rules: Dict[str, Any]) -> List[str]:
        """
        Validate a single parameter against its rules.

        Args:
            param_path: Path to the parameter (e.g., "speech_recognition.model")
            value: Value to validate
            rules: Validation rules for this parameter

        Returns:
            List of validation errors
        """
        errors = []

        # Check type
        expected_type = rules.get("type")
        if expected_type and not isinstance(value, expected_type):
            errors.append(f"Parameter '{param_path}' has incorrect type. Expected {expected_type.__name__}, got {type(value).__name__}")

        # Check allowed values
        allowed_values = rules.get("allowed_values")
        if allowed_values and value not in allowed_values:
            errors.append(f"Parameter '{param_path}' has invalid value '{value}'. Allowed values: {allowed_values}")

        # Check pattern (for strings)
        pattern = rules.get("pattern")
        if pattern and isinstance(value, str):
            if not re.match(pattern, value):
                errors.append(f"Parameter '{param_path}' does not match required pattern: {pattern}")

        # Check min/max values
        if isinstance(value, (int, float)):
            min_value = rules.get("min_value")
            max_value = rules.get("max_value")

            if min_value is not None and value < min_value:
                errors.append(f"Parameter '{param_path}' is below minimum value of {min_value}. Current value: {value}")

            if max_value is not None and value > max_value:
                errors.append(f"Parameter '{param_path}' exceeds maximum value of {max_value}. Current value: {value}")

        return errors

    def _validate_parameter_path(self, param_path: str, value: Any) -> List[str]:
        """
        Validate a parameter path that might not be explicitly defined in rules.

        Args:
            param_path: Path to the parameter
            value: Value to validate

        Returns:
            List of validation errors
        """
        errors = []

        # For now, just validate basic types for undefined parameters
        # In a real system, you might want more sophisticated validation

        return errors

    def _get_nested_value(self, config: Dict[str, Any], param_path: str) -> Any:
        """
        Get a value from a nested configuration dictionary using dot notation.

        Args:
            config: Configuration dictionary
            param_path: Parameter path (e.g., "speech_recognition.model")

        Returns:
            Value at the specified path, or None if not found
        """
        keys = param_path.split('.')
        current = config

        for key in keys:
            if isinstance(current, dict) and key in current:
                current = current[key]
            else:
                return None

        return current

    def _flatten_dict(self, d: Dict[str, Any], parent_key: str = '', sep: str = '.') -> Dict[str, Any]:
        """
        Flatten a nested dictionary using separator for nested keys.

        Args:
            d: Dictionary to flatten
            parent_key: Parent key for recursion
            sep: Separator for nested keys

        Returns:
            Flattened dictionary
        """
        items = []
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                items.extend(self._flatten_dict(v, new_key, sep=sep).items())
            else:
                items.append((new_key, v))
        return dict(items)

    def get_default_config(self) -> Dict[str, Any]:
        """
        Get a configuration dictionary with all default values filled in.

        Returns:
            Dictionary with default configuration values
        """
        default_config = {}

        for param_path, rules in self.validation_rules.items():
            default_value = rules.get("default")
            if default_value is not None:
                self._set_nested_value(default_config, param_path, default_value)

        return default_config

    def _set_nested_value(self, config: Dict[str, Any], param_path: str, value: Any):
        """
        Set a value in a nested configuration dictionary using dot notation.

        Args:
            config: Configuration dictionary
            param_path: Parameter path (e.g., "speech_recognition.model")
            value: Value to set
        """
        keys = param_path.split('.')
        current = config

        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]

        current[keys[-1]] = value


# Example usage function
def example_usage():
    """Example of how to use the ConfigValidator."""
    print("Configuration Validator example usage:")
    print("-" * 35)

    validator = ConfigValidator()

    # Example configuration
    config = {
        "speech_recognition": {
            "model": "base",
            "language": "en",
            "timeout": 5.0,
            "device": "cpu",
            "use_cache": True
        },
        "nlu": {
            "confidence_threshold": 0.7,
            "max_command_length": 200
        },
        "task_planning": {
            "max_action_sequence_length": 10,
            "action_timeout": 30.0,
            "retry_attempts": 3
        }
    }

    # Validate the configuration
    is_valid, errors = validator.validate_config(config)

    print(f"Configuration is valid: {is_valid}")
    if errors:
        print("Validation errors:")
        for error in errors:
            print(f"  - {error}")
    else:
        print("Configuration is valid!")

    # Get default configuration
    default_config = validator.get_default_config()
    print(f"\nDefault configuration has {len(validator._flatten_dict(default_config))} parameters")


if __name__ == "__main__":
    example_usage()