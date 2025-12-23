"""
Vision-Language-Action (VLA) Integration Package

This package provides integration between voice input, natural language processing,
and robotic action execution for humanoid robots.
"""

# Import main components for easy access
from .vla_node import VLANode
from .speech_recognition.speech_recognition_service import SpeechRecognitionService
from .nlu.command_parser import CommandParser
from .nlu.intent_extractor import IntentExtractor
from .cognitive_planning.action_generator import ActionGenerator
from .cognitive_planning.task_planner import TaskPlanner
from .ros_interfaces.ros_action_client import ROSActionClient
from .ros_interfaces.robot_control_interface import RobotControlInterface
from .pipeline.vla_pipeline import VLAPipeline
from .utils.performance_monitor import get_performance_monitor
from .utils.logger import get_logger
from .utils.config_validator import ConfigValidator
from .utils.cache import get_command_cache, get_whisper_cache
from .utils.security_validator import get_security_validator

__all__ = [
    'VLANode',
    'SpeechRecognitionService',
    'CommandParser',
    'IntentExtractor',
    'ActionGenerator',
    'TaskPlanner',
    'ROSActionClient',
    'RobotControlInterface',
    'VLAPipeline',
    'get_performance_monitor',
    'get_logger',
    'ConfigValidator',
    'get_command_cache',
    'get_whisper_cache',
    'get_security_validator'
]