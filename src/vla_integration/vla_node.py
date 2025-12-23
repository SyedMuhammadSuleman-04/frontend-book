"""
Basic ROS 2 Node for VLA Integration
Implements the basic ROS 2 node structure for the VLA system
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import threading
import time


class VLANode(Node):
    """
    Main ROS 2 node for the Vision-Language-Action integration system.
    """

    def __init__(self):
        super().__init__('vla_node')

        # Declare parameters with defaults
        self.declare_parameter('speech_recognition.model', 'base')
        self.declare_parameter('speech_recognition.language', 'en')
        self.declare_parameter('speech_recognition.timeout', 5.0)
        self.declare_parameter('speech_recognition.device', 'cpu')
        self.declare_parameter('speech_recognition.use_cache', True)
        self.declare_parameter('nlu.confidence_threshold', 0.7)
        self.declare_parameter('task_planning.max_action_sequence_length', 10)
        self.declare_parameter('task_planning.action_timeout', 30.0)

        # Get parameters
        self.speech_model = self.get_parameter('speech_recognition.model').value
        self.speech_language = self.get_parameter('speech_recognition.language').value
        self.speech_timeout = self.get_parameter('speech_recognition.timeout').value
        self.speech_device = self.get_parameter('speech_recognition.device').value
        self.speech_use_cache = self.get_parameter('speech_recognition.use_cache').value
        self.nlu_confidence_threshold = self.get_parameter('nlu.confidence_threshold').value
        self.max_action_sequence_length = self.get_parameter('task_planning.max_action_sequence_length').value
        self.action_timeout = self.get_parameter('task_planning.action_timeout').value

        self.get_logger().info('VLA Node initialized')

        # Initialize components (to be implemented in future phases)
        self.speech_recognition_service = None
        self.nlu_service = None
        self.task_planning_service = None
        self.ros_interfaces = None

    def initialize_components(self):
        """
        Initialize the various components of the VLA system.
        This will be expanded in later phases.
        """
        self.get_logger().info('Initializing VLA components...')

        # Initialize speech recognition service with optimization parameters
        from .speech_recognition.speech_recognition_service import SpeechRecognitionService
        self.speech_recognition_service = SpeechRecognitionService(
            model_name=self.speech_model,
            language=self.speech_language,
            confidence_threshold=self.nlu_confidence_threshold,
            device=self.speech_device,
            use_cache=self.speech_use_cache
        )

        # Initialize other services (these would be properly implemented in later phases)
        from .nlu.command_parser import CommandParser
        from .nlu.intent_extractor import IntentExtractor
        from .cognitive_planning.action_generator import ActionGenerator
        from .cognitive_planning.task_planner import TaskPlanner
        from .ros_interfaces.ros_action_client import ROSActionClient
        from .ros_interfaces.robot_control_interface import RobotControlInterface

        self.nlu_service = {
            'command_parser': CommandParser(),
            'intent_extractor': IntentExtractor()
        }
        self.task_planning_service = {
            'action_generator': ActionGenerator(),
            'task_planner': TaskPlanner(max_sequence_length=self.max_action_sequence_length)
        }
        self.ros_interfaces = {
            'action_client': ROSActionClient(self),
            'control_interface': RobotControlInterface(self)
        }

        self.get_logger().info('VLA components initialized successfully')


def main(args=None):
    rclpy.init(args=args)

    vla_node = VLANode()
    vla_node.initialize_components()

    try:
        rclpy.spin(vla_node)
    except KeyboardInterrupt:
        pass
    finally:
        vla_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()