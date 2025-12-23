"""
Robot Control Interface
Implements the interface for controlling the humanoid robot
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from typing import Dict, Any, Optional, List
import time


class RobotControlInterface:
    """
    Interface for controlling the humanoid robot through ROS 2.
    """

    def __init__(self, node: Node):
        """
        Initialize the robot control interface.

        Args:
            node: ROS 2 node to attach the control interface to
        """
        self.node = node
        self.get_logger = node.get_logger

        # QoS profile for publishers and subscribers
        qos_profile = QoSProfile(depth=10)

        # Publishers for robot control
        self.cmd_vel_publisher = self.node.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.joint_command_publisher = self.node.create_publisher(JointState, '/joint_commands', qos_profile)
        self.body_command_publisher = self.node.create_publisher(String, '/body_commands', qos_profile)
        self.head_command_publisher = self.node.create_publisher(String, '/head_commands', qos_profile)
        self.safety_publisher = self.node.create_publisher(Bool, '/safety_override', qos_profile)

        # Subscribers for robot feedback
        self.joint_state_subscriber = self.node.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, qos_profile
        )
        self.robot_status_subscriber = self.node.create_subscription(
            String, '/robot_status', self._robot_status_callback, qos_profile
        )

        # Store current robot state
        self.current_joint_states = JointState()
        self.current_robot_status = "idle"
        self.is_robot_connected = False

        self.get_logger().info('Robot Control Interface initialized')

    def _joint_state_callback(self, msg: JointState):
        """
        Callback for joint state updates.

        Args:
            msg: JointState message with current joint positions
        """
        self.current_joint_states = msg
        self.is_robot_connected = True

    def _robot_status_callback(self, msg: String):
        """
        Callback for robot status updates.

        Args:
            msg: String message with current robot status
        """
        self.current_robot_status = msg.data

    def move_to_pose(self, pose: Pose, speed: float = 0.5) -> bool:
        """
        Move the robot to a specific pose.

        Args:
            pose: Target pose for the robot
            speed: Movement speed (0.0 to 1.0)

        Returns:
            True if command was sent successfully, False otherwise
        """
        try:
            # This is a simplified implementation
            # In a real system, this would involve more complex navigation

            # Create velocity command to move toward the target
            cmd_vel = Twist()
            cmd_vel.linear.x = min(speed, 0.5)  # Limit linear speed
            cmd_vel.angular.z = 0.2  # Small turn for demonstration

            # Publish the command
            self.cmd_vel_publisher.publish(cmd_vel)

            self.get_logger().info(f"Sent move to pose command: position=({pose.position.x}, {pose.position.y})")

            return True

        except Exception as e:
            self.get_logger().error(f"Error moving to pose: {e}")
            return False

    def execute_joint_trajectory(self, joint_positions: Dict[str, float], duration: float = 2.0) -> bool:
        """
        Execute a joint trajectory to move to specific joint positions.

        Args:
            joint_positions: Dictionary mapping joint names to target positions
            duration: Time to complete the movement

        Returns:
            True if command was sent successfully, False otherwise
        """
        try:
            # Create joint state message
            joint_state = JointState()
            joint_state.header.stamp = self.node.get_clock().now().to_msg()
            joint_state.name = list(joint_positions.keys())
            joint_state.position = list(joint_positions.values())

            # Publish the joint commands
            self.joint_command_publisher.publish(joint_state)

            self.get_logger().info(f"Sent joint trajectory for {len(joint_positions)} joints")

            return True

        except Exception as e:
            self.get_logger().error(f"Error executing joint trajectory: {e}")
            return False

    def move_body_part(self, body_part: str, command: str, params: Optional[Dict[str, Any]] = None) -> bool:
        """
        Move a specific body part of the humanoid robot.

        Args:
            body_part: Which body part to move (arm, leg, torso, etc.)
            command: Command to execute (move, rotate, etc.)
            params: Additional parameters for the command

        Returns:
            True if command was sent successfully, False otherwise
        """
        try:
            # Create command string
            cmd_str = f"{body_part}:{command}"
            if params:
                for key, value in params.items():
                    cmd_str += f":{key}={value}"

            # Create and publish the command
            cmd_msg = String()
            cmd_msg.data = cmd_str
            self.body_command_publisher.publish(cmd_msg)

            self.get_logger().info(f"Sent body command: {cmd_str}")

            return True

        except Exception as e:
            self.get_logger().error(f"Error moving body part: {e}")
            return False

    def move_head(self, pan: float = 0.0, tilt: float = 0.0) -> bool:
        """
        Move the robot's head.

        Args:
            pan: Horizontal rotation in radians
            tilt: Vertical rotation in radians

        Returns:
            True if command was sent successfully, False otherwise
        """
        try:
            # Create head command string
            cmd_msg = String()
            cmd_msg.data = f"pan={pan}:tilt={tilt}"
            self.head_command_publisher.publish(cmd_msg)

            self.get_logger().info(f"Sent head movement command: pan={pan}, tilt={tilt}")

            return True

        except Exception as e:
            self.get_logger().error(f"Error moving head: {e}")
            return False

    def stop_robot(self) -> bool:
        """
        Stop all robot movement immediately.

        Returns:
            True if command was sent successfully, False otherwise
        """
        try:
            # Send zero velocity to stop movement
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)

            # Send stop command to body
            stop_msg = String()
            stop_msg.data = "stop"
            self.body_command_publisher.publish(stop_msg)

            self.get_logger().info("Sent stop command to robot")

            return True

        except Exception as e:
            self.get_logger().error(f"Error stopping robot: {e}")
            return False

    def is_robot_ready(self) -> bool:
        """
        Check if the robot is ready to receive commands.

        Returns:
            True if robot is ready, False otherwise
        """
        # Check if we have received joint states (indicating connection)
        # and if the robot status is not in an error state
        return (self.is_robot_connected and
                self.current_robot_status not in ["error", "fault", "emergency_stop"])

    def get_current_joint_positions(self) -> Dict[str, float]:
        """
        Get the current joint positions of the robot.

        Returns:
            Dictionary mapping joint names to current positions
        """
        return dict(zip(self.current_joint_states.name, self.current_joint_states.position))

    def validate_action_sequence(self, action_sequence: List[Dict[str, Any]]) -> bool:
        """
        Validate an action sequence for safety and feasibility.

        Args:
            action_sequence: List of actions to validate

        Returns:
            True if sequence is valid, False otherwise
        """
        try:
            # Check if robot is ready
            if not self.is_robot_ready():
                self.get_logger().error("Robot is not ready to execute actions")
                return False

            # Validate each action in the sequence
            for action in action_sequence:
                action_type = action.get('action_type', '')
                parameters = action.get('parameters', {})

                # Basic validation for different action types
                if action_type == "navigate_to_pose":
                    # Check if target pose is valid
                    pose = parameters.get('target_pose', {})
                    if not self._is_valid_pose(pose):
                        self.get_logger().error(f"Invalid pose in action: {pose}")
                        return False

                elif action_type in ["pick_object", "place_object"]:
                    # Check if object parameters are valid
                    obj = parameters.get('target_object', '')
                    if not obj or obj == 'unknown':
                        self.get_logger().warning(f"Unknown object in action: {action_type}")

                elif action_type == "move_base":
                    # Check if movement parameters are valid
                    direction = parameters.get('direction', '')
                    if direction not in ["forward", "backward", "left", "right"]:
                        self.get_logger().error(f"Invalid direction: {direction}")
                        return False

            return True

        except Exception as e:
            self.get_logger().error(f"Error validating action sequence: {e}")
            return False

    def _is_valid_pose(self, pose: Dict[str, Any]) -> bool:
        """
        Check if a pose is valid for navigation.

        Args:
            pose: Pose dictionary with x, y coordinates

        Returns:
            True if pose is valid, False otherwise
        """
        x = pose.get('x', 0.0)
        y = pose.get('y', 0.0)

        # Basic validation - in a real system, this would check against a map
        # For now, just ensure the values are reasonable
        return abs(x) < 100 and abs(y) < 100  # Reasonable limits

    def enable_safety_system(self, enable: bool = True) -> bool:
        """
        Enable or disable the safety system.

        Args:
            enable: True to enable safety, False to disable (use with caution!)

        Returns:
            True if command was sent successfully, False otherwise
        """
        try:
            # Create safety override message
            safety_msg = Bool()
            safety_msg.data = not enable  # Inverted logic: True means override/disable safety
            self.safety_publisher.publish(safety_msg)

            status = "enabled" if enable else "disabled"
            self.get_logger().info(f"Safety system {status}")

            return True

        except Exception as e:
            self.get_logger().error(f"Error controlling safety system: {e}")
            return False


# Example usage class
class ExampleRobotControlInterface:
    """
    Example of how to use the RobotControlInterface in a ROS 2 node.
    This is for demonstration purposes.
    """
    def __init__(self):
        # This would normally be initialized within a ROS 2 node
        pass

    def example_usage(self):
        """Example of how to use the RobotControlInterface."""
        print("Robot Control Interface example usage would go here")
        print("This would be initialized within a ROS 2 node context")


if __name__ == "__main__":
    example = ExampleRobotControlInterface()
    example.example_usage()