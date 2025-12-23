import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar configuration for the robotics documentation
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Robotics',
      items: [
        'robotics/index',
        {
          type: 'category',
          label: 'Module 1: The Robotic Nervous System (ROS 2)',
          items: [
            {
              type: 'category',
              label: 'Chapter 1: ROS 2 Core Concepts',
              items: [
                'robotics/module1/chapter-1-ros2-concepts/index',
                'robotics/module1/chapter-1-ros2-concepts/nodes-topics',
                'robotics/module1/chapter-1-ros2-concepts/services-actions',
                'robotics/module1/chapter-1-ros2-concepts/exercises'
              ],
            },
            {
              type: 'category',
              label: 'Chapter 2: Python-ROS Integration',
              items: [
                'robotics/module1/chapter-2-python-ros/index',
                'robotics/module1/chapter-2-python-ros/rclpy-integration',
                'robotics/module1/chapter-2-python-ros/ai-agent-control',
                'robotics/module1/chapter-2-python-ros/exercises'
              ],
            },
            {
              type: 'category',
              label: 'Chapter 3: Humanoid Modeling',
              items: [
                'robotics/module1/chapter-3-humanoid-modeling/index',
                'robotics/module1/chapter-3-humanoid-modeling/urdf-structure',
                'robotics/module1/chapter-3-humanoid-modeling/robot-modeling',
                'robotics/module1/chapter-3-humanoid-modeling/exercises'
              ],
            }
          ],
        },
        {
          type: 'category',
          label: 'Module 2: The Digital Twin (Gazebo & Unity)',
          items: [
            {
              type: 'category',
              label: 'Chapter 1: Simulation Fundamentals',
              items: [
                'robotics/module2/chapter-1-simulation-fundamentals/index',
                'robotics/module2/chapter-1-simulation-fundamentals/physics-concepts',
                'robotics/module2/chapter-1-simulation-fundamentals/gravity-collisions',
                'robotics/module2/chapter-1-simulation-fundamentals/exercises'
              ],
            },
            {
              type: 'category',
              label: 'Chapter 2: Virtual Environments',
              items: [
                'robotics/module2/chapter-2-virtual-environments/index',
                'robotics/module2/chapter-2-virtual-environments/unity-setup',
                'robotics/module2/chapter-2-virtual-environments/environment-building',
                'robotics/module2/chapter-2-virtual-environments/exercises'
              ],
            },
            {
              type: 'category',
              label: 'Chapter 3: Sensor Simulation',
              items: [
                'robotics/module2/chapter-3-sensor-simulation/index',
                'robotics/module2/chapter-3-sensor-simulation/lidar-camera-imu',
                'robotics/module2/chapter-3-sensor-simulation/ros2-integration',
                'robotics/module2/chapter-3-sensor-simulation/exercises'
              ],
            }
          ],
        },
        {
          type: 'category',
          label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
          items: [
            {
              type: 'category',
              label: 'Chapter 1: NVIDIA Isaac Sim',
              items: [
                'robotics/module3/chapter-1-isaac-sim/index',
                'robotics/module3/chapter-1-isaac-sim/photorealistic-simulation',
                'robotics/module3/chapter-1-isaac-sim/synthetic-data-generation',
                'robotics/module3/chapter-1-isaac-sim/exercises'
              ],
            },
            {
              type: 'category',
              label: 'Chapter 2: Isaac ROS',
              items: [
                'robotics/module3/chapter-2-isaac-ros/index',
                'robotics/module3/chapter-2-isaac-ros/vs-lam-pipelines',
                'robotics/module3/chapter-2-isaac-ros/perception-pipelines',
                'robotics/module3/chapter-2-isaac-ros/exercises'
              ],
            },
            {
              type: 'category',
              label: 'Chapter 3: Humanoid Navigation',
              items: [
                'robotics/module3/chapter-3-humanoid-navigation/index',
                'robotics/module3/chapter-3-humanoid-navigation/nav2-path-planning',
                'robotics/module3/chapter-3-humanoid-navigation/bipedal-movement',
                'robotics/module3/chapter-3-humanoid-navigation/exercises'
              ],
            }
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA)',
          items: [
            {
              type: 'category',
              label: 'Chapter 1: Voice-to-Action Pipeline',
              items: [
                'robotics/module4/chapter-1-voice-to-action'
              ],
            },
            {
              type: 'category',
              label: 'Chapter 2: Cognitive Planning for VLA',
              items: [
                'robotics/module4/chapter-2-cognitive-planning'
              ],
            },
            {
              type: 'category',
              label: 'Chapter 3: Autonomous Task Execution Pipeline',
              items: [
                'robotics/module4/chapter-3-autonomous-pipeline'
              ],
            }
          ],
        },
        'robotics/prerequisites',
        'robotics/troubleshooting',
        'robotics/references',
        'robotics/comprehensive-example',
        'robotics/cross-chapter-exercises',
        'robotics/final-project'
      ],
    },
  ],
};

export default sidebars;
