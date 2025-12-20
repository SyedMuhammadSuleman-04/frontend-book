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
