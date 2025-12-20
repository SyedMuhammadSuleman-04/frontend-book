// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Robotics',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: ROS 2 Core Concepts',
          items: [
            'robotics/chapter-1-ros2-concepts/index',
            'robotics/chapter-1-ros2-concepts/nodes-topics',
            'robotics/chapter-1-ros2-concepts/services-actions',
            'robotics/chapter-1-ros2-concepts/exercises'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Python-ROS Integration',
          items: [
            'robotics/chapter-2-python-ros/index',
            'robotics/chapter-2-python-ros/rclpy-integration',
            'robotics/chapter-2-python-ros/ai-agent-control',
            'robotics/chapter-2-python-ros/exercises'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Humanoid Modeling',
          items: [
            'robotics/chapter-3-humanoid-modeling/index',
            'robotics/chapter-3-humanoid-modeling/urdf-structure',
            'robotics/chapter-3-humanoid-modeling/robot-modeling',
            'robotics/chapter-3-humanoid-modeling/exercises'
          ],
        }
      ],
    },
  ],
};

module.exports = sidebars;