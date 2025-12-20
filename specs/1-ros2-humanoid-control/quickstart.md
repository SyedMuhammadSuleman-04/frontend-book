# Quickstart: Docusaurus Implementation for ROS 2 Educational Module

## Prerequisites

1. **System Requirements**:
   - Node.js v18.0 or higher
   - npm v8.0 or higher (or yarn v1.22+)
   - Git for version control
   - Text editor or IDE for Markdown editing

2. **Software Installation**:
   ```bash
   # Install Node.js and npm (if not already installed)
   # For Ubuntu/Debian:
   curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
   sudo apt-get install -y nodejs

   # For macOS with Homebrew:
   # brew install node

   # Verify installation
   node --version
   npm --version
   ```

## Setting Up the Docusaurus Project

1. **Create a new Docusaurus project**:
   ```bash
   # Create a new Docusaurus project
   npx create-docusaurus@latest my-robotics-book classic

   # Navigate to the project directory
   cd my-robotics-book
   ```

2. **Install additional dependencies** (if needed):
   ```bash
   # If you need additional plugins
   npm install
   ```

3. **Start the development server**:
   ```bash
   npm start
   # This command starts a local development server and opens up a browser window
   # Most changes are reflected live without having to restart the server
   ```

## Creating Module 1 Content Structure

1. **Create the robotics documentation directory**:
   ```bash
   mkdir -p docs/robotics
   mkdir -p docs/robotics/chapter-1-ros2-concepts
   mkdir -p docs/robotics/chapter-2-python-ros
   mkdir -p docs/robotics/chapter-3-humanoid-modeling
   ```

2. **Create the main robotics index page**:
   ```bash
   # Create docs/robotics/index.md
   touch docs/robotics/index.md
   ```

3. **Create the three chapter structure**:
   ```bash
   # Chapter 1 - ROS 2 Core Concepts
   touch docs/robotics/chapter-1-ros2-concepts/index.md
   touch docs/robotics/chapter-1-ros2-concepts/nodes-topics.md
   touch docs/robotics/chapter-1-ros2-concepts/services-actions.md
   touch docs/robotics/chapter-1-ros2-concepts/exercises.md

   # Chapter 2 - Python-ROS Integration
   touch docs/robotics/chapter-2-python-ros/index.md
   touch docs/robotics/chapter-2-python-ros/rclpy-integration.md
   touch docs/robotics/chapter-2-python-ros/ai-agent-control.md
   touch docs/robotics/chapter-2-python-ros/exercises.md

   # Chapter 3 - Humanoid Modeling
   touch docs/robotics/chapter-3-humanoid-modeling/index.md
   touch docs/robotics/chapter-3-humanoid-modeling/urdf-structure.md
   touch docs/robotics/chapter-3-humanoid-modeling/robot-modeling.md
   touch docs/robotics/chapter-3-humanoid-modeling/exercises.md
   ```

## Registering Pages in Sidebar

1. **Update the sidebar configuration** in `sidebars.js`:
   ```javascript
   // sidebars.js
   module.exports = {
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
   ```

## Adding Content to Pages

1. **Example content structure for a page** (`docs/robotics/chapter-1-ros2-concepts/index.md`):
   ```markdown
   ---
   sidebar_label: 'Introduction to ROS 2 Concepts'
   title: 'Introduction to ROS 2 Concepts'
   ---

   # Chapter 1: ROS 2 Core Concepts

   This chapter introduces the fundamental concepts of ROS 2, including nodes, topics, services, and actions.

   ## Learning Objectives

   By the end of this chapter, you will be able to:
   - Explain the purpose of each ROS 2 communication pattern
   - Create and run basic ROS 2 examples
   - Understand the differences between topics, services, and actions

   ## Prerequisites

   - Basic Python knowledge
   - Understanding of distributed systems concepts (helpful but not required)
   ```

## Building and Serving Documentation

1. **Build the static site**:
   ```bash
   npm run build
   # This command generates static content into the build directory
   # It creates a production build of your site
   ```

2. **Serve the built site locally**:
   ```bash
   npm run serve
   # This command serves the production build of your site locally
   # Useful for testing before deployment
   ```

## Troubleshooting

- **Docusaurus fails to start**: Check Node.js and npm versions, ensure they meet the minimum requirements
- **Pages not showing up**: Verify that the page IDs in `sidebars.js` match the file paths and frontmatter
- **Markdown not rendering**: Check that the file has proper frontmatter and is in the correct directory
- **Sidebar not updating**: Restart the development server after making changes to `sidebars.js`

## Next Steps

1. Populate each chapter with content following the learning objectives
2. Add code examples with proper syntax highlighting
3. Include diagrams and images in the static directory
4. Test the navigation and ensure all links work correctly