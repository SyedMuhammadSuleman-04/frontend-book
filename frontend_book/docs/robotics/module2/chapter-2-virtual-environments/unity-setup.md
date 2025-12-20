# Unity Setup and ROS 2 Integration

This document provides comprehensive instructions for setting up Unity for robotics simulation and integrating it with ROS 2 systems.

## Unity Installation for Robotics

### System Requirements

**Minimum Requirements**:
- Operating System: Windows 10 (64-bit) 1909+ / Windows 11 (64-bit), macOS 10.15+ (Intel), Ubuntu 20.04 LTS (64-bit)
- CPU: SSE2 instruction set support
- RAM: 8 GB
- GPU: DX10 (shader model 4.0) capabilities
- Graphics API: Direct3D 11 or OpenGL 4.5

**Recommended Requirements**:
- RAM: 16 GB or more
- GPU: Dedicated graphics card with 6GB+ VRAM (NVIDIA GTX 1060 / AMD Radeon RX 580 or equivalent)
- CPU: 8-core processor or better
- Storage: SSD with 30GB+ free space

### Installing Unity Hub and Editor

1. **Download Unity Hub**:
   - Visit https://unity.com/download
   - Download Unity Hub (free)
   - Install Unity Hub following the installation wizard

2. **Install Unity Editor**:
   - Open Unity Hub
   - Go to the "Installs" tab
   - Click "Add" to install a new Unity version
   - Select **Unity 2022.3 LTS** (Long Term Support)
   - Select the following modules during installation:
     - Android Build Support (if needed)
     - Linux Build Support (if needed)
     - Windows Build Support (if needed)
     - Visual Studio Tools for Unity (recommended)

3. **Verify Installation**:
   - Launch Unity Editor from Unity Hub
   - Create a new 3D project to confirm everything works
   - Check Help → About Unity to confirm version

### Unity Robotics Setup

1. **Install Unity Robotics Hub**:
   - Open Unity Hub and create a new 3D project
   - In the Unity Editor, go to Window → Package Manager
   - In Package Manager, select "Unity Registry" from the dropdown
   - Search for "ROS TCP Connector"
   - Install the "ROS TCP Connector" package
   - Also install "Unity Robotics Package" if available

2. **Verify Robotics Packages**:
   - Check that the packages are properly installed in Package Manager
   - Look for packages with "Unity Robotics" in the name
   - Verify you can access robotics-related components in the GameObject menu

## ROS 2 Integration Setup

### Installing ROS 2 Humble Hawksbill

**On Ubuntu 22.04**:
```bash
# Setup locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/rosSigning.key | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator

# Source ROS 2
source /opt/ros/humble/setup.bash
```

**On Windows**:
Follow the official ROS 2 Windows installation guide: https://docs.ros.org/en/humble/Installation/Windows-Install.html

### Setting Up ROS TCP Connector

The ROS TCP Connector enables communication between Unity and ROS 2:

1. **Import the Package**:
   - In Unity, ensure ROS TCP Connector is installed via Package Manager
   - The package provides scripts for ROS communication

2. **Configure Network Settings**:
   - By default, ROS TCP Connector uses localhost (127.0.0.1) and port 10000
   - Ensure firewall allows communication on the specified port
   - For network communication, configure appropriate IP addresses

3. **Test Connection**:
   - Create a simple Unity scene with ROS TCP Connector component
   - Launch ROS 2 node on the same machine
   - Verify connection can be established

### Unity Robotics Package Configuration

1. **Import Sample Scenes**:
   - Go to Unity menu: Robotics → Import Sample
   - Import the "ROS-TCP-Connector" sample scene
   - Examine the sample for integration patterns

2. **Configure ROS Settings**:
   - In Unity, go to Robotics → ROS Settings
   - Configure the ROS master URI
   - Set appropriate ROS domain ID if needed

## Development Environment Configuration

### Visual Studio Integration

1. **Install Visual Studio Tools for Unity**:
   - This should have been installed with Unity
   - If not, install from Unity Hub installer
   - Provides Unity-specific debugging and development tools

2. **Configure External Script Editor**:
   - In Unity: Edit → Preferences → External Tools
   - Set External Script Editor to Visual Studio or your preferred IDE
   - Enable "Syntax Highlighting" and other preferences

### ROS 2 Workspace Setup

1. **Create a ROS 2 Workspace**:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

2. **Environment Setup**:
   - Add ROS 2 sourcing to your shell profile:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   ```

## Testing the Integration

### Basic Connection Test

1. **Create a Simple Unity Scene**:
   - Create a new 3D scene in Unity
   - Add a cube object to the scene
   - Attach a simple script that sends position data to ROS

2. **Example Unity Script**:
```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class PositionPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/unity/position";

    // Start is called before the first frame update
    void Start()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.instance;

        // Start the repeating function to publish position
        InvokeRepeating("PublishPosition", 0.0f, 0.5f); // Publish every 0.5 seconds
    }

    void PublishPosition()
    {
        // Create the message
        PointMsg position = new PointMsg();
        position.x = transform.position.x;
        position.y = transform.position.y;
        position.z = transform.position.z;

        // Publish the message
        ros.Publish(topicName, position);
    }
}
```

3. **Test in ROS 2**:
```bash
# In a terminal, after sourcing ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Listen to the topic
ros2 topic echo /unity/position geometry_msgs/msg/Point
```

### Advanced Integration Test

1. **Create a Robot Control Scene**:
   - Import or create a simple robot model
   - Add ROS TCP Connector components
   - Implement joint control via ROS topics

2. **Example Robot Control Script**:
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    public string commandTopic = "/robot/command";
    public string feedbackTopic = "/robot/feedback";

    void Start()
    {
        ros = ROSConnection.instance;

        // Subscribe to feedback topic
        ros.Subscribe<UInt8Msg>(feedbackTopic, OnRobotFeedback);
    }

    void OnRobotFeedback(UInt8Msg feedback)
    {
        // Process robot feedback
        Debug.Log("Robot feedback received: " + feedback.data);
    }

    void SendCommand(byte command)
    {
        ros.Publish(commandTopic, new UInt8Msg(command));
    }
}
```

## Troubleshooting Common Issues

### Network Connection Issues

**Symptoms**: Unity cannot connect to ROS 2, or vice versa
**Solutions**:
- Verify both Unity and ROS 2 are on the same network
- Check firewall settings for the communication port
- Ensure IP addresses are correctly configured
- Try using localhost (127.0.0.1) for local testing

### Performance Issues

**Symptoms**: Low frame rate, lag, or delays in communication
**Solutions**:
- Reduce scene complexity during testing
- Optimize Unity graphics settings for simulation
- Use appropriate update rates for ROS communication
- Consider using lower-resolution assets for simulation

### Build and Deployment Issues

**Symptoms**: Unity application fails to run or ROS connection fails in build
**Solutions**:
- Ensure ROS TCP Connector is properly configured for the build platform
- Check that required libraries are included in the build
- Verify network configuration works in the built application

## Best Practices

### Project Organization
- Create separate scenes for different testing scenarios
- Use Unity's scene management for modular development
- Keep ROS-related scripts in a dedicated folder
- Version control for both Unity and ROS 2 code

### Communication Optimization
- Use appropriate message frequencies
- Implement message filtering where possible
- Consider using latching for static data
- Monitor bandwidth usage for network deployments

### Testing Strategy
- Test locally before moving to network deployment
- Verify message serialization/deserialization
- Monitor connection stability over time
- Test error handling and reconnection logic

## Security Considerations

When integrating Unity with ROS 2:
- Use secure network protocols where possible
- Implement authentication for critical systems
- Consider network segmentation for sensitive applications
- Regularly update both Unity and ROS 2 components

## Next Steps

After completing this setup:
1. Verify all components work together in a simple test scenario
2. Explore Unity Robotics Hub examples
3. Begin creating your first virtual environment
4. Plan the integration with your Gazebo physics simulation

This setup provides the foundation for creating complex virtual environments that can communicate with ROS 2 systems, enabling comprehensive robotics simulation and testing.