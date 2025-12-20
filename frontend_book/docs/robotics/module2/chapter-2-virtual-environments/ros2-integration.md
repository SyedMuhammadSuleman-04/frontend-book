# ROS 2 Integration with Unity Environments

This document provides comprehensive guidance on connecting Unity virtual environments with ROS 2 simulation systems, enabling bidirectional communication between Unity visualizations and ROS 2 robotics frameworks.

## Understanding ROS 2 Integration Architecture

### Integration Overview

The Unity-ROS 2 integration enables:
- **Visualization**: Unity provides high-quality visual rendering of simulation environments
- **Sensor Simulation**: Unity can generate realistic sensor data for ROS 2 nodes
- **Robot Control**: ROS 2 nodes can control robots in Unity environments
- **Data Exchange**: Bidirectional communication between Unity and ROS 2 systems

### Communication Architecture

**ROS TCP Connector**:
- Primary communication bridge between Unity and ROS 2
- Uses TCP/IP protocol for message exchange
- Supports both standard and custom ROS message types
- Handles serialization and deserialization of messages

**Message Flow**:
- ROS 2 → Unity: Robot commands, environment parameters, control signals
- Unity → ROS 2: Sensor data, robot states, environment feedback

## Setting Up ROS TCP Connector

### Installation and Configuration

1. **Install ROS TCP Connector Package**:
   - Open Unity Package Manager (Window → Package Manager)
   - Select "Unity Registry"
   - Search for "ROS TCP Connector"
   - Install the package

2. **Verify Installation**:
   - Check that `Unity.Robotics.ROSTCPConnector` namespace is available
   - Verify ROS TCP Connector component is in GameObject menu
   - Test basic connection functionality

### Basic ROS TCP Connector Setup

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class ROS2IntegrationSetup : MonoBehaviour
{
    [Header("ROS Connection Settings")]
    public string rosIPAddress = "127.0.0.1";  // Localhost
    public int rosPort = 10000;                 // Default port
    public bool autoConnect = true;

    private ROSConnection ros;

    void Start()
    {
        SetupROSConnection();
    }

    void SetupROSConnection()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.instance;

        // Configure connection settings
        if (autoConnect)
        {
            ros.Initialize(rosIPAddress, rosPort);
        }

        Debug.Log($"ROS Connection initialized: {rosIPAddress}:{rosPort}");
    }

    public void ConnectToROS()
    {
        if (ros == null)
        {
            ros = ROSConnection.instance;
        }
        ros.Initialize(rosIPAddress, rosPort);
        Debug.Log("Connected to ROS");
    }

    public void DisconnectFromROS()
    {
        if (ros != null)
        {
            ros.Disconnect();
            Debug.Log("Disconnected from ROS");
        }
    }
}
```

## Publishing Unity Data to ROS 2

### Robot State Publishing

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class RobotStatePublisher : MonoBehaviour
{
    [Header("ROS Configuration")]
    public string robotStateTopic = "/robot/state";
    public string tfTopic = "/tf";
    public float publishRate = 60f; // Hz

    [Header("Robot Configuration")]
    public string robotName = "unity_robot";
    public string baseFrame = "base_link";
    public string worldFrame = "world";

    private ROSConnection ros;
    private float publishTimer;
    private float publishInterval;

    void Start()
    {
        ros = ROSConnection.instance;
        publishInterval = 1.0f / publishRate;
        publishTimer = 0f;
    }

    void Update()
    {
        publishTimer += Time.deltaTime;

        if (publishTimer >= publishInterval)
        {
            PublishRobotState();
            publishTimer = 0f;
        }
    }

    void PublishRobotState()
    {
        // Create and populate robot state message
        OdometryMsg odometryMsg = new OdometryMsg();

        // Header
        odometryMsg.header = new HeaderMsg();
        odometryMsg.header.stamp = new TimeMsg();
        odometryMsg.header.stamp.sec = (int)Time.time;
        odometryMsg.header.stamp.nanosec = (uint)((Time.time % 1) * 1e9);
        odometryMsg.header.frame_id = worldFrame;

        // Child frame
        odometryMsg.child_frame_id = baseFrame;

        // Pose (position and orientation)
        odometryMsg.pose = new PoseWithCovarianceMsg();
        odometryMsg.pose.pose = new PoseMsg();

        // Position
        odometryMsg.pose.pose.position = new Vector3Msg(
            transform.position.x,
            transform.position.y,
            transform.position.z
        );

        // Orientation (convert Unity quaternion to ROS quaternion)
        odometryMsg.pose.pose.orientation = new QuaternionMsg(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        );

        // Twist (velocity)
        odometryMsg.twist = new TwistWithCovarianceMsg();
        odometryMsg.twist.twist = new TwistMsg();

        // For now, we'll calculate approximate velocity
        // In practice, you'd track velocity from your robot controller
        odometryMsg.twist.twist.linear = new Vector3Msg(0, 0, 0);
        odometryMsg.twist.twist.angular = new Vector3Msg(0, 0, 0);

        // Publish the message
        ros.Publish(robotStateTopic, odometryMsg);

        // Also publish TF transform
        PublishTF();
    }

    void PublishTF()
    {
        // Create TF message
        var tfMsg = new RosMessageTypes.Tf2.TFMessage();

        // Create transform
        var transformStamped = new RosMessageTypes.Geometry.TransformStampedMsg();
        transformStamped.header = new HeaderMsg();
        transformStamped.header.stamp = new TimeMsg();
        transformStamped.header.stamp.sec = (int)Time.time;
        transformStamped.header.stamp.nanosec = (uint)((Time.time % 1) * 1e9);
        transformStamped.header.frame_id = worldFrame;
        transformStamped.child_frame_id = baseFrame;

        // Translation
        transformStamped.transform.translation = new Vector3Msg(
            transform.position.x,
            transform.position.y,
            transform.position.z
        );

        // Rotation
        transformStamped.transform.rotation = new QuaternionMsg(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        );

        tfMsg.transforms = new RosMessageTypes.Geometry.TransformStampedMsg[] { transformStamped };

        // Publish TF
        ros.Publish(tfTopic, tfMsg);
    }
}
```

### Sensor Data Publishing

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using System.Collections.Generic;

public class SensorDataPublisher : MonoBehaviour
{
    [Header("Camera Configuration")]
    public Camera sensorCamera;
    public string imageTopic = "/robot/sensors/camera/image_raw";
    public string cameraInfoTopic = "/robot/sensors/camera/camera_info";
    public int imageWidth = 640;
    public int imageHeight = 480;
    public int publishRate = 30; // Hz

    [Header("LiDAR Configuration")]
    public string lidarTopic = "/robot/sensors/lidar";
    public float lidarRange = 10f;
    public int lidarResolution = 360; // Points per full rotation
    public float lidarAngleMin = -Mathf.PI;
    public float lidarAngleMax = Mathf.PI;

    [Header("IMU Configuration")]
    public string imuTopic = "/robot/sensors/imu";
    public int imuPublishRate = 100; // Hz

    private ROSConnection ros;
    private RenderTexture renderTexture;
    private Texture2D tempTexture;
    private float imagePublishInterval;
    private float imagePublishTimer;
    private float imuPublishInterval;
    private float imuPublishTimer;

    void Start()
    {
        ros = ROSConnection.instance;
        SetupCamera();
        SetupTimers();
    }

    void SetupCamera()
    {
        if (sensorCamera == null)
        {
            sensorCamera = GetComponent<Camera>();
        }

        if (sensorCamera != null)
        {
            // Create render texture for camera capture
            renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
            sensorCamera.targetTexture = renderTexture;
            tempTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        }
    }

    void SetupTimers()
    {
        imagePublishInterval = 1.0f / publishRate;
        imagePublishTimer = 0f;

        imuPublishInterval = 1.0f / imuPublishRate;
        imuPublishTimer = 0f;
    }

    void Update()
    {
        imagePublishTimer += Time.deltaTime;
        imuPublishTimer += Time.deltaTime;

        if (imagePublishTimer >= imagePublishInterval && sensorCamera != null)
        {
            PublishCameraData();
            imagePublishTimer = 0f;
        }

        if (imuPublishTimer >= imuPublishInterval)
        {
            PublishIMUData();
            imuPublishTimer = 0f;
        }

        // LiDAR can be published more frequently or on demand
        // PublishLiDARData(); // Call this when needed
    }

    void PublishCameraData()
    {
        // Capture camera image
        RenderTexture.active = renderTexture;
        tempTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        tempTexture.Apply();

        // Convert to byte array
        byte[] imageBytes = tempTexture.EncodeToJPG();

        // Create sensor message
        ImageMsg imageMsg = new ImageMsg();
        imageMsg.header = new HeaderMsg();
        imageMsg.header.stamp = new TimeMsg();
        imageMsg.header.stamp.sec = (int)Time.time;
        imageMsg.header.stamp.nanosec = (uint)((Time.time % 1) * 1e9);
        imageMsg.header.frame_id = "camera_frame";

        imageMsg.height = (uint)imageHeight;
        imageMsg.width = (uint)imageWidth;
        imageMsg.encoding = "rgb8";
        imageMsg.is_bigendian = 0;
        imageMsg.step = (uint)(imageWidth * 3); // 3 bytes per pixel for RGB
        imageMsg.data = imageBytes;

        // Publish image
        ros.Publish(imageTopic, imageMsg);

        // Publish camera info
        PublishCameraInfo();
    }

    void PublishCameraInfo()
    {
        CameraInfoMsg infoMsg = new CameraInfoMsg();
        infoMsg.header = new HeaderMsg();
        infoMsg.header.stamp = new TimeMsg();
        infoMsg.header.stamp.sec = (int)Time.time;
        infoMsg.header.stamp.nanosec = (uint)((Time.time % 1) * 1e9);
        infoMsg.header.frame_id = "camera_frame";

        infoMsg.height = (uint)imageHeight;
        infoMsg.width = (uint)imageWidth;

        // Camera matrix (intrinsic parameters)
        // These values depend on your camera configuration
        infoMsg.k = new double[] {
            500, 0, imageWidth / 2.0,   // fx, 0, cx
            0, 500, imageHeight / 2.0,  // 0, fy, cy
            0, 0, 1                     // 0, 0, 1
        };

        // Distortion coefficients (assuming no distortion for simplicity)
        infoMsg.d = new double[] { 0, 0, 0, 0, 0 };

        // Rectification matrix
        infoMsg.r = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

        // Projection matrix
        infoMsg.p = new double[] {
            500, 0, imageWidth / 2.0, 0,  // fx, 0, cx, 0
            0, 500, imageHeight / 2.0, 0, // 0, fy, cy, 0
            0, 0, 1, 0                   // 0, 0, 1, 0
        };

        ros.Publish(cameraInfoTopic, infoMsg);
    }

    public void PublishLiDARData()
    {
        // Perform raycasting to simulate LiDAR
        LaserScanMsg lidarMsg = new LaserScanMsg();
        lidarMsg.header = new HeaderMsg();
        lidarMsg.header.stamp = new TimeMsg();
        lidarMsg.header.stamp.sec = (int)Time.time;
        lidarMsg.header.stamp.nanosec = (uint)((Time.time % 1) * 1e9);
        lidarMsg.header.frame_id = "lidar_frame";

        lidarMsg.angle_min = lidarAngleMin;
        lidarMsg.angle_max = lidarAngleMax;
        lidarMsg.angle_increment = (lidarAngleMax - lidarAngleMin) / lidarResolution;
        lidarMsg.time_increment = 0; // For simulated data
        lidarMsg.scan_time = 1.0f / 10; // 10 Hz for example
        lidarMsg.range_min = 0.1f;
        lidarMsg.range_max = lidarRange;

        // Perform raycasts for each angle
        List<float> ranges = new List<float>();
        List<float> intensities = new List<float>();

        for (int i = 0; i < lidarResolution; i++)
        {
            float angle = lidarAngleMin + i * lidarMsg.angle_increment;

            // Calculate ray direction
            Vector3 rayDirection = new Vector3(
                Mathf.Cos(angle),
                0,
                Mathf.Sin(angle)
            );

            // Transform to world space relative to sensor
            rayDirection = transform.TransformDirection(rayDirection);

            // Perform raycast
            RaycastHit hit;
            if (Physics.Raycast(transform.position, rayDirection, out hit, lidarRange))
            {
                ranges.Add(hit.distance);
                intensities.Add(1.0f); // Simulated intensity
            }
            else
            {
                ranges.Add(lidarRange); // Max range if no hit
                intensities.Add(0.0f);
            }
        }

        lidarMsg.ranges = ranges.ToArray();
        lidarMsg.intensities = intensities.ToArray();

        ros.Publish(lidarTopic, lidarMsg);
    }

    void PublishIMUData()
    {
        ImuMsg imuMsg = new ImuMsg();
        imuMsg.header = new HeaderMsg();
        imuMsg.header.stamp = new TimeMsg();
        imuMsg.header.stamp.sec = (int)Time.time;
        imuMsg.header.stamp.nanosec = (uint)((Time.time % 1) * 1e9);
        imuMsg.header.frame_id = "imu_frame";

        // Orientation (from Unity rotation)
        imuMsg.orientation = new QuaternionMsg(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        );

        // Set orientation covariance (unknown, so large values)
        imuMsg.orientation_covariance = new double[] {
            -1, 0, 0, 0, 0, 0, 0, 0, 0
        };

        // Angular velocity (simulate from rotation changes)
        // In practice, you'd track this from your physics system
        imuMsg.angular_velocity = new Vector3Msg(0, 0, 0);

        // Linear acceleration (simulate gravity and motion)
        imuMsg.linear_acceleration = new Vector3Msg(
            Physics.gravity.x,
            Physics.gravity.y,
            Physics.gravity.z
        );

        ros.Publish(imuTopic, imuMsg);
    }
}
```

## Subscribing to ROS 2 Messages

### Robot Control Subscription

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class RobotControllerSubscriber : MonoBehaviour
{
    [Header("ROS Topics")]
    public string cmdVelTopic = "/robot/cmd_vel";
    public string jointCmdTopic = "/robot/joint_commands";

    [Header("Robot Configuration")]
    public float linearSpeed = 2.0f;
    public float angularSpeed = 1.0f;
    public float maxVelocity = 5.0f;

    private ROSConnection ros;
    private Rigidbody rb;
    private Vector3 targetVelocity;

    void Start()
    {
        ros = ROSConnection.instance;

        // Subscribe to ROS topics
        ros.Subscribe<TwistMsg>(cmdVelTopic, ReceiveVelocityCommand);
        ros.Subscribe<Float64MultiArrayMsg>(jointCmdTopic, ReceiveJointCommands);

        // Get rigidbody if available
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
            rb.useGravity = false;
            rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        }
    }

    void ReceiveVelocityCommand(TwistMsg twist)
    {
        // Convert ROS twist command to Unity movement
        targetVelocity = new Vector3(
            (float)twist.linear.x,
            0, // Keep Y (vertical) as 0 for ground-based robot
            (float)twist.linear.y // Map Y to Z for Unity coordinate system
        ) * linearSpeed;

        // Apply rotation
        float rotation = (float)twist.angular.z * angularSpeed;
        transform.Rotate(0, rotation * Time.deltaTime, 0);
    }

    void ReceiveJointCommands(Float64MultiArrayMsg jointArray)
    {
        // Process joint commands
        // This would typically control robot arm joints or other articulation
        Debug.Log($"Received joint commands: {jointArray.data.Length} joints");

        // Example: Apply first value to a joint (in a real implementation,
        // you'd have a more sophisticated joint control system)
        if (jointArray.data.Length > 0)
        {
            float jointValue = (float)jointArray.data[0];
            // Apply jointValue to appropriate joint
        }
    }

    void Update()
    {
        // Apply movement in Update for smooth motion
        if (rb != null)
        {
            // Limit velocity
            targetVelocity = Vector3.ClampMagnitude(targetVelocity, maxVelocity);

            // Apply movement
            rb.velocity = new Vector3(targetVelocity.x, rb.velocity.y, targetVelocity.z);
        }
        else
        {
            // Fallback for objects without rigidbody
            transform.Translate(targetVelocity * Time.deltaTime, Space.World);
        }
    }

    // Method to reset target velocity when no commands are received
    public void ResetVelocity()
    {
        targetVelocity = Vector3.zero;
    }
}
```

### Environment State Subscription

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using System.Collections.Generic;

public class EnvironmentStateSubscriber : MonoBehaviour
{
    [Header("Environment State Topics")]
    public string objectStateTopic = "/environment/object_states";
    public string globalStateTopic = "/environment/global_state";
    public string taskCommandTopic = "/environment/task_commands";

    [Header("Environment Objects")]
    public List<GameObject> controllableObjects = new List<GameObject>();

    private ROSConnection ros;
    private Dictionary<string, GameObject> objectLookup = new Dictionary<string, GameObject>();

    void Start()
    {
        ros = ROSConnection.instance;

        // Subscribe to environment state topics
        ros.Subscribe<StringMsg>(objectStateTopic, ReceiveObjectState);
        ros.Subscribe<StringMsg>(globalStateTopic, ReceiveGlobalState);
        ros.Subscribe<StringMsg>(taskCommandTopic, ReceiveTaskCommand);

        // Build object lookup dictionary
        BuildObjectLookup();
    }

    void BuildObjectLookup()
    {
        foreach (GameObject obj in controllableObjects)
        {
            if (!string.IsNullOrEmpty(obj.name))
            {
                objectLookup[obj.name] = obj;
            }
        }
    }

    void ReceiveObjectState(StringMsg stateMsg)
    {
        try
        {
            // Parse state message (format: "object_name:property:value")
            string[] parts = stateMsg.data.Split(':');
            if (parts.Length >= 3)
            {
                string objectName = parts[0];
                string property = parts[1];
                string value = parts[2];

                if (objectLookup.ContainsKey(objectName))
                {
                    GameObject targetObject = objectLookup[objectName];
                    ApplyObjectState(targetObject, property, value);
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error parsing object state: {e.Message}");
        }
    }

    void ApplyObjectState(GameObject obj, string property, string value)
    {
        switch (property.ToLower())
        {
            case "position":
                // Parse position string (format: "x,y,z")
                string[] posValues = value.Split(',');
                if (posValues.Length == 3)
                {
                    Vector3 newPos = new Vector3(
                        float.Parse(posValues[0]),
                        float.Parse(posValues[1]),
                        float.Parse(posValues[2])
                    );
                    obj.transform.position = newPos;
                }
                break;

            case "rotation":
                // Parse rotation string (format: "x,y,z,w" for quaternion)
                string[] rotValues = value.Split(',');
                if (rotValues.Length == 4)
                {
                    Quaternion newRot = new Quaternion(
                        float.Parse(rotValues[0]),
                        float.Parse(rotValues[1]),
                        float.Parse(rotValues[2]),
                        float.Parse(rotValues[3])
                    );
                    obj.transform.rotation = newRot;
                }
                break;

            case "color":
                // Parse color string (format: "r,g,b" where values are 0-1)
                string[] colorValues = value.Split(',');
                if (colorValues.Length == 3)
                {
                    Color newColor = new Color(
                        float.Parse(colorValues[0]),
                        float.Parse(colorValues[1]),
                        float.Parse(colorValues[2])
                    );

                    // Apply color to material
                    Renderer renderer = obj.GetComponent<Renderer>();
                    if (renderer != null)
                    {
                        renderer.material.color = newColor;
                    }
                }
                break;

            case "active":
                // Parse boolean state
                bool isActive = value.ToLower() == "true";
                obj.SetActive(isActive);
                break;

            default:
                Debug.LogWarning($"Unknown property: {property}");
                break;
        }
    }

    void ReceiveGlobalState(StringMsg stateMsg)
    {
        // Handle global environment state changes
        Debug.Log($"Global state update: {stateMsg.data}");

        // Example: Change lighting based on time of day
        if (stateMsg.data.Contains("timeofday"))
        {
            string[] parts = stateMsg.data.Split(':');
            if (parts.Length >= 2 && parts[0] == "timeofday")
            {
                ChangeEnvironmentLighting(parts[1]);
            }
        }
    }

    void ChangeEnvironmentLighting(string timeOfDay)
    {
        // Find the main directional light
        Light[] lights = FindObjectsOfType<Light>();
        foreach (Light light in lights)
        {
            if (light.type == LightType.Directional)
            {
                switch (timeOfDay.ToLower())
                {
                    case "day":
                        light.color = Color.white;
                        light.intensity = 1.0f;
                        break;
                    case "night":
                        light.color = new Color(0.4f, 0.4f, 0.6f);
                        light.intensity = 0.3f;
                        break;
                    case "dusk":
                    case "dawn":
                        light.color = new Color(1.0f, 0.7f, 0.4f);
                        light.intensity = 0.7f;
                        break;
                }
                break;
            }
        }
    }

    void ReceiveTaskCommand(StringMsg cmdMsg)
    {
        // Handle task-specific commands
        Debug.Log($"Task command received: {cmdMsg.data}");

        // Example commands: "reset", "start", "pause", "scenario:X"
        switch (cmdMsg.data.ToLower())
        {
            case "reset":
                ResetEnvironment();
                break;
            case "start":
                StartScenario();
                break;
            case "pause":
                PauseScenario();
                break;
            default:
                if (cmdMsg.data.StartsWith("scenario:"))
                {
                    string scenario = cmdMsg.data.Substring(9); // Remove "scenario:" prefix
                    LoadScenario(scenario);
                }
                break;
        }
    }

    void ResetEnvironment()
    {
        Debug.Log("Resetting environment to initial state");
        // Reset all objects to initial positions/states
        foreach (GameObject obj in controllableObjects)
        {
            // Reset position, rotation, and other properties
            // This would require storing initial states
        }
    }

    void StartScenario()
    {
        Debug.Log("Starting scenario");
        // Enable physics, start animations, etc.
    }

    void PauseScenario()
    {
        Debug.Log("Pausing scenario");
        // Pause physics, stop animations, etc.
    }

    void LoadScenario(string scenarioName)
    {
        Debug.Log($"Loading scenario: {scenarioName}");
        // Load specific scenario configuration
    }
}
```

## Advanced Integration Patterns

### Custom Message Types

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

// Define custom message structure
[System.Serializable]
public struct UnityEnvironmentState
{
    public string environment_id;
    public double timestamp;
    public UnityObjectState[] object_states;
    public UnitySensorData[] sensor_data;
}

[System.Serializable]
public struct UnityObjectState
{
    public string object_id;
    public double[] position; // [x, y, z]
    public double[] rotation; // [x, y, z, w] quaternion
    public string state; // "idle", "moving", "active", etc.
}

[System.Serializable]
public struct UnitySensorData
{
    public string sensor_id;
    public string sensor_type; // "camera", "lidar", "imu", etc.
    public float[] sensor_values;
}

public class CustomMessageHandler : MonoBehaviour
{
    public string customTopic = "/unity/environment_state";

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;
        // Subscribe to custom message type
        ros.Subscribe<UnityEnvironmentStateMsg>(customTopic, HandleCustomMessage);
    }

    void HandleCustomMessage(UnityEnvironmentStateMsg msg)
    {
        Debug.Log($"Received custom message: {msg.environment_id}");
        // Process custom environment state
    }
}

// Custom message class that follows ROS message conventions
public class UnityEnvironmentStateMsg : Message
{
    public const string k_RosMessageName = "unity_env_msgs/UnityEnvironmentState";
    public override string RosMessageName => k_RosMessageName;

    public string environment_id;
    public TimeMsg timestamp;
    public UnityObjectStateMsg[] object_states;
    public UnitySensorDataMsg[] sensor_data;

    public UnityEnvironmentStateMsg()
    {
        this.environment_id = "";
        this.timestamp = new TimeMsg();
        this.object_states = new UnityObjectStateMsg[0];
        this.sensor_data = new UnitySensorDataMsg[0];
    }

    public UnityEnvironmentStateMsg(string environment_id, TimeMsg timestamp, UnityObjectStateMsg[] object_states, UnitySensorDataMsg[] sensor_data)
    {
        this.environment_id = environment_id;
        this.timestamp = timestamp;
        this.object_states = object_states;
        this.sensor_data = sensor_data;
    }

    public override void Deserialize(byte[] data)
    {
        int offset = 0;
        environment_id = Serializer.DeserializeString(data, ref offset);
        timestamp = Serializer.Deserialize<TimeMsg>(data, ref offset);
        object_states = Serializer.DeserializeArray<UnityObjectStateMsg>(data, ref offset);
        sensor_data = Serializer.DeserializeArray<UnitySensorDataMsg>(data, ref offset);
    }

    public override byte[] Serialize()
    {
        var serializedData = new List<byte>();
        serializedData.AddRange(Serializer.Serialize(environment_id));
        serializedData.AddRange(Serializer.Serialize(timestamp));
        serializedData.AddRange(Serializer.SerializeArray(object_states));
        serializedData.AddRange(Serializer.SerializeArray(sensor_data));
        return serializedData.ToArray();
    }
}
```

### Performance Optimization

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;

public class OptimizedROSIntegration : MonoBehaviour
{
    [Header("Performance Settings")]
    public float messageBatchInterval = 0.1f; // Batch messages every 100ms
    public int maxMessagesPerBatch = 50;

    private ROSConnection ros;
    private Dictionary<string, List<Message>> messageBatches = new Dictionary<string, List<Message>>();
    private float batchTimer;

    void Start()
    {
        ros = ROSConnection.instance;
        batchTimer = 0f;
    }

    void Update()
    {
        batchTimer += Time.deltaTime;

        if (batchTimer >= messageBatchInterval)
        {
            FlushMessageBatches();
            batchTimer = 0f;
        }
    }

    public void QueueMessageForBatch(string topic, Message msg)
    {
        if (!messageBatches.ContainsKey(topic))
        {
            messageBatches[topic] = new List<Message>();
        }

        messageBatches[topic].Add(msg);

        // If batch is getting large, flush early
        if (messageBatches[topic].Count >= maxMessagesPerBatch)
        {
            FlushTopicBatch(topic);
        }
    }

    void FlushMessageBatches()
    {
        foreach (string topic in new List<string>(messageBatches.Keys))
        {
            FlushTopicBatch(topic);
        }
    }

    void FlushTopicBatch(string topic)
    {
        if (messageBatches.ContainsKey(topic) && messageBatches[topic].Count > 0)
        {
            // For now, send messages individually
            // In a real system, you might implement actual batching
            foreach (Message msg in messageBatches[topic])
            {
                ros.Publish(topic, msg);
            }

            messageBatches[topic].Clear();
        }
    }

    // Connection status monitoring
    public bool IsROSCConnected()
    {
        return ros != null && ros.IsConnected;
    }

    public void ReconnectToROS()
    {
        if (ros != null)
        {
            ros.Disconnect();
        }
        ros = ROSConnection.instance;
        // Reinitialize connection with previous settings
    }
}
```

## Testing and Validation

### Integration Testing Framework

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections;
using System.Collections.Generic;

public class ROSIntegrationTester : MonoBehaviour
{
    [Header("Test Configuration")]
    public string testRobotName = "test_robot";
    public float testDuration = 30f;
    public bool runAutomatedTests = true;

    private ROSConnection ros;
    private bool testsRunning = false;
    private float testTimer = 0f;

    void Start()
    {
        ros = ROSConnection.instance;

        if (runAutomatedTests)
        {
            StartCoroutine(RunIntegrationTests());
        }
    }

    IEnumerator RunIntegrationTests()
    {
        Debug.Log("Starting ROS Integration Tests...");

        // Test 1: Connection
        if (TestConnection())
        {
            Debug.Log("✓ Connection test passed");
        }
        else
        {
            Debug.LogError("✗ Connection test failed");
            yield break;
        }

        yield return new WaitForSeconds(1f);

        // Test 2: Publisher functionality
        if (TestPublisher())
        {
            Debug.Log("✓ Publisher test passed");
        }
        else
        {
            Debug.LogError("✗ Publisher test failed");
        }

        yield return new WaitForSeconds(1f);

        // Test 3: Subscriber functionality
        if (TestSubscriber())
        {
            Debug.Log("✓ Subscriber test passed");
        }
        else
        {
            Debug.LogError("✗ Subscriber test failed");
        }

        yield return new WaitForSeconds(1f);

        // Test 4: Message serialization
        if (TestMessageSerialization())
        {
            Debug.Log("✓ Message serialization test passed");
        }
        else
        {
            Debug.LogError("✗ Message serialization test failed");
        }

        Debug.Log("Integration tests completed!");
    }

    bool TestConnection()
    {
        return ros.IsConnected;
    }

    bool TestPublisher()
    {
        // Publish a simple test message
        var testMsg = new RosMessageTypes.Std.String();
        testMsg.data = $"Test message from Unity at {Time.time}";

        try
        {
            ros.Publish("/test_topic", testMsg);
            return true;
        }
        catch
        {
            return false;
        }
    }

    bool TestSubscriber()
    {
        try
        {
            // Subscribe to a test topic
            ros.Subscribe<RosMessageTypes.Std.String>("/test_response", (msg) =>
            {
                Debug.Log($"Received test response: {msg.data}");
            });

            return true;
        }
        catch
        {
            return false;
        }
    }

    bool TestMessageSerialization()
    {
        try
        {
            // Test creating and serializing a complex message
            var poseMsg = new RosMessageTypes.Geometry.PoseMsg();
            poseMsg.position = new RosMessageTypes.Geometry.Vector3Msg(1.0, 2.0, 3.0);
            poseMsg.orientation = new RosMessageTypes.Geometry.QuaternionMsg(0, 0, 0, 1);

            // The serialization happens internally when publishing
            var poseStampedMsg = new RosMessageTypes.Geometry.PoseStampedMsg();
            poseStampedMsg.pose = poseMsg;

            return true;
        }
        catch
        {
            return false;
        }
    }

    // Manual test methods for debugging
    public void TestManualConnection()
    {
        if (ros != null)
        {
            Debug.Log($"ROS Connection Status: {ros.IsConnected}");
            Debug.Log($"ROS IP: {ros.HostIp}, Port: {ros.Port}");
        }
    }

    public void TestManualPublish()
    {
        var msg = new RosMessageTypes.Std.String();
        msg.data = $"Manual test at {System.DateTime.Now}";
        ros.Publish("/manual_test", msg);
        Debug.Log("Published manual test message");
    }
}
```

## Best Practices for Unity-ROS 2 Integration

### Architecture Best Practices

1. **Separation of Concerns**: Keep ROS communication logic separate from Unity game logic
2. **Error Handling**: Implement robust error handling for network disconnections
3. **Performance Monitoring**: Monitor message rates and connection stability
4. **Security**: Consider network security for production deployments
5. **Scalability**: Design for multiple robots and complex environments

### Performance Considerations

1. **Message Rate**: Balance update frequency with network capacity
2. **Data Compression**: Consider compressing large data like images
3. **Threading**: Be aware of Unity's single-threaded nature
4. **Batching**: Group related messages when possible
5. **Caching**: Cache frequently accessed data

The Unity-ROS 2 integration enables powerful robotics simulation and visualization capabilities, allowing for realistic testing and development of robotic systems in virtual environments.