# ROS 2 Integration for Sensor Simulation

This document provides comprehensive guidance on integrating sensor simulation with ROS 2 systems, covering message formats, communication patterns, and integration best practices for robotics applications.

## Understanding ROS 2 Sensor Integration

### Core Sensor Message Types

**sensor_msgs Package**:
- `sensor_msgs/msg/LaserScan`: LiDAR and range finder data
- `sensor_msgs/msg/Image`: Camera image data
- `sensor_msgs/msg/CameraInfo`: Camera calibration parameters
- `sensor_msgs/msg/Imu`: Inertial measurement unit data
- `sensor_msgs/msg/MagneticField`: Magnetometer data
- `sensor_msgs/msg/Temperature`: Temperature sensor data
- `sensor_msgs/msg/JointState`: Joint position/velocity/effort data

### Message Structure and Standards

**LaserScan Message**:
```python
# Header information
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id

# Measurement parameters
float32 angle_min         # Start angle of the scan [rad]
float32 angle_max         # End angle of the scan [rad]
float32 angle_increment   # Angular distance between measurements [rad]
float32 time_increment    # Time between measurements [seconds]
float32 scan_time         # Time between scans [seconds]
float32 range_min         # Minimum range value [m]
float32 range_max         # Maximum range value [m]
float32[] ranges          # Range data [m]
float32[] intensities     # Intensity data [device dependent]
```

**Image Message**:
```python
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id

uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
```

**IMU Message**:
```python
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

## ROS 2 Sensor Publisher Implementation

### LiDAR Data Publisher

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <random>

class LidarPublisher : public rclcpp::Node
{
public:
    LidarPublisher() : Node("lidar_publisher")
    {
        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/robot/sensors/lidar", 10);

        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz update rate
            std::bind(&LidarPublisher::publish_scan, this));

        // Initialize random number generator for noise
        rng_.seed(std::random_device{}());
        noise_dist_ = std::normal_distribution<float>(0.0, 0.01);  // 1cm standard deviation

        RCLCPP_INFO(this->get_logger(), "LiDAR publisher initialized");
    }

private:
    void publish_scan()
    {
        auto message = sensor_msgs::msg::LaserScan();

        // Header
        message.header.stamp = this->now();
        message.header.frame_id = "lidar_frame";

        // LiDAR parameters
        message.angle_min = -M_PI;  // -180 degrees
        message.angle_max = M_PI;   // 180 degrees
        message.angle_increment = 2 * M_PI / 720;  // 0.5 degree resolution
        message.time_increment = 0.0;  // No time increment for static simulation
        message.scan_time = 0.1;  // 10 Hz
        message.range_min = 0.1;  // 10 cm minimum range
        message.range_max = 30.0; // 30 m maximum range

        // Generate sample data
        int num_readings = static_cast<int>((message.angle_max - message.angle_min) / message.angle_increment);
        message.ranges.resize(num_readings);
        message.intensities.resize(num_readings);

        // Simulate LiDAR readings with noise
        for (int i = 0; i < num_readings; ++i) {
            float angle = message.angle_min + i * message.angle_increment;

            // Simulate some environment features (walls, obstacles)
            float simulated_range = simulate_environment(angle);

            // Add noise to the measurement
            float noisy_range = simulated_range + noise_dist_(rng_);

            // Ensure range is within valid bounds
            if (noisy_range < message.range_min) {
                message.ranges[i] = std::numeric_limits<float>::quiet_NaN();
            } else if (noisy_range > message.range_max) {
                message.ranges[i] = message.range_max + 1.0;  // Indicate max range exceeded
            } else {
                message.ranges[i] = noisy_range;
            }

            // Simulate intensity (based on reflectance)
            message.intensities[i] = 100.0 + (rand() % 50);  // Random intensity with some variation
        }

        publisher_->publish(message);
    }

    float simulate_environment(float angle)
    {
        // Simulate a simple environment with walls
        // This is a placeholder - in real simulation, this would come from Gazebo
        float distance = message.range_max;  // Default to max range (no obstacle)

        // Example: simulate a rectangular room
        float x = 5.0;  // Room width
        float y = 5.0;  // Room height

        // Calculate distances to walls
        float dist_to_front = y / 2.0;
        float dist_to_back = y / 2.0;
        float dist_to_right = x / 2.0;
        float dist_to_left = x / 2.0;

        // Simplified calculation - in real implementation, use actual robot position
        if (angle >= -M_PI/4 && angle < M_PI/4) {
            distance = dist_to_front;  // Front wall
        } else if (angle >= M_PI/4 && angle < 3*M_PI/4) {
            distance = dist_to_right;  // Right wall
        } else if (angle >= -3*M_PI/4 && angle < -M_PI/4) {
            distance = dist_to_left;   // Left wall
        } else {
            distance = dist_to_back;   // Back wall
        }

        return distance;
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Random number generation for noise
    std::mt19937 rng_;
    std::normal_distribution<float> noise_dist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

### Camera Data Publisher

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher")
    {
        // Create publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/robot/sensors/camera/image_raw", 10);
        info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/robot/sensors/camera/camera_info", 10);

        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 Hz
            std::bind(&CameraPublisher::publish_image, this));

        // Initialize camera parameters
        initialize_camera_parameters();

        RCLCPP_INFO(this->get_logger(), "Camera publisher initialized");
    }

private:
    void initialize_camera_parameters()
    {
        // Camera resolution
        width_ = 640;
        height_ = 480;

        // Camera intrinsic parameters (for a 60-degree FOV camera)
        fx_ = 525.0;  // Focal length in x
        fy_ = 525.0;  // Focal length in y
        cx_ = width_ / 2.0;   // Principal point x
        cy_ = height_ / 2.0;  // Principal point y

        // Create OpenCV Mat for image generation
        simulated_image_ = cv::Mat(height_, width_, CV_8UC3);
    }

    void publish_image()
    {
        // Generate simulated image
        generate_simulated_image();

        // Convert OpenCV Mat to ROS Image message
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = this->now();
        cv_image.header.frame_id = "camera_frame";
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.image = simulated_image_;

        // Publish image
        image_pub_->publish(*cv_image.toImageMsg());

        // Publish camera info
        publish_camera_info();
    }

    void generate_simulated_image()
    {
        // Create a simple simulated environment
        simulated_image_ = cv::Scalar(100, 100, 100);  // Gray background

        // Add some geometric shapes to simulate objects
        cv::circle(simulated_image_, cv::Point(width_/2, height_/2), 50, cv::Scalar(0, 255, 0), -1);  // Green circle
        cv::rectangle(simulated_image_, cv::Point(100, 100), cv::Point(200, 200), cv::Scalar(255, 0, 0), -1);  // Red square
        cv::line(simulated_image_, cv::Point(0, height_/2), cv::Point(width_, height_/2), cv::Scalar(0, 0, 255), 2);  // Blue line

        // Add some noise to make it more realistic
        cv::Mat noise = cv::Mat(simulated_image_.size(), CV_8UC3);
        cv::randu(noise, cv::Scalar(0, 0, 0), cv::Scalar(10, 10, 10));
        cv::add(simulated_image_, noise, simulated_image_);
    }

    void publish_camera_info()
    {
        auto camera_info_msg = sensor_msgs::msg::CameraInfo();

        // Header
        camera_info_msg.header.stamp = this->now();
        camera_info_msg.header.frame_id = "camera_frame";

        // Image dimensions
        camera_info_msg.height = height_;
        camera_info_msg.width = width_;

        // Distortion parameters (assuming no distortion for simplicity)
        camera_info_msg.distortion_model = "plumb_bob";
        camera_info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};  // No distortion

        // Camera intrinsic matrix (K)
        camera_info_msg.k = {
            fx_, 0.0, cx_,
            0.0, fy_, cy_,
            0.0, 0.0, 1.0
        };

        // Rectification matrix (R) - identity for monocular camera
        camera_info_msg.r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        // Projection matrix (P)
        camera_info_msg.p = {
            fx_, 0.0, cx_, 0.0,
            0.0, fy_, cy_, 0.0,
            0.0, 0.0, 1.0, 0.0
        };

        info_pub_->publish(camera_info_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Camera parameters
    int width_, height_;
    double fx_, fy_, cx_, cy_;

    // Simulated image
    cv::Mat simulated_image_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

### IMU Data Publisher

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <random>

class IMUPublisher : public rclcpp::Node
{
public:
    IMUPublisher() : Node("imu_publisher")
    {
        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/robot/sensors/imu", 10);

        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100 Hz update rate
            std::bind(&IMUPublisher::publish_imu, this));

        // Initialize random number generators for noise
        rng_.seed(std::random_device{}());
        gyro_noise_dist_ = std::normal_distribution<double>(0.0, 0.001);  // 1 mrad/s std
        accel_noise_dist_ = std::normal_distribution<double>(0.0, 0.01);   // 1 cm/s² std
        orientation_noise_dist_ = std::normal_distribution<double>(0.0, 0.001);  // Small orientation noise

        // Initialize state variables
        roll_ = 0.0;
        pitch_ = 0.0;
        yaw_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "IMU publisher initialized");
    }

private:
    void publish_imu()
    {
        auto message = sensor_msgs::msg::Imu();

        // Header
        message.header.stamp = this->now();
        message.header.frame_id = "imu_frame";

        // Simulate orientation (with some drift for realism)
        simulate_orientation();

        // Set orientation (convert Euler angles to quaternion)
        double qx, qy, qz, qw;
        euler_to_quaternion(roll_ + orientation_noise_dist_(rng_),
                           pitch_ + orientation_noise_dist_(rng_),
                           yaw_ + orientation_noise_dist_(rng_),
                           qx, qy, qz, qw);

        message.orientation.x = qx;
        message.orientation.y = qy;
        message.orientation.z = qz;
        message.orientation.w = qw;

        // Orientation covariance (unknown, so set to large values)
        for (int i = 0; i < 9; ++i) {
            message.orientation_covariance[i] = (i == 0 || i == 4 || i == 8) ? 0.01 : 0.0;
        }

        // Simulate angular velocity (gyroscope)
        message.angular_velocity.x = simulate_gyro_x();
        message.angular_velocity.y = simulate_gyro_y();
        message.angular_velocity.z = simulate_gyro_z();

        // Angular velocity covariance
        for (int i = 0; i < 9; ++i) {
            message.angular_velocity_covariance[i] = (i == 0 || i == 4 || i == 8) ? 0.01 : 0.0;
        }

        // Simulate linear acceleration (accelerometer)
        message.linear_acceleration.x = simulate_accel_x();
        message.linear_acceleration.y = simulate_accel_y();
        message.linear_acceleration.z = simulate_accel_z();

        // Linear acceleration covariance
        for (int i = 0; i < 9; ++i) {
            message.linear_acceleration_covariance[i] = (i == 0 || i == 4 || i == 8) ? 0.01 : 0.0;
        }

        publisher_->publish(message);
    }

    void simulate_orientation()
    {
        // Simulate slow orientation changes
        static double time_counter = 0.0;
        time_counter += 0.01;  // 100 Hz * dt

        // Add some slow oscillations to simulate robot movement
        roll_ = 0.01 * sin(time_counter * 0.5);   // Slow roll oscillation
        pitch_ = 0.02 * cos(time_counter * 0.3);  // Slow pitch oscillation
        yaw_ += 0.001;  // Slow yaw rotation
    }

    double simulate_gyro_x()
    {
        // Simulate gyroscope reading with noise
        double ideal_value = 0.01 * cos(time_counter_ * 0.5);  // Simulated slow movement
        return ideal_value + gyro_noise_dist_(rng_);
    }

    double simulate_gyro_y()
    {
        double ideal_value = -0.02 * sin(time_counter_ * 0.3);
        return ideal_value + gyro_noise_dist_(rng_);
    }

    double simulate_gyro_z()
    {
        double ideal_value = 0.001;  // Slow yaw rate
        return ideal_value + gyro_noise_dist_(rng_);
    }

    double simulate_accel_x()
    {
        // Simulate accelerometer reading with gravity component
        double gravity_component = 9.81 * sin(roll_);  // Gravity in x-direction
        double motion_component = 0.1 * cos(time_counter_ * 2.0);  // Simulated motion
        return gravity_component + motion_component + accel_noise_dist_(rng_);
    }

    double simulate_accel_y()
    {
        double gravity_component = 9.81 * sin(pitch_);
        double motion_component = 0.1 * sin(time_counter_ * 2.0);
        return gravity_component + motion_component + accel_noise_dist_(rng_);
    }

    double simulate_accel_z()
    {
        // Gravity should mostly be in z-direction when robot is upright
        double gravity_component = 9.81 * cos(roll_) * cos(pitch_);
        double motion_component = 0.05 * cos(time_counter_ * 1.5);
        return gravity_component + motion_component + accel_noise_dist_(rng_);
    }

    void euler_to_quaternion(double roll, double pitch, double yaw,
                            double& qx, double& qy, double& qz, double& qw)
    {
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);

        qw = cr * cp * cy + sr * sp * sy;
        qx = sr * cp * cy - cr * sp * sy;
        qy = cr * sp * cy + sr * cp * sy;
        qz = cr * cp * sy - sr * sp * cy;
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Random number generation for noise
    std::mt19937 rng_;
    std::normal_distribution<double> gyro_noise_dist_;
    std::normal_distribution<double> accel_noise_dist_;
    std::normal_distribution<double> orientation_noise_dist_;

    // State variables
    double roll_, pitch_, yaw_;
    double time_counter_ = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

## Sensor Data Subscribers

### Sensor Data Processing Node

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class SensorProcessor : public rclcpp::Node
{
public:
    SensorProcessor() : Node("sensor_processor")
    {
        // Create subscriptions
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/robot/sensors/lidar", 10,
            std::bind(&SensorProcessor::lidar_callback, this, std::placeholders::_1));

        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/robot/sensors/camera/image_raw", 10,
            std::bind(&SensorProcessor::camera_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/robot/sensors/imu", 10,
            std::bind(&SensorProcessor::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Sensor processor initialized");
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process LiDAR data
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,  // Log every 1 second
            "LiDAR: %zu points, range [%f, %f]m, angle [%f, %f]rad",
            msg->ranges.size(),
            msg->range_min,
            msg->range_max,
            msg->angle_min,
            msg->angle_max
        );

        // Example: find minimum distance in front of robot
        find_obstacles_in_front(msg);
    }

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS Image to OpenCV Mat
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // Process image (example: detect edges)
            cv::Mat gray, edges;
            cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
            cv::Canny(gray, edges, 50, 150);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,  // Log every 1 second
                "Camera: %dx%d image, %s encoding",
                msg->width,
                msg->height,
                msg->encoding.c_str()
            );

            // Display image (optional - for debugging)
            // cv::imshow("Camera Feed", cv_ptr->image);
            // cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Process IMU data
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,  // Log every 1 second
            "IMU: Angular Vel=(%.3f, %.3f, %.3f), Linear Accel=(%.3f, %.3f, %.3f)",
            msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
            msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z
        );

        // Extract orientation from quaternion
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        RCLCPP_DEBUG_STREAM(
            this->get_logger(),
            "Orientation: Roll=" << roll << ", Pitch=" << pitch << ", Yaw=" << yaw
        );
    }

    void find_obstacles_in_front(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Find minimum distance in front of robot (±30 degrees)
        double min_distance = std::numeric_limits<double>::infinity();
        int count = 0;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;

            // Check if angle is in front of robot (±30 degrees)
            if (angle >= -M_PI/6 && angle <= M_PI/6) {
                if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < min_distance) {
                    min_distance = msg->ranges[i];
                }
                count++;
            }
        }

        if (count > 0) {
            RCLCPP_DEBUG_STREAM(
                this->get_logger(),
                "Front obstacle: min distance = " << min_distance << "m, " << count << " points"
            );
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorProcessor>());
    rclcpp::shutdown();
    return 0;
}
```

## Unity-ROS 2 Sensor Integration

### Unity Sensor Data Publisher

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using System.Collections.Generic;

public class UnitySensorPublisher : MonoBehaviour
{
    [Header("Sensor Topics")]
    public string lidarTopic = "/robot/sensors/lidar";
    public string cameraTopic = "/robot/sensors/camera/image_raw";
    public string cameraInfoTopic = "/robot/sensors/camera/camera_info";
    public string imuTopic = "/robot/sensors/imu";

    [Header("Camera Configuration")]
    public Camera sensorCamera;
    public int imageWidth = 640;
    public int imageHeight = 480;

    [Header("LiDAR Configuration")]
    public float lidarRange = 10f;
    public int lidarResolution = 360;
    public float lidarAngleMin = -Mathf.PI;
    public float lidarAngleMax = Mathf.PI;

    [Header("IMU Configuration")]
    public float imuPublishRate = 100f; // Hz

    private ROSConnection ros;
    private RenderTexture renderTexture;
    private Texture2D tempTexture;
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
        imuPublishInterval = 1.0f / imuPublishRate;
        imuPublishTimer = 0f;
    }

    void Update()
    {
        imuPublishTimer += Time.deltaTime;

        if (imuPublishTimer >= imuPublishInterval)
        {
            PublishIMUData();
            PublishCameraData();
            PublishLiDARData();
            imuPublishTimer = 0f;
        }
    }

    void PublishCameraData()
    {
        if (sensorCamera == null) return;

        // Capture camera image
        RenderTexture.active = renderTexture;
        tempTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        tempTexture.Apply();

        // Convert to byte array
        byte[] imageBytes = tempTexture.EncodeToJPG();

        // Create and publish image message
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

        ros.Publish(cameraTopic, imageMsg);

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
        float focalLength = 500f; // Adjust based on your camera FOV
        infoMsg.k = new double[] {
            focalLength, 0, imageWidth / 2.0,
            0, focalLength, imageHeight / 2.0,
            0, 0, 1
        };

        // Distortion coefficients (assuming no distortion)
        infoMsg.d = new double[] { 0, 0, 0, 0, 0 };

        // Identity rectification matrix
        infoMsg.r = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

        // Projection matrix
        infoMsg.p = new double[] {
            focalLength, 0, imageWidth / 2.0, 0,
            0, focalLength, imageHeight / 2.0, 0,
            0, 0, 1, 0
        };

        ros.Publish(cameraInfoTopic, infoMsg);
    }

    void PublishLiDARData()
    {
        LaserScanMsg lidarMsg = new LaserScanMsg();
        lidarMsg.header = new HeaderMsg();
        lidarMsg.header.stamp = new TimeMsg();
        lidarMsg.header.stamp.sec = (int)Time.time;
        lidarMsg.header.stamp.nanosec = (uint)((Time.time % 1) * 1e9);
        lidarMsg.header.frame_id = "lidar_frame";

        lidarMsg.angle_min = lidarAngleMin;
        lidarMsg.angle_max = lidarAngleMax;
        lidarMsg.angle_increment = (lidarAngleMax - lidarAngleMin) / lidarResolution;
        lidarMsg.time_increment = 0;
        lidarMsg.scan_time = 1.0f / 10; // 10 Hz
        lidarMsg.range_min = 0.1f;
        lidarMsg.range_max = lidarRange;

        // Perform raycasts for each angle
        List<float> ranges = new List<float>();
        List<float> intensities = new List<float>();

        for (int i = 0; i < lidarResolution; i++)
        {
            float angle = lidarAngleMin + i * lidarMsg.angle_increment;

            // Calculate ray direction in world space
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
                // Simulate intensity based on surface properties
                float intensity = 100f + Random.Range(0f, 20f);
                intensities.Add(intensity);
            }
            else
            {
                ranges.Add(float.PositiveInfinity); // No obstacle detected
                intensities.Add(0f);
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

        // Orientation from Unity rotation (convert to ROS coordinate system)
        imuMsg.orientation = new QuaternionMsg(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        );

        // Set orientation covariance (unknown, so large values)
        imuMsg.orientation_covariance = new double[] { -1, 0, 0, 0, 0, 0, 0, 0, 0 };

        // Angular velocity (simulate from rotation changes)
        imuMsg.angular_velocity = new Vector3Msg(0, 0, 0); // Placeholder

        // Linear acceleration (include gravity)
        imuMsg.linear_acceleration = new Vector3Msg(
            Physics.gravity.x,
            Physics.gravity.y,
            Physics.gravity.z
        );

        ros.Publish(imuTopic, imuMsg);
    }
}
```

## Sensor Data Processing Pipelines

### Perception Pipeline Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PerceptionPipeline : public rclcpp::Node
{
public:
    PerceptionPipeline() : Node("perception_pipeline")
    {
        // Create synchronized subscriptions using message_filters
        lidar_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
            this, "/robot/sensors/lidar");
        imu_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(
            this, "/robot/sensors/imu");

        // Synchronize messages based on timestamps
        sync_ = std::make_shared<SynchronizerPolicy>(SyncPolicy(10),
                                                    *lidar_sub_, *imu_sub_);
        sync_->registerCallback(&PerceptionPipeline::sensor_sync_callback, this);

        // Publishers for processed data
        obstacle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/perception/obstacles", 10);
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/perception/pointcloud", 10);

        RCLCPP_INFO(this->get_logger(), "Perception pipeline initialized");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::LaserScan, sensor_msgs::msg::Imu>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    void sensor_sync_callback(
        const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg,
        const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,  // Log every 2 seconds
            "Synchronized sensor data received at %f",
            lidar_msg->header.stamp.sec + lidar_msg->header.stamp.nanosec / 1e9
        );

        // Process LiDAR data to detect obstacles
        process_lidar_data(lidar_msg);

        // Fuse IMU data for orientation correction
        process_imu_data(imu_msg);

        // Combine sensor data for comprehensive perception
        fuse_sensor_data(lidar_msg, imu_msg);
    }

    void process_lidar_data(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Convert LaserScan to PointCloud for easier processing
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (std::isfinite(msg->ranges[i])) {
                float angle = msg->angle_min + i * msg->angle_increment;

                pcl::PointXYZ point;
                point.x = msg->ranges[i] * cos(angle);
                point.y = msg->ranges[i] * sin(angle);
                point.z = 0.0;  // 2D scan

                cloud->points.push_back(point);
            }
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;

        // Convert to ROS message and publish
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header = msg->header;

        pointcloud_pub_->publish(cloud_msg);

        // Detect obstacles and clusters
        detect_obstacles_from_pointcloud(cloud, msg->header);
    }

    void detect_obstacles_from_pointcloud(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const std_msgs::msg::Header& header)
    {
        // Simple clustering for obstacle detection
        // In practice, use more sophisticated clustering algorithms
        visualization_msgs::msg::MarkerArray marker_array;

        // Create a marker for each detected obstacle cluster
        // (This is a simplified example)
        for (size_t i = 0; i < cloud->points.size(); i += 10) {  // Downsample for visualization
            if (i < cloud->points.size()) {
                visualization_msgs::msg::Marker marker;
                marker.header = header;
                marker.ns = "obstacles";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.pose.position.x = cloud->points[i].x;
                marker.pose.position.y = cloud->points[i].y;
                marker.pose.position.z = 0.5;  // Half height
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.3;  // Diameter
                marker.scale.y = 0.3;
                marker.scale.z = 1.0;  // Height

                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 0.8f;

                marker.lifetime = rclcpp::Duration::from_seconds(1.0);

                marker_array.markers.push_back(marker);
            }
        }

        obstacle_pub_->publish(marker_array);
    }

    void process_imu_data(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Process IMU data for orientation and motion estimation
        // This could include filtering, integration, or fusion with other sensors
        RCLCPP_DEBUG(this->get_logger(), "Processing IMU data");
    }

    void fuse_sensor_data(
        const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg,
        const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        // Combine LiDAR and IMU data for improved perception
        // This could include:
        // - IMU-based motion compensation for LiDAR
        // - Orientation correction for sensor data
        // - Temporal fusion of measurements
        RCLCPP_DEBUG(this->get_logger(), "Fusing sensor data");
    }

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> lidar_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_sub_;
    std::shared_ptr<Synchronizer> sync_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};
```

## Quality of Service (QoS) Settings

### Appropriate QoS for Sensor Data

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class SensorPublisherWithQoS : public rclcpp::Node
{
public:
    SensorPublisherWithQoS() : Node("qos_sensor_publisher")
    {
        // LiDAR - requires high frequency, can drop older messages
        rclcpp::QoS lidar_qos_profile(10);  // history depth of 10
        lidar_qos_profile.reliable();       // reliable delivery
        lidar_qos_profile.durability_volatile();  // volatile durability
        lidar_qos_profile.deadline(rclcpp::Duration::from_seconds(0.1));  // 100ms deadline

        lidar_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/robot/sensors/lidar", lidar_qos_profile);

        // Camera - may need more reliable delivery
        rclcpp::QoS camera_qos_profile(5);
        camera_qos_profile.reliable();
        camera_qos_profile.durability_volatile();
        camera_qos_profile.deadline(rclcpp::Duration::from_seconds(0.033));  // 30fps deadline

        camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/robot/sensors/camera/image_raw", camera_qos_profile);

        // IMU - high frequency, less critical for reliability
        rclcpp::QoS imu_qos_profile(20);
        imu_qos_profile.best_effort();  // Best effort delivery
        imu_qos_profile.durability_volatile();
        imu_qos_profile.deadline(rclcpp::Duration::from_seconds(0.01));  // 100Hz deadline

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/robot/sensors/imu", imu_qos_profile);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};
```

## Best Practices for Sensor Integration

### Message Naming Conventions

**Standard Naming**:
- `/robot_name/sensor_type/frame_id/sensor_data`
- Examples:
  - `/robot1/sensors/lidar/scan`
  - `/robot1/sensors/camera/front/image_raw`
  - `/robot1/sensors/imu/body/data`

### Data Validation

```cpp
bool validate_lidar_data(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Check message header
    if (msg->header.frame_id.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("validator"), "LiDAR message has empty frame_id");
        return false;
    }

    // Check parameter consistency
    if (msg->angle_increment <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("validator"), "Invalid angle_increment: %f", msg->angle_increment);
        return false;
    }

    if (msg->range_min >= msg->range_max) {
        RCLCPP_ERROR(rclcpp::get_logger("validator"), "Invalid range parameters: min=%f, max=%f",
                    msg->range_min, msg->range_max);
        return false;
    }

    // Check data size
    int expected_size = std::ceil((msg->angle_max - msg->angle_min) / msg->angle_increment);
    if (msg->ranges.size() != expected_size) {
        RCLCPP_WARN(rclcpp::get_logger("validator"),
                   "Range data size mismatch: expected=%d, actual=%zu",
                   expected_size, msg->ranges.size());
        // This might be acceptable depending on implementation
    }

    return true;
}
```

### Performance Monitoring

```cpp
class SensorPerformanceMonitor
{
public:
    void update_latency_stats(double latency_ms)
    {
        latencies_.push_back(latency_ms);

        if (latencies_.size() > 1000) {
            latencies_.erase(latencies_.begin());  // Keep last 1000 samples
        }
    }

    void update_frequency_stats(double actual_freq)
    {
        frequencies_.push_back(actual_freq);

        if (frequencies_.size() > 1000) {
            frequencies_.erase(frequencies_.begin());
        }
    }

    void print_statistics()
    {
        if (latencies_.empty() || frequencies_.empty()) {
            return;
        }

        double avg_latency = std::accumulate(latencies_.begin(), latencies_.end(), 0.0) / latencies_.size();
        double avg_frequency = std::accumulate(frequencies_.begin(), frequencies_.end(), 0.0) / frequencies_.size();

        RCLCPP_INFO(rclcpp::get_logger("monitor"),
                   "Performance: Avg latency=%.2fms, Avg freq=%.2fHz",
                   avg_latency, avg_frequency);
    }

private:
    std::vector<double> latencies_;
    std::vector<double> frequencies_;
};
```

ROS 2 integration for sensor simulation enables comprehensive robotics applications by providing standardized interfaces for sensor data publication and consumption. Proper implementation of sensor integration patterns ensures reliable and efficient communication between simulation environments and robotics software stacks.