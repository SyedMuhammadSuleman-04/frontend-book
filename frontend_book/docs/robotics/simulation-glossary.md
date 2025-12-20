# Simulation Glossary and Terminology

This glossary provides definitions for key terms used in digital twin simulation with Gazebo, Unity, and ROS 2.

## A

**API (Application Programming Interface)**: A set of rules and protocols for building and interacting with software applications. In robotics, APIs enable communication between different software components.

**Artificial Potential Fields**: A navigation method that treats the robot as a particle moving under the influence of attractive forces toward the goal and repulsive forces away from obstacles.

## B

**Behavior-Based Robotics**: An approach to robotics that structures robot control as a collection of task-oriented behaviors.

**Bounding Box**: A rectangular box that completely encloses a 3D object, used for collision detection and spatial queries.

## C

**Collision Detection**: The computational problem of detecting the intersection of two or more geometric objects in a 3D environment.

**Coordinate Frame**: A system for specifying positions and orientations in space, typically defined by a position vector and rotation matrix.

**Cross-Validation**: A technique for assessing how the results of a statistical analysis will generalize to an independent data set.

## D

**Digital Twin**: A virtual representation of a physical system that enables understanding, prediction, and optimization of the physical system's performance.

**Differential Drive**: A common wheel configuration for mobile robots using two wheels mounted parallel to each other and independently driven.

**Dynamic Simulation**: Simulation that takes into account the forces and torques that cause motion, as opposed to kinematic simulation which only considers motion without forces.

## E

**End-Effector**: The device at the end of a robot arm that interacts with the environment, such as a gripper or tool.

**Environment Mapping**: The process of creating a representation of the robot's environment based on sensor data.

**Euclidean Distance**: The "ordinary" straight-line distance between two points in Euclidean space.

## F

**Forward Kinematics**: The use of kinematic equations to compute the position of the end-effector from specified values of joint parameters.

**Friction Coefficient**: A dimensionless scalar value that describes the ratio of the force of friction between two bodies and the force pressing them together.

## G

**Gazebo**: A 3D simulation environment for autonomous robots that provides realistic physics simulation and sensor models.

**Gimbal Lock**: A loss of one degree of freedom in a three-dimensional, three-gimbal mechanism that occurs when the axes of two of the three gimbals are driven into a parallel configuration.

## H

**Haptic Feedback**: The use of touch sensation in human-computer interaction, allowing users to feel virtual objects.

**Heuristic**: A technique designed for solving a problem more quickly when classic methods are too slow, or for finding an approximate solution when classic methods fail to find any exact solution.

## I

**IMU (Inertial Measurement Unit)**: An electronic device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body.

**Inverse Kinematics**: The mathematical process of calculating the variable joint parameters needed to place the end of a kinematic chain, such as a robot manipulator or animation character, in a given position and orientation.

**Iterative Closest Point (ICP)**: An algorithm employed to minimize the distance between two point clouds.

## J

**Jacobian Matrix**: A matrix of all first-order partial derivatives of a vector-valued function.

**Joint**: A connection between two or more links in a robot that allows relative motion between them.

## K

**Kinematics**: The branch of mechanics concerned with the motion of objects without reference to the forces that cause the motion.

**Kinetic Energy**: The energy that a body possesses by virtue of being in motion.

## L

**LIDAR (Light Detection and Ranging)**: A remote sensing method that uses light in the form of a pulsed laser to measure distances.

**Linear Interpolation**: A method of curve fitting using linear polynomials to construct new data points within the range of a discrete set of known data points.

**Localization**: The process of determining the position and orientation of a robot within a known or unknown environment.

## M

**Mapping**: The process of creating a representation of the robot's environment based on sensor data.

**Monte Carlo Method**: A computational algorithm that relies on repeated random sampling to obtain numerical results.

**Motor Controller**: A device that governs the operation of an electric motor.

## N

**Navigation Stack**: A collection of ROS packages that implement mobile robot navigation functionality.

**Nearest Neighbor**: An algorithm for finding the closest point in a set of points to a given point.

**Node**: A process that performs computation in ROS.

## O

**Odometry**: The use of data from motion sensors to estimate change in position over time.

**Occupancy Grid**: A probabilistic 2D representation of an environment that divides the space into discrete cells, each containing the probability of occupancy.

**Odom Frame**: A continuously-growing coordinate frame in ROS that represents the robot's path through space.

## P

**Path Planning**: The computational process of finding a valid and optimal path from a start point to a goal point.

**PID Controller**: A control loop feedback mechanism widely used in industrial control systems.

**Point Cloud**: A set of data points in space, typically representing the external surface of an object.

**Pose**: The position and orientation of a rigid body in space.

## Q

**Quaternion**: A mathematical construct that extends complex numbers and is used to represent rotations in 3D space without suffering from gimbal lock.

## R

**Real-Time Factor**: The ratio of simulation time to real time, indicating how fast a simulation runs compared to real time.

**Robot Operating System (ROS)**: A flexible framework for writing robot software that provides services designed for a heterogeneous computer cluster.

**ROS 2**: The second generation of the Robot Operating System with improved security, real-time support, and distributed architecture.

**ROS Package**: A reusable software module in ROS that contains libraries, executables, scripts, or other files.

**ROS Topic**: A named bus over which nodes exchange messages in a publish/subscribe communication pattern.

**ROS Service**: A synchronous request/response communication pattern in ROS.

**ROS Action**: A goal-oriented communication pattern in ROS that supports feedback and cancellation.

## S

**Sensor Fusion**: The combining of data from multiple sensors to achieve improved accuracy and more useful information than could be achieved by using a single sensor alone.

**Simulation**: The imitation of the operation of a real-world process or system over time.

**SLAM (Simultaneous Localization and Mapping)**: The computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

**State Machine**: A computational model used to design computer programs and logic circuits, characterized by a number of states, transitions between those states, and actions.

## T

**TF (Transforms)**: The name of the ROS package that lets the user keep track of multiple coordinate frames over time.

**Topic**: A named bus over which nodes exchange messages in ROS.

**Trajectory**: A path that a moving object follows through space as a function of time.

**Twist**: A ROS message type that represents the velocity of a robot in free space.

## U

**URDF (Unified Robot Description Format)**: An XML format for representing a robot model including kinematic and dynamic information.

**Unity**: A cross-platform game engine that can be used for creating interactive 3D simulations and visualizations.

**Unity Robotics Hub**: A collection of tools and packages that enable robotics simulation and development in Unity.

## V

**Vector Field Histogram**: A real-time motion planning method for mobile robots that uses a local map of obstacles to find a safe path.

**Velocity Obstacle**: A motion planning concept that represents the set of robot velocities that would result in a collision with an obstacle.

**Visualization**: The process of creating images, diagrams, or animations to communicate information about data or systems.

## W

**World Frame**: The global coordinate frame in a simulation environment that serves as the reference for all other coordinate frames.

**Waypoint**: A reference point in physical space used for planning or navigation.

## X, Y, Z

**X, Y, Z Axes**: The three perpendicular axes that define 3D Cartesian coordinate space, typically X (forward/backward), Y (left/right), and Z (up/down) in ROS conventions.

**Zero Moment Point (ZMP)**: A concept used in robotics and biomechanics to propose a stability criterion for legged locomotion.

## Common Abbreviations

- **API**: Application Programming Interface
- **CPU**: Central Processing Unit
- **GPU**: Graphics Processing Unit
- **IMU**: Inertial Measurement Unit
- **LIDAR**: Light Detection and Ranging
- **ROS**: Robot Operating System
- **SLAM**: Simultaneous Localization and Mapping
- **TF**: Transforms
- **URDF**: Unified Robot Description Format
- **VR**: Virtual Reality
- **AR**: Augmented Reality
- **AI**: Artificial Intelligence
- **ML**: Machine Learning
- **PID**: Proportional-Integral-Derivative
- **ICP**: Iterative Closest Point
- **NRF**: Non-Functional Requirements
- **SLO**: Service Level Objective
- **QoS**: Quality of Service

## Simulation-Specific Terms

**Digital Twin**: A virtual representation of a physical system that enables understanding, prediction, and optimization of the physical system's performance.

**Physics Engine**: Software that provides an approximate simulation of physical systems such as rigid body dynamics, soft body dynamics, and fluid dynamics.

**Real-time Simulation**: A simulation that runs at the same rate as real time, often used for interactive applications.

**Sensor Simulation**: The process of generating realistic sensor data in a simulation environment that mimics real sensor behavior.

**ROS Bridge**: A tool or system that enables communication between ROS-based systems and external systems (like Unity).

**Simulation Fidelity**: The accuracy with which a simulation represents the real system it is modeling.

**Validation**: The process of checking that a simulation correctly represents the real-world system it is intended to model.

**Verification**: The process of checking that a simulation is implemented correctly according to its specifications.

This glossary provides a comprehensive reference for the terminology used throughout the digital twin simulation curriculum.