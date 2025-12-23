# Data Model: NVIDIA Isaac™ AI-Robot Brain

## Key Entities

### Simulation Environment
- **Name**: SimulationEnvironment
- **Fields**:
  - id: string (unique identifier)
  - name: string (display name)
  - description: string (environment description)
  - physicsProperties: PhysicsProperties (gravity, friction, etc.)
  - lightingConditions: LightingConditions (ambient, directional lights)
  - objects: Array<SimulatedObject> (interactive objects in environment)
  - sensors: Array<SensorConfig> (sensor configurations)
- **Relationships**: Contains multiple SimulatedObject and SensorConfig
- **Validation**: Must have valid physics properties and at least one sensor
- **State transitions**: Active → Paused → Stopped

### PhysicsProperties
- **Name**: PhysicsProperties
- **Fields**:
  - gravity: Vector3 (gravity vector)
  - friction: number (friction coefficient)
  - restitution: number (bounciness factor)
  - timeStep: number (physics simulation time step)
- **Validation**: Gravity must be valid vector, friction between 0-1

### HumanoidRobotModel
- **Name**: HumanoidRobotModel
- **Fields**:
  - id: string (unique identifier)
  - name: string (robot name)
  - urdfPath: string (URDF file path)
  - jointCount: number (number of joints)
  - linkCount: number (number of links)
  - sensors: Array<SensorConfig> (attached sensors)
  - actuators: Array<ActuatorConfig> (joint actuators)
  - balanceConstraints: BalanceConstraints (balance and stability parameters)
- **Relationships**: Contains multiple SensorConfig and ActuatorConfig
- **Validation**: Must have valid URDF path, at least one joint

### SensorConfig
- **Name**: SensorConfig
- **Fields**:
  - id: string (unique identifier)
  - type: SensorType (camera, lidar, imu, etc.)
  - position: Vector3 (position relative to robot)
  - orientation: Vector3 (orientation relative to robot)
  - parameters: Object (sensor-specific parameters)
  - enabled: boolean (is sensor active)
- **Validation**: Position and orientation must be valid, type must be supported

### SensorType
- **Name**: SensorType
- **Values**:
  - CAMERA: RGB camera sensor
  - LIDAR: LiDAR range sensor
  - IMU: Inertial measurement unit
  - FORCE_TORQUE: Force/torque sensor
  - GPS: Global positioning system
- **Validation**: Must be one of the defined values

### PerceptionPipeline
- **Name**: PerceptionPipeline
- **Fields**:
  - id: string (unique identifier)
  - name: string (pipeline name)
  - description: string (pipeline description)
  - nodes: Array<PipelineNode> (processing nodes in pipeline)
  - inputSensors: Array<SensorConfig> (input sensor configurations)
  - output: PerceptionOutput (pipeline output)
  - hardwareAcceleration: boolean (uses GPU acceleration)
- **Relationships**: Contains multiple PipelineNode, references multiple SensorConfig
- **Validation**: Must have at least one processing node, valid input sensors

### PipelineNode
- **Name**: PipelineNode
- **Fields**:
  - id: string (unique identifier)
  - name: string (node name)
  - type: PipelineNodeType (vs-lam, object detection, etc.)
  - parameters: Object (node-specific parameters)
  - inputPorts: Array<PortConfig> (input connections)
  - outputPorts: Array<PortConfig> (output connections)
- **Validation**: Must have valid type and properly connected ports

### PipelineNodeType
- **Name**: PipelineNodeType
- **Values**:
  - VSLAM: Visual Simultaneous Localization and Mapping
  - OBJECT_DETECTION: Object detection and classification
  - DEPTH_ESTIMATION: Depth map generation
  - FEATURE_TRACKING: Feature point tracking
  - SENSOR_FUSION: Multiple sensor data fusion
- **Validation**: Must be one of the defined values

### NavigationPlan
- **Name**: NavigationPlan
- **Fields**:
  - id: string (unique identifier)
  - goal: Vector3 (navigation goal position)
  - path: Array<Vector3> (calculated path waypoints)
  - constraints: NavigationConstraints (path constraints)
  - status: NavigationStatus (current navigation status)
  - robotModel: HumanoidRobotModel (robot executing plan)
  - environment: SimulationEnvironment (environment for navigation)
- **Relationships**: References HumanoidRobotModel and SimulationEnvironment
- **Validation**: Goal must be reachable, path must be valid

### NavigationConstraints
- **Name**: NavigationConstraints
- **Fields**:
  - maxVelocity: number (maximum velocity)
  - maxAcceleration: number (maximum acceleration)
  - balanceThreshold: number (balance maintenance threshold)
  - footstepConstraints: FootstepConstraints (bipedal movement constraints)
  - obstacleClearance: number (minimum distance from obstacles)
- **Validation**: Values must be positive and within safe limits

### FootstepConstraints
- **Name**: FootstepConstraints
- **Fields**:
  - stepHeight: number (maximum step height)
  - stepLength: number (maximum step length)
  - stepWidth: number (step width)
  - swingHeight: number (foot swing height)
  - stanceWidth: number (stance width for stability)
- **Validation**: All values must be positive and physically feasible

### NavigationStatus
- **Name**: NavigationStatus
- **Values**:
  - IDLE: Navigation not started
  - PLANNING: Path is being calculated
  - EXECUTING: Path is being followed
  - PAUSED: Navigation temporarily stopped
  - COMPLETED: Goal reached successfully
  - FAILED: Navigation failed
- **Validation**: Must be one of the defined values

### SyntheticDataSample
- **Name**: SyntheticDataSample
- **Fields**:
  - id: string (unique identifier)
  - environment: SimulationEnvironment (source environment)
  - robotModel: HumanoidRobotModel (robot in scene)
  - sensorData: Array<SensorData> (sensor readings)
  - annotations: Array<Annotation> (ground truth annotations)
  - timestamp: Date (capture time)
  - qualityMetrics: QualityMetrics (data quality metrics)
- **Relationships**: References SimulationEnvironment and HumanoidRobotModel, contains multiple SensorData and Annotation
- **Validation**: Must have valid sensor data and annotations

### SensorData
- **Name**: SensorData
- **Fields**:
  - id: string (unique identifier)
  - sensorType: SensorType (type of sensor)
  - data: any (sensor-specific data format)
  - timestamp: Date (capture time)
  - frameId: string (coordinate frame)
  - metadata: Object (sensor-specific metadata)
- **Validation**: Data format must match sensor type

### Annotation
- **Name**: Annotation
- **Fields**:
  - id: string (unique identifier)
  - type: AnnotationType (object detection, segmentation, etc.)
  - data: Object (annotation-specific data)
  - confidence: number (annotation confidence score)
  - label: string (object label)
- **Validation**: Confidence must be between 0-1, label must be valid

### AnnotationType
- **Name**: AnnotationType
- **Values**:
  - BOUNDING_BOX: 2D/3D bounding box annotation
  - SEMANTIC_SEGMENTATION: Pixel-level segmentation
  - INSTANCE_SEGMENTATION: Instance-level segmentation
  - KEYPOINTS: Key point annotations
  - DEPTH: Depth map annotation
- **Validation**: Must be one of the defined values

### QualityMetrics
- **Name**: QualityMetrics
- **Fields**:
  - photorealismScore: number (how photorealistic the image is)
  - annotationAccuracy: number (accuracy of annotations)
  - lightingVariation: number (lighting condition variation)
  - sensorNoise: number (simulated sensor noise level)
  - physicsAccuracy: number (physics simulation accuracy)
- **Validation**: All scores must be between 0-1

## Relationships

### Primary Relationships
- SimulationEnvironment 1 → * SimulatedObject (contains objects)
- SimulationEnvironment 1 → * SensorConfig (contains sensors)
- HumanoidRobotModel 1 → * SensorConfig (has sensors)
- HumanoidRobotModel 1 → * ActuatorConfig (has actuators)
- PerceptionPipeline 1 → * PipelineNode (contains nodes)
- PerceptionPipeline * → * SensorConfig (uses sensors as input)
- NavigationPlan 1 → 1 HumanoidRobotModel (executed by)
- NavigationPlan 1 → 1 SimulationEnvironment (executed in)
- SyntheticDataSample 1 → * SensorData (contains data)
- SyntheticDataSample 1 → * Annotation (has annotations)

### Validation Rules
1. A SimulationEnvironment must have at least one sensor to be valid for data generation
2. A HumanoidRobotModel must have valid URDF and at least one joint to be functional
3. A PerceptionPipeline must have at least one PipelineNode to process data
4. A NavigationPlan must have a reachable goal within the environment
5. All coordinates and parameters must be within physically feasible ranges
6. Quality metrics must be between 0 and 1 for normalized scoring