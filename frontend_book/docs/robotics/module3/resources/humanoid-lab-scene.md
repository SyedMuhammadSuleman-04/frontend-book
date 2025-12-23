# Humanoid Lab Scene Setup

This document describes how to create and configure the example USD scene files for the humanoid lab environment.

## Scene Overview

The humanoid lab scene provides a controlled environment for testing humanoid robots with various sensors and scenarios. It includes:

- **Main Lab Area**: Open space for robot navigation and testing
- **Obstacle Course**: Various obstacles to test navigation capabilities
- **Sensor Testing Area**: Areas designed for sensor validation
- **Charging Station**: Designated area for robot recharging

## USD Scene Structure

### Basic Scene Hierarchy
```
World
├── GroundPlane
├── Lighting
│   ├── DirectionalLight
│   └── AmbientLight
├── LabEnvironment
│   ├── Walls
│   ├── Doors
│   └── Furniture
├── Obstacles
│   ├── StaticObstacles
│   └── DynamicObstacles
├── HumanoidRobot
└── Sensors
    ├── Camera
    ├── LiDAR
    └── IMU
```

## Creating the USD Scene

### Using Isaac Sim GUI

1. **Launch Isaac Sim**
   - Start Isaac Sim application
   - Create a new scene (File → New Scene)

2. **Set Up Ground Plane**
   - Add a plane primitive (Create → Primitive → Plane)
   - Scale to appropriate size (e.g., 10m x 10m)
   - Add physics properties for collision
   - Apply appropriate material for realistic appearance

3. **Configure Lighting**
   - Add directional light (Create → Light → Distant Light)
   - Set direction to simulate sun (typically at 45-degree angle)
   - Adjust intensity (typically 3000-5000 lux equivalent)
   - Add ambient light for realistic illumination

4. **Create Lab Environment**
   - Add walls using cube primitives or custom meshes
   - Position walls to create lab boundaries
   - Add furniture (tables, chairs) for realistic environment
   - Ensure appropriate spacing for robot navigation

5. **Add Obstacles**
   - Create static obstacles (cubes, cylinders) for navigation testing
   - Position obstacles to create challenging paths
   - Add dynamic obstacles if testing moving object avoidance
   - Ensure obstacles have appropriate physics properties

6. **Import Humanoid Robot**
   - Import your humanoid robot model (File → Import → URDF or drag USD file)
   - Position the robot appropriately in the scene
   - Configure robot physics and joint properties
   - Add sensors to the robot as needed

### USD File Content Example

Here's an example of what the USD file structure might look like:

```usd
#usda 1.0

def Xform "World" (
    prepend references = </Isaac/Environments/Room.usd>
)
{
    def Xform "GroundPlane"
    {
        def Mesh "Plane"
        {
            float3[] points = [(-5, 0, -5), (5, 0, -5), (5, 0, 5), (-5, 0, 5)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            float2[] primvars:st = [(0, 0), (10, 0), (10, 10), (0, 10)] (
                interpolation = "vertex"
            )
        }
    }

    def Xform "Lighting"
    {
        def DistantLight "DirectionalLight"
        {
            float intensity = 5000
            color3f color = (1, 1, 1)
            float angle = 0.5
        }
    }

    def Xform "LabEnvironment"
    {
        def Xform "Walls"
        {
            # Wall definitions here
        }

        def Xform "Furniture"
        {
            # Table, chair definitions here
        }
    }

    def Xform "Obstacles"
    {
        def Cube "StaticObstacle1"
        {
            double3 xformOp:translate = (2, 0, 2)
            double3 size = (0.5, 0.5, 0.5)
        }
    }

    # Robot and sensors would be referenced here
}
```

## Scene Parameters

### Physical Properties
- **Gravity**: Set to -9.81 m/s² for Earth-like gravity
- **Friction**: Set to realistic values (0.5-0.8 for most surfaces)
- **Restitution**: Set to appropriate bounciness (0.1-0.3 for most objects)

### Environmental Settings
- **Dimensions**: Main lab area of 10m x 10m minimum
- **Ceiling Height**: 3m minimum for humanoid robot operation
- **Lighting**: 5000 lux equivalent for realistic vision
- **Materials**: Realistic materials for accurate sensor simulation

## Scene Variants

### Basic Lab Scene
- Simple rectangular room
- Minimal obstacles
- Basic lighting
- Ideal for initial robot testing

### Complex Lab Scene
- Multiple rooms and corridors
- Various obstacle types
- Variable lighting conditions
- Advanced scenarios for comprehensive testing

### Outdoor Scene
- Simulated outdoor environment
- Natural lighting conditions
- Terrain variations
- Weather simulation (optional)

## Scene Validation

### Visual Validation
1. **Check Scene Layout**: Ensure all elements are properly positioned
2. **Verify Lighting**: Check for realistic illumination
3. **Validate Materials**: Confirm materials look realistic
4. **Test Camera Views**: Verify scene looks correct from robot perspective

### Physics Validation
1. **Collision Detection**: Test that all objects have proper collision geometry
2. **Gravity Simulation**: Verify objects behave with realistic physics
3. **Robot Interaction**: Test that robot can navigate and interact properly
4. **Sensor Simulation**: Validate that sensors work correctly in the environment

### Performance Validation
1. **Frame Rate**: Ensure scene runs at acceptable frame rate (30+ FPS)
2. **Memory Usage**: Verify scene doesn't exceed memory limits
3. **Physics Accuracy**: Check that physics simulation remains stable
4. **Rendering Quality**: Validate that visual quality meets requirements

## Scene Customization

### Modifying Existing Scene
1. **Open USD File**: Load the scene in Isaac Sim
2. **Make Changes**: Modify elements using Isaac Sim tools
3. **Save Changes**: Save to update the USD file
4. **Validate**: Test that changes work as expected

### Creating New Scenes
1. **Start from Template**: Use existing scene as base
2. **Modify Layout**: Change room layout and obstacles
3. **Adjust Parameters**: Update lighting and physics properties
4. **Test Functionality**: Verify scene works with robot and sensors

## Troubleshooting Common Issues

### Scene Loading Issues
- **Issue**: Scene fails to load properly
  - **Solution**: Check USD file syntax and file paths
- **Issue**: Missing textures or materials
  - **Solution**: Verify asset paths and file locations
- **Issue**: Physics not working correctly
  - **Solution**: Check physics properties and collision settings

### Performance Issues
- **Issue**: Low frame rate
  - **Solution**: Simplify geometry or reduce lighting complexity
- **Issue**: Physics instability
  - **Solution**: Adjust physics timestep and solver settings
- **Issue**: Memory errors
  - **Solution**: Reduce scene complexity or increase available memory

## Best Practices

### Scene Design
- Keep scenes simple but functional
- Use appropriate level of detail
- Maintain consistent coordinate systems
- Plan for different lighting conditions

### Asset Management
- Organize assets in logical directory structure
- Use consistent naming conventions
- Maintain version control for scene files
- Document scene parameters and configurations

### Testing
- Test scenes with actual robot models
- Validate all sensors work in the scene
- Check performance on target hardware
- Verify physics behavior is realistic

## Resources

- [USD Documentation](https://graphics.pixar.com/usd/release/index.html)
- [Isaac Sim Scene Creation Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_scenes.html)
- [USD Best Practices](https://graphics.pixar.com/usd/release/tutorials.html)

## Next Steps

After creating your scene:
1. Test with your humanoid robot model
2. Validate sensor performance in the scene
3. Adjust parameters as needed
4. Create variations for different testing scenarios