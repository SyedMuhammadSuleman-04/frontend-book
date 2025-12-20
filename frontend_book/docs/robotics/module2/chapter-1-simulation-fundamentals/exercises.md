# Chapter 1 Exercises: Gazebo Simulation Fundamentals

These exercises will help you practice and reinforce the concepts of physics simulation, gravity, and collision handling in Gazebo.

## Exercise 1: Basic Physics Model Creation

### Objective
Create a simple robot model with proper physics properties and verify its behavior in Gazebo.

### Tasks
1. **Create a Simple Robot Model**:
   - Design a URDF file for a simple wheeled robot
   - Include proper mass and inertia values for each link
   - Define collision and visual geometry for all links

2. **Physics Properties**:
   - Set realistic mass values (e.g., base: 5kg, wheels: 0.5kg each)
   - Calculate appropriate inertia tensors for cylindrical wheels
   - Use the parallel axis theorem if needed for offset masses

3. **Implementation**:
   ```xml
   <robot name="simple_robot">
     <link name="base_link">
       <inertial>
         <mass value="5.0"/>
         <origin xyz="0 0 0.1" rpy="0 0 0"/>
         <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.2"/>
       </inertial>
       <collision>
         <origin xyz="0 0 0.1" rpy="0 0 0"/>
         <geometry>
           <box size="0.5 0.3 0.2"/>
         </geometry>
       </collision>
       <visual>
         <origin xyz="0 0 0.1" rpy="0 0 0"/>
         <geometry>
           <box size="0.5 0.3 0.2"/>
         </geometry>
       </visual>
     </link>
     <!-- Add wheel links here -->
   </robot>
   ```

4. **Validation**:
   - Spawn the robot in Gazebo
   - Verify it falls to the ground due to gravity
   - Check that it remains stable without jittering

### Expected Outcome
The robot should spawn, fall to the ground due to gravity, and remain stable without oscillating or falling through the ground plane.

### Questions to Consider
- How do mass and inertia values affect the robot's behavior?
- What happens if you change the gravity value in the world file?
- How does the shape of collision geometry affect stability?

## Exercise 2: Gravity Experimentation

### Objective
Investigate the effects of different gravity values on robot behavior.

### Tasks
1. **Create Multiple World Files**:
   - `earth_gravity.world`: Standard Earth gravity (9.8 m/s²)
   - `moon_gravity.world`: Moon gravity (1.62 m/s²)
   - `zero_gravity.world`: No gravity (0 m/s²)

2. **Experiment Setup**:
   - Use the same robot model in each world
   - Place identical objects in each environment
   - Document the differences in behavior

3. **Test Scenarios**:
   - Drop a sphere from the same height in each world
   - Compare fall times and impact velocities
   - Test robot locomotion in different gravity conditions

4. **Analysis**:
   - Measure fall times and compare with theoretical values
   - Calculate the relationship between gravity and fall time
   - Document how gravity affects robot stability

### Expected Outcome
- Objects should fall faster in higher gravity environments
- Fall time should be inversely proportional to the square root of gravity
- Robot stability may vary with gravity conditions

### Questions to Consider
- How does reduced gravity affect robot locomotion?
- What challenges arise in zero gravity conditions?
- How might these gravity differences affect robot design?

## Exercise 3: Collision Detection and Response

### Objective
Create various collision scenarios to understand how Gazebo handles different collision types.

### Tasks
1. **Create Collision Test Objects**:
   - Sphere with radius 0.1m
   - Box with dimensions 0.2m × 0.2m × 0.2m
   - Cylinder with radius 0.1m and height 0.2m

2. **Setup Collision Scenarios**:
   - Drop each object from the same height
   - Create ramps with different angles
   - Set up collision between moving objects

3. **Vary Material Properties**:
   - Different friction coefficients (0.1, 0.5, 1.0, 2.0)
   - Different restitution coefficients (0.0, 0.1, 0.5, 0.9)
   - Document the effects of each property

4. **Validation Tests**:
   - Measure bounce height vs. restitution coefficient
   - Test sliding vs. rolling on inclined planes
   - Verify momentum conservation in collisions

### Expected Outcome
- Objects with higher restitution should bounce higher
- Objects with higher friction should slide less on inclined surfaces
- Collision responses should follow physics principles

### Questions to Consider
- How do different shapes affect collision behavior?
- What is the relationship between friction coefficient and sliding angle?
- How does the time step affect collision accuracy?

## Exercise 4: Robot Stability Analysis

### Objective
Analyze factors that affect robot stability in simulation.

### Tasks
1. **Create Robot Variants**:
   - Robot with low center of mass
   - Robot with high center of mass
   - Robot with wide base
   - Robot with narrow base

2. **Test Stability Scenarios**:
   - Apply lateral forces to each robot
   - Test on inclined surfaces of different angles
   - Add oscillating masses to test dynamic stability

3. **Measure Stability Metrics**:
   - Maximum stable incline angle
   - Minimum force to tip the robot
   - Recovery time from perturbations

4. **Analysis**:
   - Compare stability of different designs
   - Relate findings to real-world robot design
   - Document optimal design parameters

### Expected Outcome
- Robots with lower center of mass should be more stable
- Wider bases should provide better stability
- Stability metrics should correlate with design parameters

### Questions to Consider
- How does mass distribution affect stability?
- What are the trade-offs between stability and mobility?
- How do these stability principles apply to real robots?

## Exercise 5: Physics Parameter Optimization

### Objective
Optimize physics parameters for the best balance of accuracy and performance.

### Tasks
1. **Parameter Variation**:
   - Time step: 0.001s, 0.005s, 0.01s, 0.02s
   - Solver iterations: 10, 50, 100, 200
   - Real-time factor target: 0.5, 1.0, 2.0

2. **Performance Metrics**:
   - Measure real-time factor achieved
   - Monitor CPU usage
   - Test simulation stability
   - Check accuracy of physics behavior

3. **Accuracy Tests**:
   - Free fall with known acceleration
   - Pendulum with known period
   - Collision with known momentum conservation

4. **Optimization Process**:
   - Find the largest time step that maintains stability
   - Determine minimum solver iterations for accuracy
   - Balance performance with physical accuracy

### Expected Outcome
- Smaller time steps should provide better accuracy but lower performance
- Higher solver iterations should improve accuracy but reduce performance
- Optimal parameters will balance both requirements

### Questions to Consider
- What is the minimum acceptable real-time factor for your application?
- How do you prioritize accuracy vs. performance?
- What parameters have the most significant impact on performance?

## Exercise 6: Complex Physics Scenario

### Objective
Create a complex scenario that combines multiple physics concepts.

### Tasks
1. **Environment Design**:
   - Create a multi-level environment with platforms at different heights
   - Add various obstacles (spheres, boxes, cylinders)
   - Include ramps and inclined surfaces

2. **Robot Design**:
   - Create a robot capable of navigating the environment
   - Include appropriate sensors (optional, for navigation)
   - Design for the specific physics challenges

3. **Physics Challenges**:
   - Robot must navigate from bottom to top level
   - Avoid obstacles while maintaining stability
   - Handle different surface types with varying friction

4. **Validation**:
   - Test robot's ability to complete the challenge
   - Verify physics behavior matches expectations
   - Document any unexpected behaviors

### Expected Outcome
The robot should be able to navigate the complex environment while exhibiting realistic physics behavior including proper gravity response, collision detection, and stability.

### Questions to Consider
- How do you design a robot for specific physics challenges?
- What physics parameters are most critical for complex scenarios?
- How do you validate that complex physics behavior is correct?

## Exercise 7: Error Detection and Debugging

### Objective
Practice identifying and fixing common physics simulation errors.

### Tasks
1. **Intentionally Create Errors**:
   - Incorrect mass values (negative or zero)
   - Improper inertia tensors (non-physical values)
   - Missing collision geometry
   - Incorrect coordinate frames

2. **Error Identification**:
   - Run simulations with errors
   - Observe unusual behaviors
   - Use Gazebo's visualization tools to identify issues

3. **Fix Procedures**:
   - Correct each error systematically
   - Verify fixes resolve the issues
   - Document the debugging process

4. **Prevention Strategies**:
   - Develop validation checks for URDF files
   - Create testing procedures for physics models
   - Document common error patterns

### Expected Outcome
You should be able to identify common physics simulation errors and apply appropriate fixes to ensure proper simulation behavior.

### Questions to Consider
- What are the most common physics errors you encountered?
- How can you prevent these errors in the future?
- What debugging tools are most helpful for physics issues?

## Assessment Criteria

Each exercise will be assessed based on:

### Implementation (40%)
- Correct implementation of physics models
- Proper use of URDF/SDF syntax
- Appropriate parameter values

### Analysis (30%)
- Quality of experimental design
- Accuracy of measurements and observations
- Depth of understanding demonstrated

### Documentation (20%)
- Clear explanation of procedures
- Proper recording of results
- Thoughtful analysis of findings

### Problem-Solving (10%)
- Ability to troubleshoot issues
- Creative approaches to challenges
- Application of physics principles

## Submission Requirements

For each exercise, submit:

1. **Code Files**: URDF, SDF, and launch files used
2. **Documentation**: Description of procedures, results, and analysis
3. **Screenshots**: Key simulation states or behaviors
4. **Reflection**: What you learned and challenges encountered

## Extension Activities

For advanced students:

1. **Implement a physics validation tool** that automatically checks URDF files
2. **Create a parameter optimization script** that finds optimal physics settings
3. **Develop a physics behavior prediction system** that estimates simulation outcomes
4. **Build a comparative analysis tool** that compares simulation vs. theoretical physics

## Resources

- [Gazebo Physics Documentation](http://gazebosim.org/tutorials?tut=physics_params)
- [URDF Inertial Tutorial](http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20Properties%20to%20a%20URDF%20Model)
- [Physics Validation Techniques](http://gazebosim.org/tutorials?tut=validation)

## Next Steps

After completing these exercises, you should have a solid understanding of physics simulation fundamentals in Gazebo. The next chapter will build on these concepts by introducing virtual environment creation in Unity, where you'll learn to create the visual counterparts to these physics simulations.