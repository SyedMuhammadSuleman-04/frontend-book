# Environment Building in Unity

This document provides comprehensive guidance on creating virtual environments in Unity for robotics simulation, including objects, lighting, and interactive elements.

## Environment Design Principles

### Realistic Simulation Requirements

Creating effective virtual environments for robotics simulation requires balancing visual fidelity with computational efficiency:

**Visual Realism**:
- Accurate geometric representation of real-world objects
- Proper material properties for sensor simulation
- Consistent lighting conditions
- Appropriate textures and surface details

**Physical Accuracy**:
- Correct object sizes and proportions
- Realistic material properties (reflectivity, transparency)
- Accurate surface properties for interaction
- Proper collision geometry

**Performance Considerations**:
- Optimized polygon counts for real-time simulation
- Efficient texture usage
- Proper level of detail (LOD) systems
- Appropriate batching for static objects

### Planning Your Environment

Before building, consider:
- **Purpose**: What robot behaviors will be tested?
- **Scale**: What size environment is needed?
- **Complexity**: How detailed should objects be?
- **Lighting**: What lighting conditions are required?
- **Interactivity**: Which objects need to respond to robots?

## Basic Environment Setup

### Creating a New Scene

1. **Start with a New 3D Project**:
   - File → New Project
   - Select 3D (Built-in Render Pipeline or URP/HDRP)
   - Name your environment (e.g., "Warehouse_Simulation")

2. **Set Up Basic Scene Elements**:
   - Remove default camera and lighting
   - Add a plane as the ground/floor
   - Set appropriate scale (1 Unity unit = 1 meter for robotics)

3. **Configure Scene Settings**:
   - Set gravity in Physics settings (Edit → Project Settings → Physics)
   - Configure collision layers if needed
   - Set up appropriate render settings

### Ground Plane Configuration

```csharp
// Create a ground plane that matches your simulation requirements
public class GroundPlane : MonoBehaviour
{
    public float size = 10f;  // Size in meters
    public PhysicMaterial groundMaterial;

    void Start()
    {
        // Configure the ground plane
        var plane = GetComponent<MeshCollider>();
        if (plane != null)
        {
            // Set appropriate material properties
            GetComponent<Renderer>().material = new Material(Shader.Find("Standard"));
            GetComponent<Renderer>().material.color = Color.gray;
        }
    }
}
```

## Creating Static Environment Objects

### Basic Shapes and Structures

**Creating Walls**:
```csharp
public class WallCreator : MonoBehaviour
{
    public float wallHeight = 3f;
    public float wallThickness = 0.2f;
    public float wallLength = 5f;

    void CreateWall()
    {
        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.transform.localScale = new Vector3(wallLength, wallHeight, wallThickness);
        wall.GetComponent<Renderer>().material.color = Color.white;

        // Add realistic material properties
        wall.GetComponent<Renderer>().material = CreateWallMaterial();
    }

    Material CreateWallMaterial()
    {
        Material mat = new Material(Shader.Find("Standard"));
        mat.color = Color.white;
        mat.SetFloat("_Metallic", 0.1f);
        mat.SetFloat("_Smoothness", 0.2f);
        return mat;
    }
}
```

**Creating Furniture and Obstacles**:
- Use primitive shapes (cubes, spheres, cylinders) for basic objects
- Combine multiple primitives for complex shapes
- Apply appropriate materials for visual and physical properties
- Add collision detection for interaction

### Advanced Object Creation

**Procedural Environment Generation**:
```csharp
public class ProceduralEnvironment : MonoBehaviour
{
    public GameObject[] obstaclePrefabs;
    public int obstacleCount = 10;
    public Vector2 environmentBounds = new Vector2(20, 20);

    void GenerateEnvironment()
    {
        for (int i = 0; i < obstacleCount; i++)
        {
            Vector3 position = new Vector3(
                Random.Range(-environmentBounds.x / 2, environmentBounds.x / 2),
                0,
                Random.Range(-environmentBounds.y / 2, environmentBounds.y / 2)
            );

            GameObject obstacle = Instantiate(
                obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)],
                position,
                Quaternion.identity
            );

            // Ensure obstacles don't overlap
            if (Physics.CheckSphere(position, 1f))
            {
                Destroy(obstacle);
                i--; // Retry this iteration
            }
        }
    }
}
```

## Lighting Systems

### Types of Lighting for Robotics

**Directional Lighting** (Sun/Sky):
- Simulates outdoor lighting conditions
- Creates realistic shadows
- Affects all objects in the scene uniformly

**Point Lighting**:
- Simulates localized light sources (lamps, ceiling lights)
- Creates realistic lighting falloff
- Can simulate indoor environments

**Area Lighting**:
- Provides soft, realistic lighting
- Good for simulating large light sources
- More computationally expensive

### Configuring Realistic Lighting

```csharp
public class EnvironmentLighting : MonoBehaviour
{
    public Light mainLight;  // Usually a directional light
    public Color lightColor = Color.white;
    public float intensity = 1.0f;
    public float shadowStrength = 0.8f;

    void ConfigureLighting()
    {
        if (mainLight == null)
        {
            // Create main directional light if it doesn't exist
            GameObject lightObj = new GameObject("Main Light");
            mainLight = lightObj.AddComponent<Light>();
            mainLight.type = LightType.Directional;
            mainLight.transform.rotation = Quaternion.Euler(50, -30, 0);
        }

        // Configure light properties
        mainLight.color = lightColor;
        mainLight.intensity = intensity;
        mainLight.shadows = LightShadows.Soft;
        mainLight.shadowStrength = shadowStrength;
    }

    // Configure for different lighting conditions
    public void SetDayLighting()
    {
        mainLight.color = Color.white;
        mainLight.intensity = 1.0f;
        mainLight.transform.rotation = Quaternion.Euler(50, -30, 0);
    }

    public void SetNightLighting()
    {
        mainLight.color = new Color(0.8f, 0.8f, 1.0f); // Cool white
        mainLight.intensity = 0.3f;
        mainLight.transform.rotation = Quaternion.Euler(130, -30, 0);
    }

    public void SetIndoorLighting()
    {
        mainLight.color = new Color(0.95f, 0.9f, 0.8f); // Warm white
        mainLight.intensity = 0.8f;
        mainLight.shadows = LightShadows.Hard;
    }
}
```

### Dynamic Lighting Control

For simulating changing lighting conditions:

```csharp
public class DynamicLightingController : MonoBehaviour
{
    public AnimationCurve intensityCurve;
    public float cycleDuration = 86400f; // 24 hours in seconds
    private float startTime;

    void Start()
    {
        startTime = Time.time;
    }

    void Update()
    {
        float elapsed = (Time.time - startTime) % cycleDuration;
        float normalizedTime = elapsed / cycleDuration;

        float intensity = intensityCurve.Evaluate(normalizedTime);
        RenderSettings.ambientLight = Color.white * intensity * 0.2f;
    }
}
```

## Interactive Elements

### Physics-Based Interactions

**Movable Objects**:
```csharp
public class InteractiveObject : MonoBehaviour
{
    public float mass = 1.0f;
    public bool isMovable = true;
    public PhysicMaterial physicMaterial;

    void Start()
    {
        if (isMovable)
        {
            Rigidbody rb = gameObject.AddComponent<Rigidbody>();
            rb.mass = mass;
            rb.useGravity = true;

            // Configure collision properties
            if (physicMaterial != null)
            {
                GetComponent<Collider>().material = physicMaterial;
            }
        }
    }

    // Optional: Visual feedback when robot interacts
    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.tag == "Robot")
        {
            // Visual or audio feedback
            GetComponent<Renderer>().material.color = Color.red;
            Invoke("ResetColor", 0.5f);
        }
    }

    void ResetColor()
    {
        GetComponent<Renderer>().material.color = Color.white;
    }
}
```

### Trigger-Based Interactions

**Sensor Zones**:
```csharp
public class SensorZone : MonoBehaviour
{
    public string zoneName = "sensor_zone";
    public bool isTrigger = true;

    void Start()
    {
        GetComponent<Collider>().isTrigger = isTrigger;
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Robot"))
        {
            OnRobotEnter(other.gameObject);
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Robot"))
        {
            OnRobotExit(other.gameObject);
        }
    }

    void OnRobotEnter(GameObject robot)
    {
        Debug.Log($"Robot entered zone: {zoneName}");

        // Send message to ROS or trigger specific behavior
        // This could be where you publish to a ROS topic
    }

    void OnRobotExit(GameObject robot)
    {
        Debug.Log($"Robot exited zone: {zoneName}");
    }
}
```

## Material and Surface Properties

### Creating Realistic Materials

For sensor simulation, materials need to accurately represent real-world properties:

```csharp
public class MaterialManager : MonoBehaviour
{
    public Material CreateSurfaceMaterial(string name, float metallic, float smoothness, Color color)
    {
        Material mat = new Material(Shader.Find("Standard"));
        mat.name = name;
        mat.color = color;
        mat.SetFloat("_Metallic", metallic);
        mat.SetFloat("_Smoothness", smoothness);

        // For sensor simulation, consider reflectivity properties
        mat.SetFloat("_SpecularHighlights", metallic > 0.1f ? 1.0f : 0.0f);

        return mat;
    }

    public void SetupEnvironmentMaterials()
    {
        // Concrete floor
        Material concrete = CreateSurfaceMaterial("Concrete", 0.1f, 0.2f, Color.gray);

        // Metal surfaces
        Material metal = CreateSurfaceMaterial("Metal", 0.9f, 0.8f, Color.blue);

        // Plastic surfaces
        Material plastic = CreateSurfaceMaterial("Plastic", 0.2f, 0.5f, Color.red);

        // Wood surfaces
        Material wood = CreateSurfaceMaterial("Wood", 0.0f, 0.3f, new Color(0.6f, 0.4f, 0.2f));
    }
}
```

### Physic Materials for Realistic Physics

```csharp
public class PhysicMaterialManager : MonoBehaviour
{
    public PhysicMaterial CreatePhysicMaterial(string name, float staticFriction, float dynamicFriction, float bounciness)
    {
        PhysicMaterial physMat = new PhysicMaterial(name);
        physMat.staticFriction = staticFriction;
        physMat.dynamicFriction = dynamicFriction;
        physMat.bounciness = bounciness;

        return physMat;
    }

    public void SetupEnvironmentPhysicMaterials()
    {
        // Typical floor surfaces
        PhysicMaterial concrete = CreatePhysicMaterial("Concrete", 0.8f, 0.6f, 0.1f);
        PhysicMaterial wood = CreatePhysicMaterial("Wood", 0.4f, 0.3f, 0.05f);
        PhysicMaterial carpet = CreatePhysicMaterial("Carpet", 0.9f, 0.8f, 0.0f);

        // Robot interaction surfaces
        PhysicMaterial rubber = CreatePhysicMaterial("Rubber", 1.0f, 0.9f, 0.3f);
    }
}
```

## Advanced Environment Features

### Level of Detail (LOD) System

For performance optimization with complex environments:

```csharp
using UnityEngine;

public class EnvironmentLOD : MonoBehaviour
{
    public LODGroup lodGroup;
    public float[] lodDistances = {10f, 30f, 60f}; // Distances for each LOD level

    void Start()
    {
        SetupLOD();
    }

    void SetupLOD()
    {
        lodGroup = gameObject.AddComponent<LODGroup>();

        LOD[] lods = new LOD[3];

        // LOD 0: High detail (close)
        Renderer[] highDetailRenderers = GetComponentsInChildren<Renderer>();
        lods[0] = new LOD(0.5f, highDetailRenderers);

        // LOD 1: Medium detail (medium distance)
        Renderer[] mediumDetailRenderers = GetMediumDetailRenderers();
        lods[1] = new LOD(0.25f, mediumDetailRenderers);

        // LOD 2: Low detail (far)
        Renderer[] lowDetailRenderers = GetLowDetailRenderers();
        lods[2] = new LOD(0.05f, lowDetailRenderers);

        lodGroup.SetLODs(lods);
        lodGroup.RecalculateBounds();
    }

    Renderer[] GetMediumDetailRenderers()
    {
        // Return simplified version renderers
        return new Renderer[0]; // Implementation depends on your specific models
    }

    Renderer[] GetLowDetailRenderers()
    {
        // Return very simplified version renderers
        return new Renderer[0]; // Implementation depends on your specific models
    }
}
```

### Occlusion Culling

For large environments, implement occlusion culling:

1. Go to Window → Rendering → Occlusion Culling
2. Set up occluder and occludee objects
3. Bake the occlusion data (Window → Rendering → Lighting → Generate Lighting)

### Terrain Systems

For large outdoor environments:

```csharp
public class TerrainManager : MonoBehaviour
{
    public Terrain terrain;
    public float terrainSize = 1000f;
    public float terrainHeight = 100f;

    void CreateTerrain()
    {
        // Create terrain programmatically
        GameObject terrainObj = Terrain.CreateTerrainGameObject(new TerrainData());
        terrain = terrainObj.GetComponent<Terrain>();

        // Configure terrain size
        terrain.terrainData.size = new Vector3(terrainSize, terrainHeight, terrainSize);

        // Apply textures and details
        SetupTerrainTextures();
    }

    void SetupTerrainTextures()
    {
        // Create and assign terrain textures
        SplatPrototype[] textures = new SplatPrototype[2];

        // Grass texture
        textures[0] = new SplatPrototype();
        textures[0].texture = Resources.Load<Texture2D>("GrassTexture");
        textures[0].tileSize = new Vector2(20f, 20f);

        // Dirt texture
        textures[1] = new SplatPrototype();
        textures[1].texture = Resources.Load<Texture2D>("DirtTexture");
        textures[1].tileSize = new Vector2(15f, 15f);

        terrain.terrainData.splatPrototypes = textures;
    }
}
```

## Environment Optimization

### Performance Optimization Techniques

**Object Pooling**:
```csharp
public class ObjectPool : MonoBehaviour
{
    [System.Serializable]
    public class Pool
    {
        public string tag;
        public GameObject prefab;
        public int size;
    }

    public List<Pool> pools;
    public Dictionary<string, Queue<GameObject>> poolDictionary;

    void Start()
    {
        poolDictionary = new Dictionary<string, Queue<GameObject>>();

        foreach (Pool pool in pools)
        {
            Queue<GameObject> objectPool = new Queue<GameObject>();

            for (int i = 0; i < pool.size; i++)
            {
                GameObject obj = Instantiate(pool.prefab);
                obj.SetActive(false);
                objectPool.Enqueue(obj);
            }

            poolDictionary.Add(pool.tag, objectPool);
        }
    }

    public GameObject SpawnFromPool(string tag, Vector3 position, Quaternion rotation)
    {
        if (!poolDictionary.ContainsKey(tag))
        {
            Debug.LogWarning("Pool with tag " + tag + " doesn't exist.");
            return null;
        }

        GameObject objectToSpawn = poolDictionary[tag].Dequeue();
        objectToSpawn.SetActive(true);
        objectToSpawn.transform.position = position;
        objectToSpawn.transform.rotation = rotation;

        poolDictionary[tag].Enqueue(objectToSpawn);
        return objectToSpawn;
    }
}
```

### Batch Static Objects

For static environment elements:
- Mark static objects with the "Static" checkbox in Inspector
- Use Static Batch to combine static meshes
- Use Occlusion Culling for large environments

## Quality Assurance for Environments

### Validation Checklist

**Visual Validation**:
- [ ] Objects appear at correct scale
- [ ] Lighting looks realistic
- [ ] Textures are properly applied
- [ ] No visual artifacts or glitches

**Physical Validation**:
- [ ] Collision detection works properly
- [ ] Objects have appropriate physical properties
- [ ] No penetration between objects
- [ ] Physics behavior is realistic

**Performance Validation**:
- [ ] Frame rate remains stable
- [ ] Memory usage is reasonable
- [ ] Loading times are acceptable
- [ ] No performance drops during interaction

### Testing Procedures

**Manual Testing**:
1. Navigate through environment manually
2. Test all interactive elements
3. Verify lighting changes work correctly
4. Check collision responses

**Automated Testing**:
```csharp
public class EnvironmentTester : MonoBehaviour
{
    public void TestEnvironmentIntegrity()
    {
        // Check for overlapping objects
        TestObjectSpacing();

        // Verify all required objects exist
        TestObjectPresence();

        // Check lighting settings
        TestLighting();

        // Validate collision properties
        TestCollisions();
    }

    void TestObjectSpacing()
    {
        Collider[] colliders = FindObjectsOfType<Collider>();
        for (int i = 0; i < colliders.Length; i++)
        {
            for (int j = i + 1; j < colliders.Length; j++)
            {
                if (colliders[i].bounds.Intersects(colliders[j].bounds))
                {
                    Debug.LogWarning($"Overlapping objects detected: {colliders[i].name} and {colliders[j].name}");
                }
            }
        }
    }

    void TestObjectPresence()
    {
        // Verify critical environment objects exist
        if (FindObjectOfType<Light>() == null)
        {
            Debug.LogWarning("No lighting found in environment");
        }

        if (GameObject.FindGameObjectsWithTag("Ground").Length == 0)
        {
            Debug.LogWarning("No ground plane found in environment");
        }
    }
}
```

## Best Practices

### Environment Design

1. **Start Simple**: Begin with basic shapes, add complexity gradually
2. **Use Real-World Measurements**: Maintain accurate scale for robotics applications
3. **Plan for Performance**: Consider computational requirements from the start
4. **Modular Design**: Create reusable components and prefabs
5. **Documentation**: Keep track of environment specifications and requirements

### Material and Physics

1. **Consistent Properties**: Use consistent material properties throughout
2. **Realistic Values**: Use real-world physics parameters where possible
3. **Sensor Considerations**: Consider how materials will appear to robot sensors
4. **Performance vs. Accuracy**: Balance visual fidelity with simulation performance

### Organization and Management

1. **Prefab Usage**: Create prefabs for reusable environment elements
2. **Layer Management**: Use Unity layers for different object types
3. **Naming Conventions**: Use consistent naming for objects and materials
4. **Version Control**: Properly version control environment files

## Integration with ROS 2

### Publishing Environment State

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class EnvironmentStatePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string environmentStateTopic = "/environment/state";

    void Start()
    {
        ros = ROSConnection.instance;
        InvokeRepeating("PublishEnvironmentState", 0.0f, 1.0f); // Every second
    }

    void PublishEnvironmentState()
    {
        // Create a message with environment information
        // This could include object positions, lighting state, etc.
        // For now, using a simple example:

        // In practice, you would create a custom message type
        // or use existing ROS message types to represent environment state
    }
}
```

Creating well-designed virtual environments is crucial for effective robotics simulation. These environments should accurately represent real-world conditions while maintaining the performance needed for real-time simulation and testing.