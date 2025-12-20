# Creating Basic Unity Scenes with Terrain and Objects

This document provides detailed guidance on creating basic Unity scenes with terrain and objects for robotics simulation, covering the fundamental techniques needed to build effective virtual environments.

## Understanding Unity Scene Structure

### Basic Scene Components

A Unity scene consists of several key components:

**Hierarchy**: The organizational structure of all objects in the scene
**Inspector**: Properties and components of the selected object
**Scene View**: The visual editor for positioning and manipulating objects
**Game View**: The camera's perspective of the scene
**Project Window**: Assets, scripts, and resources for the scene

### Setting Up Your First Scene

1. **Create a New Scene**:
   - File → New Scene
   - Select 3D (for robotics simulation)
   - Save the scene (Ctrl/Cmd + S) with an appropriate name

2. **Configure Scene Settings**:
   - Set the scene scale appropriately (1 Unity unit = 1 meter for robotics)
   - Configure physics settings (Edit → Project Settings → Physics)
   - Set default gravity to match real-world conditions (-9.81 m/s² on Y-axis)

## Creating and Configuring Terrain

### Adding Terrain to Your Scene

1. **Create Terrain**:
   - Right-click in Hierarchy window
   - Go to 3D Object → Terrain
   - A default terrain will be created at position (0, 0, 0)

2. **Configure Terrain Settings**:
   - Select the Terrain object in the Hierarchy
   - In the Inspector, you'll see the Terrain component
   - Adjust the terrain size using the "Terrain Data" settings
   - For robotics simulation, a common size is 100x100 meters

### Terrain Tools and Sculpting

**Raise/Lower Terrain**:
- Select the "Raise/Lower Terrain" tool from the Terrain toolbar
- Adjust brush size and strength in the Inspector
- Click and drag in the Scene View to modify terrain height
- Use this to create hills, valleys, and natural-looking terrain

**Paint Texture**:
- Select the "Paint Texture" tool
- Choose from available textures or import your own
- Paint different surface types (grass, dirt, rock, etc.)
- This affects both visual appearance and material properties

**Paint Trees**:
- Use the "Paint Trees" tool to add vegetation
- Import tree prefabs or use default ones
- Paint trees onto the terrain for realistic environments

**Paint Details**:
- Add grass, flowers, and other small details
- Enhance the visual realism of your environment
- Be mindful of performance impact

### Terrain Optimization for Robotics

For robotics simulation, consider these terrain settings:

```csharp
// Example script for configuring terrain programmatically
using UnityEngine;

public class TerrainConfigurator : MonoBehaviour
{
    public Terrain terrain;
    public float terrainSize = 100f; // 100m x 100m
    public float terrainHeight = 10f; // 10m maximum height variation

    void Start()
    {
        if (terrain == null)
        {
            terrain = GetComponent<Terrain>();
        }

        // Configure terrain size
        terrain.terrainData.size = new Vector3(terrainSize, terrainHeight, terrainSize);

        // Configure terrain resolution for performance
        terrain.terrainData.heightmapResolution = 513; // Good balance of detail and performance
        terrain.terrainData.alphamapResolution = 256;  // Texture resolution
        terrain.terrainData.detailResolution = 256;    // Detail object resolution
    }

    // Method to create a flat terrain (useful for testing)
    public void CreateFlatTerrain()
    {
        TerrainData terrainData = terrain.terrainData;
        float[,] heights = new float[terrainData.heightmapResolution, terrainData.heightmapResolution];

        // Set all height values to 0 (flat terrain)
        for (int i = 0; i < terrainData.heightmapResolution; i++)
        {
            for (int j = 0; j < terrainData.heightmapResolution; j++)
            {
                heights[i, j] = 0f;
            }
        }

        terrainData.SetHeights(0, 0, heights);
    }
}
```

## Adding Static Objects

### Using Primitive Objects

Unity provides several primitive objects that are perfect for basic environment creation:

**Cube** (GameObject → 3D Object → Cube):
- Ideal for buildings, boxes, walls
- Easy to scale and position
- Good collision detection

**Sphere** (GameObject → 3D Object → Sphere):
- Useful for obstacles, balls, rounded objects
- Natural collision shape
- Good for physics testing

**Cylinder** (GameObject → 3D Object → Cylinder):
- Pillars, columns, barrels
- Can be oriented horizontally as tubes
- Good for circular obstacles

**Plane** (GameObject → 3D Object → Plane):
- Additional flat surfaces
- Custom floor or platform creation
- Can be used for custom terrain sections

### Positioning and Scaling Objects

1. **Select the Object**: Click on the object in the Hierarchy or Scene View
2. **Use Transform Tools**:
   - Move Tool (W): Translate the object in 3D space
   - Rotate Tool (E): Rotate the object around its axes
   - Scale Tool (R): Change the object's size
3. **Precise Positioning**: Use the Inspector to enter exact values

### Creating Complex Structures

**Combining Primitives**:
```csharp
using UnityEngine;

public class StructureBuilder : MonoBehaviour
{
    public GameObject BuildSimpleHouse(Vector3 position)
    {
        // Create main structure
        GameObject house = new GameObject("House");
        house.transform.position = position;

        // Main building
        GameObject mainBuilding = GameObject.CreatePrimitive(PrimitiveType.Cube);
        mainBuilding.transform.SetParent(house.transform);
        mainBuilding.transform.localPosition = Vector3.zero;
        mainBuilding.transform.localScale = new Vector3(5f, 3f, 4f);
        mainBuilding.GetComponent<Renderer>().material.color = Color.white;

        // Roof
        GameObject roof = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        roof.transform.SetParent(house.transform);
        roof.transform.localPosition = new Vector3(0, 1.5f, 0);
        roof.transform.localScale = new Vector3(6f, 1f, 5f);
        roof.transform.rotation = Quaternion.Euler(90, 0, 0); // Rotate to be flat
        roof.GetComponent<Renderer>().material.color = Color.red;

        // Door
        GameObject door = GameObject.CreatePrimitive(PrimitiveType.Cube);
        door.transform.SetParent(house.transform);
        door.transform.localPosition = new Vector3(0, -0.5f, 2.01f); // Slightly in front of wall
        door.transform.localScale = new Vector3(1.5f, 2f, 0.1f);
        door.GetComponent<Renderer>().material.color = Color.brown;

        return house;
    }
}
```

## Material and Visual Properties

### Applying Materials

Materials control how objects appear and interact physically:

1. **Create New Material**:
   - In Project window, right-click → Create → Material
   - Name the material appropriately (e.g., "Concrete", "Metal", "Wood")

2. **Configure Material Properties**:
   - **Albedo**: Base color of the material
   - **Metallic**: How metallic the surface appears (0-1)
   - **Smoothness**: How smooth/reflective the surface is (0-1)
   - **Normal Map**: Surface detail without geometry
   - **Occlusion**: Ambient light occlusion

### Physics Materials

Physics materials control friction and bounciness:

1. **Create Physics Material**:
   - Right-click in Project → Create → Physics Material
   - Adjust Dynamic Friction, Static Friction, and Bounciness

2. **Apply to Objects**:
   - Select the object's Collider component
   - Assign the Physics Material in the Material field

```csharp
using UnityEngine;

public class MaterialApplier : MonoBehaviour
{
    public Material[] materials;
    public PhysicMaterial[] physicMaterials;

    public void ApplyMaterial(GameObject target, int materialIndex, int physicIndex = -1)
    {
        if (materialIndex < materials.Length)
        {
            target.GetComponent<Renderer>().material = materials[materialIndex];
        }

        if (physicIndex >= 0 && physicIndex < physicMaterials.Length)
        {
            target.GetComponent<Collider>().material = physicMaterials[physicIndex];
        }
    }

    // Example: Apply different materials for different robot interaction zones
    public void SetupRobotInteractionZones()
    {
        GameObject[] zones = GameObject.FindGameObjectsWithTag("InteractionZone");

        foreach (GameObject zone in zones)
        {
            // Apply high-friction material for robot stopping zones
            ApplyMaterial(zone, 0, 0); // Visual material 0, Physics material 0
        }
    }
}
```

## Lighting Setup

### Basic Lighting Configuration

1. **Directional Light** (Simulates Sun):
   - Usually present by default
   - Provides overall scene illumination
   - Can cast shadows

2. **Position and Configure**:
   - Rotate the light to desired angle (typically 30-60 degrees)
   - Adjust intensity (default 1 is good for outdoors)
   - Configure shadow settings for realism

```csharp
using UnityEngine;

public class LightingController : MonoBehaviour
{
    public Light mainLight;

    void Start()
    {
        if (mainLight == null)
        {
            // Find the main directional light
            mainLight = GameObject.Find("Directional Light").GetComponent<Light>();
        }

        ConfigureOutdoorLighting();
    }

    public void ConfigureOutdoorLighting()
    {
        if (mainLight != null)
        {
            mainLight.type = LightType.Directional;
            mainLight.color = Color.white;
            mainLight.intensity = 1.0f;
            mainLight.shadows = LightShadows.Soft;
            mainLight.shadowStrength = 0.8f;

            // Set to typical outdoor angle
            mainLight.transform.rotation = Quaternion.Euler(50, -30, 0);
        }
    }

    public void ConfigureIndoorLighting()
    {
        if (mainLight != null)
        {
            mainLight.color = new Color(0.95f, 0.9f, 0.8f); // Warm white
            mainLight.intensity = 0.8f;
            mainLight.shadows = LightShadows.Hard;
            mainLight.transform.rotation = Quaternion.Euler(130, -30, 0);
        }
    }
}
```

## Scene Organization and Management

### Using Layers and Tags

**Layers**: Used for rendering and physics calculations
- Assign objects to different layers for specific processing
- Useful for robot sensors that need to ignore certain objects

**Tags**: Used for finding and categorizing objects
- Assign meaningful tags (e.g., "Robot", "Obstacle", "Ground")
- Use GameObject.FindWithTag() for easy object access

### Prefabs for Reusability

Create prefabs for commonly used objects:

1. **Create the Object**: Build your object with all components
2. **Drag to Project**: Drag from Hierarchy to Project window
3. **Use in Scene**: Drag prefab from Project to Scene to instantiate

```csharp
using UnityEngine;

public class EnvironmentBuilder : MonoBehaviour
{
    public GameObject[] prefabs; // Store your prefabs here
    public Transform environmentParent; // Parent object for organization

    public void CreateObstacleCourse()
    {
        // Create a simple obstacle course
        for (int i = 0; i < 5; i++)
        {
            Vector3 position = new Vector3(i * 3f, 0, 0);

            // Alternate between different obstacle types
            GameObject obstacle = Instantiate(prefabs[i % prefabs.Length], position, Quaternion.identity);
            obstacle.transform.SetParent(environmentParent);

            // Add some variation
            obstacle.transform.localScale = new Vector3(
                Random.Range(0.8f, 1.2f),
                Random.Range(0.5f, 2.0f),
                Random.Range(0.8f, 1.2f)
            );
        }
    }

    public void OrganizeScene()
    {
        // Create organizational parent objects
        if (environmentParent == null)
        {
            environmentParent = new GameObject("Environment").transform;
        }

        GameObject staticObjects = new GameObject("Static Objects");
        staticObjects.transform.SetParent(environmentParent);

        GameObject dynamicObjects = new GameObject("Dynamic Objects");
        dynamicObjects.transform.SetParent(environmentParent);

        GameObject lighting = new GameObject("Lighting");
        lighting.transform.SetParent(environmentParent);
    }
}
```

## Performance Optimization

### Static vs Dynamic Objects

**Mark Static Objects**:
- Select objects that won't move during simulation
- Check "Static" in the Inspector
- This enables Unity optimizations like static batching

### Object Pooling for Dynamic Objects

```csharp
using System.Collections.Generic;
using UnityEngine;

public class ObjectPooler : MonoBehaviour
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

## Testing Your Scene

### Basic Functionality Tests

1. **Physics Test**: Drop objects to verify gravity and collision
2. **Navigation Test**: Move objects to ensure no unexpected collisions
3. **Performance Test**: Monitor frame rate with all objects active
4. **Scale Test**: Verify objects are appropriately sized for robotics

### Scene Validation Script

```csharp
using UnityEngine;

public class SceneValidator : MonoBehaviour
{
    public void ValidateScene()
    {
        Debug.Log("=== Scene Validation ===");

        // Check for common issues
        CheckMissingColliders();
        CheckObjectOverlap();
        CheckLighting();

        Debug.Log("Scene validation complete.");
    }

    void CheckMissingColliders()
    {
        GameObject[] allObjects = GameObject.FindGameObjectsWithTag("Untagged");
        int missingColliderCount = 0;

        foreach (GameObject obj in allObjects)
        {
            if (obj.GetComponent<Collider>() == null)
            {
                // Only warn for objects that should have colliders
                if (obj.GetComponent<Renderer>() != null)
                {
                    missingColliderCount++;
                    Debug.LogWarning($"Object {obj.name} has no collider but has a renderer.");
                }
            }
        }

        if (missingColliderCount == 0)
        {
            Debug.Log("✓ All objects with renderers have colliders.");
        }
    }

    void CheckObjectOverlap()
    {
        Collider[] allColliders = FindObjectsOfType<Collider>();
        int overlapCount = 0;

        for (int i = 0; i < allColliders.Length; i++)
        {
            for (int j = i + 1; j < allColliders.Length; j++)
            {
                if (allColliders[i].bounds.Intersects(allColliders[j].bounds))
                {
                    overlapCount++;
                    Debug.LogWarning($"Overlap detected between {allColliders[i].name} and {allColliders[j].name}");
                }
            }
        }

        if (overlapCount == 0)
        {
            Debug.Log("✓ No object overlaps detected.");
        }
    }

    void CheckLighting()
    {
        Light[] lights = FindObjectsOfType<Light>();
        if (lights.Length == 0)
        {
            Debug.LogWarning("No lights found in scene - environment may be too dark.");
        }
        else
        {
            Debug.Log($"✓ Found {lights.Length} lights in scene.");
        }
    }
}
```

## Best Practices for Robotics Simulation

### Scale and Proportion
- Maintain 1:1 scale (1 Unity unit = 1 meter)
- Use realistic object sizes for accurate sensor simulation
- Consider robot dimensions when placing obstacles

### Material Properties
- Use realistic friction coefficients for accurate physics
- Consider how materials will appear to robot sensors
- Balance visual quality with performance requirements

### Performance Considerations
- Optimize geometry for real-time performance
- Use appropriate texture resolutions
- Implement LOD systems for complex objects
- Test performance with target frame rates

Creating basic Unity scenes with terrain and objects forms the foundation for complex robotics simulation environments. These techniques provide the building blocks for more advanced features like dynamic elements, sensor simulation, and ROS integration.