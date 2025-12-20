# Complex Virtual Environments for Robot Testing

This document provides comprehensive guidance on creating complex virtual environments for comprehensive robot testing, including detailed scenarios that challenge various robot capabilities.

## Understanding Complex Environments

### Characteristics of Complex Environments

**Multi-Modal Challenges**:
- Environments that test multiple robot capabilities simultaneously
- Scenarios requiring navigation, manipulation, and perception
- Realistic task combinations that mirror real-world applications

**Dynamic Elements**:
- Moving obstacles and changing environmental conditions
- Time-dependent challenges and events
- Interactive elements that respond to robot actions

**Environmental Complexity**:
- Varied terrain types and surface properties
- Multiple rooms, levels, or areas with different characteristics
- Complex lighting and weather conditions

### Planning Complex Environments

Before building, consider:
- **Primary Objectives**: What robot capabilities will be tested?
- **Secondary Challenges**: What additional skills should be exercised?
- **Realism vs. Control**: Balance realistic scenarios with controlled testing
- **Scalability**: Can the environment be modified for different difficulty levels?
- **Repeatability**: Can tests be repeated with consistent conditions?

## Warehouse Environment Example

### Overview
A realistic warehouse environment for testing logistics robots, including shelves, pallets, forklifts, and dynamic obstacles.

### Implementation

```csharp
using UnityEngine;
using System.Collections.Generic;

public class WarehouseEnvironment : MonoBehaviour
{
    [Header("Environment Dimensions")]
    public Vector2 warehouseSize = new Vector2(50f, 30f);
    public float ceilingHeight = 8f;

    [Header("Shelving Configuration")]
    public int shelfRows = 8;
    public int shelfDepth = 3;
    public float aisleWidth = 3f;
    public float shelfHeight = 3f;
    public float shelfWidth = 1.5f;
    public float shelfDepthSize = 0.8f;

    [Header("Obstacle Configuration")]
    public GameObject[] obstaclePrefabs;
    public int dynamicObstacleCount = 5;
    public float obstacleSpeed = 1f;

    [Header("Robot Spawn Points")]
    public List<Transform> spawnPoints = new List<Transform>();

    private List<GameObject> environmentObjects = new List<GameObject>();

    void Start()
    {
        BuildWarehouse();
        AddObstacles();
        SetupRobotSpawns();
    }

    void BuildWarehouse()
    {
        // Create floor
        CreateFloor();

        // Create walls
        CreateWalls();

        // Create shelving units
        CreateShelving();

        // Create loading dock
        CreateLoadingDock();

        // Add lighting
        AddWarehouseLighting();
    }

    void CreateFloor()
    {
        GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
        floor.transform.position = new Vector3(warehouseSize.x / 2, 0, warehouseSize.y / 2);
        floor.transform.localScale = new Vector3(warehouseSize.x / 10, 1, warehouseSize.y / 10);
        floor.name = "WarehouseFloor";

        // Apply warehouse-appropriate material
        Renderer floorRenderer = floor.GetComponent<Renderer>();
        if (floorRenderer != null)
        {
            Material concreteMat = new Material(Shader.Find("Standard"));
            concreteMat.color = new Color(0.6f, 0.6f, 0.6f);
            concreteMat.SetFloat("_Metallic", 0.1f);
            concreteMat.SetFloat("_Smoothness", 0.2f);
            floorRenderer.material = concreteMat;
        }

        environmentObjects.Add(floor);
    }

    void CreateWalls()
    {
        float wallHeight = ceilingHeight;

        // Create 4 walls
        CreateWall(new Vector3(0, wallHeight / 2, warehouseSize.y / 2),
                  new Vector3(0.1f, wallHeight, warehouseSize.y));
        CreateWall(new Vector3(warehouseSize.x, wallHeight / 2, warehouseSize.y / 2),
                  new Vector3(0.1f, wallHeight, warehouseSize.y));
        CreateWall(new Vector3(warehouseSize.x / 2, wallHeight / 2, 0),
                  new Vector3(warehouseSize.x, wallHeight, 0.1f));
        CreateWall(new Vector3(warehouseSize.x / 2, wallHeight / 2, warehouseSize.y),
                  new Vector3(warehouseSize.x, wallHeight, 0.1f));
    }

    GameObject CreateWall(Vector3 position, Vector3 size)
    {
        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.transform.position = position;
        wall.transform.localScale = size;
        wall.name = "Wall";

        // Apply wall material
        Renderer wallRenderer = wall.GetComponent<Renderer>();
        if (wallRenderer != null)
        {
            Material wallMat = new Material(Shader.Find("Standard"));
            wallMat.color = new Color(0.8f, 0.8f, 0.8f);
            wallRenderer.material = wallMat;
        }

        // Remove collider if it's a door opening
        // (This would be customized based on warehouse design)

        environmentObjects.Add(wall);
        return wall;
    }

    void CreateShelving()
    {
        float rowSpacing = (warehouseSize.x - (shelfDepth * shelfDepthSize)) / (shelfRows + 1);
        float shelfSpacing = (warehouseSize.y - 4f) / (shelfDepth + 1); // Leave space for aisles

        for (int row = 0; row < shelfRows; row++)
        {
            for (int depth = 0; depth < shelfDepth; depth++)
            {
                Vector3 shelfPos = new Vector3(
                    rowSpacing + row * (rowSpacing + shelfDepthSize),
                    shelfHeight / 2,
                    2f + shelfSpacing + depth * (shelfSpacing + shelfWidth)
                );

                CreateShelfUnit(shelfPos, row, depth);
            }
        }
    }

    GameObject CreateShelfUnit(Vector3 position, int row, int depth)
    {
        GameObject shelf = new GameObject($"Shelf_R{row}_D{depth}");
        shelf.transform.position = position;

        // Main shelf structure
        GameObject shelfMain = GameObject.CreatePrimitive(PrimitiveType.Cube);
        shelfMain.transform.SetParent(shelf.transform);
        shelfMain.transform.localPosition = Vector3.zero;
        shelfMain.transform.localScale = new Vector3(shelfDepthSize, shelfHeight, shelfWidth);
        shelfMain.GetComponent<Renderer>().material.color = Color.grey;

        // Add shelf levels
        int levels = 4;
        float levelHeight = shelfHeight / levels;
        for (int i = 1; i < levels; i++)
        {
            GameObject shelfLevel = GameObject.CreatePrimitive(PrimitiveType.Cube);
            shelfLevel.transform.SetParent(shelf.transform);
            shelfLevel.transform.localPosition = new Vector3(0, i * levelHeight, 0);
            shelfLevel.transform.localScale = new Vector3(shelfDepthSize, 0.05f, shelfWidth);
            shelfLevel.GetComponent<Renderer>().material.color = Color.black;
        }

        // Add collision
        BoxCollider mainCollider = shelfMain.GetComponent<BoxCollider>();
        if (mainCollider != null)
        {
            mainCollider.material = CreatePhysicsMaterial("Shelf", 0.6f, 0.5f, 0.1f);
        }

        environmentObjects.Add(shelf);
        return shelf;
    }

    void CreateLoadingDock()
    {
        // Create loading dock area with dock level and truck space
        GameObject dockArea = GameObject.CreatePrimitive(PrimitiveType.Cube);
        dockArea.transform.position = new Vector3(warehouseSize.x - 5f, 0.5f, warehouseSize.y - 3f);
        dockArea.transform.localScale = new Vector3(8f, 1f, 6f);
        dockArea.GetComponent<Renderer>().material.color = Color.grey;

        // Create dock level (slightly higher than floor)
        GameObject dockLevel = GameObject.CreatePrimitive(PrimitiveType.Cube);
        dockLevel.transform.position = new Vector3(warehouseSize.x - 5f, 0.8f, warehouseSize.y - 3f);
        dockLevel.transform.localScale = new Vector3(6f, 0.3f, 4f);
        dockLevel.GetComponent<Renderer>().material.color = Color.grey;

        environmentObjects.Add(dockArea);
        environmentObjects.Add(dockLevel);
    }

    void AddWarehouseLighting()
    {
        // Create overhead lighting similar to warehouse fixtures
        int lightsX = Mathf.CeilToInt(warehouseSize.x / 8f);
        int lightsZ = Mathf.CeilToInt(warehouseSize.y / 8f);

        for (int x = 0; x < lightsX; x++)
        {
            for (int z = 0; z < lightsZ; z++)
            {
                Vector3 lightPos = new Vector3(
                    4f + x * 8f,
                    ceilingHeight - 0.5f,
                    4f + z * 8f
                );

                CreateWarehouseLight(lightPos);
            }
        }
    }

    GameObject CreateWarehouseLight(Vector3 position)
    {
        GameObject lightObj = new GameObject("WarehouseLight");
        lightObj.transform.position = position;

        Light light = lightObj.AddComponent<Light>();
        light.type = LightType.Point;
        light.color = new Color(0.95f, 0.95f, 1.0f); // Cool white
        light.intensity = 1.5f;
        light.range = 10f;
        light.shadows = LightShadows.Hard;

        environmentObjects.Add(lightObj);
        return lightObj;
    }

    void AddObstacles()
    {
        // Add static obstacles (pallets, equipment)
        AddStaticObstacles();

        // Add dynamic obstacles (people, automated carts)
        AddDynamicObstacles();
    }

    void AddStaticObstacles()
    {
        // Create pallets
        for (int i = 0; i < 10; i++)
        {
            Vector3 pos = new Vector3(
                Random.Range(5f, warehouseSize.x - 5f),
                0.1f,
                Random.Range(5f, warehouseSize.y - 5f)
            );

            GameObject pallet = GameObject.CreatePrimitive(PrimitiveType.Cube);
            pallet.transform.position = pos;
            pallet.transform.localScale = new Vector3(1.2f, 0.15f, 1.0f);
            pallet.GetComponent<Renderer>().material.color = Color.brown;
            pallet.name = $"Pallet_{i}";

            // Add collision properties
            BoxCollider collider = pallet.GetComponent<BoxCollider>();
            if (collider != null)
            {
                collider.material = CreatePhysicsMaterial("Pallet", 0.5f, 0.4f, 0.05f);
            }

            environmentObjects.Add(pallet);
        }
    }

    void AddDynamicObstacles()
    {
        for (int i = 0; i < dynamicObstacleCount; i++)
        {
            Vector3 spawnPos = new Vector3(
                Random.Range(2f, warehouseSize.x - 2f),
                0.5f,
                Random.Range(2f, warehouseSize.y - 2f)
            );

            GameObject obstacle = CreateDynamicObstacle(spawnPos, i);
            if (obstacle != null)
            {
                // Add movement script
                WarehousePedestrian pedestrian = obstacle.AddComponent<WarehousePedestrian>();
                pedestrian.SetEnvironmentBounds(new Vector2(warehouseSize.x, warehouseSize.y));
                pedestrian.SetMovementSpeed(obstacleSpeed);
            }
        }
    }

    GameObject CreateDynamicObstacle(Vector3 position, int id)
    {
        if (obstaclePrefabs.Length == 0) return null;

        GameObject obstacle = Instantiate(obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)]);
        obstacle.transform.position = position;
        obstacle.name = $"DynamicObstacle_{id}";

        // Add basic collision
        CapsuleCollider capsule = obstacle.GetComponent<CapsuleCollider>();
        if (capsule != null)
        {
            capsule.material = CreatePhysicsMaterial("Obstacle", 0.3f, 0.3f, 0.2f);
        }

        return obstacle;
    }

    void SetupRobotSpawns()
    {
        // Create predefined spawn points for robots
        // Entry point
        GameObject entrySpawn = new GameObject("RobotSpawn_Entry");
        entrySpawn.transform.position = new Vector3(2f, 0.5f, 2f);
        spawnPoints.Add(entrySpawn);

        // Center point
        GameObject centerSpawn = new GameObject("RobotSpawn_Center");
        centerSpawn.transform.position = new Vector3(warehouseSize.x / 2, 0.5f, warehouseSize.y / 2);
        spawnPoints.Add(centerSpawn);

        // Loading dock area
        GameObject dockSpawn = new GameObject("RobotSpawn_Dock");
        dockSpawn.transform.position = new Vector3(warehouseSize.x - 8f, 0.5f, warehouseSize.y - 5f);
        spawnPoints.Add(dockSpawn);
    }

    PhysicMaterial CreatePhysicsMaterial(string name, float staticFriction, float dynamicFriction, float bounciness)
    {
        PhysicMaterial mat = new PhysicMaterial(name);
        mat.staticFriction = staticFriction;
        mat.dynamicFriction = dynamicFriction;
        mat.bounciness = bounciness;
        return mat;
    }

    // Public methods for environment control
    public Vector2 GetWarehouseSize()
    {
        return warehouseSize;
    }

    public List<Transform> GetSpawnPoints()
    {
        return spawnPoints;
    }

    public Transform GetRandomSpawnPoint()
    {
        if (spawnPoints.Count > 0)
        {
            return spawnPoints[Random.Range(0, spawnPoints.Count)];
        }
        return null;
    }
}

// Supporting script for dynamic obstacles
public class WarehousePedestrian : MonoBehaviour
{
    public Vector2 bounds;
    public float speed = 1f;
    public float changeDirectionInterval = 3f;

    private Vector3 targetDirection;
    private float lastDirectionChange;

    void Start()
    {
        ChangeDirection();
        lastDirectionChange = Time.time;
    }

    void Update()
    {
        if (Time.time - lastDirectionChange > changeDirectionInterval)
        {
            ChangeDirection();
            lastDirectionChange = Time.time;
        }

        Move();
        CheckBounds();
    }

    void ChangeDirection()
    {
        targetDirection = new Vector3(Random.Range(-1f, 1f), 0, Random.Range(-1f, 1f)).normalized;
    }

    void Move()
    {
        transform.Translate(targetDirection * speed * Time.deltaTime);
    }

    void CheckBounds()
    {
        Vector3 pos = transform.position;
        pos.x = Mathf.Clamp(pos.x, 1f, bounds.x - 1f);
        pos.z = Mathf.Clamp(pos.z, 1f, bounds.y - 1f);
        transform.position = pos;
    }

    public void SetEnvironmentBounds(Vector2 newBounds)
    {
        bounds = newBounds;
    }

    public void SetMovementSpeed(float newSpeed)
    {
        speed = newSpeed;
    }
}
```

## Urban Environment Example

### Overview
An urban environment for testing navigation, obstacle avoidance, and interaction in city-like settings.

### Implementation

```csharp
using UnityEngine;
using System.Collections.Generic;

public class UrbanEnvironment : MonoBehaviour
{
    [Header("Environment Configuration")]
    public Vector2 cityBlockSize = new Vector2(30f, 30f);
    public int blocksX = 5;
    public int blocksY = 5;
    public float streetWidth = 8f;

    [Header("Building Configuration")]
    public float minBuildingHeight = 5f;
    public float maxBuildingHeight = 20f;
    public float buildingDepth = 15f;

    [Header("Traffic Elements")]
    public GameObject[] vehiclePrefabs;
    public int trafficDensity = 3;
    public float trafficSpeed = 5f;

    [Header("Pedestrian Areas")]
    public int pedestrianCount = 10;
    public float pedestrianSpeed = 1.5f;

    private List<GameObject> cityObjects = new List<GameObject>();

    void Start()
    {
        BuildCityGrid();
        AddBuildings();
        AddTraffic();
        AddPedestrians();
        AddInfrastructure();
    }

    void BuildCityGrid()
    {
        // Create the basic street grid
        for (int x = 0; x <= blocksX; x++)
        {
            for (int z = 0; z < blocksY; z++)
            {
                CreateStreetSegment(
                    new Vector3(x * (cityBlockSize.x + streetWidth), 0, z * (cityBlockSize.y + streetWidth)),
                    new Vector3(streetWidth, 0.1f, cityBlockSize.y),
                    "VerticalStreet"
                );
            }
        }

        for (int z = 0; z <= blocksY; z++)
        {
            for (int x = 0; x < blocksX; x++)
            {
                CreateStreetSegment(
                    new Vector3(x * (cityBlockSize.x + streetWidth), 0, z * (cityBlockSize.y + streetWidth)),
                    new Vector3(cityBlockSize.x, 0.1f, streetWidth),
                    "HorizontalStreet"
                );
            }
        }
    }

    GameObject CreateStreetSegment(Vector3 position, Vector3 size, string name)
    {
        GameObject street = GameObject.CreatePrimitive(PrimitiveType.Cube);
        street.transform.position = new Vector3(position.x + size.x / 2, position.y, position.z + size.z / 2);
        street.transform.localScale = size;
        street.name = name;

        // Apply road material
        Renderer streetRenderer = street.GetComponent<Renderer>();
        if (streetRenderer != null)
        {
            Material roadMat = new Material(Shader.Find("Standard"));
            roadMat.color = new Color(0.2f, 0.2f, 0.2f);
            roadMat.SetFloat("_Metallic", 0.1f);
            roadMat.SetFloat("_Smoothness", 0.1f);
            streetRenderer.material = roadMat;
        }

        cityObjects.Add(street);
        return street;
    }

    void AddBuildings()
    {
        for (int x = 0; x < blocksX; x++)
        {
            for (int z = 0; z < blocksY; z++)
            {
                // Randomly decide whether to place a building in this block
                if (Random.value > 0.3f) // 70% chance of building
                {
                    Vector3 blockCenter = new Vector3(
                        x * (cityBlockSize.x + streetWidth) + cityBlockSize.x / 2,
                        0,
                        z * (cityBlockSize.y + streetWidth) + cityBlockSize.y / 2
                    );

                    CreateBuilding(blockCenter, x, z);
                }
            }
        }
    }

    void CreateBuilding(Vector3 position, int blockX, int blockZ)
    {
        float height = Random.Range(minBuildingHeight, maxBuildingHeight);
        float width = Random.Range(cityBlockSize.x * 0.6f, cityBlockSize.x * 0.9f);
        float depth = Random.Range(cityBlockSize.y * 0.6f, cityBlockSize.y * 0.9f);

        GameObject building = GameObject.CreatePrimitive(PrimitiveType.Cube);
        building.transform.position = new Vector3(position.x, height / 2, position.z);
        building.transform.localScale = new Vector3(width, height, depth);
        building.name = $"Building_{blockX}_{blockZ}";

        // Apply building material
        Renderer buildingRenderer = building.GetComponent<Renderer>();
        if (buildingRenderer != null)
        {
            Material buildingMat = new Material(Shader.Find("Standard"));
            buildingMat.color = GetRandomBuildingColor();
            buildingMat.SetFloat("_Metallic", 0.2f);
            buildingMat.SetFloat("_Smoothness", 0.3f);
            buildingRenderer.material = buildingMat;
        }

        cityObjects.Add(building);
    }

    Color GetRandomBuildingColor()
    {
        Color[] colors = {
            new Color(0.7f, 0.7f, 0.8f), // Light gray
            new Color(0.6f, 0.5f, 0.4f), // Brown
            new Color(0.5f, 0.6f, 0.7f), // Blue-gray
            new Color(0.8f, 0.7f, 0.6f), // Light brown
            new Color(0.4f, 0.5f, 0.6f)  // Dark blue-gray
        };

        return colors[Random.Range(0, colors.Length)];
    }

    void AddTraffic()
    {
        // Create traffic paths along streets
        for (int i = 0; i < trafficDensity; i++)
        {
            // Choose a random street
            bool isHorizontal = Random.value > 0.5f;
            int blockIndex = Random.Range(0, isHorizontal ? blocksX : blocksY);

            Vector3 startPos;
            Vector3 endPos;

            if (isHorizontal)
            {
                startPos = new Vector3(
                    0,
                    0.5f,
                    blockIndex * (cityBlockSize.y + streetWidth) + cityBlockSize.y / 2
                );
                endPos = new Vector3(
                    blocksX * (cityBlockSize.x + streetWidth),
                    0.5f,
                    blockIndex * (cityBlockSize.y + streetWidth) + cityBlockSize.y / 2
                );
            }
            else
            {
                startPos = new Vector3(
                    blockIndex * (cityBlockSize.x + streetWidth) + cityBlockSize.x / 2,
                    0.5f,
                    0
                );
                endPos = new Vector3(
                    blockIndex * (cityBlockSize.x + streetWidth) + cityBlockSize.x / 2,
                    0.5f,
                    blocksY * (cityBlockSize.y + streetWidth)
                );
            }

            CreateTrafficVehicle(startPos, endPos, i);
        }
    }

    void CreateTrafficVehicle(Vector3 startPos, Vector3 endPos, int id)
    {
        if (vehiclePrefabs.Length == 0) return;

        GameObject vehicle = Instantiate(vehiclePrefabs[Random.Range(0, vehiclePrefabs.Length)]);
        vehicle.transform.position = startPos;
        vehicle.name = $"TrafficVehicle_{id}";

        // Add traffic controller
        UrbanTrafficController trafficCtrl = vehicle.AddComponent<UrbanTrafficController>();
        trafficCtrl.SetRoute(startPos, endPos, trafficSpeed);

        cityObjects.Add(vehicle);
    }

    void AddPedestrians()
    {
        for (int i = 0; i < pedestrianCount; i++)
        {
            Vector3 spawnPos = new Vector3(
                Random.Range(0, blocksX * (cityBlockSize.x + streetWidth)),
                0.5f,
                Random.Range(0, blocksY * (cityBlockSize.y + streetWidth))
            );

            CreatePedestrian(spawnPos, i);
        }
    }

    void CreatePedestrian(Vector3 position, int id)
    {
        GameObject pedestrian = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        pedestrian.transform.position = position;
        pedestrian.transform.localScale = new Vector3(0.4f, 0.4f, 0.4f);
        pedestrian.name = $"Pedestrian_{id}";
        pedestrian.GetComponent<Renderer>().material.color = Random.ColorHSV();

        // Add pedestrian controller
        UrbanPedestrianController pedCtrl = pedestrian.AddComponent<UrbanPedestrianController>();
        pedCtrl.SetEnvironmentSize(new Vector2(
            blocksX * (cityBlockSize.x + streetWidth),
            blocksY * (cityBlockSize.y + streetWidth)
        ));
        pedCtrl.SetMovementSpeed(pedestrianSpeed);

        cityObjects.Add(pedestrian);
    }

    void AddInfrastructure()
    {
        // Add traffic lights
        AddTrafficLights();

        // Add crosswalks
        AddCrosswalks();

        // Add street lights
        AddStreetLights();
    }

    void AddTrafficLights()
    {
        for (int x = 0; x < blocksX; x++)
        {
            for (int z = 0; z < blocksY; z++)
            {
                Vector3 lightPos = new Vector3(
                    x * (cityBlockSize.x + streetWidth) + cityBlockSize.x,
                    5f,
                    z * (cityBlockSize.y + streetWidth) + cityBlockSize.y
                );

                CreateTrafficLight(lightPos, x, z);
            }
        }
    }

    GameObject CreateTrafficLight(Vector3 position, int x, int z)
    {
        GameObject light = new GameObject($"TrafficLight_{x}_{z}");
        light.transform.position = position;

        // Create light pole
        GameObject pole = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        pole.transform.SetParent(light.transform);
        pole.transform.localPosition = Vector3.zero;
        pole.transform.localScale = new Vector3(0.2f, 4f, 0.2f);
        pole.GetComponent<Renderer>().material.color = Color.black;

        // Create light housing
        GameObject housing = GameObject.CreatePrimitive(PrimitiveType.Cube);
        housing.transform.SetParent(light.transform);
        housing.transform.localPosition = new Vector3(0, 3.5f, 0);
        housing.transform.localScale = new Vector3(0.8f, 0.8f, 0.3f);
        housing.GetComponent<Renderer>().material.color = Color.black;

        // Create actual lights (simplified)
        for (int i = 0; i < 3; i++)
        {
            GameObject lightBulb = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            lightBulb.transform.SetParent(light.transform);
            lightBulb.transform.localPosition = new Vector3(0, 3.5f - i * 0.4f, 0.2f);
            lightBulb.transform.localScale = new Vector3(0.15f, 0.15f, 0.15f);
            lightBulb.GetComponent<Renderer>().material.color = i == 0 ? Color.red : (i == 1 ? Color.yellow : Color.green);
        }

        cityObjects.Add(light);
        return light;
    }

    void AddCrosswalks()
    {
        // Add crosswalks at intersections
        for (int x = 0; x < blocksX; x++)
        {
            for (int z = 0; z < blocksY; z++)
            {
                Vector3 crosswalkPos = new Vector3(
                    x * (cityBlockSize.x + streetWidth) + cityBlockSize.x,
                    0.01f,
                    z * (cityBlockSize.y + streetWidth) + cityBlockSize.y
                );

                CreateCrosswalk(crosswalkPos);
            }
        }
    }

    GameObject CreateCrosswalk(Vector3 position)
    {
        GameObject crosswalk = GameObject.CreatePrimitive(PrimitiveType.Cube);
        crosswalk.transform.position = position;
        crosswalk.transform.localScale = new Vector3(4f, 0.02f, 4f);
        crosswalk.name = "Crosswalk";

        // Apply crosswalk material (white stripes)
        Renderer crosswalkRenderer = crosswalk.GetComponent<Renderer>();
        if (crosswalkRenderer != null)
        {
            Material crosswalkMat = new Material(Shader.Find("Standard"));
            crosswalkMat.color = Color.white;
            crosswalkRenderer.material = crosswalkMat;
        }

        cityObjects.Add(crosswalk);
        return crosswalk;
    }

    void AddStreetLights()
    {
        // Add street lights along main roads
        for (int x = 0; x <= blocksX; x++)
        {
            for (int z = 0; z < blocksY; z++)
            {
                Vector3 lightPos = new Vector3(
                    x * (cityBlockSize.x + streetWidth) + cityBlockSize.x / 2,
                    8f,
                    z * (cityBlockSize.y + streetWidth)
                );

                CreateStreetLight(lightPos, "Vertical", x, z);
            }
        }

        for (int z = 0; z <= blocksY; z++)
        {
            for (int x = 0; x < blocksX; x++)
            {
                Vector3 lightPos = new Vector3(
                    x * (cityBlockSize.x + streetWidth),
                    8f,
                    z * (cityBlockSize.y + streetWidth) + cityBlockSize.y / 2
                );

                CreateStreetLight(lightPos, "Horizontal", x, z);
            }
        }
    }

    GameObject CreateStreetLight(Vector3 position, string direction, int x, int z)
    {
        GameObject lightObj = new GameObject($"StreetLight_{direction}_{x}_{z}");
        lightObj.transform.position = position;

        // Light pole
        GameObject pole = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        pole.transform.SetParent(lightObj.transform);
        pole.transform.localPosition = Vector3.zero;
        pole.transform.localScale = new Vector3(0.3f, 8f, 0.3f);
        pole.GetComponent<Renderer>().material.color = Color.gray;

        // Light fixture
        GameObject fixture = GameObject.CreatePrimitive(PrimitiveType.Cube);
        fixture.transform.SetParent(lightObj.transform);
        fixture.transform.localPosition = new Vector3(0, 7.5f, 0);
        fixture.transform.localScale = new Vector3(1.5f, 0.5f, 0.5f);
        fixture.GetComponent<Renderer>().material.color = Color.black;

        // Actual light
        Light light = lightObj.AddComponent<Light>();
        light.type = LightType.Spot;
        light.color = new Color(0.9f, 0.8f, 0.6f); // Warm white
        light.intensity = 2f;
        light.range = 15f;
        light.spotAngle = 60f;
        light.shadows = LightShadows.Soft;

        // Point light downward
        lightObj.transform.rotation = Quaternion.Euler(90, 0, 0);

        cityObjects.Add(lightObj);
        return lightObj;
    }

    public List<GameObject> GetCityObjects()
    {
        return cityObjects;
    }
}

// Supporting scripts for urban elements
public class UrbanTrafficController : MonoBehaviour
{
    public Vector3 startPos;
    public Vector3 endPos;
    public float speed = 5f;

    private Vector3 direction;

    public void SetRoute(Vector3 start, Vector3 end, float moveSpeed)
    {
        startPos = start;
        endPos = end;
        speed = moveSpeed;
        direction = (endPos - startPos).normalized;

        transform.position = startPos;
    }

    void Update()
    {
        transform.Translate(direction * speed * Time.deltaTime);

        // Reset position if reached end
        if (Vector3.Distance(transform.position, endPos) < 1f)
        {
            transform.position = startPos;
        }
    }
}

public class UrbanPedestrianController : MonoBehaviour
{
    public Vector2 environmentSize;
    public float speed = 1.5f;
    public float changeDirectionInterval = 5f;

    private Vector3 targetDirection;
    private float lastDirectionChange;
    private float wanderRadius = 3f;

    void Start()
    {
        ChangeDirection();
        lastDirectionChange = Time.time;
    }

    void Update()
    {
        if (Time.time - lastDirectionChange > changeDirectionInterval)
        {
            ChangeDirection();
            lastDirectionChange = Time.time;
        }

        Move();
        CheckBounds();
    }

    void ChangeDirection()
    {
        // Move in a random direction, but generally toward center
        Vector3 centerDirection = (new Vector3(environmentSize.x / 2, 0, environmentSize.y / 2) - transform.position).normalized;
        targetDirection = Vector3.Slerp(centerDirection, Random.insideUnitSphere, 0.7f);
        targetDirection.y = 0; // Keep horizontal
        targetDirection.Normalize();
    }

    void Move()
    {
        transform.Translate(targetDirection * speed * Time.deltaTime);
    }

    void CheckBounds()
    {
        Vector3 pos = transform.position;
        pos.x = Mathf.Clamp(pos.x, 1f, environmentSize.x - 1f);
        pos.z = Mathf.Clamp(pos.z, 1f, environmentSize.y - 1f);
        transform.position = pos;
    }

    public void SetEnvironmentSize(Vector2 size)
    {
        environmentSize = size;
    }

    public void SetMovementSpeed(float newSpeed)
    {
        speed = newSpeed;
    }
}
```

## Laboratory Environment Example

### Overview
A laboratory environment for testing precision manipulation, sensor calibration, and controlled experimentation.

### Implementation

```csharp
using UnityEngine;
using System.Collections.Generic;

public class LaboratoryEnvironment : MonoBehaviour
{
    [Header("Lab Dimensions")]
    public Vector2 labSize = new Vector2(20f, 15f);
    public float ceilingHeight = 4f;

    [Header("Workstation Configuration")]
    public int workstations = 3;
    public float workstationSpacing = 6f;

    [Header("Equipment")]
    public GameObject[] labEquipmentPrefabs;
    public int equipmentCount = 8;

    [Header("Precision Elements")]
    public bool includeCleanRoom = false;
    public bool includeChemicalFumeHood = true;
    public bool includePrecisionInstruments = true;

    private List<GameObject> labObjects = new List<GameObject>();

    void Start()
    {
        BuildLaboratory();
        AddWorkstations();
        AddEquipment();
        AddPrecisionElements();
        AddLighting();
    }

    void BuildLaboratory()
    {
        // Create floor
        CreateLabFloor();

        // Create walls
        CreateLabWalls();

        // Create ceiling
        CreateLabCeiling();

        // Add safety features
        AddSafetyFeatures();
    }

    void CreateLabFloor()
    {
        GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
        floor.transform.position = new Vector3(labSize.x / 2, 0, labSize.y / 2);
        floor.transform.localScale = new Vector3(labSize.x / 10, 1, labSize.y / 10);
        floor.name = "LabFloor";

        // Apply laboratory-appropriate material (clean, non-slip)
        Renderer floorRenderer = floor.GetComponent<Renderer>();
        if (floorRenderer != null)
        {
            Material labFloorMat = new Material(Shader.Find("Standard"));
            labFloorMat.color = new Color(0.9f, 0.9f, 0.95f); // Clean white
            labFloorMat.SetFloat("_Metallic", 0.0f);
            labFloorMat.SetFloat("_Smoothness", 0.3f);
            floorRenderer.material = labFloorMat;
        }

        // Add physics properties
        PhysicMaterial floorPhysMat = new PhysicMaterial();
        floorPhysMat.staticFriction = 0.8f;
        floorPhysMat.dynamicFriction = 0.7f;
        floorPhysMat.bounciness = 0.0f;
        floor.GetComponent<Collider>().material = floorPhysMat;

        labObjects.Add(floor);
    }

    void CreateLabWalls()
    {
        float wallHeight = ceilingHeight;

        // Create 4 walls
        CreateWall(new Vector3(0, wallHeight / 2, labSize.y / 2),
                  new Vector3(0.2f, wallHeight, labSize.y), "LabWall");
        CreateWall(new Vector3(labSize.x, wallHeight / 2, labSize.y / 2),
                  new Vector3(0.2f, wallHeight, labSize.y), "LabWall");
        CreateWall(new Vector3(labSize.x / 2, wallHeight / 2, 0),
                  new Vector3(labSize.x, wallHeight, 0.2f), "LabWall");
        CreateWall(new Vector3(labSize.x / 2, wallHeight / 2, labSize.y),
                  new Vector3(labSize.x, wallHeight, 0.2f), "LabWall");
    }

    GameObject CreateWall(Vector3 position, Vector3 size, string name)
    {
        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.transform.position = position;
        wall.transform.localScale = size;
        wall.name = name;

        // Apply lab wall material
        Renderer wallRenderer = wall.GetComponent<Renderer>();
        if (wallRenderer != null)
        {
            Material wallMat = new Material(Shader.Find("Standard"));
            wallMat.color = new Color(0.95f, 0.95f, 0.95f); // Clean white
            wallMat.SetFloat("_Metallic", 0.1f);
            wallMat.SetFloat("_Smoothness", 0.2f);
            wallRenderer.material = wallMat;
        }

        labObjects.Add(wall);
        return wall;
    }

    void CreateLabCeiling()
    {
        GameObject ceiling = GameObject.CreatePrimitive(PrimitiveType.Plane);
        ceiling.transform.position = new Vector3(labSize.x / 2, ceilingHeight, labSize.y / 2);
        ceiling.transform.localScale = new Vector3(labSize.x / 10, 1, labSize.y / 10);
        ceiling.transform.rotation = Quaternion.Euler(180, 0, 0); // Flip to face down
        ceiling.name = "LabCeiling";

        // Apply ceiling material
        Renderer ceilingRenderer = ceiling.GetComponent<Renderer>();
        if (ceilingRenderer != null)
        {
            Material ceilingMat = new Material(Shader.Find("Standard"));
            ceilingMat.color = Color.white;
            ceilingMat.SetFloat("_Metallic", 0.0f);
            ceilingMat.SetFloat("_Smoothness", 0.1f);
            ceilingRenderer.material = ceilingMat;
        }

        labObjects.Add(ceiling);
    }

    void AddSafetyFeatures()
    {
        // Emergency shower
        CreateEmergencyShower(new Vector3(2f, 0.5f, labSize.y - 2f));

        // Eye wash station
        CreateEyeWashStation(new Vector3(labSize.x - 2f, 0.5f, labSize.y - 2f));

        // Safety shower
        CreateSafetyShower(new Vector3(2f, 0.5f, 2f));

        // Fire extinguisher
        CreateFireExtinguisher(new Vector3(labSize.x - 2f, 1.5f, 2f));
    }

    GameObject CreateEmergencyShower(Vector3 position)
    {
        GameObject shower = new GameObject("EmergencyShower");
        shower.transform.position = position;

        // Main pole
        GameObject pole = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        pole.transform.SetParent(shower.transform);
        pole.transform.localPosition = Vector3.zero;
        pole.transform.localScale = new Vector3(0.1f, 2f, 0.1f);
        pole.GetComponent<Renderer>().material.color = Color.gray;

        // Shower head
        GameObject head = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        head.transform.SetParent(shower.transform);
        head.transform.localPosition = new Vector3(0, 1.8f, 0);
        head.transform.localScale = new Vector3(0.3f, 0.3f, 0.3f);
        head.GetComponent<Renderer>().material.color = Color.gray;

        labObjects.Add(shower);
        return shower;
    }

    GameObject CreateEyeWashStation(Vector3 position)
    {
        GameObject station = new GameObject("EyeWashStation");
        station.transform.position = position;

        // Base
        GameObject baseObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        baseObj.transform.SetParent(station.transform);
        baseObj.transform.localPosition = Vector3.zero;
        baseObj.transform.localScale = new Vector3(0.3f, 0.8f, 0.2f);
        baseObj.GetComponent<Renderer>().material.color = Color.gray;

        // Eye wash heads
        for (int i = 0; i < 2; i++)
        {
            GameObject head = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            head.transform.SetParent(station.transform);
            head.transform.localPosition = new Vector3(-0.1f + i * 0.2f, 0.6f, 0.15f);
            head.transform.localScale = new Vector3(0.05f, 0.1f, 0.05f);
            head.GetComponent<Renderer>().material.color = Color.white;
        }

        labObjects.Add(station);
        return station;
    }

    GameObject CreateSafetyShower(Vector3 position)
    {
        GameObject shower = new GameObject("SafetyShower");
        shower.transform.position = position;

        // Base
        GameObject baseObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        baseObj.transform.SetParent(shower.transform);
        baseObj.transform.localPosition = Vector3.zero;
        baseObj.transform.localScale = new Vector3(0.4f, 0.1f, 0.4f);
        baseObj.GetComponent<Renderer>().material.color = Color.gray;

        // Vertical pipe
        GameObject pipe = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        pipe.transform.SetParent(shower.transform);
        pipe.transform.localPosition = Vector3.zero;
        pipe.transform.localScale = new Vector3(0.05f, 1.5f, 0.05f);
        pipe.GetComponent<Renderer>().material.color = Color.gray;

        // Shower head
        GameObject head = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        head.transform.SetParent(shower.transform);
        head.transform.localPosition = new Vector3(0, 1.4f, 0);
        head.transform.localScale = new Vector3(0.4f, 0.1f, 0.4f);
        head.GetComponent<Renderer>().material.color = Color.gray;

        labObjects.Add(shower);
        return shower;
    }

    GameObject CreateFireExtinguisher(Vector3 position)
    {
        GameObject extinguisher = new GameObject("FireExtinguisher");
        extinguisher.transform.position = new Vector3(position.x, position.y + 0.5f, position.z);

        // Main body
        GameObject body = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        body.transform.SetParent(extinguisher.transform);
        body.transform.localPosition = Vector3.zero;
        body.transform.localScale = new Vector3(0.2f, 0.6f, 0.2f);
        body.GetComponent<Renderer>().material.color = Color.red;

        // Handle
        GameObject handle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        handle.transform.SetParent(extinguisher.transform);
        handle.transform.localPosition = new Vector3(0.15f, 0.1f, 0);
        handle.transform.localScale = new Vector3(0.1f, 0.05f, 0.05f);
        handle.GetComponent<Renderer>().material.color = Color.black;

        labObjects.Add(extinguisher);
        return extinguisher;
    }

    void AddWorkstations()
    {
        float startX = 3f; // Leave space from wall
        float zPos = labSize.y / 2;

        for (int i = 0; i < workstations; i++)
        {
            Vector3 stationPos = new Vector3(startX + i * workstationSpacing, 0, zPos);
            CreateWorkstation(stationPos, i);
        }
    }

    GameObject CreateWorkstation(Vector3 position, int id)
    {
        GameObject station = new GameObject($"Workstation_{id}");
        station.transform.position = position;

        // Main table
        GameObject table = GameObject.CreatePrimitive(PrimitiveType.Cube);
        table.transform.SetParent(station.transform);
        table.transform.localPosition = new Vector3(0, 0.75f, 0);
        table.transform.localScale = new Vector3(2f, 0.1f, 1f);
        table.GetComponent<Renderer>().material.color = Color.white;

        // Table legs
        for (int i = 0; i < 4; i++)
        {
            float x = (i % 2 == 0) ? -0.9f : 0.9f;
            float z = (i < 2) ? -0.4f : 0.4f;

            GameObject leg = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            leg.transform.SetParent(station.transform);
            leg.transform.localPosition = new Vector3(x, 0.35f, z);
            leg.transform.localScale = new Vector3(0.05f, 0.7f, 0.05f);
            leg.GetComponent<Renderer>().material.color = Color.gray;
        }

        // Back panel
        GameObject panel = GameObject.CreatePrimitive(PrimitiveType.Cube);
        panel.transform.SetParent(station.transform);
        panel.transform.localPosition = new Vector3(0, 1.2f, -0.5f);
        panel.transform.localScale = new Vector3(2f, 0.8f, 0.02f);
        panel.GetComponent<Renderer>().material.color = Color.lightGray;

        labObjects.Add(station);
        return station;
    }

    void AddEquipment()
    {
        for (int i = 0; i < equipmentCount; i++)
        {
            if (labEquipmentPrefabs.Length > 0)
            {
                Vector3 pos = new Vector3(
                    Random.Range(2f, labSize.x - 2f),
                    0.1f,
                    Random.Range(2f, labSize.y - 2f)
                );

                GameObject equipment = Instantiate(labEquipmentPrefabs[Random.Range(0, labEquipmentPrefabs.Length)]);
                equipment.transform.position = pos;
                equipment.name = $"LabEquipment_{i}";

                labObjects.Add(equipment);
            }
        }
    }

    void AddPrecisionElements()
    {
        if (includeCleanRoom)
        {
            CreateCleanRoomArea();
        }

        if (includeChemicalFumeHood)
        {
            CreateChemicalFumeHood();
        }

        if (includePrecisionInstruments)
        {
            AddPrecisionInstruments();
        }
    }

    void CreateCleanRoomArea()
    {
        // Create a designated clean room area with special properties
        GameObject cleanArea = new GameObject("CleanRoomArea");
        cleanArea.transform.position = new Vector3(labSize.x * 0.75f, 0.01f, labSize.y / 2);

        GameObject cleanFloor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cleanFloor.transform.SetParent(cleanArea.transform);
        cleanFloor.transform.localPosition = Vector3.zero;
        cleanFloor.transform.localScale = new Vector3(6f, 0.02f, 4f);
        cleanFloor.GetComponent<Renderer>().material.color = Color.blue;
        cleanFloor.GetComponent<Renderer>().material.color = new Color(0.8f, 0.9f, 1.0f, 0.3f); // Semi-transparent

        // Add clean room signage
        CreateCleanRoomSign(new Vector3(labSize.x * 0.75f, 1.5f, labSize.y / 2 - 2.5f));

        labObjects.Add(cleanArea);
    }

    GameObject CreateCleanRoomSign(Vector3 position)
    {
        GameObject sign = new GameObject("CleanRoomSign");
        sign.transform.position = position;

        GameObject signBoard = GameObject.CreatePrimitive(PrimitiveType.Cube);
        signBoard.transform.SetParent(sign.transform);
        signBoard.transform.localPosition = Vector3.zero;
        signBoard.transform.localScale = new Vector3(1f, 0.5f, 0.05f);
        signBoard.GetComponent<Renderer>().material.color = Color.yellow;

        // Could add text mesh here for "Clean Room" label

        labObjects.Add(sign);
        return sign;
    }

    void CreateChemicalFumeHood()
    {
        GameObject fumeHood = new GameObject("ChemicalFumeHood");
        fumeHood.transform.position = new Vector3(2f, 0, 2f);

        // Main structure
        GameObject mainUnit = GameObject.CreatePrimitive(PrimitiveType.Cube);
        mainUnit.transform.SetParent(fumeHood.transform);
        mainUnit.transform.localPosition = new Vector3(0, 0.8f, 0);
        mainUnit.transform.localScale = new Vector3(1.5f, 0.8f, 0.6f);
        mainUnit.GetComponent<Renderer>().material.color = Color.gray;

        // Hood
        GameObject hood = GameObject.CreatePrimitive(PrimitiveType.Cube);
        hood.transform.SetParent(fumeHood.transform);
        hood.transform.localPosition = new Vector3(0, 1.3f, -0.3f);
        hood.transform.localScale = new Vector3(1.2f, 0.1f, 0.4f);
        hood.GetComponent<Renderer>().material.color = Color.gray;

        // Sash (movable)
        GameObject sash = GameObject.CreatePrimitive(PrimitiveType.Cube);
        sash.transform.SetParent(fumeHood.transform);
        sash.transform.localPosition = new Vector3(0, 1.1f, -0.25f);
        sash.transform.localScale = new Vector3(1.1f, 0.4f, 0.02f);
        sash.GetComponent<Renderer>().material.color = new Color(0.8f, 0.8f, 0.9f, 0.7f); // Semi-transparent

        labObjects.Add(fumeHood);
    }

    void AddPrecisionInstruments()
    {
        // Add high-precision measurement equipment
        string[] instrumentNames = {"Microscope", "Spectrometer", "CalibrationRig", "PrecisionScale"};

        for (int i = 0; i < instrumentNames.Length; i++)
        {
            Vector3 pos = new Vector3(
                Random.Range(labSize.x * 0.6f, labSize.x - 2f),
                0.1f,
                Random.Range(2f, labSize.y * 0.4f)
            );

            GameObject instrument = CreatePrecisionInstrument(pos, instrumentNames[i], i);
            labObjects.Add(instrument);
        }
    }

    GameObject CreatePrecisionInstrument(Vector3 position, string name, int id)
    {
        GameObject instrument = new GameObject($"{name}_{id}");
        instrument.transform.position = position;

        // Base
        GameObject baseObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        baseObj.transform.SetParent(instrument.transform);
        baseObj.transform.localPosition = new Vector3(0, 0.2f, 0);
        baseObj.transform.localScale = new Vector3(0.8f, 0.4f, 0.6f);
        baseObj.GetComponent<Renderer>().material.color = Color.black;

        // Main instrument body
        GameObject body = GameObject.CreatePrimitive(PrimitiveType.Cube);
        body.transform.SetParent(instrument.transform);
        body.transform.localPosition = new Vector3(0, 0.6f, 0);
        body.transform.localScale = new Vector3(0.6f, 0.3f, 0.5f);
        body.GetComponent<Renderer>().material.color = Color.silver;

        // Add specific details based on instrument type
        switch (name)
        {
            case "Microscope":
                // Eyepiece
                GameObject eyepiece = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                eyepiece.transform.SetParent(instrument.transform);
                eyepiece.transform.localPosition = new Vector3(0.2f, 0.8f, 0);
                eyepiece.transform.localScale = new Vector3(0.05f, 0.15f, 0.05f);
                eyepiece.GetComponent<Renderer>().material.color = Color.black;
                break;

            case "Spectrometer":
                // Display screen
                GameObject screen = GameObject.CreatePrimitive(PrimitiveType.Cube);
                screen.transform.SetParent(instrument.transform);
                screen.transform.localPosition = new Vector3(0, 0.65f, 0.25f);
                screen.transform.localScale = new Vector3(0.4f, 0.15f, 0.01f);
                screen.GetComponent<Renderer>().material.color = Color.green;
                break;

            case "CalibrationRig":
                // Calibration points
                for (int i = 0; i < 3; i++)
                {
                    GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    point.transform.SetParent(instrument.transform);
                    point.transform.localPosition = new Vector3(-0.2f + i * 0.2f, 0.7f, 0.2f);
                    point.transform.localScale = new Vector3(0.03f, 0.03f, 0.03f);
                    point.GetComponent<Renderer>().material.color = Color.red;
                }
                break;

            case "PrecisionScale":
                // Display
                GameObject display = GameObject.CreatePrimitive(PrimitiveType.Cube);
                display.transform.SetParent(instrument.transform);
                display.transform.localPosition = new Vector3(0, 0.6f, 0.2f);
                display.transform.localScale = new Vector3(0.3f, 0.1f, 0.01f);
                display.GetComponent<Renderer>().material.color = Color.blue;
                break;
        }

        return instrument;
    }

    void AddLighting()
    {
        // Create laboratory-appropriate lighting
        int lightsX = Mathf.CeilToInt(labSize.x / 4f);
        int lightsZ = Mathf.CeilToInt(labSize.y / 4f);

        for (int x = 0; x < lightsX; x++)
        {
            for (int z = 0; z < lightsZ; z++)
            {
                Vector3 lightPos = new Vector3(
                    2f + x * 4f,
                    ceilingHeight - 0.2f,
                    2f + z * 4f
                );

                CreateLabLight(lightPos);
            }
        }
    }

    GameObject CreateLabLight(Vector3 position)
    {
        GameObject lightObj = new GameObject("LabLight");
        lightObj.transform.position = position;

        // Light fixture
        GameObject fixture = GameObject.CreatePrimitive(PrimitiveType.Cube);
        fixture.transform.SetParent(lightObj.transform);
        fixture.transform.localPosition = Vector3.zero;
        fixture.transform.localScale = new Vector3(0.8f, 0.1f, 0.2f);
        fixture.GetComponent<Renderer>().material.color = Color.white;

        // Actual light
        Light light = lightObj.AddComponent<Light>();
        light.type = LightType.Point;
        light.color = new Color(0.95f, 0.95f, 1.0f); // Clean white
        light.intensity = 1.2f;
        light.range = 8f;
        light.shadows = LightShadows.Soft;

        labObjects.Add(lightObj);
        return lightObj;
    }

    public List<GameObject> GetLabObjects()
    {
        return labObjects;
    }

    public Vector2 GetLabSize()
    {
        return labSize;
    }
}
```

## Performance and Optimization

### Managing Complex Environments

```csharp
using UnityEngine;
using System.Collections.Generic;

public class EnvironmentOptimizer : MonoBehaviour
{
    public float optimizationDistance = 50f;
    public int maxActiveObjects = 100;

    private List<GameObject> allEnvironmentObjects = new List<GameObject>();
    private Dictionary<GameObject, float> objectDistances = new Dictionary<GameObject, float>();

    void Start()
    {
        CollectEnvironmentObjects();
    }

    void Update()
    {
        UpdateObjectDistances();
        OptimizeEnvironment();
    }

    void CollectEnvironmentObjects()
    {
        // This would be called by each environment system
        // For now, we'll assume objects register themselves
    }

    void UpdateObjectDistances()
    {
        Vector3 playerPos = Vector3.zero; // Replace with actual robot position
        foreach (GameObject obj in allEnvironmentObjects)
        {
            if (obj != null)
            {
                float distance = Vector3.Distance(obj.transform.position, playerPos);
                objectDistances[obj] = distance;
            }
        }
    }

    void OptimizeEnvironment()
    {
        // Implement Level of Detail (LOD) systems
        // Implement object pooling
        // Implement occlusion culling
    }

    public void RegisterEnvironmentObject(GameObject obj)
    {
        if (!allEnvironmentObjects.Contains(obj))
        {
            allEnvironmentObjects.Add(obj);
        }
    }

    public void UnregisterEnvironmentObject(GameObject obj)
    {
        allEnvironmentObjects.Remove(obj);
        if (objectDistances.ContainsKey(obj))
        {
            objectDistances.Remove(obj);
        }
    }
}
```

## Testing and Validation

### Environment Validation System

```csharp
using UnityEngine;
using System.Collections.Generic;

public class EnvironmentValidator : MonoBehaviour
{
    public void ValidateEnvironment(GameObject environmentRoot)
    {
        Debug.Log("=== Environment Validation ===");

        // Check for common issues
        CheckPhysicsProperties(environmentRoot);
        CheckLightingSetup(environmentRoot);
        CheckObjectDensity(environmentRoot);
        CheckNavigationClearance(environmentRoot);

        Debug.Log("Environment validation complete.");
    }

    void CheckPhysicsProperties(GameObject root)
    {
        Collider[] colliders = root.GetComponentsInChildren<Collider>();
        int staticColliders = 0;
        int dynamicColliders = 0;

        foreach (Collider col in colliders)
        {
            if (col.GetComponent<Rigidbody>() != null)
            {
                dynamicColliders++;
            }
            else
            {
                staticColliders++;
            }
        }

        Debug.Log($"Physics validation: {staticColliders} static, {dynamicColliders} dynamic colliders");
    }

    void CheckLightingSetup(GameObject root)
    {
        Light[] lights = root.GetComponentsInChildren<Light>();
        float totalIntensity = 0f;

        foreach (Light light in lights)
        {
            totalIntensity += light.intensity;
        }

        float avgIntensity = lights.Length > 0 ? totalIntensity / lights.Length : 0;
        Debug.Log($"Lighting validation: {lights.Length} lights, average intensity {avgIntensity:F2}");
    }

    void CheckObjectDensity(GameObject root)
    {
        Renderer[] renderers = root.GetComponentsInChildren<Renderer>();
        BoxCollider[] colliders = root.GetComponentsInChildren<BoxCollider>();

        // Calculate approximate density
        float totalVolume = 0f;
        foreach (BoxCollider col in colliders)
        {
            totalVolume += col.size.x * col.size.y * col.size.z;
        }

        Debug.Log($"Object density validation: {renderers.Length} renderers, estimated volume {totalVolume:F2}");
    }

    void CheckNavigationClearance(GameObject root)
    {
        // Check for minimum clearance for robot navigation
        CapsuleCollider[] robotColliders = root.GetComponentsInChildren<CapsuleCollider>();

        foreach (CapsuleCollider robotCol in robotColliders)
        {
            // Check if robot can navigate through environment
            // This would involve more complex pathfinding validation
        }
    }
}
```

Creating complex virtual environments requires careful planning and implementation to ensure they provide realistic and challenging scenarios for robot testing. These environments should balance complexity with performance, providing rich interaction possibilities while maintaining stable simulation performance.