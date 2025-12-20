# Lighting and Material Properties in Unity

This document provides comprehensive guidance on configuring lighting and material properties in Unity for robotics simulation, covering both visual appearance and physical properties that affect robot sensors.

## Understanding Unity Lighting System

### Types of Lights in Unity

**Directional Light**:
- Simulates distant light sources like the sun
- Light rays are parallel across the entire scene
- Affects all objects uniformly
- Essential for outdoor robotics environments
- Casts shadows across the entire scene

**Point Light**:
- Simulates light sources that emit in all directions (light bulbs, robot lights)
- Light intensity decreases with distance (inverse square law)
- Limited range based on intensity and attenuation
- Useful for indoor environments and robot-mounted lights

**Spot Light**:
- Simulates focused light beams (flashlights, laser scanners)
- Conical light distribution with inner and outer angles
- Intensity decreases both with distance and angle from center
- Perfect for simulating robot sensors and focused lighting

**Area Light**:
- Simulates large, soft light sources (windows, large panels)
- Provides soft, realistic shadows
- More computationally expensive
- Available in certain render pipelines (requires HDRP/URP with specific settings)

### Light Properties and Configuration

**Basic Light Properties**:
- **Color**: The color of the emitted light (affects all objects it illuminates)
- **Intensity**: Brightness of the light (measured in different ways depending on light type)
- **Range**: Maximum distance for point and spot lights
- **Spot Angle**: Width of the cone for spot lights

**Shadow Properties**:
- **Type**: Hard shadows (fast) or Soft shadows (more realistic but slower)
- **Strength**: How dark the shadows appear (0 = no shadows, 1 = fully dark shadows)
- **Bias**: Prevents shadow acne (self-shadowing artifacts)
- **Normal Bias**: Additional bias for grazing angles

### Configuring Realistic Lighting for Robotics

```csharp
using UnityEngine;

public class RoboticsLightingManager : MonoBehaviour
{
    public Light mainLight;  // Usually directional light for outdoor scenes
    public float outdoorIntensity = 1.0f;
    public float indoorIntensity = 0.8f;
    public Color outdoorColor = Color.white;
    public Color indoorColor = new Color(0.95f, 0.9f, 0.8f); // Warm white

    void Start()
    {
        ConfigureMainLight();
    }

    public void ConfigureMainLight()
    {
        if (mainLight == null)
        {
            // Find the main light in the scene
            Light[] lights = FindObjectsOfType<Light>();
            foreach (Light light in lights)
            {
                if (light.type == LightType.Directional)
                {
                    mainLight = light;
                    break;
                }
            }

            // If no directional light exists, create one
            if (mainLight == null)
            {
                GameObject lightObj = new GameObject("Main Light");
                mainLight = lightObj.AddComponent<Light>();
                mainLight.type = LightType.Directional;
                mainLight.transform.position = new Vector3(0, 10, 0);
                mainLight.transform.rotation = Quaternion.Euler(50, -30, 0);
            }
        }

        // Configure basic properties
        mainLight.color = outdoorColor;
        mainLight.intensity = outdoorIntensity;
        mainLight.shadows = LightShadows.Soft;
        mainLight.shadowStrength = 0.8f;
        mainLight.shadowBias = 0.05f;
        mainLight.shadowNormalBias = 0.4f;
    }

    public void SetOutdoorLighting()
    {
        mainLight.color = outdoorColor;
        mainLight.intensity = outdoorIntensity;
        mainLight.transform.rotation = Quaternion.Euler(50, -30, 0); // Daytime angle
    }

    public void SetIndoorLighting()
    {
        mainLight.color = indoorColor;
        mainLight.intensity = indoorIntensity;
        mainLight.shadows = LightShadows.Hard; // Indoor lights often have harder shadows
    }

    public void SetNightLighting()
    {
        mainLight.color = new Color(0.8f, 0.8f, 1.0f); // Cool white/blue for night
        mainLight.intensity = outdoorIntensity * 0.3f; // Much dimmer
        mainLight.transform.rotation = Quaternion.Euler(130, -30, 0); // Nighttime angle
    }
}
```

## Material Properties for Robotics Simulation

### Standard Shader Properties

**Albedo (Base Color)**:
- The base color of the material
- Affects both visual appearance and sensor reflection
- Should represent real-world material colors accurately
- RGB values determine color, A (alpha) determines transparency

**Metallic**:
- Controls how metallic the surface appears (0 = non-metallic, 1 = metallic)
- Affects reflection properties
- Non-metallic objects have colored reflections, metallic objects have white reflections
- Important for sensor simulation (metallic surfaces reflect differently)

**Smoothness (Roughness Inverse)**:
- Controls surface smoothness (0 = rough, 1 = smooth)
- Affects how light scatters across the surface
- Smooth surfaces have sharp reflections, rough surfaces have diffused reflections
- Critical for realistic sensor data simulation

**Normal Map**:
- Simulates surface detail without adding geometry
- Affects how light interacts with surface details
- Can simulate textures like wood grain or fabric weave
- Enhances visual realism without performance cost

### Creating Materials for Different Surfaces

```csharp
using UnityEngine;

public class MaterialLibrary : MonoBehaviour
{
    // Pre-configured materials for common robotics surfaces
    public Material concreteMaterial;
    public Material metalMaterial;
    public Material plasticMaterial;
    public Material rubberMaterial;
    public Material grassMaterial;
    public Material glassMaterial;

    void Start()
    {
        CreateRoboticsMaterials();
    }

    public void CreateRoboticsMaterials()
    {
        // Concrete - common for indoor/outdoor floors
        concreteMaterial = CreateMaterial("Concrete", Color.gray, 0.1f, 0.3f);
        concreteMaterial.EnableKeyword("_NORMALMAP");

        // Metal - for machinery, structural elements
        metalMaterial = CreateMaterial("Metal", Color.blue, 0.9f, 0.8f);

        // Plastic - for robot components, containers
        plasticMaterial = CreateMaterial("Plastic", Color.red, 0.2f, 0.5f);

        // Rubber - for wheels, grippers, safety surfaces
        rubberMaterial = CreateMaterial("Rubber", Color.black, 0.1f, 0.2f);

        // Grass - for outdoor environments
        grassMaterial = CreateMaterial("Grass", new Color(0.2f, 0.6f, 0.2f), 0.05f, 0.1f);

        // Glass - for windows, sensors, protective barriers
        glassMaterial = CreateMaterial("Glass", new Color(0.8f, 0.9f, 1.0f, 0.3f), 0.9f, 0.9f);
        glassMaterial.SetFloat("_Mode", 3); // Transparent mode
        glassMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        glassMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        glassMaterial.SetInt("_ZWrite", 0);
        glassMaterial.DisableKeyword("_ALPHATEST_ON");
        glassMaterial.EnableKeyword("_ALPHABLEND_ON");
        glassMaterial.DisableKeyword("_ALPHAPREMULTIPLY_ON");
        glassMaterial.renderQueue = 3000;
    }

    Material CreateMaterial(string name, Color color, float metallic, float smoothness)
    {
        Material mat = new Material(Shader.Find("Standard"));
        mat.name = name;
        mat.color = color;
        mat.SetColor("_Color", color);
        mat.SetFloat("_Metallic", metallic);
        mat.SetFloat("_Smoothness", smoothness);
        mat.SetFloat("_BumpScale", 1.0f);
        mat.SetFloat("_OcclusionStrength", 1.0f);
        mat.SetFloat("_Parallax", 0.02f);
        mat.SetFloat("_ZWrite", 1.0f);
        mat.DisableKeyword("_ALPHATEST_ON");
        mat.DisableKeyword("_ALPHABLEND_ON");
        mat.DisableKeyword("_ALPHAPREMULTIPLY_ON");

        return mat;
    }

    // Method to create materials optimized for sensor simulation
    public Material CreateSensorOptimizedMaterial(string name, Color color, float metallic, float smoothness, float specular)
    {
        Material mat = CreateMaterial(name, color, metallic, smoothness);

        // Additional properties for sensor simulation
        mat.SetFloat("_SpecularHighlights", metallic > 0.1f ? 1.0f : 0.0f);
        mat.SetFloat("_Glossiness", smoothness);

        return mat;
    }
}
```

## Advanced Material Properties for Sensor Simulation

### Reflectance Properties

For robotics applications, especially when simulating sensors like LiDAR, the reflectance properties of materials are crucial:

```csharp
using UnityEngine;

public class SensorMaterialProperties : MonoBehaviour
{
    [System.Serializable]
    public class SensorMaterial
    {
        public string name;
        public Material material;
        public float reflectance; // 0-1, how much light is reflected
        public float absorption;  // 0-1, how much light is absorbed
        public float transmission; // 0-1, how much light passes through
        public bool isReflective; // Whether the material creates strong reflections
        public float roughness;   // Surface roughness affecting reflection spread
    }

    public SensorMaterial[] sensorMaterials;

    void Start()
    {
        ConfigureSensorMaterials();
    }

    void ConfigureSensorMaterials()
    {
        // Configure materials with sensor-specific properties
        foreach (SensorMaterial sensorMat in sensorMaterials)
        {
            // Apply visual properties
            if (sensorMat.material != null)
            {
                sensorMat.material.SetFloat("_Metallic", sensorMat.reflective ? 0.9f : 0.1f);
                sensorMat.material.SetFloat("_Smoothness", 1.0f - sensorMat.roughness);
            }
        }
    }

    // Get material properties for sensor simulation
    public SensorMaterial GetMaterialForSensor(string materialName)
    {
        foreach (SensorMaterial sensorMat in sensorMaterials)
        {
            if (sensorMat.name == materialName)
            {
                return sensorMat;
            }
        }
        return sensorMaterials[0]; // Return first material as default
    }

    // Calculate expected sensor return based on material properties
    public float CalculateSensorReturn(string materialName, float incidentAngle)
    {
        SensorMaterial mat = GetMaterialForSensor(materialName);

        // Simplified calculation - in reality, this would be more complex
        float angleFactor = Mathf.Cos(incidentAngle);
        float returnStrength = mat.reflectance * angleFactor;

        return Mathf.Clamp01(returnStrength);
    }
}
```

## Lighting Scenarios for Different Environments

### Indoor Environment Lighting

```csharp
using UnityEngine;

public class IndoorLightingController : MonoBehaviour
{
    public Light[] ceilingLights;
    public float indoorIntensity = 0.8f;
    public Color indoorColor = new Color(0.95f, 0.9f, 0.8f); // Warm white
    public float lightSpacing = 5f; // Distance between ceiling lights

    public void SetupCeilingLighting(Vector2 roomSize)
    {
        // Calculate number of lights needed
        int lightsX = Mathf.CeilToInt(roomSize.x / lightSpacing);
        int lightsZ = Mathf.CeilToInt(roomSize.y / lightSpacing);

        // Create ceiling lights
        for (int x = 0; x < lightsX; x++)
        {
            for (int z = 0; z < lightsZ; z++)
            {
                Vector3 position = new Vector3(
                    -roomSize.x / 2 + (x + 0.5f) * lightSpacing,
                    2.5f, // Ceiling height
                    -roomSize.y / 2 + (z + 0.5f) * lightSpacing
                );

                CreateCeilingLight(position);
            }
        }
    }

    GameObject CreateCeilingLight(Vector3 position)
    {
        GameObject lightObj = new GameObject("Ceiling Light");
        lightObj.transform.position = position;

        Light light = lightObj.AddComponent<Light>();
        light.type = LightType.Point;
        light.color = indoorColor;
        light.intensity = indoorIntensity;
        light.range = 8f; // Range for the light
        light.spotAngle = 120f; // Wide angle for ceiling light
        light.shadows = LightShadows.Hard;

        return lightObj;
    }

    public void AddAmbientLighting()
    {
        RenderSettings.ambientLight = indoorColor * 0.3f;
        RenderSettings.ambientIntensity = 0.5f;
    }
}
```

### Outdoor Environment Lighting

```csharp
using UnityEngine;

public class OutdoorLightingController : MonoBehaviour
{
    public AnimationCurve dayNightIntensity;
    public float dayDuration = 120f; // 2 minutes for full day/night cycle in simulation
    public Color dayColor = Color.white;
    public Color sunsetColor = new Color(1.0f, 0.7f, 0.4f);
    public Color nightColor = new Color(0.4f, 0.4f, 0.6f);

    private float startTime;
    private Light sunLight;

    void Start()
    {
        startTime = Time.time;
        FindSunLight();
    }

    void FindSunLight()
    {
        Light[] lights = FindObjectsOfType<Light>();
        foreach (Light light in lights)
        {
            if (light.type == LightType.Directional)
            {
                sunLight = light;
                break;
            }
        }

        if (sunLight == null)
        {
            GameObject sunObj = new GameObject("Sun");
            sunLight = sunObj.AddComponent<Light>();
            sunLight.type = LightType.Directional;
        }

        ConfigureSunLight();
    }

    void ConfigureSunLight()
    {
        sunLight.color = dayColor;
        sunLight.intensity = 1.0f;
        sunLight.shadows = LightShadows.Soft;
        sunLight.shadowStrength = 0.8f;
    }

    void Update()
    {
        UpdateDayNightCycle();
    }

    void UpdateDayNightCycle()
    {
        float elapsed = (Time.time - startTime) % dayDuration;
        float normalizedTime = elapsed / dayDuration;

        // Update sun position
        float rotation = normalizedTime * 360f;
        sunLight.transform.rotation = Quaternion.Euler(
            90 - Mathf.Abs(rotation - 180), // Sun rises and sets
            rotation,
            0
        );

        // Update intensity based on time of day
        float intensity = dayNightIntensity.Evaluate(normalizedTime);
        sunLight.intensity = intensity;

        // Update color based on sun position
        if (rotation < 45 || rotation > 315)
        {
            // Dawn/dusk
            sunLight.color = sunsetColor;
        }
        else if (rotation > 135 && rotation < 225)
        {
            // Night
            sunLight.color = nightColor;
        }
        else
        {
            // Day
            sunLight.color = dayColor;
        }
    }
}
```

## Performance Optimization for Lighting

### Light Baking and Real-time Lighting

**Real-time Lighting**:
- Lights that change during gameplay
- More flexible but computationally expensive
- Good for dynamic lighting scenarios

**Baked Lighting**:
- Pre-calculated lighting that's very fast at runtime
- Static objects only
- Perfect for static environment lighting

```csharp
using UnityEngine;

public class LightingOptimizer : MonoBehaviour
{
    public bool useBakedLighting = true;
    public GameObject[] staticObjects;
    public Light[] realTimeLights;

    public void OptimizeLighting()
    {
        if (useBakedLighting)
        {
            SetupBakedLighting();
        }
        else
        {
            SetupRealtimeLighting();
        }
    }

    void SetupBakedLighting()
    {
        // Mark static objects
        foreach (GameObject obj in staticObjects)
        {
            obj.isStatic = true;
        }

        // Configure lights for baking
        foreach (Light light in realTimeLights)
        {
            light.lightmapBakeType = LightmapBakeType.Baked;
            light.shadows = LightShadows.Baked;
        }

        // Configure environment for baked lighting
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
        RenderSettings.ambientSkyColor = new Color(0.212f, 0.227f, 0.259f, 1f);
        RenderSettings.ambientEquatorColor = new Color(0.114f, 0.125f, 0.133f, 1f);
        RenderSettings.ambientGroundColor = new Color(0.047f, 0.043f, 0.035f, 1f);
    }

    void SetupRealtimeLighting()
    {
        // Configure lights for real-time rendering
        foreach (Light light in realTimeLights)
        {
            light.lightmapBakeType = LightmapBakeType.Realtime;
            light.shadows = LightShadows.Soft;
        }

        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Skybox;
    }
}
```

## Material Optimization for Performance

### Texture Atlasing and Material Sharing

```csharp
using UnityEngine;
using System.Collections.Generic;

public class MaterialOptimizer : MonoBehaviour
{
    public List<Material> materialPool = new List<Material>();
    public Dictionary<string, Material> materialCache = new Dictionary<string, Material>();

    // Get or create a material with specific properties
    public Material GetOrCreateMaterial(string key, Color color, float metallic, float smoothness)
    {
        if (materialCache.ContainsKey(key))
        {
            return materialCache[key];
        }

        // Check if we have a similar material in the pool
        Material reusable = FindReusableMaterial(color, metallic, smoothness);
        if (reusable != null)
        {
            materialCache[key] = reusable;
            return reusable;
        }

        // Create new material
        Material newMat = CreateOptimizedMaterial(color, metallic, smoothness);
        materialCache[key] = newMat;
        materialPool.Add(newMat);

        return newMat;
    }

    Material FindReusableMaterial(Color color, float metallic, float smoothness)
    {
        // Find a material with similar properties to reuse
        foreach (Material mat in materialPool)
        {
            Color matColor = mat.GetColor("_Color");
            float matMetallic = mat.GetFloat("_Metallic");
            float matSmoothness = mat.GetFloat("_Smoothness");

            // Check if properties are similar enough to reuse
            if (Color.Approximately(matColor, color) &&
                Mathf.Approximately(matMetallic, metallic) &&
                Mathf.Approximately(matSmoothness, smoothness))
            {
                return mat;
            }
        }
        return null;
    }

    Material CreateOptimizedMaterial(Color color, float metallic, float smoothness)
    {
        Material mat = new Material(Shader.Find("Standard"));
        mat.color = color;
        mat.SetColor("_Color", color);
        mat.SetFloat("_Metallic", metallic);
        mat.SetFloat("_Smoothness", smoothness);

        // Optimize for performance
        mat.enableInstancing = true; // Enable GPU instancing for similar objects

        return mat;
    }
}
```

## Best Practices for Robotics Simulation

### Lighting Best Practices

1. **Consistent Intensity**: Maintain realistic light intensities for sensor simulation
2. **Shadow Configuration**: Properly configure shadows to match real-world conditions
3. **Performance Balance**: Balance visual quality with computational performance
4. **Environment Matching**: Ensure lighting matches the intended operational environment

### Material Best Practices

1. **Realistic Properties**: Use material properties that match real-world materials
2. **Sensor Consideration**: Consider how materials will appear to robot sensors
3. **Performance Optimization**: Use appropriate texture resolutions and material complexity
4. **Consistency**: Maintain consistent material properties across similar objects

### Validation and Testing

```csharp
using UnityEngine;

public class LightingMaterialValidator : MonoBehaviour
{
    public void ValidateLightingSetup()
    {
        Light[] lights = FindObjectsOfType<Light>();
        int totalLights = lights.Length;
        int realtimeLights = 0;
        int bakedLights = 0;

        foreach (Light light in lights)
        {
            if (light.lightmapBakeType == LightmapBakeType.Realtime)
                realtimeLights++;
            else
                bakedLights++;
        }

        Debug.Log($"Lighting Validation: {totalLights} total lights, {realtimeLights} realtime, {bakedLights} baked");

        // Check for common issues
        if (totalLights > 10 && realtimeLights > 5)
        {
            Debug.LogWarning("High number of realtime lights may impact performance");
        }

        // Validate materials
        ValidateMaterials();
    }

    void ValidateMaterials()
    {
        Renderer[] renderers = FindObjectsOfType<Renderer>();
        int transparentMaterials = 0;
        int reflectiveMaterials = 0;

        foreach (Renderer renderer in renderers)
        {
            foreach (Material mat in renderer.materials)
            {
                float metallic = mat.GetFloat("_Metallic");
                float alpha = mat.color.a;

                if (alpha < 1.0f)
                    transparentMaterials++;

                if (metallic > 0.8f)
                    reflectiveMaterials++;
            }
        }

        Debug.Log($"Material Validation: {transparentMaterials} transparent, {reflectiveMaterials} highly reflective materials");
    }
}
```

Understanding and properly configuring lighting and material properties is crucial for creating realistic robotics simulation environments. These properties not only affect visual appearance but also significantly impact how robot sensors perceive and interact with the virtual environment.