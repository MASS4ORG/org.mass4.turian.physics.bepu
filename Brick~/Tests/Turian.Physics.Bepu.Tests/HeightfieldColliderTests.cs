using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>Heightfield colliders: terrain from a height grid, its sampling and its serialization.</summary>
public sealed class HeightfieldColliderTests : IDisposable
{
    const double Dt = 1d / 60d;
    readonly ServiceProvider services;
    readonly PhysicsScene scene;
    readonly Node root = new() { Name = "root" };

    /// <summary>Creates a single-threaded scene backed by Bepu.</summary>
    public HeightfieldColliderTests()
    {
        var settings = new AppSettings();
        settings.Loaded.Use(new PhysicsSettings { Multithreaded = false });
        scene = new PhysicsScene(settings, null, null, new BepuPhysicsWorldFactory());
        services = new ServiceCollection().AddSingleton(scene).BuildServiceProvider();
    }

    /// <inheritdoc />
    public void Dispose()
    {
        scene.Dispose();
        services.Dispose();
    }

    Node Add(string name, Vector3 position, params Component[] components)
    {
        var node = new Node { Name = name, Position = position, Parent = root };
        foreach (var component in components)
        {
            node.AddComponent(component);
        }

        root.Children.Add(node);
        return node;
    }

    // A grid rising along X from 0 to the full height.
    static float[,] Ramp(int resolution)
    {
        var heights = new float[resolution, resolution];
        for (var z = 0; z < resolution; z++)
        {
            for (var x = 0; x < resolution; x++)
            {
                heights[z, x] = x / (float)(resolution - 1);
            }
        }

        return heights;
    }

    void Run(int ticks)
    {
        for (var i = 0; i < ticks; i++)
        {
            scene.FixedTick(i + 1, Dt);
        }
    }

    /// <summary>A ball dropped on a flat terrain rests on its surface, so its triangles face up.</summary>
    [Fact]
    public void Ball_RestsOnTheSurface()
    {
        var terrain = new HeightfieldColliderComponent { Size = new Vector3(10, 4, 10) };
        terrain.SetHeights(new float[,] { { 0.5f, 0.5f, 0.5f }, { 0.5f, 0.5f, 0.5f }, { 0.5f, 0.5f, 0.5f } });
        Add("terrain", new Vector3(-5, 0, -5), terrain);
        var ball = Add("ball", new Vector3(0.3f, 4, 0.2f), new RigidbodyComponent(),
            new SphereColliderComponent { Radius = 0.25f });
        root.Awake(null, services);

        Run(120);

        Assert.Equal(2f + 0.25f, ball.GlobalTransform.Position.Y, 0.05f);
        Assert.Equal(2f, terrain.SampleHeight(ball.GlobalTransform.Position), 1e-4f);
    }

    /// <summary>Heights between samples interpolate, follow the node's scale and clamp to the grid's edge.</summary>
    [Fact]
    public void SampleHeight_InterpolatesAndClamps()
    {
        var terrain = new HeightfieldColliderComponent { Size = new Vector3(10, 2, 10) };
        terrain.SetHeights(Ramp(5));
        var node = Add("terrain", new Vector3(100, 3, 0), terrain);

        Assert.Equal(0.5f, terrain.SampleHeight(new Vector3(102.5f, 50, 7)), 1e-4f);
        Assert.Equal(2f, terrain.SampleHeight(new Vector3(500, 0, 5)), 1e-4f);
        Assert.Equal(0f, terrain.SampleHeight(new Vector3(-500, 0, 5)), 1e-4f);
        node.Scale = new Vector3(1, 3, 1);
        Assert.Equal(1.5f, terrain.SampleHeight(new Vector3(102.5f, 0, 7)), 1e-4f);
        Assert.Equal(0f, new HeightfieldColliderComponent().SampleHeight(Vector3.Zero));
    }

    /// <summary>A ball rolls down a ramp of terrain towards its low side.</summary>
    [Fact]
    public void Ball_RollsDownhill()
    {
        var terrain = new HeightfieldColliderComponent { Size = new Vector3(20, 6, 20) };
        terrain.SetHeights(Ramp(9));
        Add("terrain", new Vector3(-10, 0, -10), terrain);
        var ball = Add("ball", new Vector3(5, 6, 0), new RigidbodyComponent(), new SphereColliderComponent());
        root.Awake(null, services);

        Run(90);

        Assert.True(ball.GlobalTransform.Position.X < 4f, $"the ball stayed at {ball.GlobalTransform.Position}");
        Assert.True(ball.GlobalTransform.Position.Y > terrain.SampleHeight(ball.GlobalTransform.Position));
    }

    /// <summary>An incomplete grid is not simulated, a bad grid is refused, and settings survive a save.</summary>
    [Fact]
    public void IncompleteGrids_AndSerialization()
    {
        TypeRegistry.ScanAssembly(typeof(HeightfieldColliderComponent).Assembly);
        var empty = new HeightfieldColliderComponent { Resolution = 3, Heights = [0f, 1f] };
        Add("empty", Vector3.Zero, empty);
        root.Awake(null, services);
        Run(1);

        Assert.False(empty.HasShape);
        Assert.Equal(0, scene.BodyCount);
        Assert.Throws<ArgumentException>(() => empty.SetHeights(new float[2, 3]));
        Assert.Throws<ArgumentException>(() => empty.SetHeights(new float[1, 1]));
        Assert.Equal(2, new HeightfieldColliderComponent { Resolution = 0 }.Resolution);

        var node = new Node();
        var terrain = new HeightfieldColliderComponent { Size = new Vector3(8, 3, 6) };
        terrain.SetHeights(Ramp(3));
        node.AddComponent(terrain);
        var loaded = Serializer.LoadData<Node>(Serializer.Serialize(node))!
            .GetComponent<HeightfieldColliderComponent>()!;
        Assert.Equal((3, new Vector3(8, 3, 6)), (loaded.Resolution, loaded.Size));
        Assert.Equal(terrain.Heights, loaded.Heights);
        var mirrored = loaded.Describe(new Vector3(-1, 1, 1), out _);
        Assert.Equal(ShapeType.TriangleMesh, mirrored.Type);
        Assert.Equal(new Vector3(-4, 1.5f, 0), mirrored.Points![1]);
        Assert.NotEqual(loaded.Describe(Vector3.One, out _).Indices, mirrored.Indices);
    }
}
