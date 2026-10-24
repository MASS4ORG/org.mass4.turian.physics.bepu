using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>The physics scene's statistics and its layer helpers for scripts.</summary>
public sealed class PhysicsStatisticsTests
{
    const double Dt = 1d / 60d;

    static (PhysicsScene Scene, ServiceProvider Services) Scene(PhysicsSettings physics,
        TagsAndLayersSettings? layers = null)
    {
        var settings = new AppSettings();
        settings.Loaded.Use(physics);
        if (layers is not null)
        {
            settings.Loaded.Use(layers);
        }

        var scene = new PhysicsScene(settings, new LayerFilter(settings), null, new BepuPhysicsWorldFactory());
        return (scene, new ServiceCollection().AddSingleton(scene).BuildServiceProvider());
    }

    /// <summary>A settled pile reports sleeping bodies, the static floor, contacts and the step's cost.</summary>
    [Fact]
    public void Statistics_CountBodiesAndTimeSteps()
    {
        var (scene, services) = Scene(new PhysicsSettings { Multithreaded = false });
        using var _ = services;
        using var __ = scene;
        var root = new Node();
        var floor = new Node { Position = new Vector3(0, -0.5f, 0), Parent = root };
        floor.AddComponent(new BoxColliderComponent { Size = new Vector3(20, 1, 20) });
        root.Children.Add(floor);
        for (var i = 0; i < 3; i++)
        {
            var box = new Node { Position = new Vector3(i * 2f, 0.5f, 0), Parent = root };
            box.AddComponent(new RigidbodyComponent());
            box.AddComponent(new BoxColliderComponent());
            root.Children.Add(box);
        }

        root.Awake(null, services);
        Assert.Equal(default, scene.Statistics);

        for (var i = 0; i < 240; i++)
        {
            scene.FixedTick(i + 1, Dt);
        }

        var statistics = scene.Statistics;
        Assert.Equal((4, 1, 3, 0, 240L), (statistics.Bodies, statistics.StaticBodies, statistics.SleepingBodies,
            statistics.AwakeBodies, statistics.Steps));
        Assert.Equal(0, statistics.Joints);
        Assert.True(statistics.StepMilliseconds >= 0d);
        Assert.True(statistics.SimulateMilliseconds >= statistics.StepMilliseconds);

        scene.Simulate(1f / 60f);
        Assert.True(scene.Statistics.SimulateMilliseconds > 0d);
        root.Children[1].GetComponent<RigidbodyComponent>()!.WakeUp();
        Assert.Equal(1, scene.Statistics.AwakeBodies);
    }

    /// <summary>Touching pairs count, and exits do not.</summary>
    [Fact]
    public void Statistics_CountTouchingPairs()
    {
        var (scene, services) = Scene(new PhysicsSettings { Multithreaded = false });
        using var _ = services;
        using var __ = scene;
        var root = new Node();
        var floor = new Node { Position = new Vector3(0, -0.5f, 0), Parent = root };
        floor.AddComponent(new BoxColliderComponent { Size = new Vector3(20, 1, 20) });
        var ball = new Node { Position = new Vector3(0, 0.5f, 0), Parent = root };
        ball.AddComponent(new RigidbodyComponent());
        ball.AddComponent(new SphereColliderComponent());
        root.Children.Add(floor);
        root.Children.Add(ball);
        root.Awake(null, services);

        scene.Simulate(1f / 60f);
        Assert.Equal(1, scene.Statistics.ContactPairs);
        ball.Position = new Vector3(0, 30, 0);
        scene.Simulate(1f / 60f);
        Assert.Equal(0, scene.Statistics.ContactPairs);
    }

    /// <summary>Without a backend the statistics stay empty.</summary>
    [Fact]
    public void Statistics_WithoutBackend()
    {
        using var scene = new PhysicsScene(factory: new NullFactory());

        scene.Simulate(1f / 60f);

        Assert.Equal(default, scene.Statistics);
    }

    /// <summary>The settings' matrix is edited in both directions at once and a session starts from it.</summary>
    [Fact]
    public void LayerCollision_IsSymmetric()
    {
        var physics = new PhysicsSettings { Multithreaded = false, LayerCollisionMatrix = [] };
        physics.SetLayerCollision(3, 5, false);
        physics.SetLayerCollision(4, 4, false);

        Assert.Equal(32, physics.LayerCollisionMatrix.Count);
        Assert.False(physics.GetLayerCollision(5, 3));
        Assert.False(physics.GetLayerCollision(3, 5));
        Assert.False(physics.GetLayerCollision(4, 4));
        Assert.True(physics.GetLayerCollision(3, 4));
        physics.SetLayerCollision(5, 3, true);
        Assert.True(physics.GetLayerCollision(3, 5));
        physics.SetLayerCollision(3, 5, false);
        Assert.Throws<ArgumentOutOfRangeException>(() => physics.SetLayerCollision(32, 0, true));
        Assert.Throws<ArgumentOutOfRangeException>(() => physics.GetLayerCollision(0, -1));

        var (scene, services) = Scene(physics);
        using var _ = services;
        using var __ = scene;
        Assert.True(scene.GetIgnoreLayerCollision(5, 3));
        Assert.False(scene.GetIgnoreLayerCollision(3, 4));
    }

    /// <summary>Layer names resolve to indices and masks from the project's tags and layers, as in Unity.</summary>
    [Fact]
    public void LayerNames_ResolveFromTheProject()
    {
        var layers = new TagsAndLayersSettings();
        layers.PhysicsLayers.Single(static l => l.Index == 3).Name = "Player";
        layers.PhysicsLayers.Single(static l => l.Index == 8).Name = "Ground";
        var (scene, services) = Scene(new PhysicsSettings { Multithreaded = false }, layers);
        using var _ = services;
        using var __ = scene;

        Assert.Equal(3, scene.NameToLayer("Player"));
        Assert.Equal(0, scene.NameToLayer("Default"));
        Assert.Equal(-1, scene.NameToLayer("Water"));
        Assert.Equal("Ground", scene.LayerToName(8));
        Assert.Equal(string.Empty, scene.LayerToName(40));
        Assert.Equal((1u << 3) | (1u << 8), scene.GetMask("Player", "Ground", "Water").Value);
        Assert.Equal(0u, scene.GetMask().Value);

        using var bare = new PhysicsScene(factory: new NullFactory());
        Assert.Equal(7, bare.NameToLayer("Layer 7"));
    }

    sealed class NullFactory : IPhysicsWorldFactory
    {
        public IPhysicsWorld Create(PhysicsWorldSettings settings) => null!;
    }
}
