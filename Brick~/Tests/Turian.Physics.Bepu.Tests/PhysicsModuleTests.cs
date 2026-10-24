using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>The physics module's session wiring, world lifetime and the physics brick without a backend.</summary>
[Collection(SerialTests.Name)]
public sealed class PhysicsModuleTests
{
    static ServiceProvider Session()
    {
        var settings = new AppSettings();
        settings.Loaded.Use(new PhysicsSettings { Multithreaded = false });
        return new ServiceCollection()
            .AddSingleton<IAppSettings>(settings)
            .AddSingleton<LayerFilter>()
            .AddSingleton<IPhysicsQueries, PhysicsQueries>()
            .AddSingleton(new SimulationClock())
            .AddEngineModules(typeof(PhysicsModule).Assembly)
            .BuildServiceProvider();
    }

    static Node Scene(IServiceProvider services)
    {
        var root = new Node();
        var ground = new Node { Position = new Vector3(0, -0.5f, 0), Parent = root };
        ground.AddComponent(new BoxColliderComponent { Size = new Vector3(20, 1, 20) });
        var box = new Node { Position = new Vector3(0, 2, 0), Parent = root };
        box.AddComponent(new RigidbodyComponent());
        box.AddComponent(new BoxColliderComponent());
        root.Children.Add(ground);
        root.Children.Add(box);
        root.Awake(null, services);
        return root;
    }

    /// <summary>The module registers one scene as the session's fixed-tick system and query backend.</summary>
    [Fact]
    public void Module_ForwardsTheSceneToTickerAndQueries()
    {
        using var services = Session();

        var scene = services.GetRequiredService<PhysicsScene>();
        Assert.Same(scene, Assert.Single(services.GetServices<IFixedTickSystem>()));
        Assert.Same(scene, services.GetRequiredService<IPhysicsQueryBackend>());
        Assert.True(scene.IsSimulating);
        Scene(services);
        scene.FixedTick(1, 1d / 60d);

        var hit = services.GetRequiredService<IPhysicsQueries>().Raycast(new Ray(new Vector3(0, 10, 0), -Vector3.UnitY));
        Assert.NotNull(hit);
        Assert.Equal(2.5f, hit.Value.Point.Y, 0.05f);
    }

    /// <summary>Starting and stopping a session three times leaves no backend world alive.</summary>
    [Fact]
    public void Sessions_DisposeTheirWorlds()
    {
        var before = BepuPhysicsWorld.LiveCount;
        for (var i = 0; i < 3; i++)
        {
            var services = Session();
            var root = Scene(services);
            var ticker = new SceneTicker(new SceneManager(new AssetDatabase()), services.GetRequiredService<SimulationClock>())
            {
                Systems = [.. services.GetServices<IFixedTickSystem>()],
            };
            ticker.Tick(0.1);
            Assert.Equal(before + 1, BepuPhysicsWorld.LiveCount);
            Assert.NotNull(root);
            services.Dispose();
        }

        Assert.Equal(before, BepuPhysicsWorld.LiveCount);
    }

}
