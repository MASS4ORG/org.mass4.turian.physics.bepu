using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>The physics module's session wiring, world lifetime and the physics brick without a backend.</summary>
[Collection(SerialTests.Name)]
public sealed class PhysicsModuleTests
{
    /// <summary>A running game's composed services and lifetimes, with the first session begun.</summary>
    sealed class Game : IDisposable
    {
        public Game()
        {
            var settings = new AppSettings();
            settings.Loaded.Use(new PhysicsSettings { Multithreaded = false });
            Services = new ServiceCollection()
                .AddSingleton<IAppSettings>(settings)
                .AddSingleton<LayerFilter>()
                .AddSingleton<IPhysicsQueries, PhysicsQueries>()
                .AddSingleton(new SimulationClock())
                .AddSingleton<ISessionLifetime>(_ => Lifetimes!)
                .AddEngineModules(typeof(PhysicsModule).Assembly)
                .BuildServiceProvider();
            var host = new LifetimeHost(new SubsystemRegistry(Services.GetServices<SubsystemDescriptor>()), Services);
            Lifetimes = new GameLifetimes(host, Scenes, static _ => Task.CompletedTask);
            Lifetimes.Start();
            SceneServices = new SubsystemServiceProvider(Services, host);
        }

        public ServiceProvider Services { get; }
        public SceneManager Scenes { get; } = new(new AssetDatabase());
        public GameLifetimes Lifetimes { get; }
        public IServiceProvider SceneServices { get; }
        public PhysicsScene Physics => Lifetimes.Session!.Get<PhysicsScene>();

        public void Dispose()
        {
            Lifetimes.Dispose();
            Services.Dispose();
        }
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

    /// <summary>The session owns one world; scene objects receive it and core's queries answer from it.</summary>
    [Fact]
    public void Module_GivesTheSessionsWorldToScenesAndQueries()
    {
        using var game = new Game();

        var scene = game.Physics;
        Assert.Empty(game.Services.GetServices<IFixedTickSystem>());
        Assert.True(scene.IsSimulating);
        var root = Scene(game.SceneServices);
        Assert.Same(scene, root.Children[1].GetComponent<RigidbodyComponent>()!.Physics);
        scene.FixedTick(1, 1d / 60d);

        var hit = game.Services.GetRequiredService<IPhysicsQueries>().Raycast(new Ray(new Vector3(0, 10, 0), -Vector3.UnitY));
        Assert.NotNull(hit);
        Assert.Equal(2.5f, hit.Value.Point.Y, 0.05f);
    }

    /// <summary>Restarting the session three times replaces its world each time, and ending the game frees the last.</summary>
    [Fact]
    public void Sessions_DisposeTheirWorlds()
    {
        var before = BepuPhysicsWorld.LiveCount;
        using (var game = new Game())
        {
            var ticker = new SceneTicker(game.Scenes, game.Services.GetRequiredService<SimulationClock>())
            {
                Lifetimes = game.Lifetimes,
            };
            for (var i = 0; i < 3; i++)
            {
                var world = game.Physics;
                Assert.NotNull(Scene(game.SceneServices));
                ticker.Tick(0.1);
                Assert.Equal(before + 1, BepuPhysicsWorld.LiveCount);

                game.Lifetimes.Restart();
                ticker.Tick(0.1);
                Assert.NotSame(world, game.Physics);
                Assert.Equal(before + 1, BepuPhysicsWorld.LiveCount);
            }
        }

        Assert.Equal(before, BepuPhysicsWorld.LiveCount);
    }
}
