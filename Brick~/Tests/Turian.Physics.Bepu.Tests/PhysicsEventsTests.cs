using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>Collision and trigger callbacks, layer filtering and continuous detection through the front-end.</summary>
public sealed class PhysicsEventsTests : IDisposable
{
    const double Dt = 1d / 60d;
    readonly ServiceProvider services;
    readonly PhysicsScene scene;
    readonly Node root = new() { Name = "root" };

    sealed class Recorder : Component, ICollisionHandler, ITriggerHandler
    {
        public List<string> Log { get; } = [];

        public Collision? Last { get; private set; }

        public void OnCollisionEnter(Collision collision) => Record("collision-enter", collision);

        public void OnCollisionStay(Collision collision) => Record("collision-stay", collision);

        public void OnCollisionExit(Collision collision) => Record("collision-exit", collision);

        public void OnTriggerEnter(ColliderComponent other) => Log.Add($"trigger-enter:{other.Node!.Name}");

        public void OnTriggerStay(ColliderComponent other) => Log.Add($"trigger-stay:{other.Node!.Name}");

        public void OnTriggerExit(ColliderComponent other) => Log.Add($"trigger-exit:{other.Node!.Name}");

        void Record(string kind, Collision collision)
        {
            Log.Add($"{kind}:{collision.Node!.Name}");
            Last = collision;
        }
    }

    /// <summary>Creates a single-threaded scene whose layers 1 and 2 ignore each other.</summary>
    public PhysicsEventsTests()
    {
        var settings = new AppSettings();
        var physics = new PhysicsSettings { Multithreaded = false };
        physics.LayerCollisionMatrix[1] = ~(1u << 2);
        physics.LayerCollisionMatrix[2] = ~(1u << 1);
        settings.Loaded.Use(physics);
        scene = new PhysicsScene(settings, null, null, new BepuPhysicsWorldFactory());
        services = new ServiceCollection().AddSingleton(scene).BuildServiceProvider();
    }

    /// <inheritdoc />
    public void Dispose()
    {
        scene.Dispose();
        services.Dispose();
    }

    Node Add(string name, Vector3 position, int layer, params Component[] components)
    {
        var node = new Node { Name = name, Position = position, PhysicsLayer = layer, Parent = root };
        foreach (var component in components)
        {
            node.AddComponent(component);
        }

        root.Children.Add(node);
        return node;
    }

    Node Ground(int layer = 0) =>
        Add("ground", new Vector3(0, -0.5f, 0), layer, new BoxColliderComponent { Size = new Vector3(20, 1, 20) });

    void Run(int ticks)
    {
        for (var i = 0; i < ticks; i++)
        {
            scene.FixedTick(i + 1, Dt);
        }
    }

    static List<string> Distinct(IEnumerable<string> log)
    {
        var result = new List<string>();
        foreach (var entry in log)
        {
            if (result.Count == 0 || result[^1] != entry)
            {
                result.Add(entry);
            }
        }

        return result;
    }

    /// <summary>
    /// A body's node receives the collisions of a collider on its child: enter, then stays, then exit once moved away,
    /// with contact points facing the receiver.
    /// </summary>
    [Fact]
    public void Collisions_ReachTheBodyNodeInOrder()
    {
        Ground();
        var recorder = new Recorder();
        var ball = Add("ball", new Vector3(0, 1, 0), 0, new RigidbodyComponent(), recorder);
        var child = new Node { Name = "shape", Parent = ball };
        child.AddComponent(new SphereColliderComponent());
        ball.Children.Add(child);
        root.Awake(null, services);

        Run(60);
        Assert.NotNull(recorder.Last);
        Assert.NotEmpty(recorder.Last.Contacts);
        Assert.True(recorder.Last.GetContact(0).Normal.Y > 0.9f, "the normal should push the ball up");
        Assert.Same(recorder.Last.Contacts[0].ThisCollider, child.GetComponent<SphereColliderComponent>());
        ball.Position = new Vector3(0, 10, 0);
        Run(2);

        Assert.Equal(["collision-enter:ground", "collision-stay:ground", "collision-exit:ground"],
            Distinct(recorder.Log));
    }

    /// <summary>A ball falling through a trigger reports enter, stays and exit on both sides, and passes through.</summary>
    [Fact]
    public void Triggers_ReportEnterStayExitOnBothSides()
    {
        var zone = new Recorder();
        Add("zone", Vector3.Zero, 0, new BoxColliderComponent { Size = new Vector3(4, 1, 4), IsTrigger = true }, zone);
        var ball = new Recorder();
        Add("ball", new Vector3(0, 2, 0), 0, new RigidbodyComponent(), new SphereColliderComponent { Radius = 0.25f },
            ball);
        root.Awake(null, services);

        Run(90);

        Assert.Equal(["trigger-enter:ball", "trigger-stay:ball", "trigger-exit:ball"], Distinct(zone.Log));
        Assert.Equal(["trigger-enter:zone", "trigger-stay:zone", "trigger-exit:zone"], Distinct(ball.Log));
    }

    /// <summary>A disabled handler and a handler that throws do not stop the others.</summary>
    [Fact]
    public void Handlers_SkipDisabledAndSurviveExceptions()
    {
        Ground();
        var disabled = new Recorder { IsActive = false };
        var enabled = new Recorder();
        Add("ball", new Vector3(0, 0.6f, 0), 0, new RigidbodyComponent(), new SphereColliderComponent(), new Thrower(),
            disabled, enabled);
        root.Awake(null, services);

        Run(10);

        Assert.Empty(disabled.Log);
        Assert.NotEmpty(enabled.Log);
    }

    sealed class Thrower : Component, ICollisionHandler
    {
        public void OnCollisionEnter(Collision collision) => throw new InvalidOperationException("script bug");
    }

    /// <summary>A handler added to a body already touching the ground receives that contact from the next step.</summary>
    [Fact]
    public void Handlers_AddedLaterReceiveOngoingContacts()
    {
        Ground();
        var ball = Add("ball", new Vector3(0, 0.6f, 0), 0, new RigidbodyComponent(), new SphereColliderComponent());
        root.Awake(null, services);
        Run(10);

        var recorder = new Recorder();
        ball.AddComponent(recorder);
        Run(2);

        Assert.Contains("collision-stay:ground", recorder.Log);
    }

    /// <summary>
    /// A sleeping box woken by a ball landing on it keeps touching the ground: its ground contact stays without an
    /// exit, though the pair was not tested in the step that woke it.
    /// </summary>
    [Fact]
    public void WakingASleepingBody_KeepsItsRestingContacts()
    {
        Ground();
        var recorder = new Recorder();
        var box = new RigidbodyComponent();
        Add("box", new Vector3(0, 0.5f, 0), 0, box, new BoxColliderComponent(), recorder);
        root.Awake(null, services);
        Run(180);
        Assert.True(box.IsSleeping(), "the box should be asleep before the ball lands");
        recorder.Log.Clear();

        var ball = new Node { Name = "ball", Position = new Vector3(0, 1.6f, 0) };
        root.Children.Add(ball);
        ball.Awake(root);
        ball.AddComponent(new RigidbodyComponent());
        ball.AddComponent(new SphereColliderComponent { Radius = 0.25f });
        Run(60);

        Assert.Contains("collision-enter:ball", recorder.Log);
        Assert.DoesNotContain("collision-exit:ground", recorder.Log);
    }

    /// <summary>Disabling a resting body's rigidbody removes its body, and its node still receives the exit.</summary>
    [Fact]
    public void DisablingTheRigidbody_ReportsTheExit()
    {
        Ground();
        var recorder = new Recorder();
        var body = new RigidbodyComponent();
        Add("ball", new Vector3(0, 0.6f, 0), 0, body, new SphereColliderComponent(), recorder);
        root.Awake(null, services);
        Run(10);

        body.IsActive = false;
        Run(2);

        Assert.Equal("collision-exit:ground", recorder.Log[^1]);
    }

    /// <summary>The project's layer matrix lets layer 2 fall through layer 1 while layer 0 still lands.</summary>
    [Fact]
    public void LayerMatrix_FiltersContacts()
    {
        Ground(layer: 1);
        var ghost = Add("ghost", new Vector3(-2, 1, 0), 2, new RigidbodyComponent(), new BoxColliderComponent());
        var solid = Add("solid", new Vector3(2, 1, 0), 0, new RigidbodyComponent(), new BoxColliderComponent());
        root.Awake(null, services);

        Run(60);

        Assert.True(ghost.GlobalTransform.Position.Y < -1f);
        Assert.Equal(0.5f, solid.GlobalTransform.Position.Y, 0.05f);
        Assert.True(scene.GetIgnoreLayerCollision(1, 2));
        Assert.False(scene.GetIgnoreLayerCollision(0, 1));
    }

    /// <summary>Layers and single pairs can be ignored at run time, and restored.</summary>
    [Fact]
    public void IgnoreLayerAndPair_ApplyAtRunTime()
    {
        var ground = Ground(layer: 3);
        var layered = Add("layered", new Vector3(-2, 1, 0), 4, new RigidbodyComponent(), new BoxColliderComponent());
        var paired = Add("paired", new Vector3(2, 1, 0), 0, new RigidbodyComponent(), new BoxColliderComponent());
        root.Awake(null, services);
        scene.IgnoreLayerCollision(3, 4);
        scene.IgnoreCollision(ground.GetComponent<BoxColliderComponent>()!, paired.GetComponent<BoxColliderComponent>()!);

        Run(60);

        Assert.True(scene.GetIgnoreLayerCollision(4, 3));
        Assert.True(layered.GlobalTransform.Position.Y < -1f);
        Assert.True(paired.GlobalTransform.Position.Y < -1f);
        scene.IgnoreLayerCollision(3, 4, false);
        Assert.False(scene.GetIgnoreLayerCollision(3, 4));
        Assert.Throws<ArgumentOutOfRangeException>(() => scene.IgnoreLayerCollision(32, 0));
    }

    /// <summary>
    /// Queries honor layer masks and trigger interaction, and core's <see cref="IPhysicsQueries"/> filters the same
    /// colliders by physics layer.
    /// </summary>
    [Fact]
    public void Queries_HonorMasksTriggersAndTheCoreAdapter()
    {
        Ground(layer: 1);
        Add("trigger", new Vector3(0, 3, 0), 0, new BoxColliderComponent { IsTrigger = true });
        root.Awake(null, services);
        Run(1);
        var top = new Vector3(0, 10, 0);

        Assert.True(scene.Raycast(top, -Vector3.UnitY, out var hit));
        Assert.Equal("trigger", hit.Node!.Name);
        Assert.True(scene.Raycast(top, -Vector3.UnitY, out hit, queryTriggerInteraction: QueryTriggerInteraction.Ignore));
        Assert.Equal("ground", hit.Node!.Name);
        Assert.False(scene.Raycast(top, -Vector3.UnitY, out _, layerMask: LayerMask.FromLayer(2),
            queryTriggerInteraction: QueryTriggerInteraction.Ignore));
        Assert.Equal(2, scene.RaycastAll(top, -Vector3.UnitY, queryTriggerInteraction: QueryTriggerInteraction.Collide).Length);
        Assert.Empty(scene.OverlapSphere(new Vector3(0, 3, 0), 0.1f, queryTriggerInteraction: QueryTriggerInteraction.Ignore));

        var queries = new PhysicsQueries(new LayerFilter(), scene);
        var ray = new Ray(top, -Vector3.UnitY);
        Assert.Equal("ground", queries.Raycast(ray, layerMask: LayerMask.FromLayer(1))!.Value.Node.Name);
        Assert.Null(queries.Raycast(ray, layerMask: LayerMask.FromLayer(5)));
    }

    /// <summary>A compound body hit by a ray reports one hit per collider, never two for the same one.</summary>
    [Fact]
    public void RaycastAll_ReportsEachColliderOnce()
    {
        var body = Add("body", new Vector3(0, 0, 0), 0, new RigidbodyComponent { IsKinematic = true },
            new BoxColliderComponent(), new BoxColliderComponent { Center = new Vector3(0, 2, 0) });
        root.Awake(null, services);
        Run(1);

        var hits = scene.RaycastAll(new Vector3(0, 10, 0), -Vector3.UnitY);

        Assert.Equal(2, hits.Length);
        Assert.Equal(2, hits.Select(static h => h.Collider).Distinct().Count());
        Assert.True(hits[0].Distance < hits[1].Distance);
        Assert.All(hits, h => Assert.Same(body.GetComponent<RigidbodyComponent>(), h.Rigidbody));
    }

    /// <summary>
    /// A 400 m/s bullet stops at a 20 cm wall in both detection modes: the backend's speculative contacts already
    /// catch it, and continuous mode adds a sweep for fast rotation.
    /// </summary>
    [Fact]
    public void FastBullets_DoNotTunnelThroughWalls()
    {
        Add("wall", new Vector3(0, 5, 0), 0, new BoxColliderComponent { Size = new Vector3(4, 4, 0.2f) });
        var discrete = Bullet(-1f, CollisionDetectionMode.Discrete);
        var continuous = Bullet(1f, CollisionDetectionMode.Continuous);
        root.Awake(null, services);
        Run(1);
        discrete.LinearVelocity = continuous.LinearVelocity = new Vector3(0, 0, 400);

        Run(10);

        Assert.True(discrete.Position.Z < 0f, $"discrete bullet reached {discrete.Position.Z}");
        Assert.True(continuous.Position.Z < 0f, $"continuous bullet reached {continuous.Position.Z}");
    }

    RigidbodyComponent Bullet(float x, CollisionDetectionMode mode)
    {
        var body = new RigidbodyComponent { UseGravity = false, CollisionDetectionMode = mode };
        Add($"bullet{x}", new Vector3(x, 5, -3), 0, body, new SphereColliderComponent { Radius = 0.05f });
        return body;
    }
}
