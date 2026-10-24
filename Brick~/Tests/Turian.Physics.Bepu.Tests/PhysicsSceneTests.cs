using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>The physics front-end driving the Bepu backend through hand-built scenes.</summary>
public sealed class PhysicsSceneTests : IDisposable
{
    /// <summary>Collider churn reuses assignment storage instead of allocating arrays proportional to the scene.</summary>
    [Fact]
    public void AssignmentStorageIsReusedAcrossColliderChurn()
    {
        for (var i = 0; i < 2048; i++)
        {
            Add($"body{i}", new Vector3(i * 3, 0, 0),
                new RigidbodyComponent { IsKinematic = true, UseGravity = false }, new BoxColliderComponent());
        }
        var node = Add("churn", new Vector3(-5, 0, 0), new BoxColliderComponent());
        var collider = node.GetComponent<BoxColliderComponent>()!;
        root.Awake(null, services);
        for (var i = 0; i < 20; i++)
        {
            collider.IsActive = !collider.IsActive;
            scene.FixedTick(i + 1, Dt);
        }
        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var i = 0; i < 200; i++)
        {
            collider.IsActive = !collider.IsActive;
            scene.FixedTick(i + 21, Dt);
        }
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.Equal(2049, scene.BodyCount);
        Assert.True(allocated < 1048576, $"Collider churn allocated {allocated} bytes after warm-up.");
    }

    const double Dt = 1d / 60d;
    readonly ServiceProvider services;
    readonly PhysicsScene scene;
    readonly Node root = new() { Name = "root" };

    /// <summary>Creates a single-threaded scene backed by Bepu.</summary>
    public PhysicsSceneTests()
    {
        scene = new PhysicsScene(Settings(), null, null, new BepuPhysicsWorldFactory());
        services = new ServiceCollection().AddSingleton(scene).BuildServiceProvider();
    }

    /// <inheritdoc />
    public void Dispose()
    {
        scene.Dispose();
        services.Dispose();
    }

    static AppSettings Settings()
    {
        var settings = new AppSettings();
        settings.Loaded.Use(new PhysicsSettings { Multithreaded = false });
        return settings;
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

    void Ground(PhysicsMaterial? material = null) =>
        Add("ground", new Vector3(0, -0.5f, 0), new BoxColliderComponent { Size = new Vector3(20, 1, 20), Material = material });

    (Node Node, RigidbodyComponent Body) Box(Vector3 position, bool kinematic = false)
    {
        var body = new RigidbodyComponent { IsKinematic = kinematic };
        return (Add("box", position, body, new BoxColliderComponent()), body);
    }

    void Run(int ticks, Action<int>? beforeTick = null)
    {
        for (var i = 0; i < ticks; i++)
        {
            beforeTick?.Invoke(i);
            scene.FixedTick(i + 1, Dt);
        }
    }

    /// <summary>Three boxes dropped on each other come to rest stacked and fall asleep.</summary>
    [Fact]
    public void Stack_Settles()
    {
        Ground();
        var boxes = Enumerable.Range(0, 3).Select(i => Box(new Vector3(0, 0.55f + i * 1.1f, 0))).ToArray();
        root.Awake(null, services);

        Run(300);

        for (var i = 0; i < boxes.Length; i++)
        {
            Assert.Equal(0.5f + i, boxes[i].Node.GlobalTransform.Position.Y, 0.05f);
            Assert.Equal(0f, boxes[i].Node.GlobalTransform.Position.X, 0.05f);
        }

        Assert.All(boxes, static box => Assert.True(box.Body.IsSleeping()));
        Assert.Equal(4, scene.BodyCount);
    }

    /// <summary>A rubber ball rebounds from the ground while an ice ball barely leaves it.</summary>
    [Fact]
    public void Rubber_BouncesHigherThanIce()
    {
        Ground();
        var rubber = new PhysicsMaterial { Bounciness = 0.9f, BounceCombine = PhysicsMaterialCombine.Maximum };
        var ice = new PhysicsMaterial { StaticFriction = 0.05f, DynamicFriction = 0.05f };
        var bouncy = Add("rubber", new Vector3(-3, 3, 0), new RigidbodyComponent(),
            new SphereColliderComponent { Material = rubber });
        var dull = Add("ice", new Vector3(3, 3, 0), new RigidbodyComponent(),
            new SphereColliderComponent { Material = ice });
        root.Awake(null, services);
        var bouncyBody = bouncy.GetComponent<RigidbodyComponent>()!;
        float bouncyPeak = 0f, dullPeak = 0f;
        var landed = false;

        Run(180, _ =>
        {
            landed |= bouncyBody.LinearVelocity.Y > 0.5f;
            if (landed)
            {
                bouncyPeak = MathF.Max(bouncyPeak, bouncy.GlobalTransform.Position.Y);
                dullPeak = MathF.Max(dullPeak, dull.GlobalTransform.Position.Y);
            }
        });

        Assert.True(bouncyPeak > 1.5f, $"rubber rebounded to {bouncyPeak}");
        Assert.True(dullPeak < 0.7f, $"ice rebounded to {dullPeak}");
    }

    /// <summary>A kinematic box moved with <see cref="RigidbodyComponent.MovePosition"/> shoves a resting box.</summary>
    [Fact]
    public void KinematicMovePosition_PushesDynamicBody()
    {
        Ground();
        var (_, pusher) = Box(new Vector3(-2, 0.5f, 0), kinematic: true);
        var (pushed, _) = Box(new Vector3(0, 0.5f, 0));
        root.Awake(null, services);

        Run(90, i => pusher.MovePosition(new Vector3(-2f + 3f * MathF.Min(i + 1, 60) / 60f, 0.5f, 0)));

        Assert.Equal(1f, pusher.Position.X, 0.01f);
        Assert.True(pushed.GlobalTransform.Position.X > 1.5f, $"pushed box is at {pushed.GlobalTransform.Position.X}");
    }

    /// <summary>Frozen axes hold against velocity, spin and contact.</summary>
    [Fact]
    public void Constraints_HoldFrozenAxes()
    {
        Ground();
        var (node, body) = Box(new Vector3(1, 2, 1));
        body.Constraints = RigidbodyConstraints.FreezePositionX | RigidbodyConstraints.FreezePositionZ |
                           RigidbodyConstraints.FreezeRotation;
        root.Awake(null, services);
        Run(1);
        body.LinearVelocity = new Vector3(5, 0, -5);
        body.AngularVelocity = new Vector3(3, 2, 1);

        Run(120);

        var pose = node.GlobalTransform;
        Assert.Equal(1f, pose.Position.X, 0.01f);
        Assert.Equal(1f, pose.Position.Z, 0.01f);
        Assert.Equal(0.5f, pose.Position.Y, 0.05f);
        Assert.True(MathF.Abs(Quaternion.Dot(pose.Orientation, Quaternion.Identity)) > 0.9999f);
    }

    /// <summary>An interpolated body shows a blend of its last two ticks and resumes from the simulated pose.</summary>
    [Fact]
    public void Interpolation_BlendsTheLastTwoPoses()
    {
        var body = new RigidbodyComponent { Interpolation = RigidbodyInterpolation.Interpolate };
        var node = Add("falling", new Vector3(0, 10, 0), body, new SphereColliderComponent());
        root.Awake(null, services);
        Run(2);
        var previous = node.GlobalTransform.Position.Y;
        scene.FixedTick(3, Dt);
        var current = node.GlobalTransform.Position.Y;

        scene.AfterFixedTicks(0.25);

        Assert.Equal(previous + (current - previous) * 0.25f, node.GlobalTransform.Position.Y, 1e-4f);
        Assert.Equal(current, body.Position.Y, 1e-4f);
        scene.FixedTick(4, Dt);
        Assert.True(node.GlobalTransform.Position.Y < current, "the blended pose was taken as a teleport");
    }

    /// <summary>The scene steps as a ticker system, and a script moving a node teleports its body.</summary>
    [Fact]
    public void Ticker_StepsTheSceneAndScriptsTeleportBodies()
    {
        var (node, body) = Box(new Vector3(0, 5, 0));
        var sceneManager = new SceneManager(new AssetDatabase());
        root.Awake(null, services);
        sceneManager.AdoptScene(Guid.NewGuid(), root);
        var ticker = new SceneTicker(sceneManager, new SimulationClock()) { Systems = [scene] };

        ticker.Tick(0.5);
        Assert.True(node.GlobalTransform.Position.Y < 5f);
        node.Position = new Vector3(3, 20, 0);
        ticker.Tick(Dt);

        Assert.Equal(3f, body.Position.X, 0.01f);
        Assert.True(body.Position.Y > 19f);
    }

    /// <summary>A dynamic body teleports on <see cref="RigidbodyComponent.MovePosition"/> and accelerates under forces.</summary>
    [Fact]
    public void DynamicBodies_TeleportOnMoveAndRespondToForces()
    {
        var (node, body) = Box(new Vector3(0, 5, 0));
        body.UseGravity = false;
        root.Awake(null, services);
        Run(1);

        body.MovePosition(new Vector3(4, 5, 0));
        body.MoveRotation(Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI / 2f));
        Run(1);
        Assert.Equal(4f, node.GlobalTransform.Position.X, 0.01f);

        body.AddForce(Vector3.UnitX, ForceMode.VelocityChange);
        body.AddForceAtPosition(new Vector3(0, 0, 1), body.WorldCenterOfMass - Vector3.UnitX, ForceMode.Impulse);
        body.AddTorque(Vector3.UnitY, ForceMode.Acceleration);
        Run(1);

        Assert.Equal(1f, body.LinearVelocity.X, 0.01f);
        Assert.True(body.LinearVelocity.Z > 0f);
        Assert.True(body.AngularVelocity.Y > 0f);
    }

    /// <summary>Rays, sweeps and overlaps find the nearest collider, skip what they start inside, and honor masks.</summary>
    [Fact]
    public void Queries_FindCollidersAndHonorMasks()
    {
        Ground();
        var box = Add("box", new Vector3(0, 2, 0), new BoxColliderComponent()).GetComponent<BoxColliderComponent>()!;
        root.Awake(null, services);
        Run(1);
        var top = new Vector3(0, 10, 0);

        Assert.True(scene.Raycast(top, -Vector3.UnitY, out var hit));
        Assert.Same(box, hit.Collider);
        Assert.Equal(7.5f, hit.Distance, 0.01f);
        Assert.Equal(2, scene.RaycastAll(top, -Vector3.UnitY).Length);
        Span<RaycastHit> one = new RaycastHit[1];
        Assert.Equal(1, scene.RaycastNonAlloc(top, -Vector3.UnitY, one));
        Assert.False(scene.Raycast(top, -Vector3.UnitY, 5f));
        Assert.False(scene.Raycast(top, Vector3.Zero));
        Assert.False(scene.Raycast(top, -Vector3.UnitY, layerMask: LayerMask.Nothing));
        Assert.True(scene.SphereCast(top, 0.25f, -Vector3.UnitY, out hit));
        Assert.Equal(7.25f, hit.Distance, 0.02f);
        Assert.True(scene.BoxCast(top, new Vector3(0.25f), -Vector3.UnitY, out _, Quaternion.Identity));
        Assert.True(scene.CapsuleCast(top, top + Vector3.UnitY, 0.25f, -Vector3.UnitY, out _));
        Assert.Equal([box], scene.OverlapSphere(new Vector3(0, 2, 0), 0.1f));
        Assert.Equal(2, scene.OverlapCapsule(new Vector3(0, -0.5f, 0), new Vector3(0, 2, 0), 0.1f).Length);
        Assert.True(scene.CheckSphere(new Vector3(0, 2, 0), 0.1f));
        Assert.False(scene.CheckBox(new Vector3(0, 5, 0), new Vector3(0.1f), Quaternion.Identity));
        var backend = (IPhysicsQueryBackend)scene;
        Assert.Equal(2, backend.RaycastAll(new Ray(top, -Vector3.UnitY), 20f).Count());
        Assert.Contains(box.Node!, backend.OverlapBox(new Bounds(new Vector3(-0.1f, 1.9f, -0.1f), new Vector3(0.1f, 2.1f, 0.1f))));
        Assert.NotEmpty(backend.BoxCastAll(new Bounds(new Vector3(-0.1f, 9.9f, -0.1f), new Vector3(0.1f, 10.1f, 0.1f)),
            -Vector3.UnitY, 20f));
    }

    /// <summary>Disabling a rigidbody leaves its collider static, and disabling a collider removes its body.</summary>
    [Fact]
    public void DisablingComponents_RebuildsTheBodies()
    {
        Ground();
        var (node, body) = Box(new Vector3(0, 3, 0));
        root.Awake(null, services);
        Run(1);
        Assert.Equal(2, scene.BodyCount);

        body.IsActive = false;
        var height = node.GlobalTransform.Position.Y;
        Run(30);
        Assert.Equal(2, scene.BodyCount);
        Assert.Equal(height, node.GlobalTransform.Position.Y);
        Assert.Null(node.GetComponent<BoxColliderComponent>()!.AttachedRigidbody);

        node.GetComponent<BoxColliderComponent>()!.IsActive = false;
        Run(1);
        Assert.Equal(1, scene.BodyCount);
    }

    /// <summary>Spawning and removing bodies regroups the colliders without rebuilding the bodies left unchanged.</summary>
    [Fact]
    public void AddingAndRemovingBodies_KeepsTheOthersBuilt()
    {
        Ground();
        var (_, first) = Box(new Vector3(0, 3, 0));
        root.Awake(null, services);
        Run(1);
        var handle = scene.HandleOf(first);

        // Spawned as at run time: the node wakes under the running root, then receives its components.
        var second = new Node { Name = "spawned", Position = new Vector3(3, 3, 0) };
        root.Children.Add(second);
        second.Awake(root);
        var secondBody = new RigidbodyComponent();
        second.AddComponent(secondBody);
        second.AddComponent(new BoxColliderComponent());
        Run(1);
        Assert.Equal(3, scene.BodyCount);
        Assert.Equal(handle, scene.HandleOf(first));

        second.RemoveComponent(secondBody);
        Run(1);
        Assert.Equal(3, scene.BodyCount);
        Assert.Equal(handle, scene.HandleOf(first));
        Assert.Null(second.GetComponent<BoxColliderComponent>()!.AttachedRigidbody);
    }

    /// <summary>Overlapping convex colliders report the move that separates them; apart, they report nothing.</summary>
    [Fact]
    public void ComputePenetration_SeparatesOverlappingColliders()
    {
        var a = new BoxColliderComponent();
        var b = new BoxColliderComponent();
        Add("a", Vector3.Zero, a);
        Add("b", Vector3.Zero, b);
        root.Awake(null, services);

        Assert.True(scene.ComputePenetration(a, new Vector3(0.75f, 0, 0), Quaternion.Identity, b, Vector3.Zero,
            Quaternion.Identity, out var direction, out var distance));
        Assert.Equal(0.25f, distance, 0.01f);
        Assert.Equal(1f, MathF.Abs(direction.X), 0.01f);
        Assert.False(scene.ComputePenetration(a, new Vector3(3, 0, 0), Quaternion.Identity, b, Vector3.Zero,
            Quaternion.Identity, out _, out _));
    }

    /// <summary>Every physics component keeps its authored values through a scene save and load.</summary>
    [Fact]
    public void Components_RoundTripThroughSerialization()
    {
        TypeRegistry.ScanAssembly(typeof(RigidbodyComponent).Assembly);
        var node = new Node { Name = "saved" };
        node.AddComponent(new RigidbodyComponent
        {
            Mass = 3f,
            LinearDamping = 0.2f,
            AngularDamping = 0.3f,
            UseGravity = false,
            GravityScale = 2f,
            IsKinematic = true,
            Interpolation = RigidbodyInterpolation.Interpolate,
            CollisionDetectionMode = CollisionDetectionMode.Continuous,
            Constraints = RigidbodyConstraints.FreezeRotationY,
            CenterOfMass = new Vector3(0, 1, 0),
            AutomaticCenterOfMass = false,
        });
        node.AddComponent(new BoxColliderComponent
        {
            Size = new Vector3(1, 2, 3),
            Center = new Vector3(0, 1, 0),
            IsTrigger = true,
            IncludeLayers = LayerMask.FromLayer(3),
            ExcludeLayers = LayerMask.FromLayer(4),
        });
        node.AddComponent(new SphereColliderComponent { Radius = 2f });
        node.AddComponent(new CapsuleColliderComponent { Radius = 0.25f, Height = 3f, Direction = 2 });
        node.AddComponent(new CylinderColliderComponent { Radius = 0.75f, Height = 4f, Direction = 0 });

        var loaded = Serializer.LoadData<Node>(Serializer.Serialize(node))!;

        var body = loaded.GetComponent<RigidbodyComponent>()!;
        Assert.Equal((3f, 0.2f, 0.3f, false, 2f, true), (body.Mass, body.LinearDamping, body.AngularDamping,
            body.UseGravity, body.GravityScale, body.IsKinematic));
        Assert.Equal(RigidbodyInterpolation.Interpolate, body.Interpolation);
        Assert.Equal(CollisionDetectionMode.Continuous, body.CollisionDetectionMode);
        Assert.Equal(RigidbodyConstraints.FreezeRotationY, body.Constraints);
        Assert.Equal((new Vector3(0, 1, 0), false), (body.CenterOfMass, body.AutomaticCenterOfMass));
        var box = loaded.GetComponent<BoxColliderComponent>()!;
        Assert.Equal((new Vector3(1, 2, 3), new Vector3(0, 1, 0), true), (box.Size, box.Center, box.IsTrigger));
        Assert.Equal((LayerMask.FromLayer(3), LayerMask.FromLayer(4)), (box.IncludeLayers, box.ExcludeLayers));
        Assert.Equal(2f, loaded.GetComponent<SphereColliderComponent>()!.Radius);
        var capsule = loaded.GetComponent<CapsuleColliderComponent>()!;
        Assert.Equal((0.25f, 3f, 2), (capsule.Radius, capsule.Height, capsule.Direction));
        var cylinder = loaded.GetComponent<CylinderColliderComponent>()!;
        Assert.Equal((0.75f, 4f, 0), (cylinder.Radius, cylinder.Height, cylinder.Direction));
    }
}
