using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>Joint components driving Bepu joints through the front-end, in Unity's units.</summary>
public sealed class JointComponentTests : IDisposable
{
    const double Dt = 1d / 60d;
    readonly ServiceProvider services;
    readonly PhysicsScene scene;
    readonly Node root = new() { Name = "root" };

    /// <summary>Creates a single-threaded scene backed by Bepu.</summary>
    public JointComponentTests()
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

    (Node Node, RigidbodyComponent Body) Box(Vector3 position, params Component[] extra)
    {
        var body = new RigidbodyComponent();
        var node = new Node { Name = $"box{root.Children.Count}", Position = position, Parent = root };
        node.AddComponent(body);
        node.AddComponent(new BoxColliderComponent { Size = new Vector3(0.5f) });
        foreach (var component in extra)
        {
            node.AddComponent(component);
        }

        root.Children.Add(node);
        return (node, body);
    }

    void Run(int ticks, Action? beforeTick = null)
    {
        for (var i = 0; i < ticks; i++)
        {
            beforeTick?.Invoke();
            scene.FixedTick(i + 1, Dt);
        }
    }

    static float YawDegrees(Node node)
    {
        var q = node.GlobalTransform.Orientation;
        return 2f * MathF.Atan2(q.Y, q.W) * 180f / MathF.PI;
    }

    /// <summary>A door hinged to the world turns counter-clockwise under a positive motor and stays on its hinge.</summary>
    [Fact]
    public void HingeMotor_TurnsTheDoorInDegreesPerSecond()
    {
        var hinge = new HingeJointComponent
        {
            Axis = Vector3.UnitY,
            Anchor = new Vector3(-0.5f, 0, 0),
            UseMotor = true,
            MotorSpeed = 90f,
            MotorForce = 100f,
        };
        var (door, body) = Box(new Vector3(0, 2, 0), hinge);
        root.Awake(null, services);

        Run(30);

        Assert.Equal(1, scene.JointCount);
        Assert.Equal(MathF.PI / 2f, body.AngularVelocity.Y, 0.05f);
        Assert.True(YawDegrees(door) > 20f);
        Assert.Equal(new Vector3(-0.5f, 2, 0), hinge.ConnectedAnchor);
        var hingePoint = Vector3.Transform(hinge.Anchor, door.GlobalTransform.Matrix4X4());
        Assert.True(Vector3.Distance(hingePoint, hinge.ConnectedAnchor) < 0.02f);
    }

    /// <summary>A hinge limit in degrees stops a pushed door at its maximum angle.</summary>
    [Fact]
    public void HingeLimit_StopsAtTheMaximumAngle()
    {
        var (door, body) = Box(new Vector3(0, 2, 0), new HingeJointComponent
        {
            Axis = Vector3.UnitY,
            UseLimits = true,
            MinAngle = -10f,
            MaxAngle = 30f,
        });
        root.Awake(null, services);

        Run(120, () => body.AddTorque(new Vector3(0, 2, 0)));

        Assert.Equal(30f, YawDegrees(door), 2f);
    }

    /// <summary>A slider follows its axis turned by the node: here, local X points up the world.</summary>
    [Fact]
    public void SliderMotor_FollowsTheNodeAxis()
    {
        var slider = new SliderJointComponent { UseMotor = true, MotorSpeed = 1f, MotorForce = 1000f };
        var (node, _) = Box(new Vector3(0, 2, 0), slider);
        node.Orientation = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2f);
        root.Awake(null, services);

        Run(60);

        Assert.Equal(3f, node.GlobalTransform.Position.Y, 0.05f);
        Assert.Equal(0f, node.GlobalTransform.Position.X, 0.01f);
    }

    /// <summary>A chain of hinged and socketed links connected to each other hangs from the world and holds.</summary>
    [Fact]
    public void Chain_Holds()
    {
        var links = new List<(Node Node, RigidbodyComponent Body)>();
        for (var i = 0; i < 4; i++)
        {
            JointComponent joint = i % 2 == 0
                ? new BallSocketJointComponent { Anchor = new Vector3(-0.5f, 0, 0) }
                : new HingeJointComponent { Axis = Vector3.UnitZ, Anchor = new Vector3(-0.5f, 0, 0) };
            var link = Box(new Vector3(i + 0.5f, 10, 0), joint);
            joint.ConnectedBody = i == 0 ? null : links[^1].Body;
            links.Add(link);
        }

        root.Awake(null, services);
        var lowest = float.MaxValue;
        Run(300, () => lowest = MathF.Min(lowest, links[^1].Node.GlobalTransform.Position.Y));

        Assert.Equal(4, scene.JointCount);
        for (var i = 1; i < links.Count; i++)
        {
            Assert.Equal(1f, Vector3.Distance(links[i].Node.GlobalTransform.Position,
                links[i - 1].Node.GlobalTransform.Position), 0.05f);
        }

        Assert.True(lowest < 6.5f, $"the chain's end only swung down to {lowest}");
    }

    /// <summary>A fixed joint carries a second body along; rebuilding a body recreates its joint.</summary>
    [Fact]
    public void Fixed_SurvivesBodyRebuilds()
    {
        var (anchor, anchorBody) = Box(new Vector3(0, 5, 0));
        var weld = new FixedJointComponent();
        var (carried, _) = Box(new Vector3(1, 5, 0), weld);
        weld.ConnectedBody = anchorBody;
        root.Awake(null, services);
        Run(1);

        anchorBody.Mass = 5f;
        Run(30);

        Assert.Equal(1, scene.JointCount);
        Assert.Equal(1f, Vector3.Distance(anchor.GlobalTransform.Position, carried.GlobalTransform.Position), 0.02f);
        Assert.True(anchor.GlobalTransform.Position.Y < 4.5f, "the pair should fall together");
    }

    /// <summary>
    /// A rope keeps its length as placed, a spring settles at its placed length, and disabling a joint frees its body.
    /// </summary>
    [Fact]
    public void DistanceAndSpring_UseThePlacedLengths()
    {
        var rope = new DistanceJointComponent { ConnectedAnchor = new Vector3(0, 10, 0), AutoConfigureConnectedAnchor = false };
        var (hanging, hangingBody) = Box(new Vector3(2, 10, 0), rope);
        var spring = new SpringJointComponent
        {
            ConnectedAnchor = new Vector3(5, 10, 0),
            AutoConfigureConnectedAnchor = false,
            Frequency = 4f,
            DampingRatio = 1f,
        };
        var (bouncing, _) = Box(new Vector3(5, 8, 0), spring);
        root.Awake(null, services);

        Run(240, () => hangingBody.AddForce(new Vector3(0, 0, 0.1f)));

        Assert.Equal(2f, Vector3.Distance(hanging.GlobalTransform.Position, new Vector3(0, 10, 0)), 0.05f);
        Assert.True(hanging.GlobalTransform.Position.Y < 8.5f, "the rope should swing down");
        Assert.Equal(2f, Vector3.Distance(bouncing.GlobalTransform.Position, new Vector3(5, 10, 0)), 0.25f);

        rope.IsActive = false;
        Run(30);
        Assert.Equal(1, scene.JointCount);
        Assert.True(Vector3.Distance(hanging.GlobalTransform.Position, new Vector3(0, 10, 0)) > 2.5f);
    }

    /// <summary>Joints and their connected-body references survive a scene save and load.</summary>
    [Fact]
    public void Joints_RoundTripThroughSerialization()
    {
        TypeRegistry.ScanAssembly(typeof(JointComponent).Assembly);
        var (_, target) = Box(Vector3.Zero);
        var hinge = new HingeJointComponent
        {
            Axis = Vector3.UnitZ,
            UseLimits = true,
            MinAngle = -20f,
            MaxAngle = 60f,
            UseMotor = true,
            MotorSpeed = 45f,
            MotorForce = 3f,
            UseSpring = true,
            SpringFrequency = 5f,
            SpringDamping = 0.5f,
            SpringTargetAngle = 15f,
            Anchor = new Vector3(1, 0, 0),
            EnableCollision = true,
            AutoConfigureConnectedAnchor = false,
            ConnectedAnchor = new Vector3(0, 1, 0),
        };
        Box(Vector3.One, hinge, new FixedJointComponent(),
            new SliderJointComponent { UseLimits = true, MinDistance = -2f, MaxDistance = 3f, UseMotor = true, MotorSpeed = 2f },
            new BallSocketJointComponent { UseSwingLimit = true, SwingLimit = 30f, SwingAxis = Vector3.UnitZ },
            new DistanceJointComponent { MinDistance = 1f, MaxDistance = 4f },
            new SpringJointComponent { Frequency = 6f, DampingRatio = 0.1f, RestDistance = 2f, MaxDistance = 5f });
        hinge.ConnectedBody = target;

        var loaded = Serializer.LoadData<Node>(Serializer.Serialize(root))!;

        var box = loaded.Children[1];
        var h = box.GetComponent<HingeJointComponent>()!;
        Assert.Same(loaded.Children[0].GetComponent<RigidbodyComponent>(), h.ConnectedBody);
        Assert.Equal((Vector3.UnitZ, true, -20f, 60f), (h.Axis, h.UseLimits, h.MinAngle, h.MaxAngle));
        Assert.Equal((true, 45f, 3f), (h.UseMotor, h.MotorSpeed, h.MotorForce));
        Assert.Equal((true, 5f, 0.5f, 15f), (h.UseSpring, h.SpringFrequency, h.SpringDamping, h.SpringTargetAngle));
        Assert.Equal((new Vector3(1, 0, 0), new Vector3(0, 1, 0), true, false),
            (h.Anchor, h.ConnectedAnchor, h.EnableCollision, h.AutoConfigureConnectedAnchor));
        Assert.NotNull(box.GetComponent<FixedJointComponent>());
        var s = box.GetComponent<SliderJointComponent>()!;
        Assert.Equal((true, -2f, 3f, true, 2f), (s.UseLimits, s.MinDistance, s.MaxDistance, s.UseMotor, s.MotorSpeed));
        var b = box.GetComponent<BallSocketJointComponent>()!;
        Assert.Equal((true, 30f, Vector3.UnitZ), (b.UseSwingLimit, b.SwingLimit, b.SwingAxis));
        var d = box.GetComponent<DistanceJointComponent>()!;
        Assert.Equal((1f, 4f), (d.MinDistance, d.MaxDistance));
        var sp = box.GetComponent<SpringJointComponent>()!;
        Assert.Equal((6f, 0.1f, 2f, 5f), (sp.Frequency, sp.DampingRatio, sp.RestDistance, sp.MaxDistance));
    }

    /// <summary>A joint without a body, or waiting for its connected body, creates nothing until it can.</summary>
    [Fact]
    public void Joints_WaitForTheirBodies()
    {
        var orphan = new Node { Parent = root };
        orphan.AddComponent(new FixedJointComponent());
        root.Children.Add(orphan);
        var waiting = new FixedJointComponent();
        var (_, other) = Box(new Vector3(3, 5, 0));
        Box(new Vector3(4, 5, 0), waiting);
        waiting.ConnectedBody = other;
        other.IsActive = false;
        root.Awake(null, services);

        Run(1);
        Assert.Equal(0, scene.JointCount);
        other.IsActive = true;
        Run(1);
        Assert.Equal(1, scene.JointCount);
    }
}
