using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>
/// The Bepu joints through the engine-neutral contract. Values are of the first body relative to the second, and
/// positive angles turn counter-clockwise around the axis (right-hand rule); motors are checked by velocity and
/// limits and springs by position, so a doubled sign error cannot hide.
/// </summary>
public sealed class BepuJointTests : IDisposable
{
    const float Dt = 1f / 60f;
    readonly IPhysicsWorld world = new BepuPhysicsWorldFactory().Create(new PhysicsWorldSettings { Gravity = Vector3.Zero });
    ulong nextId = 1;

    /// <inheritdoc />
    public void Dispose() => world.Dispose();

    BodyHandle Box(Vector3 position, float mass = 1f) =>
        world.CreateBody(BodyDesc.Dynamic(new PhysicsPose(position, Quaternion.Identity),
            new ColliderDesc(world.CreateShape(ShapeDesc.Box(new Vector3(0.5f))), PhysicsPose.Identity,
                CollisionFilter.Default, MaterialDesc.Default, false, nextId++)) with
        {
            Mass = new PhysicsMass(mass, Vector3.Zero, true),
            SleepThreshold = -1f,
        });

    void Run(int steps, Action? beforeStep = null)
    {
        for (var i = 0; i < steps; i++)
        {
            beforeStep?.Invoke();
            world.Step(Dt);
        }
    }

    // The signed rotation of a body around a world axis.
    float Angle(BodyHandle body, Vector3 axis)
    {
        var q = world.GetPose(body).Orientation;
        return 2f * MathF.Atan2(Vector3.Dot(new Vector3(q.X, q.Y, q.Z), axis), q.W);
    }

    JointDesc Hinge(BodyHandle body) => new()
    {
        Type = JointType.Hinge,
        BodyA = body,
        AnchorA = Vector3.Zero,
        AnchorB = world.GetPose(body).Position,
        AxisA = Vector3.UnitY,
        AxisB = Vector3.UnitY,
    };

    JointDesc Slider(BodyHandle body) => new()
    {
        Type = JointType.Slider,
        BodyA = body,
        AnchorA = Vector3.Zero,
        AnchorB = world.GetPose(body).Position,
        AxisA = Vector3.UnitX,
        AxisB = Vector3.UnitX,
    };

    /// <summary>A positive hinge motor spins the body counter-clockwise around the axis.</summary>
    [Fact]
    public void HingeMotor_SpinsCounterClockwise()
    {
        var body = Box(Vector3.Zero);
        world.CreateJoint(Hinge(body) with { Motor = new JointMotor(true, 2f, 100f) });

        Run(60);

        Assert.Equal(2f, world.GetVelocity(body).Angular.Y, 0.05f);
        Assert.True(Angle(body, Vector3.UnitY) > 0f);
    }

    /// <summary>A hinge limit stops a torque at the matching end of the range, and the hinge holds its anchor.</summary>
    [Theory]
    [InlineData(1f, 0.5f)]
    [InlineData(-1f, -0.25f)]
    public void HingeLimit_StopsAtTheMatchingEnd(float torque, float expected)
    {
        var body = Box(new Vector3(0, 2, 0));
        world.CreateJoint(Hinge(body) with { Limit = new JointLimit(true, -0.25f, 0.5f) });

        Run(120, () => world.ApplyAngularImpulse(body, new Vector3(0, torque * Dt, 0)));

        Assert.Equal(expected, Angle(body, Vector3.UnitY), 0.03f);
        Assert.Equal(new Vector3(0, 2, 0), world.GetPose(body).Position, new Vector3Comparer(0.01f));
    }

    /// <summary>A hinge spring settles at a positive target angle.</summary>
    [Fact]
    public void HingeSpring_SettlesAtItsTarget()
    {
        var body = Box(Vector3.Zero);
        world.CreateJoint(Hinge(body) with { Spring = new JointSpring(true, 4f, 1f, 0.6f) });

        Run(180);

        Assert.Equal(0.6f, Angle(body, Vector3.UnitY), 0.02f);
    }

    /// <summary>A positive slider motor moves the body along the axis.</summary>
    [Fact]
    public void SliderMotor_MovesAlongTheAxis()
    {
        var body = Box(Vector3.Zero);
        world.CreateJoint(Slider(body) with { Motor = new JointMotor(true, 1.5f, 100f) });

        Run(60);

        Assert.Equal(1.5f, world.GetVelocity(body).Linear.X, 0.05f);
        Assert.True(world.GetPose(body).Position.X > 1f);
        Assert.Equal(0f, world.GetPose(body).Position.Y, 0.01f);
    }

    /// <summary>A slider limit stops a push at the matching end, and the slider keeps the body from turning.</summary>
    [Theory]
    [InlineData(1f, 1f)]
    [InlineData(-1f, -0.5f)]
    public void SliderLimit_StopsAtTheMatchingEnd(float push, float expected)
    {
        var body = Box(Vector3.Zero);
        world.CreateJoint(Slider(body) with { Limit = new JointLimit(true, -0.5f, 1f) });

        Run(120, () =>
        {
            world.ApplyImpulse(body, new Vector3(push * 5f * Dt, push * Dt, 0), world.GetCenterOfMass(body));
            world.ApplyAngularImpulse(body, new Vector3(0, Dt, 0));
        });

        Assert.Equal(expected, world.GetPose(body).Position.X, 0.03f);
        Assert.Equal(0f, world.GetPose(body).Position.Y, 0.01f);
        Assert.Equal(0f, Angle(body, Vector3.UnitY), 0.01f);
    }

    /// <summary>A fixed joint keeps two bodies' offset and relative rotation while one is spun and pushed.</summary>
    [Fact]
    public void Fixed_KeepsTheRelativePose()
    {
        var a = Box(Vector3.Zero);
        var b = Box(new Vector3(1, 0, 0));
        world.CreateJoint(new JointDesc
        {
            Type = JointType.Fixed,
            BodyA = a,
            BodyB = b,
            AnchorA = new Vector3(0.5f, 0, 0),
            AnchorB = new Vector3(-0.5f, 0, 0),
            RelativeOrientation = Quaternion.Identity,
        });

        Run(60, () =>
        {
            world.ApplyImpulse(a, new Vector3(0, 2 * Dt, 0), world.GetCenterOfMass(a));
            world.ApplyAngularImpulse(a, new Vector3(0, 0, Dt));
        });

        var poseA = world.GetPose(a);
        var poseB = world.GetPose(b);
        Assert.Equal(1f, Vector3.Distance(poseA.Position, poseB.Position), 0.02f);
        Assert.True(MathF.Abs(Quaternion.Dot(poseA.Orientation, poseB.Orientation)) > 0.999f);
    }

    /// <summary>A chain of ball sockets hanging from the world keeps its links together under gravity.</summary>
    [Fact]
    public void BallSocketChain_Holds()
    {
        world.Gravity = new Vector3(0, -9.81f, 0);
        var links = Enumerable.Range(0, 5).Select(i => Box(new Vector3(i + 0.5f, 10, 0))).ToArray();
        world.CreateJoint(new JointDesc
        {
            Type = JointType.BallSocket,
            BodyA = links[0],
            AnchorA = new Vector3(-0.5f, 0, 0),
            AnchorB = new Vector3(0, 10, 0),
        });
        for (var i = 1; i < links.Length; i++)
        {
            world.CreateJoint(new JointDesc
            {
                Type = JointType.BallSocket,
                BodyA = links[i],
                BodyB = links[i - 1],
                AnchorA = new Vector3(-0.5f, 0, 0),
                AnchorB = new Vector3(0.5f, 0, 0),
            });
        }

        Run(300);

        for (var i = 1; i < links.Length; i++)
        {
            Assert.Equal(1f, Vector3.Distance(world.GetPose(links[i]).Position, world.GetPose(links[i - 1]).Position), 0.05f);
        }

        Assert.True(world.GetPose(links[^1]).Position.Y < 7f, "the chain should hang down");
    }

    /// <summary>A ball socket's swing limit keeps the body within a cone of its axis.</summary>
    [Fact]
    public void BallSocketSwingLimit_HoldsTheCone()
    {
        var body = Box(new Vector3(0, -1, 0));
        world.CreateJoint(new JointDesc
        {
            Type = JointType.BallSocket,
            BodyA = body,
            AnchorA = new Vector3(0, 1, 0),
            AnchorB = Vector3.Zero,
            AxisA = -Vector3.UnitY,
            AxisB = -Vector3.UnitY,
            Limit = new JointLimit(true, 0f, 0.4f),
        });

        Run(120, () => world.ApplyImpulse(body, new Vector3(3 * Dt, 0, 0), world.GetCenterOfMass(body)));

        var hang = Vector3.Normalize(world.GetPose(body).Position);
        Assert.Equal(0.4f, MathF.Acos(Vector3.Dot(hang, -Vector3.UnitY)), 0.05f);
    }

    /// <summary>A distance joint keeps its anchors within the range; a spring settles at its rest distance.</summary>
    [Fact]
    public void DistanceAndSpring_KeepTheirRanges()
    {
        var limited = Box(new Vector3(0, 0, 0));
        var sprung = Box(new Vector3(0, 0, 5));
        world.CreateJoint(new JointDesc
        {
            Type = JointType.Distance,
            BodyA = limited,
            AnchorB = new Vector3(0, 0, 0),
            Limit = new JointLimit(true, 0.5f, 2f),
        });
        world.CreateJoint(new JointDesc
        {
            Type = JointType.Spring,
            BodyA = sprung,
            AnchorB = new Vector3(0, 0, 5),
            Spring = new JointSpring(true, 3f, 1f, 1.5f),
        });

        Run(180, () => world.ApplyImpulse(limited, new Vector3(4 * Dt, 0, 0), world.GetCenterOfMass(limited)));

        Assert.Equal(2f, world.GetPose(limited).Position.Length(), 0.05f);
        Assert.Equal(1.5f, Vector3.Distance(world.GetPose(sprung).Position, new Vector3(0, 0, 5)), 0.05f);
    }

    /// <summary>Updating a joint applies its new settings; removing it, or its body, frees the other body.</summary>
    [Fact]
    public void UpdateAndRemove_ChangeTheConstraint()
    {
        var body = Box(Vector3.Zero);
        var joint = world.CreateJoint(Hinge(body) with { Motor = new JointMotor(true, 1f, 100f) });
        Run(30);
        world.UpdateJoint(joint, Hinge(body) with { Motor = new JointMotor(true, -3f, 100f) });
        Run(30);
        Assert.Equal(-3f, world.GetVelocity(body).Angular.Y, 0.05f);

        world.RemoveJoint(joint);
        world.SetVelocity(body, new PhysicsVelocity(Vector3.UnitX, Vector3.Zero));
        Run(30);

        Assert.True(world.GetPose(body).Position.X > 0.4f);
        Assert.Throws<ArgumentException>(() => world.UpdateJoint(joint, Hinge(body)));
    }

    sealed class Vector3Comparer(float tolerance) : IEqualityComparer<Vector3>
    {
        public bool Equals(Vector3 x, Vector3 y) => Vector3.Distance(x, y) <= tolerance;

        public int GetHashCode(Vector3 obj) => 0;
    }
}
