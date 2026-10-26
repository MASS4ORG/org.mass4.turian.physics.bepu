using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>The BepuPhysics backend through the engine-neutral world contract.</summary>
public sealed class BepuPhysicsWorldTests : IDisposable
{
    const float Dt = 1f / 60f;
    readonly IPhysicsWorld world = new BepuPhysicsWorldFactory().Create(new PhysicsWorldSettings());

    /// <inheritdoc />
    public void Dispose() => world.Dispose();

    ColliderDesc Collider(ShapeDesc shape, ulong id, bool trigger = false, MaterialDesc? material = null,
        CollisionFilter? filter = null, PhysicsPose? local = null) =>
        new(world.CreateShape(shape), local ?? PhysicsPose.Identity, filter ?? CollisionFilter.Default,
            material ?? MaterialDesc.Default, trigger, id);

    BodyHandle Ground(ulong id = 100, MaterialDesc? material = null) =>
        world.CreateStatic(new StaticDesc(new PhysicsPose(new Vector3(0, -0.5f, 0), Quaternion.Identity),
            [Collider(ShapeDesc.Box(new Vector3(20, 1, 20)), id, material: material)]));

    BodyHandle Ball(Vector3 position, ulong id, MaterialDesc? material = null) =>
        world.CreateBody(BodyDesc.Dynamic(new PhysicsPose(position, Quaternion.Identity),
            Collider(ShapeDesc.Sphere(0.5f), id, material: material)));

    void Run(int steps)
    {
        for (var i = 0; i < steps; i++)
        {
            world.Step(Dt);
        }
    }

    /// <summary>A fast sphere stops at the face of a wall instead of sinking deep into it before being pushed out.</summary>
    [Fact]
    public void FastBody_StopsAtTheWallFace()
    {
        world.CreateStatic(new StaticDesc(new PhysicsPose(new Vector3(0, 5, 0), Quaternion.Identity),
            [Collider(ShapeDesc.Box(new Vector3(4, 4, 1)), 100)]));
        var bullet = world.CreateBody(BodyDesc.Dynamic(new PhysicsPose(new Vector3(0, 5, -3), Quaternion.Identity),
            Collider(ShapeDesc.Sphere(0.5f), 1)) with
        {
            GravityScale = 0f,
            Velocity = new PhysicsVelocity(new Vector3(0, 0, 400), Vector3.Zero),
        });

        world.Step(Dt);

        Assert.True(world.GetPose(bullet).Position.Z < -0.85f, $"sank to {world.GetPose(bullet).Position.Z}");
    }

    /// <summary>A falling box comes to rest on the ground and falls asleep.</summary>
    [Fact]
    public void Body_FallsAndRestsOnStatic()
    {
        Ground();
        var box = world.CreateBody(BodyDesc.Dynamic(new PhysicsPose(new Vector3(0, 3, 0), Quaternion.Identity),
            Collider(ShapeDesc.Box(Vector3.One), 1)));

        Run(240);

        Assert.Equal(0.5f, world.GetPose(box).Position.Y, 0.03f);
        Assert.False(world.IsAwake(box));
    }

    /// <summary>
    /// A body calm except for a one-step nudge every eighth step never stays below the threshold for Bepu's 32 steps
    /// in a row, yet still sleeps when the nudges stay under the upper threshold; clearly moving nudges keep it awake.
    /// </summary>
    [Theory]
    [InlineData(0.2f, true)]
    [InlineData(0.5f, false)]
    public void BriefSpikes_HoldTheSleepCount(float nudge, bool sleeps)
    {
        var box = world.CreateBody(BodyDesc.Dynamic(new PhysicsPose(new Vector3(0, 3, 0), Quaternion.Identity),
            Collider(ShapeDesc.Box(Vector3.One), 1)) with
        { GravityScale = 0f, SleepThreshold = 0.1f });

        for (var step = 0; step < 400 && world.IsAwake(box); step++)
        {
            var bepu = (BepuPhysicsWorld)world;
            bepu.Simulation.Bodies[bepu.Get(box).Body].Velocity =
                new BepuPhysics.BodyVelocity(new Vector3(step % 8 == 0 ? nudge : 0f, 0, 0), Vector3.Zero);
            world.Step(Dt);
        }

        Assert.Equal(sleeps, !world.IsAwake(box));
    }

    /// <summary>A body moving steadily inside the hysteresis band never earns calm steps.</summary>
    [Theory]
    [InlineData(0.2f, 0f, 0.1f, true)]
    [InlineData(0f, 0.2f, 0.1f, true)]
    [InlineData(0f, 0f, -1f, true)]
    [InlineData(0f, 0f, 0f, false)]
    public void SleepThreshold_RespectsSustainedMotionAndDisabledSleeping(float speed, float spin, float threshold,
        bool awake)
    {
        var box = world.CreateBody(BodyDesc.Dynamic(PhysicsPose.Identity,
            Collider(ShapeDesc.Box(Vector3.One), 1)) with
        {
            GravityScale = 0f,
            LinearDamping = 0f,
            AngularDamping = 0f,
            SleepThreshold = threshold,
            Velocity = new PhysicsVelocity(new Vector3(speed, 0, 0), new Vector3(0, spin, 0)),
        });

        Run(400);

        Assert.Equal(awake, world.IsAwake(box));
    }

    /// <summary>An explicit wake cancels sleep candidacy before the next native sleep pass.</summary>
    [Fact]
    public void Wake_CancelsPendingSleep()
    {
        var box = world.CreateBody(BodyDesc.Dynamic(PhysicsPose.Identity,
            Collider(ShapeDesc.Box(Vector3.One), 1)) with
        { GravityScale = 0f });
        Run(32);
        Assert.True(world.IsAwake(box));

        world.Wake(box);
        Run(5);

        Assert.True(world.IsAwake(box));
    }

    /// <summary>Waking a body starts a fresh calm period, even immediately after it fell asleep.</summary>
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void Wake_DiscardsThePreviousCalmPeriod(int command)
    {
        var box = world.CreateBody(BodyDesc.Dynamic(PhysicsPose.Identity,
            Collider(ShapeDesc.Box(Vector3.One), 1)) with
        { GravityScale = 0f });
        for (var i = 0; i < 100 && world.IsAwake(box); i++)
        {
            world.Step(Dt);
        }

        Assert.False(world.IsAwake(box));
        switch (command)
        {
            case 0:
                world.Wake(box);
                break;
            case 1:
                world.SetVelocity(box, new PhysicsVelocity(new Vector3(0.02f, 0, 0), Vector3.Zero));
                break;
            case 2:
                world.ApplyImpulse(box, new Vector3(0.02f, 0, 0), world.GetCenterOfMass(box));
                break;
            case 3:
                world.ApplyAngularImpulse(box, new Vector3(0, 0.002f, 0));
                break;
            default:
                world.SetPose(box, new PhysicsPose(Vector3.UnitY, Quaternion.Identity));
                break;
        }

        Run(5);
        Assert.True(world.IsAwake(box));
        Run(100);
        Assert.False(world.IsAwake(box));
    }

    /// <summary>A bounce returns about bounciness squared of the drop height, never more than the drop.</summary>
    [Theory]
    [InlineData(1f, PhysicsMaterialCombine.Average, 0.5f, 6f)]
    [InlineData(0.6f, PhysicsMaterialCombine.Maximum, 0.6f, 6f)]
    [InlineData(0.9f, PhysicsMaterialCombine.Maximum, 0.9f, 2.5f)]
    public void Bounciness_ReturnsAtMostTheImpactEnergy(float bounciness, PhysicsMaterialCombine combine,
        float restitution, float drop)
    {
        Ground(material: new MaterialDesc(0.6f, 0.6f, 0f));
        var ball = Ball(new Vector3(0, 0.5f + drop, 0), 1,
            new MaterialDesc(0.4f, 0.4f, bounciness, BounceCombine: combine));
        var peak = 0f;
        var landed = false;

        for (var i = 0; i < 240; i++)
        {
            world.Step(Dt);
            landed |= world.GetVelocity(ball).Linear.Y > 0.05f;
            peak = landed ? MathF.Max(peak, world.GetPose(ball).Position.Y - 0.5f) : peak;
        }

        var expected = restitution * restitution * drop;
        Assert.InRange(peak, expected * 0.5f, MathF.Min(drop, expected * 1.5f));
    }

    /// <summary>A body of several colliders bounces through the same capped rebound as a single collider.</summary>
    [Fact]
    public void Bounciness_CapsCompoundRebounds()
    {
        Ground(material: new MaterialDesc(0.6f, 0.6f, 0f));
        var bouncy = new MaterialDesc(0.4f, 0.4f, 0.6f, BounceCombine: PhysicsMaterialCombine.Maximum);
        var left = new PhysicsPose(new Vector3(-0.6f, 0, 0), Quaternion.Identity);
        var right = new PhysicsPose(new Vector3(0.6f, 0, 0), Quaternion.Identity);
        var dumbbell = world.CreateBody(BodyDesc.Dynamic(new PhysicsPose(new Vector3(0, 6.5f, 0), Quaternion.Identity),
            Collider(ShapeDesc.Sphere(0.5f), 1, material: bouncy, local: left),
            Collider(ShapeDesc.Sphere(0.5f), 2, material: bouncy, local: right)));
        var peak = 0f;
        var landed = false;

        for (var i = 0; i < 240; i++)
        {
            world.Step(Dt);
            landed |= world.GetVelocity(dumbbell).Linear.Y > 0.05f;
            peak = landed ? MathF.Max(peak, world.GetPose(dumbbell).Position.Y - 0.5f) : peak;
        }

        Assert.InRange(peak, 0.6f * 0.6f * 6f * 0.5f, 6f);
    }

    /// <summary>A bouncy ball rebounds higher than an absorbing one.</summary>
    [Fact]
    public void Bounciness_OrdersRebounds()
    {
        var rubber = new MaterialDesc(0.6f, 0.6f, 0.9f, BounceCombine: PhysicsMaterialCombine.Maximum);
        Ground(material: new MaterialDesc(0.6f, 0.6f, 0f));
        var bouncy = Ball(new Vector3(-3, 3, 0), 1, rubber);
        var dull = Ball(new Vector3(3, 3, 0), 2, new MaterialDesc(0.05f, 0.05f, 0f));
        float bouncyPeak = 0f, dullPeak = 0f;
        var landed = false;

        for (var i = 0; i < 180; i++)
        {
            world.Step(Dt);
            landed |= world.GetVelocity(bouncy).Linear.Y > 0.5f;
            if (landed)
            {
                bouncyPeak = MathF.Max(bouncyPeak, world.GetPose(bouncy).Position.Y);
                dullPeak = MathF.Max(dullPeak, world.GetPose(dull).Position.Y);
            }
        }

        Assert.True(bouncyPeak > 1.5f, $"rubber rebounded to {bouncyPeak}");
        Assert.True(dullPeak < 0.7f, $"ice rebounded to {dullPeak}");
    }

    /// <summary>Rays hit the nearest accepted collider and skip filtered layers and triggers.</summary>
    [Fact]
    public void CastRay_RespectsFiltersAndTriggers()
    {
        Ground(id: 100);
        world.CreateStatic(new StaticDesc(new PhysicsPose(new Vector3(0, 2, 0), Quaternion.Identity),
            [Collider(ShapeDesc.Box(Vector3.One), 7, filter: new CollisionFilter(1u << 3, uint.MaxValue))]));
        world.CreateStatic(new StaticDesc(new PhysicsPose(new Vector3(0, 4, 0), Quaternion.Identity),
            [Collider(ShapeDesc.Box(Vector3.One), 8, trigger: true)]));
        var down = new RaycastInput(new Vector3(0, 10, 0), new Vector3(0, -10, 0), CollisionFilter.Default);

        var closest = new ClosestHitCollector();
        world.CastRay(down, ref closest);
        var masked = new ClosestHitCollector();
        world.CastRay(down with { Filter = new CollisionFilter(uint.MaxValue, ~(1u << 3)) }, ref masked);
        var withTriggers = new ClosestHitCollector();
        world.CastRay(down with { HitTriggers = true }, ref withTriggers);

        Assert.Equal(7ul, closest.Hit.ColliderData);
        Assert.Equal(2.5f, closest.Hit.Point.Y, 0.01f);
        Assert.Equal(Vector3.UnitY, closest.Hit.Normal);
        Assert.Equal(100ul, masked.Hit.ColliderData);
        Assert.Equal(8ul, withTriggers.Hit.ColliderData);
    }

    /// <summary>Overlaps report each touching collider of a compound, and only those.</summary>
    [Fact]
    public void Overlap_ReportsCompoundChildren()
    {
        var left = Collider(ShapeDesc.Box(Vector3.One), 1, local: new PhysicsPose(new Vector3(-2, 0, 0), Quaternion.Identity));
        var right = Collider(ShapeDesc.Box(Vector3.One), 2, local: new PhysicsPose(new Vector3(2, 0, 0), Quaternion.Identity));
        world.CreateStatic(new StaticDesc(PhysicsPose.Identity, [left, right]));
        var all = new AllHitsCollector();

        world.Overlap(new OverlapInput(ShapeDesc.Sphere(0.4f),
            new PhysicsPose(new Vector3(2.6f, 0, 0), Quaternion.Identity), CollisionFilter.Default), ref all);
        var none = new AllHitsCollector();
        world.Overlap(new OverlapInput(ShapeDesc.Sphere(0.4f),
            new PhysicsPose(new Vector3(0, 0, 0), Quaternion.Identity), CollisionFilter.Default), ref none);

        Assert.Equal([2ul], all.Hits.Select(static h => h.ColliderData));
        Assert.Empty(none.Hits);
    }

    /// <summary>A sphere swept down stops on the ground with an upward normal.</summary>
    [Fact]
    public void CastShape_ReportsFractionAndNormal()
    {
        Ground();
        var closest = new ClosestHitCollector();

        world.CastShape(new ShapeCastInput(ShapeDesc.Sphere(0.5f), new Vector3(0, 5.5f, 0), Quaternion.Identity,
            new Vector3(0, -4.5f, 0), CollisionFilter.Default), ref closest);

        Assert.True(closest.HasHit);
        Assert.Equal(0.5f, closest.Hit.Fraction, 0.01f);
        Assert.True(Vector3.Dot(closest.Hit.Normal, Vector3.UnitY) > 0.99f, $"normal {closest.Hit.Normal}");
    }

    /// <summary>Distances are positive for gaps and negative for penetration, with the normal towards the first shape.</summary>
    [Fact]
    public void CalculateDistance_ReportsGapsAndPenetration()
    {
        var box = ShapeDesc.Box(Vector3.One);
        var sphere = ShapeDesc.Sphere(0.5f);

        Assert.True(world.CalculateDistance(sphere, new PhysicsPose(new Vector3(0, 1.5f, 0), Quaternion.Identity),
            box, PhysicsPose.Identity, 2f, out var gap));
        Assert.True(world.CalculateDistance(sphere, new PhysicsPose(new Vector3(0, 0.8f, 0), Quaternion.Identity),
            box, PhysicsPose.Identity, 0f, out var overlap));
        Assert.False(world.CalculateDistance(sphere, new PhysicsPose(new Vector3(0, 5f, 0), Quaternion.Identity),
            box, PhysicsPose.Identity, 1f, out _));

        Assert.Equal(0.5f, gap.Distance, 0.01f);
        Assert.Equal(-0.2f, overlap.Distance, 0.01f);
        Assert.True(Vector3.Dot(overlap.Normal, Vector3.UnitY) > 0.99f, $"normal {overlap.Normal}");
    }

    /// <summary>A body passing through a trigger reports enter, stay and exit in order.</summary>
    [Fact]
    public void Triggers_ReportEnterStayExit()
    {
        world.CreateStatic(new StaticDesc(PhysicsPose.Identity, [Collider(ShapeDesc.Box(new Vector3(4, 1, 4)), 50, trigger: true)]));
        var ball = Ball(new Vector3(0, 3, 0), 1);
        var states = new List<PhysicsEventState>();

        for (var i = 0; i < 120; i++)
        {
            world.Step(Dt);
            foreach (var trigger in world.Triggers)
            {
                Assert.Contains(50ul, new[] { trigger.ColliderA, trigger.ColliderB });
                if (states.Count == 0 || states[^1] != trigger.State)
                {
                    states.Add(trigger.State);
                }
            }
        }

        Assert.Equal([PhysicsEventState.Enter, PhysicsEventState.Stay, PhysicsEventState.Exit], states);
        Assert.True(world.GetPose(ball).Position.Y < -2f);
    }

    /// <summary>A resting body keeps reporting stay after it falls asleep, and exit once it is removed.</summary>
    [Fact]
    public void Contacts_StayWhileAsleepAndExitOnRemoval()
    {
        Ground();
        var ball = Ball(new Vector3(0, 1, 0), 1);
        Run(300);
        Assert.False(world.IsAwake(ball));

        world.Step(Dt);
        var stay = world.Contacts.ToArray();
        world.Remove(ball);
        world.Step(Dt);
        var exit = world.Contacts.ToArray();

        Assert.Equal(PhysicsEventState.Stay, Assert.Single(stay).State);
        Assert.Equal(PhysicsEventState.Exit, Assert.Single(exit).State);
    }

    /// <summary>
    /// Pairs whose bodies do not report contacts are counted but not reported, and a pair is reported again once
    /// either body reports.
    /// </summary>
    [Fact]
    public void ContactReporting_SkipsEventsButKeepsTheCount()
    {
        var ground = Ground();
        var ball = Ball(new Vector3(0, 0.5f, 0), 1);
        world.SetContactReporting(ground, false);
        world.SetContactReporting(ball, false);
        world.Step(Dt);
        Assert.Empty(world.Contacts.ToArray());
        Assert.Equal(1, world.ContactPairCount);

        world.SetContactReporting(ball, true);
        world.Step(Dt);
        Assert.Equal(PhysicsEventState.Enter, Assert.Single(world.Contacts.ToArray()).State);
        Assert.Equal(1, world.ContactPairCount);
    }

    /// <summary>Glancing contacts beyond the contact offset are not reported, and a first touch is an enter.</summary>
    [Fact]
    public void Contacts_EnterOnTouchWithImpulse()
    {
        Ground();
        Ball(new Vector3(0, 2, 0), 1);
        ContactEvent? enter = null;

        for (var i = 0; i < 60 && enter is null; i++)
        {
            world.Step(Dt);
            foreach (var contact in world.Contacts)
            {
                enter ??= contact.State == PhysicsEventState.Enter ? contact : null;
            }
        }

        Assert.NotNull(enter);
        Assert.True(enter.Value.PointCount > 0);
        Assert.True(enter.Value.Impulse.Length() > 0f);
    }

    /// <summary>Mesh triangles with counter-clockwise front faces block rays and bodies from above.</summary>
    [Fact]
    public void TriangleMesh_FrontFacesUp()
    {
        Vector3[] vertices = [new(-5, 0, -5), new(5, 0, -5), new(5, 0, 5), new(-5, 0, 5)];
        int[] indices = [0, 2, 1, 0, 3, 2];
        world.CreateStatic(new StaticDesc(PhysicsPose.Identity,
            [Collider(ShapeDesc.TriangleMesh(vertices, indices), 9)]));
        var ball = Ball(new Vector3(0, 2, 0), 1);
        var hit = new ClosestHitCollector();

        world.CastRay(new RaycastInput(new Vector3(1, 5, 1), new Vector3(1, -5, 1), CollisionFilter.Default), ref hit);
        Run(120);

        Assert.Equal(9ul, hit.Hit.ColliderData);
        Assert.Equal(0.5f, world.GetPose(ball).Position.Y, 0.05f);
    }

    /// <summary>A kinematic body moved by targets pushes a dynamic body and stops when targets stop.</summary>
    [Fact]
    public void KinematicTarget_PushesDynamics()
    {
        Ground();
        var pusher = world.CreateBody(new BodyDesc
        {
            Motion = MotionType.Kinematic,
            Pose = new PhysicsPose(new Vector3(-3, 0.5f, 0), Quaternion.Identity),
            Colliders = [Collider(ShapeDesc.Box(Vector3.One), 1)],
            SleepThreshold = 0.01f,
        });
        var crate = world.CreateBody(BodyDesc.Dynamic(new PhysicsPose(new Vector3(0, 0.5f, 0), Quaternion.Identity),
            Collider(ShapeDesc.Box(Vector3.One), 2)));

        for (var i = 1; i <= 120; i++)
        {
            world.SetKinematicTarget(pusher, new PhysicsPose(new Vector3(-3 + i * 0.03f, 0.5f, 0), Quaternion.Identity), Dt);
            world.Step(Dt);
        }

        world.Step(Dt);

        Assert.Equal(0.6f, world.GetPose(pusher).Position.X, 0.01f);
        Assert.Equal(Vector3.Zero, world.GetVelocity(pusher).Linear);
        Assert.True(world.GetPose(crate).Position.X > 1.4f, $"crate at {world.GetPose(crate).Position}");
    }

    /// <summary>Locked axes hold position and rotation under off-center impulses.</summary>
    [Fact]
    public void Locks_HoldAxes()
    {
        var body = world.CreateBody(BodyDesc.Dynamic(PhysicsPose.Identity, Collider(ShapeDesc.Box(Vector3.One), 1)) with
        {
            Locks = AxisLocks.LinearY | AxisLocks.AngularX | AxisLocks.AngularY | AxisLocks.AngularZ,
        });

        world.ApplyImpulse(body, new Vector3(1, 0, 0), new Vector3(0, 0.5f, 0.5f));
        Run(30);

        var pose = world.GetPose(body);
        Assert.Equal(0f, pose.Position.Y, 0.001f);
        Assert.True(pose.Position.X > 0.3f);
        Assert.Equal(1f, MathF.Abs(pose.Orientation.W), 0.001f);
    }

    /// <summary>Compound bodies keep their origin on the node and their center of mass on the colliders.</summary>
    [Fact]
    public void Compound_KeepsOriginAndCenterOfMass()
    {
        var body = world.CreateBody(BodyDesc.Dynamic(new PhysicsPose(new Vector3(1, 2, 3), Quaternion.Identity),
            Collider(ShapeDesc.Box(Vector3.One), 1, local: new PhysicsPose(new Vector3(0, 0, 2), Quaternion.Identity)),
            Collider(ShapeDesc.Box(Vector3.One), 2, local: new PhysicsPose(new Vector3(0, 0, 4), Quaternion.Identity))) with
        {
            GravityScale = 0f,
        });

        Assert.Equal(new Vector3(1, 2, 3), world.GetPose(body).Position);
        Assert.Equal(new Vector3(1, 2, 6), world.GetCenterOfMass(body));
        Assert.Equal(1f, world.GetMass(body), 0.001f);
    }

    /// <summary>The worker setting picks the thread count, capped by the processors, and a capped world still steps.</summary>
    [Fact]
    public void WorkerThreads_SelectTheDispatcherSize()
    {
        var processors = Environment.ProcessorCount;
        var settings = new PhysicsSettings { WorkerThreads = 2 }.ToWorldSettings();

        Assert.Equal(1, BepuPhysicsWorld.WorkerCount(settings with { Multithreaded = false }));
        Assert.Equal(Math.Min(2, processors), BepuPhysicsWorld.WorkerCount(settings));
        Assert.Equal(processors, BepuPhysicsWorld.WorkerCount(settings with { WorkerThreads = 0 }));
        Assert.Equal(processors, BepuPhysicsWorld.WorkerCount(settings with { WorkerThreads = processors + 8 }));
        using var capped = new BepuPhysicsWorld(settings);
        var ball = capped.CreateBody(BodyDesc.Dynamic(new PhysicsPose(Vector3.UnitY, Quaternion.Identity),
            new ColliderDesc(capped.CreateShape(ShapeDesc.Sphere(0.5f)), PhysicsPose.Identity, CollisionFilter.Default,
                MaterialDesc.Default, false, 1)));
        capped.Step(Dt);
        Assert.True(capped.GetPose(ball).Position.Y < 1f);
    }

    /// <summary>Removing bodies and disposing the world frees everything.</summary>
    [Fact]
    public void Remove_And_Dispose()
    {
        var ground = Ground();
        var ball = Ball(Vector3.One, 1);

        world.Remove(ball);
        world.Remove(ball);

        Assert.Equal(1, world.BodyCount);
        Assert.True(world.Contains(ground));
        Assert.False(world.Contains(ball));
        world.Dispose();
        Assert.Throws<ObjectDisposedException>(() => world.Step(Dt));
    }
}
