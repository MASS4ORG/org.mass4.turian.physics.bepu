using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>Qualifies skeleton fitting, pose transfer, physics constraints and ownership of generated ragdolls.</summary>
public sealed class RagdollTests : IDisposable
{
    const float Dt = 1f / 60;
    readonly Node root = new();
    readonly PhysicsScene scene;
    readonly ServiceProvider services;
    long tick;

    /// <summary>Creates a deterministic scene with a floor and an explicit physics backend.</summary>
    public RagdollTests()
    {
        var settings = new AppSettings();
        settings.Loaded.Use(new PhysicsSettings { Multithreaded = false });
        scene = new PhysicsScene(settings, factory: new BepuPhysicsWorldFactory());
        services = new ServiceCollection().AddSingleton(scene).BuildServiceProvider();
        var floor = Add(root, "Floor", new(0, -0.5f, 0));
        floor.AddComponent(new BoxColliderComponent { Size = new(20, 1, 20) });
        root.Awake(null, services);
    }

    /// <inheritdoc/>
    public void Dispose()
    {
        root.DestroyImmediate();
        scene.Dispose();
        services.Dispose();
    }

    static Node Add(Node parent, string name, Vector3 position)
    {
        var node = new Node { Name = name, Parent = parent, Position = position };
        parent.Children.Add(node);
        node.Awake(parent);
        return node;
    }

    (Node Model, Node Hips, Node Chest, Node Head) Skeleton()
    {
        var model = Add(root, "Model", new(0, 3, 0));
        model.Transform = model.Transform with { Rotation = new(0, 30, 0), Scale = new(1.2f, 0.8f, 1.1f) };
        var hips = Add(model, "Hips", Vector3.Zero);
        var helper = Add(hips, "Helper", new(0, 0.2f, 0));
        var chest = Add(helper, "Chest", new(0, 0.4f, 0));
        var head = Add(chest, "Head", new(0, 0.5f, 0));
        return (model, hips, chest, head);
    }

    void Step(int count, RagdollRig? rig = null)
    {
        for (var i = 0; i < count; i++)
        {
            scene.FixedTick(++tick, Dt);
            rig?.SyncToBones();
        }
    }

    /// <summary>Unordered selected bones connect through skipped helpers without changing the skeleton hierarchy.</summary>
    [Fact]
    public void FittingPreservesHierarchyAndDistributesMass()
    {
        var (model, hips, chest, head) = Skeleton();
        var parent = chest.Parent;
        using var rig = RagdollBuilder.Create(model,
        [
            new() { Bone = head, MassWeight = 1, Radius = 0.08f },
            new() { Bone = hips, MassWeight = 3, EndBone = chest },
            new() { Bone = chest, MassWeight = 2 },
        ], new() { TotalMass = 60 });
        Assert.Equal(new[] { hips, chest, head }, rig.Parts.Select(static part => part.Bone));
        Assert.Equal(new[] { 30f, 20f, 10f }, rig.Parts.Select(static part => part.Body.Mass));
        Assert.Null(rig.Parts[0].Joint);
        Assert.Same(rig.Parts[0].Body, rig.Parts[1].Joint!.ConnectedBody);
        Assert.Same(rig.Parts[1].Body, rig.Parts[2].Joint!.ConnectedBody);
        Assert.Equal(0.08f, rig.Parts[2].Collider.Radius);
        Assert.Equal(0.48f, rig.Parts[0].Collider.Height, 0.001f);
        Assert.Equal(0.25f, rig.Parts[2].Collider.Height, 0.001f);
        Assert.True(rig.IsKinematic);
        Step(1);
        Assert.Equal(1, scene.BodyCount);
        Assert.Equal(0, scene.JointCount);
        Assert.Same(parent, chest.Parent);
        Assert.Null(hips.GetComponent<RigidbodyComponent>());
    }

    /// <summary>Dynamic proxies fall, hold their joint anchors and drive the original bones at their preserved scales.</summary>
    [Fact]
    public void DynamicRigFallsAndDrivesBonesWithoutReparenting()
    {
        var (model, hips, chest, head) = Skeleton();
        var scales = new[] { hips.GlobalTransform.Scale, chest.GlobalTransform.Scale, head.GlobalTransform.Scale };
        using var rig = RagdollBuilder.FromSkeleton(model, [head, hips, chest], new() { StartKinematic = false });
        var initial = hips.GlobalTransform.Position.Y;
        Step(180, rig);
        Assert.Equal(3, rig.Parts.Count);
        Assert.Equal(4, scene.BodyCount);
        Assert.Equal(2, scene.JointCount);
        Assert.True(hips.GlobalTransform.Position.Y < initial - 1);
        Assert.All(rig.Parts, static part =>
        {
            Assert.InRange(Vector3.Distance(part.Bone.GlobalTransform.Position, part.Body.Position), 0, 0.001f);
            Assert.True(part.Body.Position.Y > -0.3f);
            Assert.Same(part.Body.Node, part.Collider.AttachedRigidbody!.Node);
        });
        for (var i = 0; i < rig.Parts.Count; i++)
        {
            Assert.Equal(scales[i], rig.Parts[i].Bone.GlobalTransform.Scale);
        }
        foreach (var part in rig.Parts.Where(static part => part.Joint is not null))
        {
            var joint = part.Joint!;
            var target = Vector3.Transform(joint.ConnectedAnchor, joint.ConnectedBody!.Node!.GlobalTransform.Matrix4X4());
            Assert.InRange(Vector3.Distance(target, part.Body.Position), 0, 0.06f);
        }
        var proxy = rig.Parts[0].Body.Position;
        rig.SyncFromBones();
        Assert.Equal(proxy, rig.Parts[0].Body.Position);
    }

    /// <summary>Animation/physics transitions clear velocities, remove physics membership and restore the captured pose.</summary>
    [Fact]
    public void ModeChangesAndBindPoseRestorationDoNotLeaveGhostBodies()
    {
        var (model, hips, chest, head) = Skeleton();
        var original = hips.Transform;
        using var rig = RagdollBuilder.FromSkeleton(model, [hips, chest, head]);
        hips.Position += Vector3.UnitX;
        rig.SyncFromBones();
        Assert.Equal(hips.GlobalTransform.Position, rig.Parts[0].Body.Node!.GlobalTransform.Position);
        rig.SyncToBones();
        rig.SetKinematic(false);
        rig.SetKinematic(false);
        Step(30, rig);
        rig.Parts[0].Body.LinearVelocity = Vector3.UnitX;
        rig.SetKinematic(true);
        Step(1);
        Assert.Equal(1, scene.BodyCount);
        Assert.Equal(0, scene.JointCount);
        Assert.Equal(Vector3.Zero, rig.Parts[0].Body.LinearVelocity);
        rig.RestoreBindPose();
        Assert.Equal(original, hips.Transform);
        rig.SetKinematic(false);
        Step(1);
        Assert.Equal(4, scene.BodyCount);
        rig.Dispose();
        rig.Dispose();
        Step(1);
        Assert.Equal(1, scene.BodyCount);
        Assert.False(hips.IsDestroyed);
        Assert.True(rig.Root.IsDestroyed);
        Assert.Throws<ObjectDisposedException>(() => rig.SyncToBones());
        Assert.Throws<ObjectDisposedException>(() => rig.SyncFromBones());
        Assert.Throws<ObjectDisposedException>(() => rig.SetKinematic(false));
        Assert.Throws<ObjectDisposedException>(() => rig.RestoreBindPose());
    }

    /// <summary>Zero-length and downward segments produce finite capsules, and an unawoken hierarchy can be set up.</summary>
    [Fact]
    public void DegenerateSegmentsUseSafeDefaultsWithoutABackend()
    {
        var model = new Node();
        var a = Add(model, "a", Vector3.Zero);
        var b = Add(a, "b", Vector3.Zero);
        b.Orientation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI);
        using var rig = RagdollBuilder.FromSkeleton(model, [a, b], new() { StartKinematic = false });
        Assert.All(rig.Parts, static part => Assert.Equal(0.25f, part.Collider.Height));
        Assert.All(rig.Parts, static part => Assert.True(float.IsFinite(part.Collider.Node!.GlobalTransform.Orientation.W)));
        model.Awake(null);
        rig.SyncToBones();
        Assert.Equal(2, rig.Parts.Count);
    }

    /// <summary>Invalid definitions fail before creating a proxy group or modifying any original bone.</summary>
    [Fact]
    public void InvalidSkeletonAndSettingsAreRejectedAtomically()
    {
        var (model, hips, chest, head) = Skeleton();
        var count = root.Children.Count;
        Assert.Throws<ArgumentException>(() => RagdollBuilder.FromSkeleton(model, []));
        Assert.Throws<ArgumentException>(() => RagdollBuilder.FromSkeleton(model, [hips, hips]));
        Assert.Throws<ArgumentException>(() => RagdollBuilder.FromSkeleton(model, [new Node()]));
        Assert.Throws<ArgumentException>(() => RagdollBuilder.FromSkeleton(root, [root]));
        foreach (var settings in new[]
                 {
                     new RagdollSettings { TotalMass = 0 }, new() { Radius = float.NaN },
                     new() { TerminalLength = -1 }, new() { SwingLimit = 181 }, new() { SwingLimit = float.NaN },
                 })
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => RagdollBuilder.FromSkeleton(model, [hips], settings));
        }
        Assert.Throws<ArgumentException>(() => RagdollBuilder.Create(model, [new() { Bone = hips, EndBone = model }]));
        Assert.Throws<ArgumentOutOfRangeException>(() => RagdollBuilder.Create(model, [new() { Bone = hips, MassWeight = 0 }]));
        Assert.Throws<ArgumentOutOfRangeException>(() => RagdollBuilder.Create(model, [new() { Bone = hips, Radius = -1 }]));
        var position = hips.Position;
        hips.Position = new(float.NaN, 0, 0);
        Assert.Throws<ArgumentException>(() => RagdollBuilder.FromSkeleton(model, [hips]));
        hips.Position = position;
        var scale = root.Scale;
        root.Scale = Vector3.Zero;
        Assert.Throws<ArgumentException>(() => RagdollBuilder.FromSkeleton(model, [hips]));
        root.Scale = scale;
        Assert.Equal(count, root.Children.Count);
        Assert.Single(model.Children);
        Assert.Same(chest, head.Parent);
    }
}
