using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>Query filtering and bounded ray hits through the scene's public API.</summary>
[Collection("Serial")]
public sealed class PhysicsQueryAcceptanceTests : IDisposable
{
    readonly PhysicsScene scene;
    readonly ServiceProvider services;
    readonly Node root = new();

    /// <summary>Creates a single-threaded world with solids and triggers on distinct layers.</summary>
    public PhysicsQueryAcceptanceTests()
    {
        var settings = new AppSettings();
        settings.Loaded.Use(new PhysicsSettings { Multithreaded = false, Gravity = Vector3.Zero });
        scene = new PhysicsScene(settings, factory: new BepuPhysicsWorldFactory());
        services = new ServiceCollection().AddSingleton(scene).BuildServiceProvider();
    }

    /// <inheritdoc />
    public void Dispose()
    {
        root.OnDestroy();
        services.Dispose();
        scene.Dispose();
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

    void Start()
    {
        root.Awake(null, services);
        scene.Simulate(1f / 60f);
    }

    /// <summary>Every sweep and overlap respects masks and explicit or project-default trigger policies.</summary>
    [Theory]
    [InlineData("sphere-cast")]
    [InlineData("box-cast")]
    [InlineData("capsule-cast")]
    [InlineData("sphere-overlap")]
    [InlineData("box-overlap")]
    [InlineData("capsule-overlap")]
    public void Queries_RespectLayersAndTriggerPolicies(string query)
    {
        var solid = new BoxColliderComponent { ExcludeLayers = LayerMask.Everything };
        var trigger = new BoxColliderComponent { IsTrigger = true };
        Add("solid", Vector3.Zero, 7, solid);
        Add("trigger", new Vector3(0, 2, 0), 9, trigger);
        Start();

        var both = LayerMask.FromLayer(7) | LayerMask.FromLayer(9);
        ColliderComponent[] all = query.EndsWith("overlap", StringComparison.Ordinal) ? [trigger, solid] : [trigger];
        Assert.Equal(all, Query(query, both, QueryTriggerInteraction.Collide));
        Assert.Equal([solid], Query(query, both, QueryTriggerInteraction.Ignore));
        Assert.Equal([solid], Query(query, LayerMask.FromLayer(7), QueryTriggerInteraction.Collide));
        Assert.Empty(Query(query, LayerMask.FromLayer(12), QueryTriggerInteraction.Collide));
        scene.Settings.QueriesHitTriggers = false;
        Assert.Equal([solid], Query(query, both, QueryTriggerInteraction.UseGlobal));
        scene.Settings.QueriesHitTriggers = true;
        Assert.Equal(all, Query(query, both, QueryTriggerInteraction.UseGlobal));
    }

    ColliderComponent[] Query(string query, LayerMask mask, QueryTriggerInteraction triggers)
    {
        var top = new Vector3(0, 5, 0);
        var center = new Vector3(0, 1, 0);
        RaycastHit hit;
        switch (query)
        {
            case "sphere-cast":
                return scene.SphereCast(top, 0.2f, -Vector3.UnitY, out hit, 10, mask, triggers) ? [hit.Collider] : [];
            case "box-cast":
                return scene.BoxCast(top, new Vector3(0.2f), -Vector3.UnitY, out hit, Quaternion.Identity, 10, mask,
                    triggers) ? [hit.Collider] : [];
            case "capsule-cast":
                return scene.CapsuleCast(top, top + Vector3.UnitX, 0.2f, -Vector3.UnitY, out hit, 10, mask, triggers)
                    ? [hit.Collider] : [];
            default:
                var overlaps = query switch
                {
                    "sphere-overlap" => scene.OverlapSphere(center, 2, mask, triggers),
                    "box-overlap" => scene.OverlapBox(center, new Vector3(2), Quaternion.Identity, mask, triggers),
                    _ => scene.OverlapCapsule(Vector3.Zero, new Vector3(0, 2, 0), 0.2f, mask, triggers),
                };
                return [.. overlaps.OrderByDescending(static collider => collider.Node!.Position.Y)];
        }
    }

    /// <summary>A layer edit without a transform change updates static and compound query filters on the next tick.</summary>
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LayerChanges_UpdateExistingColliderFilters(bool compound)
    {
        var collider = new BoxColliderComponent();
        var node = Add("shape", Vector3.Zero, 3, collider);
        if (compound)
        {
            var body = Add("body", Vector3.Zero, 0, new RigidbodyComponent { IsKinematic = true });
            root.Children.Remove(node);
            node.Parent = body;
            body.Children.Add(node);
        }

        Start();
        var top = new Vector3(0, 5, 0);
        Assert.True(scene.Raycast(top, -Vector3.UnitY, layerMask: LayerMask.FromLayer(3)));
        node.PhysicsLayer = 5;
        scene.Simulate(1f / 60f);

        Assert.False(scene.Raycast(top, -Vector3.UnitY, layerMask: LayerMask.FromLayer(3)));
        Assert.True(scene.Raycast(top, -Vector3.UnitY, layerMask: LayerMask.FromLayer(5)));
        Assert.Equal([collider], scene.OverlapSphere(Vector3.Zero, 0.1f, LayerMask.FromLayer(5)));
    }

    /// <summary>A short buffer gets the closest distinct hits in order without allocating after warm-up.</summary>
    [Fact]
    public void RaycastNonAlloc_ReturnsNearestHitsWithoutAllocating()
    {
        for (var i = 0; i < 8; i++)
        {
            Add("box" + i, new Vector3(0, i * 2, 0), i % 2, new BoxColliderComponent());
        }

        Start();
        var top = new Vector3(0, 20, 0);
        var expected = scene.RaycastAll(top, -Vector3.UnitY).Take(3).ToArray();
        var hits = new RaycastHit[3];
        for (var i = 0; i < 100; i++)
        {
            scene.RaycastNonAlloc(top, -Vector3.UnitY, hits);
        }

        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var i = 0; i < 100; i++)
        {
            scene.RaycastNonAlloc(top, -Vector3.UnitY, hits);
        }

        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.Equal(expected, hits);
        Assert.Equal(0, allocated);
        Assert.Equal(0, scene.RaycastNonAlloc(top, -Vector3.UnitY, []));
        Assert.Equal(0, scene.RaycastNonAlloc(top, Vector3.Zero, hits));
        Assert.Equal(0, scene.RaycastNonAlloc(top, -Vector3.UnitY, hits, layerMask: LayerMask.Nothing));
    }

    /// <summary>Several triangles in one mesh consume one buffer slot, leaving room for another collider.</summary>
    [Fact]
    public void RaycastNonAlloc_DeduplicatesMeshTrianglesAndSkipsContainingShapes()
    {
        var mesh = new MeshColliderComponent();
        mesh.SetGeometry([new(-2, 0, -2), new(0, 0, 2), new(2, 0, -2),
            new(-2, 2, -2), new(0, 2, 2), new(2, 2, -2),
            new(-2, 4, -2), new(0, 4, 2), new(2, 4, -2)], [0, 1, 2, 3, 4, 5, 6, 7, 8]);
        Add("mesh", Vector3.Zero, 0, mesh);
        var box = new BoxColliderComponent();
        Add("box", new Vector3(0, 3, 0), 0, box);
        var trigger = new BoxColliderComponent { IsTrigger = true };
        Add("trigger", new Vector3(0, 7, 0), 1, trigger);
        var top = new Vector3(0, 10, 0);
        Add("containing", top, 0, new BoxColliderComponent());
        Start();

        var buffer = new RaycastHit[2];
        Assert.Equal(2, scene.RaycastNonAlloc(top, -Vector3.UnitY, buffer, layerMask: LayerMask.FromLayer(0)));
        Assert.Equal([mesh, box], buffer.Select(static hit => hit.Collider));
        Assert.Equal(6f, buffer[0].Distance, 0.01f);
        Assert.Equal(6.5f, buffer[1].Distance, 0.01f);
        Assert.Equal(1, scene.RaycastNonAlloc(top, -Vector3.UnitY, buffer, layerMask: LayerMask.FromLayer(1)));
        Assert.Same(trigger, buffer[0].Collider);
        Assert.Equal(0, scene.RaycastNonAlloc(top, -Vector3.UnitY, buffer, layerMask: LayerMask.FromLayer(1),
            queryTriggerInteraction: QueryTriggerInteraction.Ignore));
    }

    /// <summary>A sleeping body changes its physical contacts when its node's layer changes.</summary>
    [Fact]
    public void LayerChanges_UpdateContactsOnSleepingBodies()
    {
        Add("ground", new Vector3(0, -0.5f, 0), 1,
            new BoxColliderComponent { Size = new Vector3(20, 1, 20) });
        var body = new RigidbodyComponent();
        var node = Add("body", new Vector3(0, 0.5f, 0), 2, body, new BoxColliderComponent());
        scene.Gravity = new Vector3(0, -9.81f, 0);
        scene.IgnoreLayerCollision(1, 3);
        Start();
        for (var i = 0; i < 120; i++)
        {
            scene.Simulate(1f / 60f);
        }

        Assert.True(body.IsSleeping());
        node.PhysicsLayer = 3;
        for (var i = 0; i < 60; i++)
        {
            scene.Simulate(1f / 60f);
        }

        Assert.True(node.Position.Y < -1f);
    }

    /// <summary>Repeated hits for one collider keep its closest face without displacing other accepted colliders.</summary>
    [Fact]
    public void BoundedCollector_KeepsNearestFacePerCollider()
    {
        var first = new BoxColliderComponent();
        var second = new BoxColliderComponent();
        var buffer = new RaycastHit[2];
        var collector = new PhysicsScene.BoundedRayCollector(buffer, 2, 10,
            new Dictionary<ulong, ColliderComponent> { [1] = first, [2] = second });
        var hit = new PhysicsQueryHit(default, 1, Vector3.Zero, Vector3.UnitY, 0.6f);

        Assert.True(collector.AddHit(hit));
        Assert.True(collector.AddHit(hit with { ColliderData = 2, Fraction = 0.8f }));
        Assert.False(collector.AddHit(hit with { Fraction = 0.7f }));
        Assert.True(collector.AddHit(hit with { Fraction = 0.2f }));
        Assert.Equal(2, collector.Count);
        Assert.Equal([first, second], buffer.Select(static value => value.Collider));
        Assert.Equal(2f, buffer[0].Distance);
        Assert.False(collector.AddHit(hit with { ColliderData = 3 }));
        Assert.False(collector.AddHit(hit with { Fraction = 0f }));
        Assert.False(collector.AddHit(hit with { Fraction = 1f }));
    }

    /// <summary>An unchanged sibling's geometry remains reusable when another collider requires synchronization.</summary>
    [Fact]
    public void ColliderState_UnchangedGeometryDoesNotRequireRebuilding()
    {
        var node = new Node();
        var collider = new BoxColliderComponent();
        node.AddComponent(collider);
        var record = new PhysicsBody(node, null);
        record.Colliders.Add(collider);
        var pose = record.NodePose();
        var (local, scale) = record.Geometry(collider, pose);
        record.ColliderStates.Add((node.TransformVersion, local, scale, node.PhysicsLayer));

        Assert.False(record.GeometryChanged(0, pose));
    }
}
