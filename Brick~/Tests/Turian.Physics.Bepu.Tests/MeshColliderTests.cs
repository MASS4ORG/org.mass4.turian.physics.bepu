using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>Mesh colliders from imported models, script geometry and convex hulls.</summary>
public sealed class MeshColliderTests : IDisposable
{
    const double Dt = 1d / 60d;
    readonly string projectRoot = Directory.CreateTempSubdirectory("turian-mesh-collider-").FullName;
    readonly PhysicsScene scene;
    readonly Node root = new() { Name = "root" };

    /// <summary>Creates a single-threaded scene backed by Bepu.</summary>
    public MeshColliderTests()
    {
        var settings = new AppSettings();
        settings.Loaded.Use(new PhysicsSettings { Multithreaded = false });
        scene = new PhysicsScene(settings, null, null, new BepuPhysicsWorldFactory());
    }

    /// <inheritdoc />
    public void Dispose()
    {
        scene.Dispose();
        Directory.Delete(projectRoot, recursive: true);
    }

    // A 20 m square floor at y = 0, facing up.
    static readonly Vector3[] FloorVertices = [new(-10, 0, -10), new(10, 0, -10), new(10, 0, 10), new(-10, 0, 10)];
    static readonly int[] FloorIndices = [0, 2, 1, 0, 3, 2];

    static readonly Vector3[] CubePoints =
    [
        new(-0.5f, -0.5f, -0.5f), new(0.5f, -0.5f, -0.5f), new(0.5f, 0.5f, -0.5f), new(-0.5f, 0.5f, -0.5f),
        new(-0.5f, -0.5f, 0.5f), new(0.5f, -0.5f, 0.5f), new(0.5f, 0.5f, 0.5f), new(-0.5f, 0.5f, 0.5f),
    ];

    ServiceProvider Services(AssetDatabase? database = null)
    {
        var services = new ServiceCollection().AddSingleton(scene);
        if (database is not null)
        {
            services.AddSingleton(database);
        }

        return services.BuildServiceProvider();
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

    (Node Node, RigidbodyComponent Body) Ball(Vector3 position)
    {
        var body = new RigidbodyComponent();
        return (Add("ball", position, body, new SphereColliderComponent()), body);
    }

    void Run(int ticks)
    {
        for (var i = 0; i < ticks; i++)
        {
            scene.FixedTick(i + 1, Dt);
        }
    }

    /// <summary>A model's triangles, read from its glTF through a sibling model component, hold up a ball.</summary>
    [Fact]
    public void ImportedModel_FormsAStaticTriangleMesh()
    {
        var database = ImportFloor(out var modelId);
        using var services = Services(database);
        Add("floor", Vector3.Zero, new ModelComponent { Model = new AssetReference<ModelAsset>(modelId) },
            new MeshColliderComponent());
        var (ball, _) = Ball(new Vector3(0, 2, 0));
        root.Awake(null, services);

        Run(90);

        Assert.Equal(0.5f, ball.GlobalTransform.Position.Y, 0.05f);
        Assert.Equal(2, scene.BodyCount);
    }

    /// <summary>Script geometry follows the node's scale, and a mirrored floor still faces up.</summary>
    [Fact]
    public void ScriptGeometry_ScalesAndKeepsItsFacingWhenMirrored()
    {
        using var services = Services();
        var collider = new MeshColliderComponent();
        collider.SetGeometry(FloorVertices, FloorIndices);
        var floor = Add("floor", Vector3.Zero, collider);
        floor.Scale = new Vector3(-0.2f, 1, 0.2f);
        var (inside, _) = Ball(new Vector3(1, 2, 0));
        var (outside, _) = Ball(new Vector3(3, 2, 0));
        root.Awake(null, services);

        Run(90);

        Assert.Equal(0.5f, inside.GlobalTransform.Position.Y, 0.05f);
        Assert.True(outside.GlobalTransform.Position.Y < -1f, "the scaled floor is only 4 m wide");
        Assert.Equal(new Vector3(2, 0, 2), collider.Bounds.Max);
    }

    /// <summary>A convex mesh collider makes a dynamic body that tumbles and lands like a box.</summary>
    [Fact]
    public void ConvexHull_SimulatesOnADynamicBody()
    {
        using var services = Services();
        Add("ground", new Vector3(0, -0.5f, 0), new BoxColliderComponent { Size = new Vector3(20, 1, 20) });
        var hull = new MeshColliderComponent { Convex = true };
        hull.SetGeometry(CubePoints);
        var body = new RigidbodyComponent();
        var crate = Add("crate", new Vector3(0, 3, 0), body, hull);
        root.Awake(null, services);

        Run(180);

        Assert.Equal(0.5f, crate.GlobalTransform.Position.Y, 0.05f);
        Assert.True(body.IsSleeping());
    }

    /// <summary>A mesh collider without geometry is skipped until it gets some, and leaves when it loses it.</summary>
    [Fact]
    public void MissingGeometry_IsSkippedUntilSet()
    {
        using var services = Services();
        var collider = new MeshColliderComponent();
        Add("floor", Vector3.Zero, collider);
        var (ball, _) = Ball(new Vector3(0, 2, 0));
        root.Awake(null, services);

        Run(1);
        Assert.False(collider.HasShape);
        Assert.Equal(1, scene.BodyCount);
        Assert.False(scene.ComputePenetration(collider, Vector3.Zero, Quaternion.Identity,
            ball.GetComponent<SphereColliderComponent>()!, Vector3.Zero, Quaternion.Identity, out _, out _));
        collider.SetGeometry(FloorVertices, FloorIndices);
        Run(90);
        Assert.Equal(2, scene.BodyCount);
        Assert.Equal(0.5f, ball.GlobalTransform.Position.Y, 0.05f);

        collider.SetGeometry(null);
        Run(1);
        Assert.Equal(1, scene.BodyCount);
    }

    /// <summary>The references and the convex flag survive a save and load, and a missing mesh asset reads nothing.</summary>
    [Fact]
    public void References_RoundTripAndMissingAssetsReadNothing()
    {
        TypeRegistry.ScanAssembly(typeof(MeshColliderComponent).Assembly);
        var mesh = Guid.NewGuid();
        var model = Guid.NewGuid();
        var node = new Node();
        node.AddComponent(new MeshColliderComponent
        {
            Convex = true,
            Mesh = new AssetReference<MeshAsset>(mesh),
            Model = new AssetReference<ModelAsset>(model),
        });

        var loaded = Serializer.LoadData<Node>(Serializer.Serialize(node))!.GetComponent<MeshColliderComponent>()!;

        Assert.True(loaded.Convex);
        Assert.Equal(mesh, loaded.Mesh!.AssetId);
        Assert.Equal(model, loaded.Model!.AssetId);
        Assert.Null(MeshGeometry.Read(new AssetDatabase(), model, mesh));
        Assert.Null(MeshGeometry.Read(new AssetDatabase(), model, Guid.Empty));
        Assert.Null(MeshGeometry.Read(new AssetDatabase(), Guid.Empty, Guid.Empty));
    }

    AssetDatabase ImportFloor(out Guid modelId)
    {
        var folder = Directory.CreateDirectory(Path.Combine(projectRoot, "Assets")).FullName;
        var glb = Path.Combine(folder, "floor.glb");
        GltfGeometryFixture.Save(glb, [.. FloorVertices.Select(static v => new Vertex(v, Vector3.One))],
            [.. FloorIndices.Select(static i => (uint)i)]);
        var asset = (ModelImportAsset)new GltfModelImporter().CreateAsset(glb);
        Serializer.Save(glb + ".meta", asset);
        var database = new AssetDatabase();
        using var importer = new AssetImporter(NullLogger.Instance, database, new SettingsService());
        importer.GenerateMetaFiles(folder);
        modelId = asset.Id;
        return database;
    }
}
