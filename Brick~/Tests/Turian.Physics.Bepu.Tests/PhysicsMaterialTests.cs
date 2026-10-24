using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>Physics materials as shared data assets: references in scenes and settings, and the project default.</summary>
public sealed class PhysicsMaterialTests : IDisposable
{
    readonly string projectRoot = Directory.CreateTempSubdirectory("turian-physics-material-").FullName;

    /// <inheritdoc />
    public void Dispose() => Directory.Delete(projectRoot, recursive: true);

    /// <summary>
    /// A collider and the physics settings save a material as a reference. A scene resolves it on load; settings read
    /// without a loader keep it pending until the session's physics resolves it.
    /// </summary>
    [Fact]
    public async Task Materials_SerializeAsSharedReferences()
    {
        TypeRegistry.ScanAssembly(typeof(PhysicsMaterial).Assembly);
        var database = new AssetDatabase();
        var assets = Directory.CreateDirectory(Path.Combine(projectRoot, "Assets")).FullName;
        var path = Path.Combine(assets, "Rubber.asset");
        var meta = new DataAssetAsset { RelativePath = path };
        File.WriteAllText($"{path}.meta", Serializer.Serialize(meta));
        Assert.True(database.RegisterAsset(meta));
        Serializer.Save<DataAsset>(path, new PhysicsMaterial { Id = meta.Id, Bounciness = 0.8f });
        var loader = new RuntimeAssetLoader(database);
        var rubber = (await loader.LoadContentAsync<PhysicsMaterial>(meta.Id))!;
        var node = new Node();
        node.AddComponent(new SphereColliderComponent { Material = rubber });

        var json = Serializer.Serialize(node);
        var loaded = Serializer.LoadData<Node>(json, loader)!;
        var settings = (PhysicsSettings)Serializer.LoadData<DataAsset>(
            Serializer.Serialize<DataAsset>(new PhysicsSettings { DefaultMaterial = rubber }))!;
        Assert.Null(settings.DefaultMaterial);
        var app = new AppSettings();
        app.Loaded.Use(settings);
        using var scene = new PhysicsScene(app, factory: new BepuPhysicsWorldFactory(), loader: loader);

        Assert.Contains($"\"$ref\": \"{meta.Id}\"", json);
        Assert.DoesNotContain("Bounciness", json);
        Assert.Same(rubber, loaded.GetComponent<SphereColliderComponent>()!.Material);
        Assert.Same(rubber, scene.Settings.DefaultMaterial);
        Assert.Equal(new MaterialDesc(0.6f, 0.6f, 0.8f), rubber.ToDesc());
    }

    /// <summary>Colliders without a material use the project's default one.</summary>
    [Fact]
    public void DefaultMaterial_AppliesToCollidersWithoutOne()
    {
        var settings = new AppSettings();
        settings.Loaded.Use(new PhysicsSettings
        {
            Multithreaded = false,
            DefaultMaterial = new PhysicsMaterial { Bounciness = 0.9f, BounceCombine = PhysicsMaterialCombine.Maximum },
        });
        using var scene = new PhysicsScene(settings, null, null, new BepuPhysicsWorldFactory());
        using var services = new ServiceCollection().AddSingleton(scene).BuildServiceProvider();
        var root = new Node();
        var ground = new Node { Position = new Vector3(0, -0.5f, 0), Parent = root };
        ground.AddComponent(new BoxColliderComponent { Size = new Vector3(20, 1, 20) });
        var ball = new Node { Position = new Vector3(0, 3, 0), Parent = root };
        var body = new RigidbodyComponent();
        ball.AddComponent(body);
        ball.AddComponent(new SphereColliderComponent());
        root.Children.Add(ground);
        root.Children.Add(ball);
        root.Awake(null, services);
        var peak = 0f;
        var landed = false;

        for (var i = 0; i < 180; i++)
        {
            scene.FixedTick(i + 1, 1d / 60d);
            landed |= body.LinearVelocity.Y > 0.5f;
            peak = landed ? MathF.Max(peak, ball.GlobalTransform.Position.Y) : peak;
        }

        Assert.True(peak > 1.5f, $"the ball rebounded to {peak}");
    }
}
