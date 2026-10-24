using System.Reflection;
using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>The 3D physics bricks <c>org.mass4.turian.physics</c> and its Bepu backend.</summary>
[Collection(SerialTests.Name)]
public sealed class PhysicsBrickTests : IDisposable
{
    const string PhysicsId = "org.mass4.turian.physics";
    const string BepuId = "org.mass4.turian.physics.bepu";

    static readonly string BepuRoot = FindBrick();
    static readonly string PhysicsRoot = typeof(PhysicsBrickTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
        .Single(static item => item.Key == "PhysicsBrickRoot").Value!;

    static string FindBrick()
    {
        for (var directory = new DirectoryInfo(AppContext.BaseDirectory); directory is not null; directory = directory.Parent)
        {
            if (File.Exists(Path.Combine(directory.FullName, "package.json")))
            {
                return directory.FullName;
            }
        }

        throw new DirectoryNotFoundException("Bepu brick root was not found.");
    }

    readonly string project = Path.Combine(Path.GetTempPath(), $"turian-physics-brick-{Guid.NewGuid():N}");

    /// <summary>Creates an empty project.</summary>
    public PhysicsBrickTests() => Directory.CreateDirectory(Path.Combine(project, "Assets"));

    /// <inheritdoc />
    public void Dispose() => Directory.Delete(project, recursive: true);

    /// <summary>The backend ships only its own assembly by hand; Bepu comes from its NuGet map.</summary>
    [Fact]
    public void BepuBrickShipsItsAssemblyAndMapsBepuPackages()
    {
        var (_, bepu) = Install();

        var synced = bepu.Manifest.Nuget.Keys.ToHashSet(StringComparer.Ordinal);
        Assert.Equal(["Turian.Engine.Physics.Bepu"], BrickAssemblies.RuntimeAssemblies(bepu)
            .Select(Path.GetFileNameWithoutExtension).Where(name => !synced.Contains(name!)));
        Assert.Equal(["BepuPhysics", "BepuUtilities"], bepu.Manifest.Nuget.Keys.Order(StringComparer.Ordinal));
        Assert.All(bepu.Manifest.Nuget.Values, static version => Assert.Equal("2.5.0-beta.33", version));
    }

    /// <summary>Loading the bricks finds the Bepu factory and registers both for module discovery.</summary>
    [Fact]
    public void LoadedBricksProvideTheFactoryAndModules()
    {
        var (physics, bepu) = Install();

        BrickAssemblies.Load([physics, bepu], NullLogger.Instance);

        Assert.IsType<BepuPhysicsWorldFactory>(PhysicsWorldFactories.Find());
        Assert.Contains(typeof(PhysicsScene).Assembly, PrecastAssemblies.Loaded);
        Assert.Contains(typeof(BepuPhysicsWorld).Assembly, PrecastAssemblies.Loaded);
        Assert.Equal(typeof(RigidbodyComponent),
            TypeRegistry.GetTypeOrThrow(new Guid("ed053cc6-a437-4a75-b33c-68a92ef6e453")));
    }

    /// <summary>Both bricks pass the brick verifier.</summary>
    [Fact]
    public void BricksVerifyClean()
    {
        var (physics, bepu) = Install();

        Assert.Empty(BrickVerifier.Verify(physics.RootPath));
        Assert.Empty(BrickVerifier.Verify(bepu.RootPath));
    }

    /// <summary>A game with the backend installed references both assemblies and the Bepu packages.</summary>
    [Fact]
    public void GeneratedProjectsReferenceBepu()
    {
        Install();
        var generated = CsProjectGenerator.GenerateUserCode(
            new BuildAppSettings { Title = "Game", ProjectAbsoluteDir = project }, NullLogger.Instance).Items;

        var references = generated.Where(static i => i.ItemType == "Reference").Select(static i => i.Include).ToList();
        var packages = generated.Where(static i => i.ItemType == "PackageReference").Select(static i => i.Include).ToList();
        Assert.Contains("Turian.Engine.Physics", references);
        Assert.Contains("Turian.Engine.Physics.Bepu", references);
        Assert.Contains("BepuPhysics", packages);
        Assert.Contains("BepuUtilities", packages);
    }

    (ResolvedPackage Physics, ResolvedPackage Bepu) Install()
    {
        new ProjectManifest { Dependencies = { [PhysicsId] = $"file:{PhysicsRoot}", [BepuId] = $"file:{BepuRoot}" } }
            .Save(project);
        var packages = ProjectPackages.Resolve(project).Packages;
        return (packages.Single(static p => p.Id == PhysicsId), packages.Single(static p => p.Id == BepuId));
    }
}
