using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>The physics brick's Scene view gizmos: collider wireframes and rigidbody contacts.</summary>
public sealed class PhysicsGizmoTests
{
    static Bounds LineBounds(Gizmos gizmos) => gizmos.WorldLines
        .Aggregate(Bounds.Empty, static (box, line) => box.Encapsulate(line.A).Encapsulate(line.B));

    static Gizmos Draw(Component component)
    {
        var gizmos = new Gizmos();
        IGizmoDrawer drawer = component is RigidbodyComponent ? new RigidbodyGizmoDrawer() : new ColliderGizmoDrawer();
        drawer.DrawGizmos(gizmos, component);
        return gizmos;
    }

    static T Placed<T>(T collider, Vector3 position, Quaternion rotation) where T : Component
    {
        var node = new Node { Position = position, Orientation = rotation };
        node.AddComponent(collider);
        node.Awake(null);
        return collider;
    }

    /// <summary>Each shape's wireframe fills the collider's world bounds, wherever and however it is turned.</summary>
    [Fact]
    public void Wireframes_MatchColliderBounds()
    {
        var turn = Quaternion.CreateFromYawPitchRoll(0.3f, 0.5f, 0.1f);
        var mesh = new MeshColliderComponent();
        mesh.SetGeometry([new(0, 0, 0), new(2, 0, 0), new(0, 1, 3)], [0, 1, 2]);
        ColliderComponent[] colliders =
        [
            new BoxColliderComponent { Size = new Vector3(1, 2, 3), Center = new Vector3(0, 1, 0) },
            new SphereColliderComponent { Radius = 0.75f },
            new CapsuleColliderComponent { Radius = 0.25f, Height = 2f, Direction = 0 },
            new CylinderColliderComponent { Radius = 0.5f, Height = 1f },
            mesh,
        ];

        foreach (var collider in colliders)
        {
            var bounds = Placed(collider, new Vector3(5, 1, -2), turn).Bounds;
            var drawn = LineBounds(Draw(collider));

            Assert.True(Vector3.Distance(bounds.Min, drawn.Min) < 0.05f && Vector3.Distance(bounds.Max, drawn.Max) < 0.05f,
                $"{collider.GetType().Name}: drawn {drawn} but bounds are {bounds}");
        }
    }

    /// <summary>A capsule's caps bulge past its cylinder by the radius at both ends.</summary>
    [Fact]
    public void Capsule_DrawsRoundedCaps()
    {
        var gizmos = Draw(Placed(new CapsuleColliderComponent { Radius = 0.5f, Height = 3f }, Vector3.Zero,
            Quaternion.Identity));

        var drawn = LineBounds(gizmos);
        Assert.Equal(1.5f, drawn.Max.Y, 0.01f);
        Assert.Equal(-1.5f, drawn.Min.Y, 0.01f);
        Assert.Contains(gizmos.WorldLines, static line => MathF.Abs(line.B.Y) > 1.4f && MathF.Abs(line.B.X) < 0.1f);
    }

    /// <summary>Colors tell static, dynamic, kinematic and trigger colliders apart; nothing draws without a shape.</summary>
    [Fact]
    public void Colors_TellCollidersApart()
    {
        var root = new Node();
        Node Child(params Component[] components)
        {
            var node = new Node { Parent = root };
            foreach (var component in components)
            {
                node.AddComponent(component);
            }

            root.Children.Add(node);
            return node;
        }

        var still = new BoxColliderComponent();
        var dynamic = new BoxColliderComponent();
        var kinematic = new BoxColliderComponent();
        var trigger = new BoxColliderComponent { IsTrigger = true };
        var empty = new MeshColliderComponent();
        Child(still, trigger, empty);
        Child(new RigidbodyComponent(), dynamic);
        Child(new RigidbodyComponent { IsKinematic = true }, kinematic);
        root.Awake(null);

        Assert.Equal(ColliderGizmoDrawer.StaticColor, ColliderGizmoDrawer.ColorOf(still));
        Assert.Equal(ColliderGizmoDrawer.DynamicColor, ColliderGizmoDrawer.ColorOf(dynamic));
        Assert.Equal(ColliderGizmoDrawer.KinematicColor, ColliderGizmoDrawer.ColorOf(kinematic));
        Assert.Equal(ColliderGizmoDrawer.TriggerColor, ColliderGizmoDrawer.ColorOf(trigger));
        Assert.Equal(ColliderGizmoDrawer.TriggerColor, Draw(trigger).WorldLines[0].Color);
        Assert.Equal(0, Draw(empty).LineCount);
        Assert.Equal(0, Draw(new BoxColliderComponent()).LineCount);
    }

    /// <summary>A mesh too large to wireframe draws its bounds instead.</summary>
    [Fact]
    public void LargeMeshes_DrawTheirBounds()
    {
        var vertices = new Vector3[3 * 5000];
        for (var i = 0; i < vertices.Length; i++)
        {
            vertices[i] = new Vector3(i % 100, 0, i / 100);
        }

        var mesh = Placed(new MeshColliderComponent(), Vector3.Zero, Quaternion.Identity);
        mesh.SetGeometry(vertices, [.. Enumerable.Range(0, vertices.Length)]);

        Assert.Equal(12, Draw(mesh).LineCount);
    }

    /// <summary>A resting body draws its center of mass, and its contacts with the ground once simulated.</summary>
    [Fact]
    public void Rigidbody_DrawsItsContacts()
    {
        var settings = new AppSettings();
        settings.Loaded.Use(new PhysicsSettings { Multithreaded = false });
        using var scene = new PhysicsScene(settings, null, null, new BepuPhysicsWorldFactory());
        using var services = new ServiceCollection().AddSingleton(scene).BuildServiceProvider();
        var root = new Node();
        var ground = new Node { Position = new Vector3(0, -0.5f, 0), Parent = root };
        ground.AddComponent(new BoxColliderComponent { Size = new Vector3(20, 1, 20) });
        var box = new Node { Position = new Vector3(0, 0.5f, 0), Parent = root };
        var body = new RigidbodyComponent();
        box.AddComponent(body);
        box.AddComponent(new BoxColliderComponent());
        root.Children.Add(ground);
        root.Children.Add(box);
        var resting = Draw(Placed(new RigidbodyComponent(), Vector3.Zero, Quaternion.Identity)).LineCount;
        // No handler receives the box's contacts, so only the debugging switch records them.
        scene.ReportAllContacts = true;
        root.Awake(null, services);

        for (var i = 0; i < 10; i++)
        {
            scene.FixedTick(i + 1, 1d / 60d);
        }

        var gizmos = Draw(body);
        Assert.Equal(3, resting);
        Assert.NotEmpty(scene.LastContacts);
        Assert.Contains(gizmos.WorldLines, static line => line.Color == RigidbodyGizmoDrawer.ContactColor);
        Assert.Contains(gizmos.WorldLines, static line => MathF.Abs(line.A.Y) < 0.05f && line.B.Y > 0.2f);
    }

    /// <summary>
    /// A joint draws its anchor and axis, and the line to its connected anchor once that is known: set by hand, or
    /// placed when the joint was created.
    /// </summary>
    [Fact]
    public void Joint_DrawsAnchorAxisAndConnection()
    {
        var hinge = Placed(new HingeJointComponent { Axis = Vector3.UnitY, Anchor = new Vector3(1, 0, 0) },
            new Vector3(0, 2, 0), Quaternion.Identity);
        var fixedJoint = Placed(new FixedJointComponent
        {
            AutoConfigureConnectedAnchor = false,
            ConnectedAnchor = new Vector3(0, 9, 0)
        }, Vector3.Zero, Quaternion.Identity);
        var drawer = new JointGizmoDrawer();
        var gizmos = new Gizmos();

        drawer.DrawGizmos(gizmos, hinge);
        var hingeLines = gizmos.WorldLines.ToList();
        gizmos.Clear();
        drawer.DrawGizmos(gizmos, fixedJoint);

        Assert.Equal(13, hingeLines.Count);
        Assert.Contains(hingeLines, static line => line.A == new Vector3(1, 1.5f, 0) && line.B == new Vector3(1, 2.5f, 0));
        Assert.Equal(13, gizmos.LineCount);
        Assert.Contains(gizmos.WorldLines, static line => line.B == new Vector3(0, 9, 0));
    }

    /// <summary>The editor's catalog finds the drawers for physics components, including derived types.</summary>
    [Fact]
    public void Catalog_FindsTheDrawers()
    {
        using var build = new BuildManager(new AppSettings(), NullLogger.Instance);
        var catalog = new GizmoDrawerCatalog(build);

        Assert.IsType<ColliderGizmoDrawer>(Assert.Single(catalog.GetDrawers(typeof(MeshColliderComponent))));
        Assert.IsType<RigidbodyGizmoDrawer>(Assert.Single(catalog.GetDrawers(typeof(RigidbodyComponent))));
        Assert.IsType<JointGizmoDrawer>(Assert.Single(catalog.GetDrawers(typeof(SpringJointComponent))));
    }
}
