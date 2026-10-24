using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>The kinematic character controller: walking, falling, steps, slopes, ceilings and platforms.</summary>
public sealed class CharacterControllerTests : IDisposable
{
    const float Dt = 1f / 60f;
    readonly ServiceProvider services;
    readonly PhysicsScene scene;
    readonly Node root = new() { Name = "root" };
    long tick;

    /// <summary>Creates a single-threaded scene backed by Bepu with a 40 m floor whose top is at y = 0.</summary>
    public CharacterControllerTests()
    {
        var settings = new AppSettings();
        settings.Loaded.Use(new PhysicsSettings { Multithreaded = false });
        scene = new PhysicsScene(settings, null, null, new BepuPhysicsWorldFactory());
        services = new ServiceCollection().AddSingleton(scene).BuildServiceProvider();
        Block("floor", new Vector3(0, -0.5f, 0), new Vector3(40, 1, 40));
    }

    /// <inheritdoc />
    public void Dispose()
    {
        scene.Dispose();
        services.Dispose();
    }

    sealed class HitRecorder : Component, IControllerColliderHitHandler
    {
        public List<ControllerColliderHit> Hits { get; } = [];

        public void OnControllerColliderHit(ControllerColliderHit hit) => Hits.Add(hit);
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

    Node Block(string name, Vector3 center, Vector3 size, Quaternion? rotation = null)
    {
        var node = Add(name, center, new BoxColliderComponent { Size = size });
        node.Orientation = rotation ?? Quaternion.Identity;
        return node;
    }

    (Node Node, CharacterControllerComponent Controller) Character(Vector3 feet, params Component[] extra)
    {
        var controller = new CharacterControllerComponent { Center = new Vector3(0, 1, 0) };
        return (Add("player", feet, [controller, .. extra]), controller);
    }

    void Start()
    {
        root.Awake(null, services);
        Step();
    }

    void Step() => scene.FixedTick(++tick, Dt);

    void Walk(CharacterControllerComponent controller, Vector3 speed, int frames)
    {
        for (var i = 0; i < frames; i++)
        {
            controller.SimpleMove(speed, Dt);
            Step();
        }
    }

    /// <summary>A character dropped above the floor falls under gravity and lands on it.</summary>
    [Fact]
    public void SimpleMove_FallsAndLands()
    {
        var (node, controller) = Character(new Vector3(0, 3, 0));
        Start();

        controller.SimpleMove(Vector3.Zero, Dt);
        Assert.False(controller.IsGrounded);
        Assert.True(controller.Velocity.Y < 0f);
        Walk(controller, Vector3.Zero, 90);

        Assert.True(controller.IsGrounded);
        Assert.Equal(CollisionFlags.Below, controller.CollisionFlags);
        Assert.Equal(0f, node.GlobalTransform.Position.Y, 0.05f);
    }

    /// <summary>Walking diagonally into a wall stops at it and slides along it.</summary>
    [Fact]
    public void Walls_BlockAndSlide()
    {
        Block("wall", new Vector3(0, 1, 3), new Vector3(20, 2, 1));
        var recorder = new HitRecorder();
        var (node, controller) = Character(Vector3.Zero, recorder);
        Start();

        Walk(controller, new Vector3(2, 0, 4), 120);

        var position = node.GlobalTransform.Position;
        Assert.Equal(2f, position.Z, 0.05f);
        Assert.True(position.X > 3f, $"the character should slide along the wall, but is at {position}");
        Assert.True((controller.CollisionFlags & CollisionFlags.Sides) != 0);
        Assert.Contains(recorder.Hits, static h => h.Node!.Name == "wall" && h.Normal.Z < -0.9f);
    }

    /// <summary>A step lower than the step offset is climbed; a taller one stops the character.</summary>
    [Theory]
    [InlineData(0.25f, true)]
    [InlineData(0.5f, false)]
    public void Steps_ClimbOnlyUpToTheOffset(float height, bool climbs)
    {
        // Deeper than the walk, so a character that climbs it is still on top at the end.
        Block("step", new Vector3(0, height / 2f, 5), new Vector3(4, height, 6));
        var (node, controller) = Character(Vector3.Zero);
        Start();
        Walk(controller, Vector3.Zero, 5);

        Walk(controller, new Vector3(0, 0, 3), 90);

        var position = node.GlobalTransform.Position;
        Assert.Equal(climbs, position.Z > 2.5f);
        Assert.Equal(climbs ? height : 0f, position.Y, 0.05f);
        Assert.True(controller.IsGrounded);
    }

    /// <summary>A gentle ramp is walked up; a steep one is not, whatever the push.</summary>
    [Theory]
    [InlineData(25f, true)]
    [InlineData(60f, false)]
    public void Slopes_ClimbOnlyUpToTheLimit(float degrees, bool climbs)
    {
        var angle = degrees * MathF.PI / 180f;
        // Half the ramp sinks under the floor; its walkable top meets the floor near z = 1.
        Block("ramp", new Vector3(0, 0, 1 + 4 * MathF.Cos(angle)), new Vector3(4, 0.2f, 8),
            Quaternion.CreateFromAxisAngle(Vector3.UnitX, -angle));
        var (node, controller) = Character(Vector3.Zero);
        Start();
        Walk(controller, Vector3.Zero, 5);

        Walk(controller, new Vector3(0, 0, 3), 150);

        var height = node.GlobalTransform.Position.Y;
        Assert.True(climbs ? height > 1f : height < 0.5f, $"the character reached {height} m on a {degrees}° ramp");
    }

    /// <summary>A jump into a low ceiling reports it and stops rising.</summary>
    [Fact]
    public void Ceilings_StopAJump()
    {
        Block("ceiling", new Vector3(0, 2.6f, 0), new Vector3(4, 0.2f, 4));
        var (node, controller) = Character(Vector3.Zero);
        Start();

        var flags = controller.Move(new Vector3(0, 1, 0));

        Assert.Equal(CollisionFlags.Above, flags);
        Assert.Equal(0.5f, node.GlobalTransform.Position.Y, 0.05f);
    }

    /// <summary>A character standing on a moving kinematic platform is carried along.</summary>
    [Fact]
    public void Platforms_CarryTheCharacter()
    {
        var platformBody = new RigidbodyComponent { IsKinematic = true };
        var platform = Add("platform", new Vector3(0, 1, 0), platformBody, new BoxColliderComponent { Size = new Vector3(4, 0.2f, 4) });
        var (node, controller) = Character(new Vector3(0, 1.2f, 0));
        Start();
        Walk(controller, Vector3.Zero, 10);
        Assert.True(controller.IsGrounded);

        for (var i = 1; i <= 60; i++)
        {
            platformBody.MovePosition(new Vector3(i * 0.05f, 1, 0));
            controller.SimpleMove(Vector3.Zero, Dt);
            Step();
        }

        Assert.Equal(platform.GlobalTransform.Position.X, node.GlobalTransform.Position.X, 0.1f);
        Assert.True(controller.IsGrounded);
    }

    /// <summary>A character placed inside a box is pushed out of it by its next move.</summary>
    [Fact]
    public void Overlaps_ArePushedOut()
    {
        Block("crate", new Vector3(0.8f, 1, 0), new Vector3(1, 2, 1));
        var (node, controller) = Character(Vector3.Zero);
        Start();

        controller.Move(Vector3.Zero);

        Assert.True(node.GlobalTransform.Position.X < -0.15f, $"the character is still at {node.GlobalTransform.Position}");
    }

    /// <summary>Without a simulating physics scene a move does nothing; settings survive a save and load.</summary>
    [Fact]
    public void WithoutPhysics_MovesNothingAndSettingsRoundTrip()
    {
        TypeRegistry.ScanAssembly(typeof(CharacterControllerComponent).Assembly);
        var node = new Node();
        var controller = new CharacterControllerComponent
        {
            Radius = 0.3f,
            Height = 1.8f,
            SlopeLimit = 50f,
            StepOffset = 0.4f,
            SkinWidth = 0.03f,
            MinMoveDistance = 0.01f,
            Center = new Vector3(0, 0.9f, 0),
        };
        node.AddComponent(controller);

        Assert.Equal(CollisionFlags.None, controller.Move(Vector3.UnitX));
        Assert.Equal(Vector3.Zero, node.Position);
        var loaded = Serializer.LoadData<Node>(Serializer.Serialize(node))!.GetComponent<CharacterControllerComponent>()!;
        Assert.Equal((0.3f, 1.8f, 50f, 0.4f, 0.03f, 0.01f), (loaded.Radius, loaded.Height, loaded.SlopeLimit,
            loaded.StepOffset, loaded.SkinWidth, loaded.MinMoveDistance));
        Assert.Equal(new Vector3(0, 0.9f, 0), loaded.Center);
        Assert.Equal(ShapeType.Capsule, loaded.Describe(Vector3.One, out _).Type);
    }

    /// <summary>Other bodies collide with the controller as with a static capsule.</summary>
    [Fact]
    public void Bodies_LandOnTheCharacter()
    {
        var (_, _) = Character(Vector3.Zero);
        var ball = Add("ball", new Vector3(0, 4, 0), new RigidbodyComponent(), new SphereColliderComponent { Radius = 0.25f });
        Start();

        for (var i = 0; i < 90; i++)
        {
            Step();
        }

        Assert.Equal(2.25f, ball.GlobalTransform.Position.Y, 0.05f);
    }
}
