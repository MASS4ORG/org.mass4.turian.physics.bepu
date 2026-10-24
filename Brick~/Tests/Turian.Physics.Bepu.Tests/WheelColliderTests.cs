using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>Wheel colliders on a four-wheeled body: suspension, drive, brakes and steering.</summary>
public sealed class WheelColliderTests : IDisposable
{
    const float Dt = 1f / 60f;
    readonly ServiceProvider services;
    readonly PhysicsScene scene;
    readonly Node root = new() { Name = "root" };
    readonly List<WheelColliderComponent> wheels = [];
    long tick;

    /// <summary>Creates a single-threaded scene backed by Bepu with a 200 m floor whose top is at y = 0.</summary>
    public WheelColliderTests()
    {
        var settings = new AppSettings();
        settings.Loaded.Use(new PhysicsSettings { Multithreaded = false });
        scene = new PhysicsScene(settings, null, null, new BepuPhysicsWorldFactory());
        services = new ServiceCollection().AddSingleton(scene).BuildServiceProvider();
        var floor = new Node { Name = "floor", Position = new Vector3(0, -0.5f, 0), Parent = root };
        floor.AddComponent(new BoxColliderComponent { Size = new Vector3(200, 1, 200) });
        root.Children.Add(floor);
    }

    /// <inheritdoc />
    public void Dispose()
    {
        scene.Dispose();
        services.Dispose();
    }

    // A 400 kg chassis whose four wheels sit at its corners, level with its origin.
    (Node Car, RigidbodyComponent Body) Car(float height = 1f)
    {
        var body = new RigidbodyComponent { Mass = 400f };
        var car = new Node { Name = "car", Position = new Vector3(0, height, 0), Parent = root };
        car.AddComponent(body);
        car.AddComponent(new BoxColliderComponent { Size = new Vector3(1.6f, 0.3f, 3f) });
        root.Children.Add(car);
        foreach (var (x, z) in new[] { (-0.8f, 1.2f), (0.8f, 1.2f), (-0.8f, -1.2f), (0.8f, -1.2f) })
        {
            var wheel = new WheelColliderComponent { Radius = 0.4f, SuspensionDistance = 0.3f };
            var node = new Node { Name = $"wheel{wheels.Count}", Position = new Vector3(x, 0, z), Parent = car };
            node.AddComponent(wheel);
            car.Children.Add(node);
            wheels.Add(wheel);
        }

        root.Awake(null, services);
        return (car, body);
    }

    void Run(int ticks, Action? control = null)
    {
        for (var i = 0; i < ticks; i++)
        {
            control?.Invoke();
            foreach (var wheel in wheels)
            {
                wheel.OnFixedUpdate(Dt);
            }

            scene.FixedTick(++tick, Dt);
        }
    }

    /// <summary>A loaded car settles with every suspension at its target position, its wheels on the ground.</summary>
    [Fact]
    public void Suspension_RestsAtItsTarget()
    {
        var (car, _) = Car();

        Run(240);

        // Wheel centers rest a radius above the floor, half their 0.3 m travel below the chassis.
        Assert.Equal(0.4f + 0.15f, car.GlobalTransform.Position.Y, 0.02f);
        Assert.All(wheels, static wheel =>
        {
            Assert.True(wheel.IsGrounded);
            Assert.Equal(100f, wheel.SprungMass, 0.01f);
            wheel.GetWorldPose(out var position, out _);
            Assert.Equal(0.4f, position.Y, 0.02f);
            Assert.True(wheel.GetGroundHit(out var hit));
            Assert.Equal(100f * 9.81f, hit.Force, 30f);
            Assert.Equal("floor", hit.Collider.Node!.Name);
        });
    }

    /// <summary>Motor torque drives the car forward and spins the wheels; brakes then stop it.</summary>
    [Fact]
    public void Motor_DrivesAndBrakesStop()
    {
        var (car, body) = Car();
        Run(60);

        Run(120, () => wheels.ForEach(static w => w.MotorTorque = 150f));

        Assert.True(body.LinearVelocity.Z > 2f, $"the car only reached {body.LinearVelocity}");
        Assert.True(car.GlobalTransform.Position.Z > 1f);
        Assert.True(wheels[0].Rpm > 0f);
        Assert.Equal(0f, car.GlobalTransform.Position.X, 0.05f);

        Run(120, () => wheels.ForEach(static w => (w.MotorTorque, w.BrakeTorque) = (0f, 1500f)));

        Assert.True(body.LinearVelocity.Length() < 0.1f, $"still moving at {body.LinearVelocity}, " +
            $"{body.AngularVelocity}, at {car.GlobalTransform.Position}, rpm {wheels[0].Rpm}");
        Assert.Equal(0f, wheels[0].Rpm, 1f);
    }

    /// <summary>A positive steering angle on the front wheels turns a moving car to the right.</summary>
    [Fact]
    public void Steering_TurnsRight()
    {
        var (car, body) = Car();
        Run(60);

        Run(180, () =>
        {
            wheels.ForEach(static w => w.MotorTorque = 100f);
            (wheels[0].SteerAngle, wheels[1].SteerAngle) = (20f, 20f);
        });

        Assert.True(car.GlobalTransform.Position.X > 0.5f, $"the car went to {car.GlobalTransform.Position}");
        Assert.True(car.GlobalTransform.Forward.X > 0.2f, "the car should face right of where it started");
        Assert.True(MathF.Abs(car.GlobalTransform.Up.Y) > 0.95f, "the car should stay upright");
        // Rolling turns the wheel around its own X, so its axle shows the steering.
        wheels[0].GetWorldPose(out _, out var steered);
        var axle = Vector3.Transform(Vector3.UnitX, steered);
        Assert.Equal(MathF.Cos(20f * MathF.PI / 180f), Vector3.Dot(axle, car.GlobalTransform.Right), 1e-3f);
        Assert.Equal(-MathF.Sin(20f * MathF.PI / 180f), Vector3.Dot(axle, car.GlobalTransform.Forward), 1e-3f);
        Assert.True(body.LinearVelocity.Length() > 1f);
    }

    /// <summary>An airborne wheel hangs fully extended; without physics it does nothing; it survives a save.</summary>
    [Fact]
    public void Airborne_AndWithoutPhysics()
    {
        TypeRegistry.ScanAssembly(typeof(WheelColliderComponent).Assembly);
        var (car, _) = Car(height: 20f);
        Run(1);

        Assert.All(wheels, static w => Assert.False(w.GetGroundHit(out _)));
        wheels[0].GetWorldPose(out var hanging, out _);
        Assert.Equal(car.GlobalTransform.Position.Y - 0.3f, hanging.Y, 0.01f);
        Assert.False(wheels[0].HasShape);
        Assert.Equal(2, scene.BodyCount);

        var detached = new WheelColliderComponent { MotorTorque = 50f };
        detached.OnFixedUpdate(Dt);
        Assert.False(detached.IsGrounded);
        Assert.True(detached.Rpm > 0f, "the motor still spins a wheel in the air");

        var node = new Node();
        node.AddComponent(new WheelColliderComponent
        {
            Radius = 0.35f,
            Mass = 15f,
            SuspensionDistance = 0.2f,
            SuspensionSpring = 20000f,
            SuspensionDamper = 2000f,
            SuspensionTargetPosition = 0.3f,
            SidewaysFriction = WheelFrictionCurve.Sideways with { Stiffness = 0.5f },
            MotorTorque = 10f,
        });
        var loaded = Serializer.LoadData<Node>(Serializer.Serialize(node))!.GetComponent<WheelColliderComponent>()!;
        Assert.Equal((0.35f, 15f, 0.2f, 20000f, 2000f, 0.3f), (loaded.Radius, loaded.Mass, loaded.SuspensionDistance,
            loaded.SuspensionSpring, loaded.SuspensionDamper, loaded.SuspensionTargetPosition));
        Assert.Equal(0.5f, loaded.SidewaysFriction.Stiffness);
        Assert.Equal(WheelFrictionCurve.Forward, loaded.ForwardFriction);
        Assert.Equal(0f, loaded.MotorTorque);
    }

    /// <summary>The friction curve rises to its extremum, eases to its asymptote and keeps it.</summary>
    [Fact]
    public void FrictionCurve_FollowsUnitysShape()
    {
        var curve = WheelFrictionCurve.Sideways with { Stiffness = 2f };

        Assert.Equal(0f, curve.Evaluate(0f));
        Assert.Equal(1f, curve.Evaluate(-0.1f), 1e-5f);
        Assert.Equal(2f, curve.Evaluate(0.2f), 1e-5f);
        Assert.Equal(1.75f, curve.Evaluate(0.35f), 1e-5f);
        Assert.Equal(1.5f, curve.Evaluate(5f), 1e-5f);
        Assert.Equal(2f, curve.Peak);
    }
}
