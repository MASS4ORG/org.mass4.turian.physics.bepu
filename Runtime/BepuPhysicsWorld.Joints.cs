namespace Turian.Engine.Physics;

public sealed partial class BepuPhysicsWorld
{
    static readonly SpringSettings Rigid = new(30f, 1f);

    readonly Dictionary<int, BepuJoint> joints = [];
    readonly Dictionary<(int, int), int> noCollisionPairs = [];
    BepuBody? worldAnchor;
    int nextJoint = 1;

    /// <inheritdoc />
    public JointHandle CreateJoint(in JointDesc description)
    {
        ThrowIfDisposed();
        var a = Get(description.BodyA);
        if (a.Motion == MotionType.Static)
        {
            throw new ArgumentException("A joint's first body cannot be static.", nameof(description));
        }

        var handle = new JointHandle(nextJoint++);
        var joint = new BepuJoint(description);
        joints.Add(handle.Id, joint);
        Build(handle, joint, a);
        return handle;
    }

    /// <inheritdoc />
    public void UpdateJoint(JointHandle joint, in JointDesc description)
    {
        ThrowIfDisposed();
        if (!joints.TryGetValue(joint.Id, out var existing))
        {
            throw new ArgumentException($"Joint {joint.Id} does not exist.", nameof(joint));
        }

        Tear(joint, existing);
        existing.Description = description;
        Build(joint, existing, Get(description.BodyA));
    }

    /// <inheritdoc />
    public void RemoveJoint(JointHandle joint)
    {
        ThrowIfDisposed();
        if (joints.Remove(joint.Id, out var existing))
        {
            Tear(joint, existing);
        }
    }

    /// <summary>Whether a joint between two bodies turned their collision off; read by the narrow phase.</summary>
    internal bool AreJointedWithoutCollision(BepuBody a, BepuBody b) =>
        noCollisionPairs.Count > 0 && noCollisionPairs.ContainsKey(PairOf(a.Handle.Id, b.Handle.Id));

    void Build(JointHandle handle, BepuJoint joint, BepuBody a)
    {
        var description = joint.Description;
        var b = description.BodyB.IsValid && Get(description.BodyB) is { Motion: not MotionType.Static } connected
            ? connected
            : WorldAnchor();
        // A static second body is replaced by the world anchor, so its frame is converted to world space.
        if (description.BodyB.IsValid && b == worldAnchor)
        {
            var pose = GetPose(description.BodyB);
            var relative = description.RelativeOrientation == default
                ? Quaternion.Identity
                : description.RelativeOrientation;
            description = description with
            {
                AnchorB = pose.TransformPoint(description.AnchorB),
                AxisB = Vector3.Transform(description.AxisB, pose.Orientation),
                RelativeOrientation = Quaternion.Normalize(relative * Quaternion.Conjugate(pose.Orientation)),
            };
        }

        var frame = new JointFrame(a, b, description);
        joint.BodyA = a;
        joint.BodyB = b;
        foreach (var constraint in Constraints(frame))
        {
            joint.Constraints.Add(constraint);
        }

        a.Joints.Add(handle);
        b.Joints.Add(handle);
        if (!description.EnableCollision && b != worldAnchor)
        {
            var key = PairOf(a.Handle.Id, b.Handle.Id);
            noCollisionPairs[key] = noCollisionPairs.GetValueOrDefault(key) + 1;
        }

        Simulation.Awakener.AwakenBody(a.Body);
        if (b != worldAnchor)
        {
            Simulation.Awakener.AwakenBody(b.Body);
        }
    }

    IEnumerable<ConstraintHandle> Constraints(JointFrame f)
    {
        var d = f.Description;
        var bodyA = f.A.Body;
        var bodyB = f.B.Body;
        var solver = Simulation.Solver;
        switch (d.Type)
        {
            case JointType.Fixed:
                yield return solver.Add(bodyA, bodyB, new Weld
                {
                    LocalOffset = f.CenterOffset,
                    LocalOrientation = f.RelativeRotation,
                    SpringSettings = Rigid,
                });
                break;
            case JointType.Hinge:
                yield return solver.Add(bodyA, bodyB, new Hinge
                {
                    LocalOffsetA = f.OffsetA,
                    LocalHingeAxisA = f.AxisA,
                    LocalOffsetB = f.OffsetB,
                    LocalHingeAxisB = f.AxisB,
                    SpringSettings = Rigid,
                });
                foreach (var extra in HingeExtras(f))
                {
                    yield return extra;
                }

                break;
            case JointType.BallSocket:
                yield return solver.Add(bodyA, bodyB, new BallSocket
                {
                    LocalOffsetA = f.OffsetA,
                    LocalOffsetB = f.OffsetB,
                    SpringSettings = Rigid,
                });
                if (d.Limit.Enabled)
                {
                    yield return solver.Add(bodyA, bodyB, new SwingLimit
                    {
                        AxisLocalA = f.AxisA,
                        AxisLocalB = f.AxisB,
                        MaximumSwingAngle = MathF.Max(d.Limit.Max, 0f),
                        SpringSettings = Rigid,
                    });
                }

                break;
            case JointType.Slider:
                foreach (var constraint in SliderConstraints(f))
                {
                    yield return constraint;
                }

                break;
            case JointType.Distance:
                yield return solver.Add(bodyA, bodyB, new DistanceLimit(f.OffsetA, f.OffsetB,
                    MathF.Max(d.Limit.Min, 0f), MathF.Max(d.Limit.Max, d.Limit.Min), Rigid));
                break;
            case JointType.Spring:
                yield return solver.Add(bodyA, bodyB, new DistanceServo(f.OffsetA, f.OffsetB,
                    MathF.Max(d.Spring.Target, 0f), Soft(d.Spring), ServoSettings.Default));
                if (d.Limit.Enabled)
                {
                    yield return solver.Add(bodyA, bodyB, new DistanceLimit(f.OffsetA, f.OffsetB,
                        MathF.Max(d.Limit.Min, 0f), MathF.Max(d.Limit.Max, d.Limit.Min), Rigid));
                }

                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(f), d.Type, "Unknown joint type.");
        }
    }

    // Joint values are of the first body relative to the second. Bepu's twist constraints measure the second body
    // against the first, so their angles are negated; its axis motor already drives the first body's velocity.
    IEnumerable<ConstraintHandle> HingeExtras(JointFrame f)
    {
        var d = f.Description;
        if (d.Limit.Enabled)
        {
            yield return Simulation.Solver.Add(f.A.Body, f.B.Body, new TwistLimit
            {
                LocalBasisA = f.BasisA,
                LocalBasisB = f.BasisB,
                MinimumAngle = -d.Limit.Max,
                MaximumAngle = -d.Limit.Min,
                SpringSettings = Rigid,
            });
        }

        if (d.Motor.Enabled)
        {
            yield return Simulation.Solver.Add(f.A.Body, f.B.Body, new AngularAxisMotor
            {
                LocalAxisA = f.AxisA,
                TargetVelocity = d.Motor.TargetVelocity,
                Settings = new MotorSettings(MathF.Max(d.Motor.MaxForce, 0f), 1e-3f),
            });
        }

        if (d.Spring.Enabled)
        {
            yield return Simulation.Solver.Add(f.A.Body, f.B.Body, new TwistServo
            {
                LocalBasisA = f.BasisA,
                LocalBasisB = f.BasisB,
                TargetAngle = -d.Spring.Target,
                SpringSettings = Soft(d.Spring),
                ServoSettings = ServoSettings.Default,
            });
        }
    }

    IEnumerable<ConstraintHandle> SliderConstraints(JointFrame f)
    {
        var d = f.Description;
        var solver = Simulation.Solver;
        yield return solver.Add(f.A.Body, f.B.Body, new PointOnLineServo
        {
            LocalOffsetA = f.OffsetA,
            LocalOffsetB = f.OffsetB,
            LocalDirection = f.AxisA,
            ServoSettings = ServoSettings.Default,
            SpringSettings = Rigid,
        });
        yield return solver.Add(f.A.Body, f.B.Body, new AngularServo
        {
            TargetRelativeRotationLocalA = f.RelativeRotation,
            ServoSettings = ServoSettings.Default,
            SpringSettings = Rigid,
        });
        if (d.Limit.Enabled)
        {
            yield return solver.Add(f.A.Body, f.B.Body, new LinearAxisLimit
            {
                LocalOffsetA = f.OffsetA,
                LocalOffsetB = f.OffsetB,
                LocalAxis = f.AxisA,
                MinimumOffset = -d.Limit.Max,
                MaximumOffset = -d.Limit.Min,
                SpringSettings = Rigid,
            });
        }

        if (d.Motor.Enabled)
        {
            yield return solver.Add(f.A.Body, f.B.Body, new LinearAxisMotor
            {
                LocalOffsetA = f.OffsetA,
                LocalOffsetB = f.OffsetB,
                LocalAxis = f.AxisA,
                TargetVelocity = -d.Motor.TargetVelocity,
                Settings = new MotorSettings(MathF.Max(d.Motor.MaxForce, 0f), 1e-3f),
            });
        }
    }

    void Tear(JointHandle handle, BepuJoint joint)
    {
        foreach (var constraint in joint.Constraints)
        {
            Simulation.Solver.Remove(constraint);
        }

        joint.Constraints.Clear();
        joint.BodyA?.Joints.Remove(handle);
        joint.BodyB?.Joints.Remove(handle);
        if (!joint.Description.EnableCollision && joint.BodyA is { } a && joint.BodyB is { } b && b != worldAnchor)
        {
            var key = PairOf(a.Handle.Id, b.Handle.Id);
            if (noCollisionPairs.TryGetValue(key, out var count) && count > 1)
            {
                noCollisionPairs[key] = count - 1;
            }
            else
            {
                noCollisionPairs.Remove(key);
            }
        }
    }

    BepuBody WorldAnchor()
    {
        if (worldAnchor is null)
        {
            var handle = CreateBody(new BodyDesc
            {
                Motion = MotionType.Kinematic,
                Pose = PhysicsPose.Identity,
                Colliders = [],
                SleepThreshold = 0f,
            });
            worldAnchor = Get(handle);
            records.Remove(handle.Id);
        }

        return worldAnchor;
    }

    static SpringSettings Soft(in JointSpring spring) =>
        new(MathF.Max(spring.Frequency, 1e-3f), MathF.Max(spring.DampingRatio, 0f));

    static (int, int) PairOf(int a, int b) => a < b ? (a, b) : (b, a);
}

/// <summary>The Bepu constraints behind one joint.</summary>
sealed class BepuJoint(JointDesc description)
{
    public JointDesc Description { get; set; } = description;

    public BepuBody? BodyA { get; set; }

    public BepuBody? BodyB { get; set; }

    public List<ConstraintHandle> Constraints { get; } = [];
}

/// <summary>A joint description expressed in the local frames of the two Bepu bodies, whose origins are their centers.</summary>
readonly struct JointFrame
{
    public JointFrame(BepuBody a, BepuBody b, in JointDesc description)
    {
        A = a;
        B = b;
        Description = description;
        var relative = description.RelativeOrientation == default ? Quaternion.Identity : description.RelativeOrientation;
        var inverseA = Quaternion.Conjugate(a.Offset.Orientation);
        var inverseB = Quaternion.Conjugate(b.Offset.Orientation);
        OffsetA = Vector3.Transform(description.AnchorA - a.Offset.Position, inverseA);
        OffsetB = Vector3.Transform(description.AnchorB - b.Offset.Position, inverseB);
        AxisA = Vector3.Transform(Unit(description.AxisA), inverseA);
        AxisB = Vector3.Transform(Unit(description.AxisB), inverseB);
        RelativeRotation = Quaternion.Normalize(inverseA * relative * b.Offset.Orientation);
        var originB = description.AnchorA - Vector3.Transform(description.AnchorB, relative);
        CenterOffset = Vector3.Transform(originB + Vector3.Transform(b.Offset.Position, relative) - a.Offset.Position,
            inverseA);
        var basis = FromTo(Vector3.UnitZ, Unit(description.AxisA));
        BasisA = Quaternion.Normalize(inverseA * basis);
        BasisB = Quaternion.Normalize(inverseB * Quaternion.Conjugate(relative) * basis);
    }

    public BepuBody A { get; }

    public BepuBody B { get; }

    public JointDesc Description { get; }

    public Vector3 OffsetA { get; }

    public Vector3 OffsetB { get; }

    public Vector3 AxisA { get; }

    public Vector3 AxisB { get; }

    /// <summary>The second body's orientation relative to the first, between Bepu frames.</summary>
    public Quaternion RelativeRotation { get; }

    /// <summary>The second body's center relative to the first's, in the first's Bepu frame.</summary>
    public Vector3 CenterOffset { get; }

    /// <summary>A basis whose Z is the joint axis, in the first body's Bepu frame.</summary>
    public Quaternion BasisA { get; }

    /// <summary>The same basis at the joint's rest pose, in the second body's Bepu frame.</summary>
    public Quaternion BasisB { get; }

    static Vector3 Unit(Vector3 axis) => axis.LengthSquared() > 1e-12f ? Vector3.Normalize(axis) : Vector3.UnitY;

    static Quaternion FromTo(Vector3 from, Vector3 to)
    {
        var dot = Vector3.Dot(from, to);
        if (dot > 0.999999f)
        {
            return Quaternion.Identity;
        }

        if (dot < -0.999999f)
        {
            return Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI);
        }

        return Quaternion.Normalize(new Quaternion(Vector3.Cross(from, to), 1f + dot));
    }
}
