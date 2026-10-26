using System.Reflection;
using Microsoft.Extensions.Logging;
using Log = Turian.Engine.Core.Log;

namespace Turian.Engine.Physics;

/// <summary>An <see cref="IPhysicsWorld"/> simulated by BepuPhysics v2.</summary>
public sealed partial class BepuPhysicsWorld : IPhysicsWorld
{
    /// <summary>
    /// The upper sleep threshold, as a multiple of the squared one: a body below three times the sleep speed keeps its
    /// calm steps but does not add to them.
    /// </summary>
    internal const float SleepHysteresis = 9f;

    static int liveCount;
    readonly BufferPool pool = new();
    readonly ThreadDispatcher? dispatcher;
    readonly Dictionary<int, BepuShape> shapes = [];
    readonly Dictionary<int, BepuBody> records = [];
    readonly HashSet<(ulong, ulong)> ignoredPairs = [];
    readonly int substeps;
    BepuBody?[] byBody = new BepuBody?[64];
    BepuBody?[] byStatic = new BepuBody?[64];
    BodyIntegration[] integration = new BodyIntegration[64];
    int[] testedAt = new int[64];
    int steps;
    int nextShape = 1;
    int nextBody = 1;
    bool disposed;

    /// <summary>Creates an empty world.</summary>
    public BepuPhysicsWorld(PhysicsWorldSettings settings)
    {
        ArgumentNullException.ThrowIfNull(settings);
        Gravity = settings.Gravity;
        var workers = WorkerCount(settings);
        if (workers > 1)
        {
            dispatcher = new ThreadDispatcher(workers);
        }

        Log.Logger.LogInformation(
            "Physics {Build}: WorkerThreads={WorkerThreads}, resolved {Workers} worker thread(s) of {Processors} logical processor(s), {Substeps} substep(s), {Iterations} solver iteration(s)",
            typeof(BepuPhysicsWorld).Assembly
                .GetCustomAttribute<AssemblyInformationalVersionAttribute>()?.InformationalVersion,
            settings.WorkerThreads, workers, Environment.ProcessorCount, Math.Max(1, settings.Substeps),
            Math.Max(1, settings.SolverIterations));

        Events = new BepuEventSink(this, dispatcher?.ThreadCount ?? 1);
        substeps = Math.Max(1, settings.Substeps);
        Simulation = Simulation.Create(pool, new BepuNarrowPhaseCallbacks(this), new BepuPoseIntegratorCallbacks(this),
            new SolveDescription(Math.Max(1, settings.SolverIterations), substeps));
        Simulation.Deterministic = settings.Deterministic;
        if (Simulation.Timestepper is DefaultTimestepper timestepper)
        {
            timestepper.BeforeCollisionDetection += (_, _) => StampTested();
        }

        Interlocked.Increment(ref liveCount);
    }

    /// <summary>The worker threads a world uses: one when single-threaded, else the setting capped by the processors.</summary>
    internal static int WorkerCount(PhysicsWorldSettings settings) => !settings.Multithreaded ? 1
        : settings.WorkerThreads > 0 ? Math.Min(settings.WorkerThreads, Environment.ProcessorCount)
        : Environment.ProcessorCount;

    /// <summary>The number of worlds created and not yet disposed in this process, for leak checks.</summary>
    internal static int LiveCount => Volatile.Read(ref liveCount);

    /// <summary>The underlying simulation, for advanced use within the Bepu brick.</summary>
    internal Simulation Simulation { get; }

    internal BepuEventSink Events { get; }

    /// <summary>
    /// The integration multipliers of each body by Bepu body handle, read for every bundle of every substep, so they
    /// are plain values rather than lookups of the body records.
    /// </summary>
    internal BodyIntegration[] Integration => integration;

    /// <summary>
    /// The contact spring frequency of the current step: half the substep rate, the stiffest Bepu keeps stable. A
    /// softer spring lets speculative contacts leak, so fast bodies sink into or pass through what they hit.
    /// </summary>
    internal float ContactFrequency { get; private set; } = 120f;

    /// <inheritdoc />
    public Vector3 Gravity { get; set; }

    /// <inheritdoc />
    public int BodyCount => records.Count;

    /// <inheritdoc />
    public PhysicsBodyCounts? BodyCounts => disposed ? new PhysicsBodyCounts() : new PhysicsBodyCounts(
        records.Count, Simulation.Statics.Count,
        records.Count - Simulation.Statics.Count - Simulation.Bodies.ActiveSet.Count +
        (worldAnchor is { } anchor && Simulation.Bodies[anchor.Body].Awake ? 1 : 0));

    /// <inheritdoc />
    public ReadOnlySpan<ContactEvent> Contacts => Events.Contacts;

    /// <inheritdoc />
    public ReadOnlySpan<TriggerEvent> Triggers => Events.Triggers;

    /// <inheritdoc />
    public int ContactPairCount => Events.ContactPairCount;

    /// <inheritdoc />
    public ShapeHandle CreateShape(in ShapeDesc description)
    {
        ThrowIfDisposed();
        var handle = new ShapeHandle(nextShape++);
        shapes.Add(handle.Id, BepuShapes.Add(Simulation.Shapes, pool, description));
        return handle;
    }

    /// <inheritdoc />
    public void RemoveShape(ShapeHandle shape)
    {
        ThrowIfDisposed();
        if (shapes.Remove(shape.Id, out var entry))
        {
            BepuShapes.Remove(Simulation.Shapes, pool, entry);
        }
    }

    /// <inheritdoc />
    public BodyHandle CreateBody(in BodyDesc description)
    {
        ThrowIfDisposed();
        if (description.Motion == MotionType.Static)
        {
            return CreateStatic(new StaticDesc(description.Pose, description.Colliders, description.UserData));
        }

        var body = NewRecord(description.Motion, description.Colliders);
        var dynamic = description.Motion == MotionType.Dynamic;
        var mass = dynamic ? MathF.Max(description.Mass.Mass, 1e-4f) : 0f;
        var built = BuildCollidable(body, dynamic ? mass : 0f, description.Mass);
        body.Mass = mass;
        body.GravityScale = description.GravityScale;
        body.LinearDamping = description.LinearDamping;
        body.AngularDamping = description.AngularDamping;
        body.Locks = description.Locks;
        var pose = body.ToCollidable(description.Pose);
        body.LockedPosition = pose.Position;
        var collidable = new CollidableDescription(body.Shape,
            description.Continuous ? ContinuousDetection.Continuous() : ContinuousDetection.Discrete);
        var activity = new BodyActivityDescription(description.SleepThreshold < 0
            ? -1f
            : description.SleepThreshold * description.SleepThreshold);
        var velocity = new BodyVelocity(description.Velocity.Linear, description.Velocity.Angular);
        var bepu = dynamic
            ? BodyDescription.CreateDynamic(pose, velocity, LockInertia(built, description.Locks), collidable, activity)
            : BodyDescription.CreateKinematic(pose, velocity, collidable, activity);
        body.Body = Simulation.Bodies.Add(bepu);
        Register(ref byBody, body.Body.Value, body);
        Register(ref integration, body.Body.Value, BodyIntegration.Of(body));
        return body.Handle;
    }

    /// <inheritdoc />
    public BodyHandle CreateStatic(in StaticDesc description)
    {
        ThrowIfDisposed();
        if (description.Colliders is not { Length: > 0 })
        {
            throw new ArgumentException("A static body needs at least one collider.", nameof(description));
        }

        var body = NewRecord(MotionType.Static, description.Colliders);
        BuildCollidable(body, 0f, PhysicsMass.Default);
        body.Static = Simulation.Statics.Add(new StaticDescription(body.ToCollidable(description.Pose), body.Shape));
        Register(ref byStatic, body.Static.Value, body);
        return body.Handle;
    }

    /// <inheritdoc />
    public void Remove(BodyHandle body)
    {
        ThrowIfDisposed();
        if (!records.Remove(body.Id, out var record))
        {
            return;
        }

        foreach (var joint in record.Joints.ToArray())
        {
            RemoveJoint(joint);
        }

        if (record.Motion == MotionType.Static)
        {
            Simulation.Statics.Remove(record.Static);
            byStatic[record.Static.Value] = null;
        }
        else
        {
            Simulation.Bodies.Remove(record.Body);
            byBody[record.Body.Value] = null;
        }

        if (record.OwnsShape)
        {
            Simulation.Shapes.RemoveAndDispose(record.Shape, pool);
        }
    }

    /// <inheritdoc />
    public bool Contains(BodyHandle body) => records.ContainsKey(body.Id);

    /// <inheritdoc />
    public PhysicsPose GetPose(BodyHandle body)
    {
        var record = Get(body);
        return record.ToOrigin(record.Motion == MotionType.Static
            ? Simulation.Statics[record.Static].Pose
            : Simulation.Bodies[record.Body].Pose);
    }

    /// <inheritdoc />
    public void SetPose(BodyHandle body, in PhysicsPose pose)
    {
        var record = Get(body);
        var target = record.ToCollidable(pose);
        if (record.Motion == MotionType.Static)
        {
            Simulation.Statics.ApplyDescription(record.Static, new StaticDescription(target, record.Shape));
            return;
        }

        var reference = Simulation.Bodies[record.Body];
        reference.Pose = target;
        WakeBody(record);
        reference.UpdateBounds();
        record.LockedPosition = target.Position;
    }

    /// <inheritdoc />
    public PhysicsVelocity GetVelocity(BodyHandle body)
    {
        var record = Get(body);
        if (record.Motion == MotionType.Static)
        {
            return default;
        }

        var velocity = Simulation.Bodies[record.Body].Velocity;
        return new PhysicsVelocity(velocity.Linear, velocity.Angular);
    }

    /// <inheritdoc />
    public void SetVelocity(BodyHandle body, in PhysicsVelocity velocity)
    {
        var record = Get(body);
        if (record.Motion == MotionType.Static)
        {
            return;
        }

        var reference = Simulation.Bodies[record.Body];
        reference.Velocity = new BodyVelocity(velocity.Linear, velocity.Angular);
        WakeBody(record);
    }

    /// <inheritdoc />
    public Vector3 GetCenterOfMass(BodyHandle body)
    {
        var record = Get(body);
        return record.Motion == MotionType.Static
            ? Simulation.Statics[record.Static].Pose.Position
            : Simulation.Bodies[record.Body].Pose.Position;
    }

    /// <inheritdoc />
    public float GetMass(BodyHandle body) => Get(body).Mass;

    /// <inheritdoc />
    public void ApplyImpulse(BodyHandle body, Vector3 impulse, Vector3 worldPoint)
    {
        var record = Get(body);
        if (record.Motion != MotionType.Dynamic)
        {
            return;
        }

        var reference = Simulation.Bodies[record.Body];
        reference.ApplyImpulse(impulse, worldPoint - reference.Pose.Position);
        WakeBody(record);
    }

    /// <inheritdoc />
    public void ApplyAngularImpulse(BodyHandle body, Vector3 angularImpulse)
    {
        var record = Get(body);
        if (record.Motion != MotionType.Dynamic)
        {
            return;
        }

        var reference = Simulation.Bodies[record.Body];
        reference.ApplyAngularImpulse(angularImpulse);
        WakeBody(record);
    }

    /// <inheritdoc />
    public void SetKinematicTarget(BodyHandle body, in PhysicsPose target, float deltaTime)
    {
        var record = Get(body);
        if (record.Motion != MotionType.Kinematic || deltaTime <= 0f)
        {
            return;
        }

        var reference = Simulation.Bodies[record.Body];
        var current = reference.Pose;
        var goal = record.ToCollidable(target);
        var delta = Quaternion.Normalize(goal.Orientation * Quaternion.Conjugate(current.Orientation));
        if (delta.W < 0f)
        {
            delta = -delta;
        }

        var angle = 2f * MathF.Acos(Math.Clamp(delta.W, -1f, 1f));
        var sin = MathF.Sqrt(MathF.Max(0f, 1f - delta.W * delta.W));
        var axis = sin > 1e-6f ? new Vector3(delta.X, delta.Y, delta.Z) / sin : Vector3.Zero;
        reference.Velocity = new BodyVelocity((goal.Position - current.Position) / deltaTime, axis * angle / deltaTime);
        WakeBody(record);
        record.HasKinematicTarget = true;
    }

    /// <inheritdoc />
    public void Wake(BodyHandle body)
    {
        var record = Get(body);
        if (record.Motion != MotionType.Static)
        {
            WakeBody(record);
        }
    }

    /// <inheritdoc />
    public void Sleep(BodyHandle body)
    {
        var record = Get(body);
        if (record.Motion != MotionType.Static && Simulation.Bodies[record.Body].Awake)
        {
            record.CalmAt = -1;
            Simulation.Sleeper.Sleep(Simulation.Bodies.HandleToLocation[record.Body.Value].SetIndex == 0
                ? Simulation.Bodies.HandleToLocation[record.Body.Value].Index
                : -1);
        }
    }

    /// <inheritdoc />
    public bool IsAwake(BodyHandle body)
    {
        var record = Get(body);
        return record.Motion != MotionType.Static && Simulation.Bodies[record.Body].Awake;
    }

    /// <inheritdoc />
    public void GetAwakePoses(List<BodyPose> poses)
    {
        ArgumentNullException.ThrowIfNull(poses);
        ThrowIfDisposed();
        poses.Clear();
        ref var active = ref Simulation.Bodies.ActiveSet;
        for (var i = 0; i < active.Count; i++)
        {
            if (Lookup(byBody, active.IndexToHandle[i].Value) is { } body)
            {
                poses.Add(new BodyPose(body.Handle, body.ToOrigin(active.DynamicsState[i].Motion.Pose)));
            }
        }
    }

    /// <inheritdoc />
    public void SetPairIgnored(ulong colliderA, ulong colliderB, bool ignored)
    {
        var key = colliderA < colliderB ? (colliderA, colliderB) : (colliderB, colliderA);
        if (ignored)
        {
            ignoredPairs.Add(key);
        }
        else
        {
            ignoredPairs.Remove(key);
        }
    }

    /// <inheritdoc />
    public void SetContactReporting(BodyHandle body, bool report) => Get(body).ReportsContacts = report;

    /// <inheritdoc />
    public void Step(float deltaTime)
    {
        ThrowIfDisposed();
        if (deltaTime <= 0f)
        {
            return;
        }

        ContactFrequency = substeps / (2f * deltaTime);
        steps++;
        Events.BeginStep();
        Simulation.Timestep(deltaTime, dispatcher);
        AfterStep();
        HoldSleepCounts();
        Events.EndStep();
    }

    /// <inheritdoc />
    public void Dispose()
    {
        if (disposed)
        {
            return;
        }

        disposed = true;
        Simulation.Dispose();
        dispatcher?.Dispose();
        pool.Clear();
        Interlocked.Decrement(ref liveCount);
    }

    internal BepuBody? Find(CollidableReference collidable) => collidable.Mobility == CollidableMobility.Static
        ? Lookup(byStatic, collidable.StaticHandle.Value)
        : Lookup(byBody, collidable.BodyHandle.Value);

    internal bool IsPairIgnored(ulong colliderA, ulong colliderB) =>
        ignoredPairs.Count > 0 && ignoredPairs.Contains(colliderA < colliderB ? (colliderA, colliderB) : (colliderB, colliderA));

    internal BepuBody Get(BodyHandle body)
    {
        ThrowIfDisposed();
        return records.TryGetValue(body.Id, out var record)
            ? record
            : throw new ArgumentException($"Body {body.Id} does not exist.", nameof(body));
    }

    static BepuBody? Lookup(BepuBody?[] table, int index) => (uint)index < (uint)table.Length ? table[index] : null;

    static void Register<T>(ref T[] table, int index, T value)
    {
        if (index >= table.Length)
        {
            Array.Resize(ref table, Math.Max(index + 1, table.Length * 2));
        }

        table[index] = value;
    }

    BepuBody NewRecord(MotionType motion, ColliderDesc[]? colliders)
    {
        var body = new BepuBody { Handle = new BodyHandle(nextBody++), Motion = motion, Colliders = colliders ?? [] };
        body.Summarize();
        records.Add(body.Handle.Id, body);
        return body;
    }

    // Clears one-step kinematic velocities and holds locked axes exactly where they were.
    void AfterStep()
    {
        foreach (var record in records.Values)
        {
            if (record.HasKinematicTarget)
            {
                record.HasKinematicTarget = false;
                Simulation.Bodies[record.Body].Velocity = default;
            }

            if ((record.Locks & (AxisLocks.LinearX | AxisLocks.LinearY | AxisLocks.LinearZ)) != 0)
            {
                HoldLinearLocks(record);
            }
        }
    }

    /// <summary>
    /// Bepu restarts a body's sleep count on any step above its threshold, so one rocking box can keep a whole settled
    /// island awake. A step between the threshold and <see cref="SleepHysteresis"/> times it holds the count instead.
    /// </summary>
    void HoldSleepCounts()
    {
        var set = Simulation.Bodies.ActiveSet;
        for (var i = 0; i < set.Count; i++)
        {
            ref var activity = ref set.Activity[i];
            var handle = set.IndexToHandle[i].Value;
            if (activity.SleepThreshold < 0f || handle >= byBody.Length || byBody[handle] is not { } record)
            {
                continue;
            }

            var velocity = set.DynamicsState[i].Motion.Velocity;
            var energy = velocity.Linear.LengthSquared() + velocity.Angular.LengthSquared();
            var held = record.CalmAt == steps - 1 && handle < testedAt.Length && testedAt[handle] == steps
                ? record.CalmSteps : 0;
            record.CalmSteps = CountCalmSteps(energy, activity.SleepThreshold, held);
            record.CalmAt = steps;
            activity.TimestepsUnderThresholdCount = (byte)Math.Min(record.CalmSteps, byte.MaxValue);
            activity.SleepCandidate = record.CalmSteps >= activity.MinimumTimestepsUnderThreshold;
        }
    }

    static int CountCalmSteps(float energy, float threshold, int held) =>
        energy <= threshold ? Math.Min(held + 1, byte.MaxValue)
        : energy < threshold * SleepHysteresis ? held : 0;

    // Commands must not inherit a prior sleep candidacy before Bepu's next sleep pass.
    void WakeBody(BepuBody record)
    {
        record.CalmAt = -1;
        var reference = Simulation.Bodies[record.Body];
        reference.Awake = true;
        reference.Activity.TimestepsUnderThresholdCount = 0;
        reference.Activity.SleepCandidate = false;
    }

    void HoldLinearLocks(BepuBody record)
    {
        var reference = Simulation.Bodies[record.Body];
        var position = reference.Pose.Position;
        var velocity = reference.Velocity.Linear;
        var locked = record.LockedPosition;
        if ((record.Locks & AxisLocks.LinearX) != 0) { position.X = locked.X; velocity.X = 0f; }
        if ((record.Locks & AxisLocks.LinearY) != 0) { position.Y = locked.Y; velocity.Y = 0f; }
        if ((record.Locks & AxisLocks.LinearZ) != 0) { position.Z = locked.Z; velocity.Z = 0f; }
        reference.Pose.Position = position;
        reference.Velocity.Linear = velocity;
    }

    static BodyInertia LockInertia(BodyInertia inertia, AxisLocks locks)
    {
        ref var tensor = ref inertia.InverseInertiaTensor;
        if ((locks & AxisLocks.AngularX) != 0) { tensor.XX = 0f; tensor.YX = 0f; tensor.ZX = 0f; }
        if ((locks & AxisLocks.AngularY) != 0) { tensor.YX = 0f; tensor.YY = 0f; tensor.ZY = 0f; }
        if ((locks & AxisLocks.AngularZ) != 0) { tensor.ZX = 0f; tensor.ZY = 0f; tensor.ZZ = 0f; }
        return inertia;
    }

    void ThrowIfDisposed() => ObjectDisposedException.ThrowIf(disposed, this);
}
