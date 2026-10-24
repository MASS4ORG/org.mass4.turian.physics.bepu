namespace Turian.Engine.Physics;

/// <summary>Filters pairs, combines materials and records contacts and trigger overlaps on the worker threads.</summary>
struct BepuNarrowPhaseCallbacks(BepuPhysicsWorld world) : INarrowPhaseCallbacks
{
    // Bouncy pairs have no speculative margin, so they sink before touching. Above Unity's default bounce threshold
    // the push-out targets bounciness times the closing speed, so the sunk depth adds no energy.
    const float SolidRecoveryVelocity = 2f;
    const float RestingRecoveryVelocity = 0.2f;
    // The bounciness above which AllowContactGeneration drops the speculative margin.
    const float BouncyPair = 0.01f;
    const float BounceThreshold = 2f;

    public readonly void Initialize(Simulation simulation)
    {
    }

    public readonly bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b,
        ref float speculativeMargin)
    {
        if (world.Find(a) is not { } bodyA || world.Find(b) is not { } bodyB)
        {
            return false;
        }

        var anyDynamic = a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
        if (!anyDynamic && !bodyA.AnyTrigger && !bodyB.AnyTrigger)
        {
            return false;
        }

        var mayTouch = !bodyA.IsCompound && !bodyB.IsCompound
            ? ChildrenMayTouch(bodyA, bodyA.ColliderIndex(0), bodyB, bodyB.ColliderIndex(0))
            : bodyA.UsesGroups || bodyB.UsesGroups ||
              ((bodyA.BelongsTo & bodyB.CollidesWith) != 0 && (bodyB.BelongsTo & bodyA.CollidesWith) != 0);
        if (!mayTouch || world.AreJointedWithoutCollision(bodyA, bodyB))
        {
            return false;
        }

        // Speculative contacts would stop a bouncy body before its contact spring compresses, so it never rebounds.
        // Only pairs the filters accept pay for combining the materials.
        if (Bounce(bodyA.Material, bodyB.Material) > 0.01f)
        {
            speculativeMargin = 0f;
        }

        return true;
    }

    public readonly bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
    {
        if (world.Find(pair.A) is not { } bodyA || world.Find(pair.B) is not { } bodyB)
        {
            return false;
        }

        return ChildrenMayTouch(bodyA, bodyA.ColliderIndex(childIndexA), bodyB, bodyB.ColliderIndex(childIndexB));
    }

    public readonly bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair,
        ref TManifold manifold, out PairMaterialProperties pairMaterial)
        where TManifold : unmanaged, IContactManifold<TManifold>
    {
        pairMaterial = default;
        if (world.Find(pair.A) is not { } bodyA || world.Find(pair.B) is not { } bodyB)
        {
            return false;
        }

        var touches = Record(workerIndex, bodyA, bodyB, ref manifold);
        // Events see the measured depths before a rebound deepens them.
        pairMaterial = Combine(bodyA, bodyB, ref manifold);
        return touches && !bodyA.AllTriggers && !bodyB.AllTriggers &&
               (pair.A.Mobility == CollidableMobility.Dynamic || pair.B.Mobility == CollidableMobility.Dynamic);
    }

    // Records a single-collider pair's contact; compounds record per child. False for a trigger or missing collider.
    readonly bool Record<TManifold>(int workerIndex, BepuBody bodyA, BepuBody bodyB, ref TManifold manifold)
        where TManifold : unmanaged, IContactManifold<TManifold>
    {
        if (bodyA.IsCompound || bodyB.IsCompound)
        {
            return true;
        }

        int colliderA = bodyA.ColliderIndex(0), colliderB = bodyB.ColliderIndex(0);
        if (colliderA < 0 || colliderB < 0)
        {
            return false;
        }

        var trigger = bodyA.Colliders[colliderA].IsTrigger || bodyB.Colliders[colliderB].IsTrigger;
        world.Events.Record(workerIndex, bodyA, colliderA, bodyB, colliderB, ref manifold, trigger);
        return !trigger;
    }

    public readonly bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA,
        int childIndexB, ref ConvexContactManifold manifold)
    {
        if (world.Find(pair.A) is not { } bodyA || world.Find(pair.B) is not { } bodyB)
        {
            return false;
        }

        // Meshes report triangles as children; only compounds map children to colliders here.
        if (!bodyA.IsCompound && !bodyB.IsCompound)
        {
            return true;
        }

        int colliderA = bodyA.ColliderIndex(childIndexA), colliderB = bodyB.ColliderIndex(childIndexB);
        if (colliderA < 0 || colliderB < 0)
        {
            return false;
        }

        var trigger = bodyA.Colliders[colliderA].IsTrigger || bodyB.Colliders[colliderB].IsTrigger;
        world.Events.Record(workerIndex, bodyA, colliderA, bodyB, colliderB, ref manifold, trigger);
        return !trigger;
    }

    public readonly void Dispose()
    {
    }

    readonly bool ChildrenMayTouch(BepuBody bodyA, int colliderA, BepuBody bodyB, int colliderB)
    {
        if (colliderA < 0 || colliderB < 0)
        {
            return false;
        }

        ref readonly var a = ref bodyA.Colliders[colliderA];
        ref readonly var b = ref bodyB.Colliders[colliderB];
        return CollisionFilter.IsCollisionEnabled(a.Filter, b.Filter) && !world.IsPairIgnored(a.UserData, b.UserData);
    }

    readonly PairMaterialProperties Combine<TManifold>(BepuBody bodyA, BepuBody bodyB, ref TManifold manifold)
        where TManifold : unmanaged, IContactManifold<TManifold>
    {
        var a = bodyA.Material;
        var b = bodyB.Material;
        var friction = MathF.Max(
            MaterialDesc.Combine(a.DynamicFriction, b.DynamicFriction, a.FrictionCombine, b.FrictionCombine), 0f);
        var bounce = Bounce(a, b);
        var closing = bounce > BouncyPair ? ClosingSpeed(bodyA, bodyB, ref manifold) : 0f;
        if (closing <= BounceThreshold)
        {
            var recovery = bounce > BouncyPair ? RestingRecoveryVelocity : SolidRecoveryVelocity;
            return new PairMaterialProperties(friction, recovery, new SpringSettings(world.ContactFrequency, 1f));
        }

        // An undamped spring pushes at depth per substep, so twice the rebound's substep travel reaches it at once.
        var rebound = bounce * closing;
        var depth = rebound / world.ContactFrequency;
        for (var i = 0; i < manifold.Count; i++)
        {
            ref var contactDepth = ref TManifold.GetDepthReference(ref manifold, i);
            contactDepth = MathF.Max(contactDepth, depth);
        }

        return new PairMaterialProperties(friction, rebound, new SpringSettings(world.ContactFrequency, 0f));
    }

    // The fastest approach along any contact normal, which points from B to A.
    readonly float ClosingSpeed<TManifold>(BepuBody a, BepuBody b, ref TManifold manifold)
        where TManifold : unmanaged, IContactManifold<TManifold>
    {
        var origin = world.CollidablePosition(a);
        var closing = 0f;
        for (var i = 0; i < manifold.Count; i++)
        {
            manifold.GetContact(i, out var offset, out var normal, out _, out _);
            var point = origin + offset;
            var relative = world.PointVelocity(a, point) - world.PointVelocity(b, point);
            closing = MathF.Max(closing, -Vector3.Dot(relative, normal));
        }

        return closing;
    }

    static float Bounce(in MaterialDesc a, in MaterialDesc b) =>
        Math.Clamp(MaterialDesc.Combine(a.Bounciness, b.Bounciness, a.BounceCombine, b.BounceCombine), 0f, 1f);
}

/// <summary>Applies gravity, per-body damping and linear locks to SIMD bundles of bodies.</summary>
struct BepuPoseIntegratorCallbacks(BepuPhysicsWorld world) : IPoseIntegratorCallbacks
{
    Vector3Wide gravity;

    public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

    public readonly bool AllowSubstepsForUnconstrainedBodies => false;

    public readonly bool IntegrateVelocityForKinematics => false;

    public readonly void Initialize(Simulation simulation)
    {
    }

    public void PrepareForIntegration(float dt) => gravity = Vector3Wide.Broadcast(world.Gravity);

    public readonly void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation,
        BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt,
        ref BodyVelocityWide velocity)
    {
        Span<float> scale = stackalloc float[Vector<float>.Count];
        Span<float> linear = stackalloc float[Vector<float>.Count];
        Span<float> angular = stackalloc float[Vector<float>.Count];
        Span<float> keepX = stackalloc float[Vector<float>.Count];
        Span<float> keepY = stackalloc float[Vector<float>.Count];
        Span<float> keepZ = stackalloc float[Vector<float>.Count];
        Span<float> spinX = stackalloc float[Vector<float>.Count];
        Span<float> spinY = stackalloc float[Vector<float>.Count];
        Span<float> spinZ = stackalloc float[Vector<float>.Count];
        var handles = world.Simulation.Bodies.ActiveSet.IndexToHandle;
        var table = world.Integration;
        for (var lane = 0; lane < Vector<float>.Count; lane++)
        {
            var index = bodyIndices[lane];
            ref readonly var factors = ref index >= 0 ? ref table[handles[index].Value] : ref BodyIntegration.Empty;
            var step = dt[lane];
            scale[lane] = factors.GravityScale;
            linear[lane] = MathF.Max(0f, 1f - factors.LinearDamping * step);
            angular[lane] = MathF.Max(0f, 1f - factors.AngularDamping * step);
            keepX[lane] = factors.Linear.X;
            keepY[lane] = factors.Linear.Y;
            keepZ[lane] = factors.Linear.Z;
            spinX[lane] = factors.Angular.X;
            spinY[lane] = factors.Angular.Y;
            spinZ[lane] = factors.Angular.Z;
        }

        var gravityStep = new Vector<float>(scale) * dt;
        var linearDamping = new Vector<float>(linear);
        var angularDamping = new Vector<float>(angular);
        velocity.Linear.X = (velocity.Linear.X + gravity.X * gravityStep) * linearDamping * new Vector<float>(keepX);
        velocity.Linear.Y = (velocity.Linear.Y + gravity.Y * gravityStep) * linearDamping * new Vector<float>(keepY);
        velocity.Linear.Z = (velocity.Linear.Z + gravity.Z * gravityStep) * linearDamping * new Vector<float>(keepZ);
        // Locked inertia stops forces from turning the body; the velocity a script set is dropped in local space.
        var local = QuaternionWide.TransformByConjugate(velocity.Angular * angularDamping, orientation);
        local.X *= new Vector<float>(spinX);
        local.Y *= new Vector<float>(spinY);
        local.Z *= new Vector<float>(spinZ);
        velocity.Angular = QuaternionWide.Transform(local, orientation);
    }

}

/// <summary>
/// One body's gravity scale, damping rates and axis multipliers for integration: 1 keeps an axis's velocity, 0 drops
/// it.
/// </summary>
readonly record struct BodyIntegration(float GravityScale, float LinearDamping, float AngularDamping, Vector3 Linear,
    Vector3 Angular)
{
    /// <summary>The factors of an empty bundle lane, which change nothing.</summary>
    public static readonly BodyIntegration Empty = new(0f, 0f, 0f, Vector3.One, Vector3.One);

    /// <summary>The factors of a dynamic body.</summary>
    public static BodyIntegration Of(BepuBody body) => new(body.GravityScale, body.LinearDamping, body.AngularDamping,
        Keep(body.Locks, AxisLocks.LinearX, AxisLocks.LinearY, AxisLocks.LinearZ),
        Keep(body.Locks, AxisLocks.AngularX, AxisLocks.AngularY, AxisLocks.AngularZ));

    static Vector3 Keep(AxisLocks locks, AxisLocks x, AxisLocks y, AxisLocks z) =>
        new(Free(locks, x), Free(locks, y), Free(locks, z));

    static float Free(AxisLocks locks, AxisLocks axis) => (locks & axis) == 0 ? 1f : 0f;
}
