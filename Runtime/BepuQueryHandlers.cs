namespace Turian.Engine.Physics;

/// <summary>Applies a query's filter and trigger choice to bodies and their colliders.</summary>
readonly struct BepuQueryFilter(BepuPhysicsWorld world, CollisionFilter filter, bool hitTriggers)
{
    /// <summary>The body behind a collidable when any of its colliders could be accepted.</summary>
    public BepuBody? Body(CollidableReference collidable) =>
        world.Find(collidable) is { } body && (filter.CollidesWith & body.BelongsTo) != 0 &&
        (hitTriggers || !body.AllTriggers)
            ? body
            : null;

    /// <summary>Whether a collider of an accepted body is accepted.</summary>
    public bool Accepts(BepuBody body, int collider) =>
        collider >= 0 && filter.Accepts(body.Colliders[collider].Filter) &&
        (hitTriggers || !body.Colliders[collider].IsTrigger);
}

/// <summary>Forwards Bepu ray hits to a collector as fractions of the ray.</summary>
struct BepuRayHandler<TCollector>(BepuQueryFilter filter, Vector3 start, Vector3 delta, TCollector collector)
    : IRayHitHandler where TCollector : IHitCollector
{
    public TCollector Collector = collector;
    bool done;

    public readonly bool AllowTest(CollidableReference collidable) => !done && filter.Body(collidable) is not null;

    public readonly bool AllowTest(CollidableReference collidable, int childIndex) =>
        !done && filter.Body(collidable) is { } body && filter.Accepts(body, body.ColliderIndex(childIndex));

    public void OnRayHit(in RayData ray, ref float maximumT, float t, Vector3 normal, CollidableReference collidable,
        int childIndex)
    {
        if (done || filter.Body(collidable) is not { } body || body.ColliderIndex(childIndex) is var collider &&
            !filter.Accepts(body, collider))
        {
            return;
        }

        var hit = new PhysicsQueryHit(body.Handle, body.Colliders[collider].UserData, start + delta * t,
            Vector3.Normalize(normal), t);
        if (Collector.AddHit(hit))
        {
            done = Collector.EarlyOutOnFirstHit;
            maximumT = done ? 0f : MathF.Min(maximumT, Collector.MaxFraction);
        }
    }
}

/// <summary>Forwards Bepu sweep hits to a collector as fractions of the sweep.</summary>
struct BepuSweepHandler<TCollector>(BepuPhysicsWorld world, BepuQueryFilter filter, Vector3 start, Vector3 delta,
    TCollector collector) : ISweepHitHandler where TCollector : IHitCollector
{
    public TCollector Collector = collector;
    bool done;

    public readonly bool AllowTest(CollidableReference collidable) => !done && filter.Body(collidable) is not null;

    public readonly bool AllowTest(CollidableReference collidable, int child) =>
        !done && filter.Body(collidable) is { } body && filter.Accepts(body, body.ColliderIndex(child));

    public void OnHit(ref float maximumT, float t, Vector3 hitLocation, Vector3 hitNormal,
        CollidableReference collidable) => Report(ref maximumT, t, hitLocation, hitNormal, collidable);

    public void OnHitAtZeroT(ref float maximumT, CollidableReference collidable)
    {
        var direction = delta.LengthSquared() > 0f ? Vector3.Normalize(delta) : Vector3.UnitY;
        Report(ref maximumT, 0f, start, -direction, collidable);
    }

    void Report(ref float maximumT, float t, Vector3 point, Vector3 normal, CollidableReference collidable)
    {
        if (done || filter.Body(collidable) is not { } body)
        {
            return;
        }

        var collider = world.NearestCollider(body, point, filter);
        if (collider < 0)
        {
            return;
        }

        var unit = normal.LengthSquared() > 0f ? Vector3.Normalize(normal) : normal;
        if (Collector.AddHit(new PhysicsQueryHit(body.Handle, body.Colliders[collider].UserData, point, unit, t)))
        {
            done = Collector.EarlyOutOnFirstHit;
            maximumT = done ? 0f : MathF.Min(maximumT, Collector.MaxFraction);
        }
    }
}

/// <summary>Collects broad-phase candidates.</summary>
struct BepuCandidates : IBreakableForEach<CollidableReference>
{
    public List<CollidableReference> Found;

    public readonly bool LoopBody(CollidableReference collidable)
    {
        Found.Add(collidable);
        return true;
    }
}

/// <summary>The deepest contact found for one collider of a batched pair.</summary>
/// <param name="Depth">Penetration depth; negative is a gap.</param>
/// <param name="Normal">The normal from the target towards the query shape.</param>
/// <param name="Offset">The contact position relative to the query shape.</param>
readonly record struct BepuDeepest(float Depth, Vector3 Normal, Vector3 Offset);

/// <summary>Gathers the deepest contact per pair and collider from a collision batch.</summary>
sealed class BepuBatchResults(BepuQueryFilter? filter)
{
    /// <summary>The target body of each pair, or null for a bare shape.</summary>
    public List<BepuBody?> Targets { get; } = [];

    /// <summary>The deepest contact per pair and target collider.</summary>
    public Dictionary<(int Pair, int Collider), BepuDeepest> Found { get; } = [];

    // The query shape is convex, so whichever side a flipped pair reports, the larger index is the target child.
    public bool Allow(int pair, int childA, int childB) =>
        filter is not { } accepted || Targets[pair] is not { IsCompound: true } body ||
        accepted.Accepts(body, body.ColliderIndex(Math.Max(childA, childB)));

    public void Child(int pair, int childA, int childB, ref ConvexContactManifold manifold)
    {
        if (Targets[pair] is { IsCompound: true } body)
        {
            Keep(pair, body.ColliderIndex(Math.Max(childA, childB)), ref manifold);
        }
    }

    public void Pair<TManifold>(int pair, ref TManifold manifold)
        where TManifold : unmanaged, IContactManifold<TManifold>
    {
        if (Targets[pair] is not { IsCompound: true })
        {
            Keep(pair, Targets[pair]?.ColliderIndex(0) ?? 0, ref manifold);
        }
    }

    void Keep<TManifold>(int pair, int collider, ref TManifold manifold)
        where TManifold : unmanaged, IContactManifold<TManifold>
    {
        for (var i = 0; i < manifold.Count; i++)
        {
            manifold.GetContact(i, out var offset, out var normal, out var depth, out _);
            if (!Found.TryGetValue((pair, collider), out var deepest) || depth > deepest.Depth)
            {
                Found[(pair, collider)] = new BepuDeepest(depth, normal, offset);
            }
        }
    }
}

/// <summary>Routes batcher results to <see cref="BepuBatchResults"/>.</summary>
struct BepuBatchCallbacks(BepuBatchResults results) : ICollisionCallbacks
{
    public readonly bool AllowCollisionTesting(int pairId, int childA, int childB) =>
        results.Allow(pairId, childA, childB);

    public readonly void OnChildPairCompleted(int pairId, int childA, int childB, ref ConvexContactManifold manifold) =>
        results.Child(pairId, childA, childB, ref manifold);

    public readonly void OnPairCompleted<TManifold>(int pairId, ref TManifold manifold)
        where TManifold : unmanaged, IContactManifold<TManifold> => results.Pair(pairId, ref manifold);
}
