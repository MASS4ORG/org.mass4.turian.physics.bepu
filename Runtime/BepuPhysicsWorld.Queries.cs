namespace Turian.Engine.Physics;

public sealed partial class BepuPhysicsWorld
{
    // The batcher's dt only matters for speculative velocity expansion, which queries do not use.
    const float BatchDeltaTime = 1f / 60f;

    /// <inheritdoc />
    public void CastRay<TCollector>(in RaycastInput input, ref TCollector collector) where TCollector : IHitCollector
    {
        ThrowIfDisposed();
        var delta = input.End - input.Start;
        if (delta.LengthSquared() <= 0f)
        {
            return;
        }

        var handler = new BepuRayHandler<TCollector>(new BepuQueryFilter(this, input.Filter, input.HitTriggers),
            input.Start, delta, collector);
        Simulation.RayCast(input.Start, delta, MathF.Min(1f, collector.MaxFraction), pool, ref handler);
        collector = handler.Collector;
    }

    /// <inheritdoc />
    public void CastShape<TCollector>(in ShapeCastInput input, ref TCollector collector)
        where TCollector : IHitCollector
    {
        ThrowIfDisposed();
        var filter = new BepuQueryFilter(this, input.Filter, input.HitTriggers);
        var delta = input.End - input.Start;
        var handler = new BepuSweepHandler<TCollector>(this, filter, input.Start, delta, collector);
        var pose = new RigidPose(input.Start, input.Orientation);
        var velocity = new BodyVelocity(delta, Vector3.Zero);
        var maximum = MathF.Min(1f, collector.MaxFraction);
        var shape = input.Shape;
        switch (shape.Type)
        {
            case ShapeType.Box:
                Simulation.Sweep(BepuShapes.ToBox(shape), pose, velocity, maximum, pool, ref handler);
                break;
            case ShapeType.Sphere:
                Simulation.Sweep(BepuShapes.ToSphere(shape), pose, velocity, maximum, pool, ref handler);
                break;
            case ShapeType.Capsule:
                Simulation.Sweep(BepuShapes.ToCapsule(shape), pose, velocity, maximum, pool, ref handler);
                break;
            case ShapeType.Cylinder:
                Simulation.Sweep(BepuShapes.ToCylinder(shape), pose, velocity, maximum, pool, ref handler);
                break;
            case ShapeType.ConvexHull:
                var hull = new ConvexHull(shape.Points ?? [], pool, out var center);
                pose.Position += Vector3.Transform(center, pose.Orientation);
                Simulation.Sweep(hull, pose, velocity, maximum, pool, ref handler);
                hull.Dispose(pool);
                break;
            default:
                throw new ArgumentException("Only convex shapes can be cast.", nameof(input));
        }

        collector = handler.Collector;
    }

    /// <inheritdoc />
    public void Overlap<TCollector>(in OverlapInput input, ref TCollector collector) where TCollector : IHitCollector
    {
        ThrowIfDisposed();
        var filter = new BepuQueryFilter(this, input.Filter, input.HitTriggers);
        var query = AddQueryShape(input.Shape);
        try
        {
            var pose = QueryPose(query, input.Pose);
            Simulation.Shapes[query.Index.Type].ComputeBounds(query.Index.Index, pose, out var min, out var max);
            var candidates = new BepuCandidates { Found = [] };
            Simulation.BroadPhase.GetOverlaps(min, max, pool, ref candidates);
            var results = new BepuBatchResults(filter);
            var batcher = new CollisionBatcher<BepuBatchCallbacks>(pool, Simulation.Shapes,
                Simulation.NarrowPhase.CollisionTaskRegistry, BatchDeltaTime, new BepuBatchCallbacks(results));
            foreach (var candidate in candidates.Found)
            {
                var body = filter.Body(candidate);
                if (body is null || !body.Shape.Exists)
                {
                    continue;
                }

                var target = CollidablePose(body);
                var continuation = new PairContinuation(results.Targets.Count);
                results.Targets.Add(body);
                batcher.Add(query.Index, body.Shape, target.Position - pose.Position, pose.Orientation,
                    target.Orientation, 0f, in continuation);
            }

            batcher.Flush();
            foreach (var ((pair, collider), deepest) in results.Found)
            {
                var body = results.Targets[pair]!;
                if (deepest.Depth < 0f || !filter.Accepts(body, collider) ||
                    !collector.AddHit(new PhysicsQueryHit(body.Handle, body.Colliders[collider].UserData,
                        pose.Position + deepest.Offset, deepest.Normal, 0f)))
                {
                    continue;
                }

                if (collector.EarlyOutOnFirstHit)
                {
                    break;
                }
            }
        }
        finally
        {
            BepuShapes.Remove(Simulation.Shapes, pool, query);
        }
    }

    /// <inheritdoc />
    public bool CalculateDistance(in ShapeDesc shapeA, in PhysicsPose poseA, in ShapeDesc shapeB,
        in PhysicsPose poseB, float maxDistance, out DistanceResult result)
    {
        ThrowIfDisposed();
        var queryB = AddQueryShape(shapeB);
        try
        {
            return Distance(shapeA, poseA, queryB.Index, QueryPose(queryB, poseB), maxDistance, out result);
        }
        finally
        {
            BepuShapes.Remove(Simulation.Shapes, pool, queryB);
        }
    }

    /// <inheritdoc />
    public bool CalculateDistance(in ShapeDesc shape, in PhysicsPose pose, BodyHandle body, ulong collider,
        float maxDistance, out DistanceResult result)
    {
        var record = Get(body);
        var index = Array.FindIndex(record.Colliders, c => c.UserData == collider);
        if (index < 0 || Array.IndexOf(record.ChildToCollider, index) < 0)
        {
            result = default;
            return false;
        }

        return Distance(shape, pose, ColliderShape(record, index), ColliderPose(record, index), maxDistance,
            out result);
    }

    /// <summary>The collider of a body nearest to a world point among those the filter accepts.</summary>
    internal int NearestCollider(BepuBody body, Vector3 point, BepuQueryFilter filter)
    {
        if (!body.IsCompound)
        {
            var single = body.ColliderIndex(0);
            return filter.Accepts(body, single) ? single : -1;
        }

        var best = -1;
        var bestDistance = float.MaxValue;
        foreach (var collider in body.ChildToCollider)
        {
            var distance = Vector3.DistanceSquared(ColliderPose(body, collider).Position, point);
            if (filter.Accepts(body, collider) && distance < bestDistance)
            {
                best = collider;
                bestDistance = distance;
            }
        }

        return best;
    }

    bool Distance(in ShapeDesc shapeA, in PhysicsPose poseA, TypedIndex shapeB, RigidPose poseB, float maxDistance,
        out DistanceResult result)
    {
        var queryA = AddQueryShape(shapeA);
        try
        {
            var pose = QueryPose(queryA, poseA);
            var results = new BepuBatchResults(null);
            results.Targets.Add(null);
            var batcher = new CollisionBatcher<BepuBatchCallbacks>(pool, Simulation.Shapes,
                Simulation.NarrowPhase.CollisionTaskRegistry, BatchDeltaTime, new BepuBatchCallbacks(results));
            var continuation = new PairContinuation(0);
            batcher.Add(queryA.Index, shapeB, poseB.Position - pose.Position, pose.Orientation, poseB.Orientation,
                MathF.Max(maxDistance, 0f), in continuation);
            batcher.Flush();
            if (!results.Found.TryGetValue((0, 0), out var deepest) || -deepest.Depth > maxDistance)
            {
                result = default;
                return false;
            }

            var point = pose.Position + deepest.Offset;
            result = new DistanceResult(-deepest.Depth, deepest.Normal, point);
            return true;
        }
        finally
        {
            BepuShapes.Remove(Simulation.Shapes, pool, queryA);
        }
    }

    BepuShape AddQueryShape(in ShapeDesc shape)
    {
        if (!shape.IsConvex)
        {
            throw new ArgumentException("Only convex shapes can be used as queries.", nameof(shape));
        }

        return BepuShapes.Add(Simulation.Shapes, pool, shape);
    }

    static RigidPose QueryPose(in BepuShape shape, in PhysicsPose pose) => new(
        pose.Position + Vector3.Transform(shape.Center, pose.Orientation), pose.Orientation);
}
