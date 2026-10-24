namespace Turian.Engine.Physics;

public sealed partial class BepuPhysicsWorld
{
    /// <summary>
    /// Whether a body was static or asleep when the last step's collision detection began, so its pairs were not
    /// tested; false for removed bodies. A body woken during detection, as by a new body landing on it, counts as
    /// untested for that step although it is awake afterwards.
    /// </summary>
    internal bool IsUntested(int id) => records.TryGetValue(id, out var record) &&
                                        (record.Motion == MotionType.Static ||
                                         (uint)record.Body.Value >= (uint)testedAt.Length ||
                                         testedAt[record.Body.Value] != steps);

    // Marks the bodies awake as detection begins: exactly those whose pairs the step tests.
    void StampTested()
    {
        ref var active = ref Simulation.Bodies.ActiveSet;
        for (var i = 0; i < active.Count; i++)
        {
            var handle = active.IndexToHandle[i].Value;
            if (handle >= testedAt.Length)
            {
                Array.Resize(ref testedAt, Math.Max(handle + 1, testedAt.Length * 2));
            }

            testedAt[handle] = steps;
        }
    }

    /// <summary>The world position of a body's collidable.</summary>
    internal Vector3 CollidablePosition(BepuBody body) => CollidablePose(body).Position;

    /// <summary>The world pose of a body's collidable.</summary>
    internal RigidPose CollidablePose(BepuBody body) => body.Motion == MotionType.Static
        ? Simulation.Statics[body.Static].Pose
        : Simulation.Bodies[body.Body].Pose;

    /// <summary>The velocity of a body at a world point; zero for statics.</summary>
    internal Vector3 PointVelocity(BepuBody body, Vector3 point)
    {
        if (body.Motion == MotionType.Static)
        {
            return Vector3.Zero;
        }

        var reference = Simulation.Bodies[body.Body];
        var velocity = reference.Velocity;
        return velocity.Linear + Vector3.Cross(velocity.Angular, point - reference.Pose.Position);
    }

    /// <summary>The inverse mass of a body; zero for statics and kinematics.</summary>
    internal float InverseMass(BepuBody body) => body.Motion == MotionType.Dynamic
        ? Simulation.Bodies[body.Body].LocalInertia.InverseMass
        : 0f;

    /// <summary>The world pose of one collider of a body.</summary>
    internal RigidPose ColliderPose(BepuBody body, int collider)
    {
        var origin = body.ToOrigin(CollidablePose(body));
        var shape = shapes[body.Colliders[collider].Shape.Id];
        var local = ChildPose(body.Colliders[collider], shape);
        return new RigidPose(origin.TransformPoint(local.Position),
            Quaternion.Normalize(origin.Orientation * local.Orientation));
    }

    /// <summary>The Bepu shape of one collider.</summary>
    internal TypedIndex ColliderShape(BepuBody body, int collider) => shapes[body.Colliders[collider].Shape.Id].Index;
}
