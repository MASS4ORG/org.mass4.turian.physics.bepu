using Microsoft.Extensions.Logging;
using Log = Turian.Engine.Core.Log;

namespace Turian.Engine.Physics;

public sealed partial class BepuPhysicsWorld
{
    // Picks a single shape when the body has one collider at its center of mass, otherwise builds a compound whose
    // origin is the center of mass; returns the inertia for a dynamic body.
    BodyInertia BuildCollidable(BepuBody body, float mass, in PhysicsMass description)
    {
        var usable = UsableColliders(body);
        body.ChildToCollider = [.. usable];
        if (usable.Count == 0)
        {
            body.Shape = default;
            return mass > 0f ? new BepuSphere(0.5f).ComputeInertia(mass) : default;
        }

        var dynamic = mass > 0f;
        if (usable.Count == 1 && (!dynamic || description.AutomaticCenterOfMass))
        {
            var collider = body.Colliders[usable[0]];
            var shape = shapes[collider.Shape.Id];
            body.Offset = ChildPose(collider, shape);
            body.Shape = shape.Index;
            return dynamic ? BepuShapes.Inertia(shape, mass) : default;
        }

        return BuildCompound(body, usable, mass, description);
    }

    BodyInertia BuildCompound(BepuBody body, List<int> usable, float mass, in PhysicsMass description)
    {
        var dynamic = mass > 0f;
        var poses = new RigidPose[usable.Count];
        var weights = new float[usable.Count];
        var volume = 0f;
        for (var i = 0; i < usable.Count; i++)
        {
            var collider = body.Colliders[usable[i]];
            var shape = shapes[collider.Shape.Id];
            poses[i] = ChildPose(collider, shape);
            weights[i] = MathF.Max(shape.Description.Volume, 1e-6f);
            volume += weights[i];
        }

        var center = Vector3.Zero;
        if (dynamic)
        {
            for (var i = 0; i < weights.Length; i++)
            {
                weights[i] = mass * weights[i] / volume;
                center += poses[i].Position * weights[i];
            }

            center = description.AutomaticCenterOfMass ? center / mass : description.CenterOfMass;
        }

        using var builder = new CompoundBuilder(pool, Simulation.Shapes, usable.Count);
        for (var i = 0; i < usable.Count; i++)
        {
            var shape = shapes[body.Colliders[usable[i]].Shape.Id];
            var pose = new RigidPose(poses[i].Position - center, poses[i].Orientation);
            if (dynamic)
            {
                builder.Add(shape.Index, pose, BepuShapes.Inertia(shape, weights[i]).InverseInertiaTensor, weights[i]);
            }
            else
            {
                builder.AddForKinematic(shape.Index, pose, 1f);
            }
        }

        Buffer<CompoundChild> children;
        var inertia = default(BodyInertia);
        if (dynamic)
        {
            builder.BuildDynamicCompound(out children, out inertia);
        }
        else
        {
            builder.BuildKinematicCompound(out children);
        }

        body.Shape = Simulation.Shapes.Add(new Compound(children));
        body.OwnsShape = true;
        body.IsCompound = true;
        body.Offset = new RigidPose(center, Quaternion.Identity);
        return inertia;
    }

    // A mesh must be the only collider of a static or kinematic body; other colliders must name a live shape.
    List<int> UsableColliders(BepuBody body)
    {
        var usable = new List<int>(body.Colliders.Length);
        for (var i = 0; i < body.Colliders.Length; i++)
        {
            if (!shapes.TryGetValue(body.Colliders[i].Shape.Id, out var shape))
            {
                throw new ArgumentException($"Shape {body.Colliders[i].Shape.Id} does not exist.");
            }

            if (!shape.Description.IsConvex && (body.Colliders.Length > 1 || body.Motion == MotionType.Dynamic))
            {
                Log.Logger.LogWarning(
                    "A triangle mesh collider must be the only collider of a static or kinematic body; it is ignored");
                continue;
            }

            usable.Add(i);
        }

        return usable;
    }

    static RigidPose ChildPose(in ColliderDesc collider, in BepuShape shape) => new(
        collider.LocalPose.Position + Vector3.Transform(shape.Center, collider.LocalPose.Orientation),
        collider.LocalPose.Orientation);
}
