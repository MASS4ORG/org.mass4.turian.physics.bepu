namespace Turian.Engine.Physics;

/// <summary>One Bepu body or static plus the per-collider data the callbacks read during a step.</summary>
sealed class BepuBody
{
    /// <summary>The handle the world hands out.</summary>
    public required BodyHandle Handle { get; init; }

    /// <summary>Static, kinematic or dynamic.</summary>
    public required MotionType Motion { get; init; }

    /// <summary>The Bepu body, when not static.</summary>
    public BepuBodyHandle Body { get; set; }

    /// <summary>The Bepu static, when static.</summary>
    public StaticHandle Static { get; set; }

    /// <summary>The collidable's pose relative to the body's origin.</summary>
    public RigidPose Offset { get; set; } = RigidPose.Identity;

    /// <summary>The colliders as described.</summary>
    public ColliderDesc[] Colliders { get; set; } = [];

    /// <summary>For each compound child, the index of its collider in <see cref="Colliders"/>.</summary>
    public int[] ChildToCollider { get; set; } = [];

    /// <summary>Whether the collidable is a compound whose children map through <see cref="ChildToCollider"/>.</summary>
    public bool IsCompound { get; set; }

    /// <summary>The collidable's shape, or a non-existing index for a shapeless body.</summary>
    public TypedIndex Shape { get; set; }

    /// <summary>Whether <see cref="Shape"/> is a compound the body created and frees.</summary>
    public bool OwnsShape { get; set; }

    /// <summary>The union of the colliders' categories.</summary>
    public uint BelongsTo { get; set; }

    /// <summary>The union of the colliders' accepted categories.</summary>
    public uint CollidesWith { get; set; }

    /// <summary>Whether any collider uses a non-zero group, so the unions cannot reject a pair.</summary>
    public bool UsesGroups { get; set; }

    /// <summary>Whether every collider is a trigger.</summary>
    public bool AllTriggers { get; set; }

    /// <summary>Whether any collider is a trigger.</summary>
    public bool AnyTrigger { get; set; }

    /// <summary>The material of the first solid collider.</summary>
    public MaterialDesc Material { get; set; } = MaterialDesc.Default;

    /// <summary>Whether its solid contacts are reported as events; workers read it during the step.</summary>
    public bool ReportsContacts { get; set; } = true;

    /// <summary>Gravity multiplier.</summary>
    public float GravityScale { get; set; }

    /// <summary>Linear damping per second.</summary>
    public float LinearDamping { get; set; }

    /// <summary>Angular damping per second.</summary>
    public float AngularDamping { get; set; }

    /// <summary>Locked axes.</summary>
    public AxisLocks Locks { get; set; }

    /// <summary>The world position of the center of mass that linear locks hold.</summary>
    public Vector3 LockedPosition { get; set; }

    /// <summary>The body's mass, zero unless dynamic.</summary>
    public float Mass { get; set; }

    /// <summary>Whether a kinematic target set the velocity for the coming step only.</summary>
    public bool HasKinematicTarget { get; set; }

    /// <summary>Steps the body has spent below its sleep threshold, held through brief spikes.</summary>
    public int CalmSteps { get; set; }

    /// <summary>The world step <see cref="CalmSteps"/> was last counted on, so a body that slept starts over.</summary>
    public int CalmAt { get; set; } = -1;

    /// <summary>The joints attached to this body.</summary>
    public List<JointHandle> Joints { get; } = [];

    /// <summary>The Bepu reference of the collidable.</summary>
    public CollidableReference Reference => Motion switch
    {
        MotionType.Static => new CollidableReference(Static),
        MotionType.Kinematic => new CollidableReference(CollidableMobility.Kinematic, Body),
        _ => new CollidableReference(CollidableMobility.Dynamic, Body),
    };

    /// <summary>The index in <see cref="Colliders"/> of a collidable child, or -1 when it has none.</summary>
    public int ColliderIndex(int child) => IsCompound
        ? (uint)child < (uint)ChildToCollider.Length ? ChildToCollider[child] : -1
        : ChildToCollider.Length > 0 ? ChildToCollider[0] : -1;

    /// <summary>Converts the pose of the body's origin into the collidable's pose.</summary>
    public RigidPose ToCollidable(in PhysicsPose origin) => new(
        origin.Position + Vector3.Transform(Offset.Position, origin.Orientation),
        Quaternion.Normalize(origin.Orientation * Offset.Orientation));

    /// <summary>Converts the collidable's pose into the pose of the body's origin.</summary>
    public PhysicsPose ToOrigin(in RigidPose collidable)
    {
        var orientation = Quaternion.Normalize(collidable.Orientation * Quaternion.Conjugate(Offset.Orientation));
        return new PhysicsPose(collidable.Position - Vector3.Transform(Offset.Position, orientation), orientation);
    }

    /// <summary>Computes the filter unions and trigger and material summaries from <see cref="Colliders"/>.</summary>
    public void Summarize()
    {
        uint belongs = 0, collides = 0;
        var groups = false;
        var solid = 0;
        foreach (var collider in Colliders)
        {
            belongs |= collider.Filter.BelongsTo;
            collides |= collider.Filter.CollidesWith;
            groups |= collider.Filter.GroupIndex != 0;
            if (!collider.IsTrigger && solid++ == 0)
            {
                Material = collider.Material;
            }
        }

        BelongsTo = belongs;
        CollidesWith = collides;
        UsesGroups = groups;
        AllTriggers = Colliders.Length > 0 && solid == 0;
        AnyTrigger = solid < Colliders.Length;
    }
}
