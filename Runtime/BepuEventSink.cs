using System.Runtime.InteropServices;

namespace Turian.Engine.Physics;

/// <summary>
/// Collects contacts and trigger overlaps per worker during a step, then merges them and derives enter, stay and
/// exit by comparing the pairs with the previous step's.
/// </summary>
sealed class BepuEventSink
{
    // Unity reports a collision once surfaces are within its default contact offset.
    const float ContactOffset = 0.01f;

    // Workers count into their own cache line.
    const int CounterStride = 16;

    readonly BepuPhysicsWorld world;
    readonly List<Raw>[] workers;
    readonly int[] unreported;
    readonly List<ContactEvent> contacts = [];
    readonly List<TriggerEvent> triggers = [];
    readonly Dictionary<PairKey, Tracked<ContactEvent>> contactPairs = [];
    readonly Dictionary<PairKey, Tracked<TriggerEvent>> triggerPairs = [];
    int step;

    public BepuEventSink(BepuPhysicsWorld world, int workerCount)
    {
        this.world = world;
        workers = new List<Raw>[Math.Max(1, workerCount)];
        unreported = new int[workers.Length * CounterStride];
        for (var i = 0; i < workers.Length; i++)
        {
            workers[i] = [];
        }
    }

    /// <summary>The contacts of the last step.</summary>
    public ReadOnlySpan<ContactEvent> Contacts => CollectionsMarshal.AsSpan(contacts);

    /// <summary>The trigger overlaps of the last step.</summary>
    public ReadOnlySpan<TriggerEvent> Triggers => CollectionsMarshal.AsSpan(triggers);

    /// <summary>The touching solid pairs of the last step: the reported ones that did not exit, and the others found.</summary>
    public int ContactPairCount { get; private set; }

    /// <summary>Clears the per-worker buffers before a step.</summary>
    public void BeginStep()
    {
        foreach (var buffer in workers)
        {
            buffer.Clear();
        }

        Array.Clear(unreported);
    }

    /// <summary>
    /// Records a manifold found by a worker; untouched speculative contacts are dropped, and touching pairs of two
    /// bodies that do not report contacts are only counted.
    /// </summary>
    public void Record<TManifold>(int worker, BepuBody a, int colliderA, BepuBody b, int colliderB,
        ref TManifold manifold, bool trigger) where TManifold : unmanaged, IContactManifold<TManifold>
    {
        var count = manifold.Count;
        if (count == 0)
        {
            return;
        }

        var deepest = float.MinValue;
        for (var i = 0; i < count; i++)
        {
            deepest = MathF.Max(deepest, manifold.GetDepth(i));
        }

        if (deepest < (trigger ? 0f : -ContactOffset))
        {
            return;
        }

        if (!trigger && !a.ReportsContacts && !b.ReportsContacts)
        {
            unreported[worker * CounterStride]++;
            return;
        }

        var contact = new ContactEvent
        {
            BodyA = a.Handle,
            BodyB = b.Handle,
            ColliderA = a.Colliders[colliderA].UserData,
            ColliderB = b.Colliders[colliderB].UserData,
        };
        if (!trigger)
        {
            Describe(ref contact, a, b, ref manifold);
        }

        workers[worker].Add(new Raw(new PairKey(a.Handle.Id, colliderA, b.Handle.Id, colliderB), trigger, contact));
    }

    /// <summary>Merges the workers' records and derives the event states.</summary>
    public void EndStep()
    {
        step++;
        foreach (var buffer in workers)
        {
            foreach (ref readonly var raw in CollectionsMarshal.AsSpan(buffer))
            {
                Merge(raw);
            }
        }

        contacts.Clear();
        triggers.Clear();
        var touching = 0;
        for (var i = 0; i < unreported.Length; i += CounterStride)
        {
            touching += unreported[i];
        }

        // Removing the current entry while enumerating is allowed and keeps the others' positions.
        foreach (var (key, pair) in contactPairs)
        {
            var state = State(key, pair, contactPairs);
            touching += state == PhysicsEventState.Exit ? 0 : 1;
            contacts.Add(pair.Event with
            {
                State = state,
                PointCount = state == PhysicsEventState.Exit ? 0 : pair.Event.PointCount,
            });
        }

        ContactPairCount = touching;

        foreach (var (key, pair) in triggerPairs)
        {
            triggers.Add(pair.Event with { State = State(key, pair, triggerPairs) });
        }
    }

    // The pair's event this step: the first and later steps it was found, or, once it is no longer found, a stay
    // while neither side was tested, since sleeping and static pairs are not, and an exit otherwise.
    PhysicsEventState State<T>(PairKey key, in Tracked<T> pair, Dictionary<PairKey, Tracked<T>> pairs)
    {
        if (pair.Seen == step)
        {
            return pair.Found == step ? PhysicsEventState.Enter : PhysicsEventState.Stay;
        }

        if (world.IsUntested(key.BodyA) && world.IsUntested(key.BodyB))
        {
            return PhysicsEventState.Stay;
        }

        pairs.Remove(key);
        return PhysicsEventState.Exit;
    }

    void Merge(in Raw raw)
    {
        var (key, contact) = Normalize(raw);
        if (raw.Trigger)
        {
            ref var trigger = ref CollectionsMarshal.GetValueRefOrAddDefault(triggerPairs, key, out var known);
            if (!known)
            {
                trigger.Found = step;
            }

            trigger.Seen = step;
            trigger.Event = new TriggerEvent(contact.BodyA, contact.BodyB, contact.ColliderA, contact.ColliderB,
                PhysicsEventState.Enter);
            return;
        }

        // Several manifolds of one pair keep the one with the most points.
        ref var pair = ref CollectionsMarshal.GetValueRefOrAddDefault(contactPairs, key, out var found);
        if (!found)
        {
            pair.Found = step;
        }

        if (pair.Seen != step || pair.Event.PointCount < contact.PointCount)
        {
            pair.Event = contact;
        }

        pair.Seen = step;
    }

    void Describe<TManifold>(ref ContactEvent contact, BepuBody a, BepuBody b, ref TManifold manifold)
        where TManifold : unmanaged, IContactManifold<TManifold>
    {
        var origin = world.CollidablePosition(a);
        var count = Math.Min(manifold.Count, 4);
        for (var i = 0; i < count; i++)
        {
            manifold.GetContact(i, out var offset, out var normal, out var depth, out _);
            contact.Points[i] = new ContactPointData(origin + offset, normal, -depth);
        }

        contact.PointCount = count;
        var point = contact.Points[0].Point;
        var relative = world.PointVelocity(a, point) - world.PointVelocity(b, point);
        contact.RelativeVelocity = relative;
        var normalSpeed = Vector3.Dot(relative, contact.Points[0].Normal);
        var inverseMass = world.InverseMass(a) + world.InverseMass(b);
        contact.Impulse = normalSpeed < 0f && inverseMass > 0f
            ? -normalSpeed / inverseMass * contact.Points[0].Normal
            : Vector3.Zero;
    }

    static (PairKey Key, ContactEvent Contact) Normalize(in Raw raw)
    {
        var key = raw.Key;
        if (key.BodyA < key.BodyB || (key.BodyA == key.BodyB && key.ColliderA <= key.ColliderB))
        {
            return (key, raw.Contact);
        }

        var contact = raw.Contact;
        var swapped = contact with
        {
            BodyA = contact.BodyB,
            BodyB = contact.BodyA,
            ColliderA = contact.ColliderB,
            ColliderB = contact.ColliderA,
            RelativeVelocity = -contact.RelativeVelocity,
            Impulse = -contact.Impulse,
        };
        for (var i = 0; i < swapped.PointCount; i++)
        {
            var point = swapped.Points[i];
            swapped.Points[i] = point with { Normal = -point.Normal };
        }

        return (new PairKey(key.BodyB, key.ColliderB, key.BodyA, key.ColliderA), swapped);
    }

    readonly record struct PairKey(int BodyA, int ColliderA, int BodyB, int ColliderB);

    readonly record struct Raw(PairKey Key, bool Trigger, ContactEvent Contact);

    /// <summary>A pair's latest event and the steps it was first and last found in.</summary>
    struct Tracked<T>
    {
        public T Event;
        public int Found;
        public int Seen;
    }
}
