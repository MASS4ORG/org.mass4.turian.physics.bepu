namespace Turian.Engine.Physics;

/// <summary>A shape added to the Bepu shape set.</summary>
/// <param name="Index">The Bepu shape index.</param>
/// <param name="Description">The description it was created from.</param>
/// <param name="Center">Where a recentered hull's origin lies in the description's space.</param>
/// <param name="Hull">The hull, for hull shapes.</param>
readonly record struct BepuShape(TypedIndex Index, ShapeDesc Description, Vector3 Center, ConvexHull Hull);

/// <summary>Converts shape descriptions into Bepu shapes.</summary>
static class BepuShapes
{
    const float MinimumSize = 1e-4f;

    /// <summary>Adds a description to the shape set.</summary>
    public static BepuShape Add(Shapes shapes, BufferPool pool, in ShapeDesc description)
    {
        switch (description.Type)
        {
            case ShapeType.ConvexHull:
                var points = description.Points ?? throw new ArgumentException("A convex hull needs points.");
                if (points.Length < 4)
                {
                    throw new ArgumentException("A convex hull needs at least four points.");
                }

                var hull = new ConvexHull(points, pool, out var center);
                return new BepuShape(shapes.Add(hull), description, center, hull);
            case ShapeType.TriangleMesh:
                return new BepuShape(shapes.Add(CreateMesh(pool, description)), description, Vector3.Zero, default);
            default:
                return new BepuShape(AddConvexPrimitive(shapes, description), description, Vector3.Zero, default);
        }
    }

    /// <summary>Returns a shape's resources to the pool.</summary>
    public static void Remove(Shapes shapes, BufferPool pool, in BepuShape shape) =>
        shapes.RemoveAndDispose(shape.Index, pool);

    /// <summary>The inertia of a convex shape with the given mass.</summary>
    public static BodyInertia Inertia(in BepuShape shape, float mass) => shape.Description.Type switch
    {
        ShapeType.Box => ToBox(shape.Description).ComputeInertia(mass),
        ShapeType.Sphere => ToSphere(shape.Description).ComputeInertia(mass),
        ShapeType.Capsule => ToCapsule(shape.Description).ComputeInertia(mass),
        ShapeType.Cylinder => ToCylinder(shape.Description).ComputeInertia(mass),
        ShapeType.ConvexHull => shape.Hull.ComputeInertia(mass),
        _ => throw new NotSupportedException("Triangle meshes cannot be dynamic."),
    };

    static TypedIndex AddConvexPrimitive(Shapes shapes, in ShapeDesc description) => description.Type switch
    {
        ShapeType.Box => shapes.Add(ToBox(description)),
        ShapeType.Sphere => shapes.Add(ToSphere(description)),
        ShapeType.Capsule => shapes.Add(ToCapsule(description)),
        ShapeType.Cylinder => shapes.Add(ToCylinder(description)),
        _ => throw new ArgumentOutOfRangeException(nameof(description), description.Type, "Unknown shape type."),
    };

    internal static BepuBox ToBox(in ShapeDesc d) =>
        new(Positive(d.Size.X), Positive(d.Size.Y), Positive(d.Size.Z));

    internal static BepuSphere ToSphere(in ShapeDesc d) => new(Positive(d.Radius));

    internal static BepuCapsule ToCapsule(in ShapeDesc d) => new(Positive(d.Radius), MathF.Max(d.Length, 0f));

    internal static BepuCylinder ToCylinder(in ShapeDesc d) => new(Positive(d.Radius), Positive(d.Length));

    static float Positive(float value) => MathF.Max(MathF.Abs(value), MinimumSize);

    // Bepu triangles collide on the side their (B - A) x (C - A) winding faces away from, so glTF's
    // counter-clockwise front faces are passed as A, C, B.
    static BepuMesh CreateMesh(BufferPool pool, in ShapeDesc description)
    {
        var vertices = description.Points ?? throw new ArgumentException("A triangle mesh needs vertices.");
        var indices = description.Indices ?? throw new ArgumentException("A triangle mesh needs indices.");
        var count = indices.Length / 3;
        if (count == 0)
        {
            throw new ArgumentException("A triangle mesh needs at least one triangle.");
        }

        pool.Take<Triangle>(count, out var triangles);
        for (var i = 0; i < count; i++)
        {
            triangles[i] = new Triangle(vertices[indices[i * 3]], vertices[indices[i * 3 + 2]],
                vertices[indices[i * 3 + 1]]);
        }

        return new BepuMesh(triangles, Vector3.One, pool);
    }
}
