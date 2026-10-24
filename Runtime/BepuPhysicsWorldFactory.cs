namespace Turian.Engine.Physics;

/// <summary>Creates BepuPhysics v2 worlds.</summary>
public sealed class BepuPhysicsWorldFactory : IPhysicsWorldFactory
{
    /// <inheritdoc />
    public IPhysicsWorld Create(PhysicsWorldSettings settings)
    {
        ArgumentNullException.ThrowIfNull(settings);
        return new BepuPhysicsWorld(settings);
    }
}
