# 3D Physics: Bepu backend

Makes [`org.mass4.turian.physics`](https://github.com/MASS4ORG/org.mass4.turian.physics) move, using [BepuPhysics v2](https://github.com/bepu/bepuphysics2).

Install it next to the physics brick. There is nothing to set up.

## Install

```sh
turian-cli brick add org.mass4.turian.physics
turian-cli brick add org.mass4.turian.physics.bepu
```

Or use the Bricks panel.

## Speed tips

In **Settings / Physics**:

- `Multithreaded` uses every CPU core.
- Fewer `SolverIterations` and `Substeps` run faster; more make stacks and joints stiffer.
- Resting bodies fall asleep and cost almost nothing.

## Limitations

- Bounces are close to Unity's, not identical.
- Mesh colliders only work on static or kinematic bodies.
- Results repeat only on the same machine.
- A body made of several colliders uses one material for all of them.

## Development

Keep a Turian checkout at `../../Turian` and the physics brick at `../org.mass4.turian.physics`, then run from Turian:

```sh
dotnet test --project ../Bricks/org.mass4.turian.physics.bepu/Brick~/Tests/Turian.Physics.Bepu.Tests/Turian.Physics.Bepu.Tests.csproj -p:NoLocalPackages=true
```

Release the physics brick before this one.
