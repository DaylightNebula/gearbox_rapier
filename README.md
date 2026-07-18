# Anarchy Rapier

A bridge between the [`anarchy`](../anarchy) ECS, the [`cell`](../cell) app/plugin
framework, [`gearbox`](../gearbox) (transforms/rendering), and
[rapier3d](https://rapier.rs), letting entities with a `Transform` drive and be driven
by a rapier physics simulation.

**Status: work in progress.** APIs, component layout, and behavior are still
changing.

## What it provides

- `PhysicsPlugin` — a `cell::Plugin` that registers the physics engine, queries, and
  the per-tick update system with an `App`.
- `PhysicsObject` — component linking an entity to a rapier rigidbody/collider, with
  queued velocity, impulses, torques, and persistent forces/torques.
- `PhysicsCharacter` — optional companion component adding a kinematic character
  controller (slope handling, autostep, snap-to-ground) on top of a `PhysicsObject`.
- `Collider` — an engine-agnostic collision shape description, converted to rapier
  shapes internally.
- `PhysicsQueries` — a resource for submitting raycasts and shape casts against the
  physics world from any system, resolved on the next physics tick.

Every tick, `PhysicsEngine` creates/refreshes rapier rigidbodies, colliders, and
character controllers for entities with a `PhysicsObject`, applies queued
velocity/impulses/forces/torques, moves kinematic characters, writes the resulting
transform back onto the ECS `Transform`, runs pending `PhysicsQueries`, and steps the
simulation.

## Dependencies

This crate depends on the sibling `anarchy`, `cell`, and `gearbox` crates (via path
dependencies), in addition to `rapier3d`, `bimap`, `dashmap`, `chrono`, `glam`, and
`mutual`.

## Testing

This crate has no tests of its own; it is tested externally, in a consuming project.
