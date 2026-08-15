# AABB Physics

A simple 2D physics library for Rust based on AABB collision detection.

The library is designed for simple games and custom game engines. It provides static and kinematic bodies with basic collision resolution and gravity.

## Features

* AABB collision detection
* Static bodies
* Kinematic bodies
* Kinematic vs static collisions
* Kinematic vs kinematic collisions
* Gravity
* Ground detection
* Basic collision resolution
* Simple API

## Installation

Add the following to your `Cargo.toml`:

```toml
[dependencies]
aabb_physics
```

For local development:

```toml
[dependencies]
aabb_physics = { path = "../aabb_physics" }
```

## Usage

Create a physics world:

```rust
use aabb_physics::*;
use macroquad::prelude::*;

let mut world = World::new();
```

Add a static body:

```rust
world.add_static(
    Vec2::new(0.0, 500.0),
    Vec2::new(800.0, 50.0),
);
```

Add a kinematic body:

```rust
let player = world.add_kinematic(
    Vec2::new(100.0, 100.0),
    Vec2::new(40.0, 40.0),
);
```

Set its velocity:

```rust
world.kinematic_mut(player).unwrap().velocity.x = 100.0;
```

Update the physics:

```rust
world.step(dt);
```

## KinematicBody

A `KinematicBody` can be controlled by your game logic.

```rust
let body = world.kinematic_mut(player).unwrap();

body.velocity.x = 200.0;
body.velocity.y = -400.0;
```

Gravity is enabled by default:

```rust
body.gravity = Vec2::new(0.0, 980.0);
```

You can disable it with:

```rust
body.gravity = Vec2::ZERO;
```

Check if the body is standing on something:

```rust
if body.grounded {
    println!("On ground");
}
```

## StaticBody

A `StaticBody` does not move and can be used for floors, walls, platforms, and other level geometry.

```rust
world.add_static(
    Vec2::new(0.0, 500.0),
    Vec2::new(800.0, 50.0),
);
```

## AABB

The `Aabb` type contains a position and size:

```rust
let aabb = Aabb::new(
    Vec2::new(100.0, 100.0),
    Vec2::new(32.0, 32.0),
);
```

You can check for intersections:

```rust
if aabb.intersects(&other) {
    println!("Collision");
}
```

## Example

The repository contains a Macroquad example with two kinematic bodies moving towards each other.

Run it with:

```bash
cargo run --example demo
```

## Current Limitations

This is a simple AABB physics system. It currently does not support:

* Rotation
* Dynamic rigid bodies
* Friction
* Restitution
* Slopes
* Circle colliders
* Polygon colliders
* Continuous collision detection

## License

MIT
