use aabb_physics::*;
use macroquad::prelude::*;

#[macroquad::main("AABB Physics - Characters")]
async fn main() {
    let mut world = World::new();

    // Ground
    world.add_static(Vec2::new(0.0, 500.0), Vec2::new(1000.0, 50.0));

    // Left character
    let left = world.add_kinematic(Vec2::new(150.0, 460.0), Vec2::new(40.0, 40.0));

    // Right character
    let right = world.add_kinematic(Vec2::new(810.0, 460.0), Vec2::new(40.0, 40.0));

    loop {
        let dt = get_frame_time();

        // Move characters towards each other
        {
            let body = world.kinematic_mut(left).unwrap();
            body.velocity.x = 200.0;
        }

        {
            let body = world.kinematic_mut(right).unwrap();
            body.velocity.x = -100.0;
        }

        world.step(dt);

        clear_background(BLACK);

        // Draw static bodies
        for body in world.statics() {
            draw_rectangle(
                body.aabb.position.x,
                body.aabb.position.y,
                body.aabb.size.x,
                body.aabb.size.y,
                GRAY,
            );
        }

        // Draw characters
        for (i, body) in world.kinematics().iter().enumerate() {
            let color = if i == 0 { RED } else { BLUE };

            draw_rectangle(
                body.aabb.position.x,
                body.aabb.position.y,
                body.aabb.size.x,
                body.aabb.size.y,
                color,
            );
        }

        draw_text(
            "Two KinematicBodies moving towards each other",
            20.0,
            40.0,
            24.0,
            WHITE,
        );

        next_frame().await;
    }
}
