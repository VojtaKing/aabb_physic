use macroquad::math::Vec2;

#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    pub position: Vec2,
    pub size: Vec2,
}

impl Aabb {
    pub fn new(position: Vec2, size: Vec2) -> Self {
        Self { position, size }
    }

    pub fn min(&self) -> Vec2 {
        self.position
    }

    pub fn max(&self) -> Vec2 {
        self.position + self.size
    }

    pub fn center(&self) -> Vec2 {
        self.position + self.size * 0.5
    }

    pub fn intersects(&self, other: &Aabb) -> bool {
        self.min().x < other.max().x
            && self.max().x > other.min().x
            && self.min().y < other.max().y
            && self.max().y > other.min().y
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BodyId(pub usize);

#[derive(Debug, Clone, Copy)]
pub struct StaticBody {
    pub id: BodyId,
    pub aabb: Aabb,
}

#[derive(Debug, Clone, Copy)]
pub struct KinematicBody {
    pub id: BodyId,
    pub aabb: Aabb,
    pub velocity: Vec2,
    pub gravity: Vec2,
    pub grounded: bool,
}

impl KinematicBody {
    pub fn new(position: Vec2, size: Vec2) -> Self {
        Self {
            id: BodyId(0),
            aabb: Aabb::new(position, size),
            velocity: Vec2::ZERO,
            gravity: Vec2::new(0.0, 980.0),
            grounded: false,
        }
    }

    pub fn position(&self) -> Vec2 {
        self.aabb.position
    }

    pub fn size(&self) -> Vec2 {
        self.aabb.size
    }

    pub fn set_position(&mut self, position: Vec2) {
        self.aabb.position = position;
    }
}

pub struct World {
    statics: Vec<StaticBody>,
    kinematics: Vec<KinematicBody>,
    next_id: usize,
}

impl World {
    pub fn new() -> Self {
        Self {
            statics: Vec::new(),
            kinematics: Vec::new(),
            next_id: 0,
        }
    }

    pub fn add_static(&mut self, position: Vec2, size: Vec2) -> BodyId {
        let id = BodyId(self.next_id);
        self.next_id += 1;

        self.statics.push(StaticBody {
            id,
            aabb: Aabb::new(position, size),
        });

        id
    }

    pub fn add_kinematic(&mut self, position: Vec2, size: Vec2) -> BodyId {
        let id = BodyId(self.next_id);
        self.next_id += 1;

        let mut body = KinematicBody::new(position, size);
        body.id = id;

        self.kinematics.push(body);

        id
    }

    pub fn kinematic(&self, id: BodyId) -> Option<&KinematicBody> {
        self.kinematics.iter().find(|body| body.id == id)
    }

    pub fn kinematic_mut(&mut self, id: BodyId) -> Option<&mut KinematicBody> {
        self.kinematics.iter_mut().find(|body| body.id == id)
    }

    pub fn statics(&self) -> &[StaticBody] {
        &self.statics
    }

    pub fn kinematics(&self) -> &[KinematicBody] {
        &self.kinematics
    }

    pub fn step(&mut self, dt: f32) {
        // Gravity + movement
        for body in &mut self.kinematics {
            body.grounded = false;
            body.velocity += body.gravity * dt;

            body.aabb.position.x += body.velocity.x * dt;
            body.aabb.position.y += body.velocity.y * dt;
        }

        // Kinematic vs Static
        for body in &mut self.kinematics {
            for collider in &self.statics {
                if !body.aabb.intersects(&collider.aabb) {
                    continue;
                }

                let overlap_x = f32::min(body.aabb.max().x, collider.aabb.max().x)
                    - f32::max(body.aabb.min().x, collider.aabb.min().x);

                let overlap_y = f32::min(body.aabb.max().y, collider.aabb.max().y)
                    - f32::max(body.aabb.min().y, collider.aabb.min().y);

                if overlap_x < overlap_y {
                    if body.aabb.center().x < collider.aabb.center().x {
                        body.aabb.position.x -= overlap_x;
                    } else {
                        body.aabb.position.x += overlap_x;
                    }

                    body.velocity.x = 0.0;
                } else {
                    if body.aabb.center().y < collider.aabb.center().y {
                        body.aabb.position.y -= overlap_y;

                        if body.velocity.y > 0.0 {
                            body.grounded = true;
                        }
                    } else {
                        body.aabb.position.y += overlap_y;
                    }

                    body.velocity.y = 0.0;
                }
            }
        }

        // Kinematic vs Kinematic
        let count = self.kinematics.len();

        for i in 0..count {
            for j in (i + 1)..count {
                let (left, right) = self.kinematics.split_at_mut(j);

                let a = &mut left[i];
                let b = &mut right[0];

                if !a.aabb.intersects(&b.aabb) {
                    continue;
                }

                let overlap_x = f32::min(a.aabb.max().x, b.aabb.max().x)
                    - f32::max(a.aabb.min().x, b.aabb.min().x);

                let overlap_y = f32::min(a.aabb.max().y, b.aabb.max().y)
                    - f32::max(a.aabb.min().y, b.aabb.min().y);

                // Horizontal collision
                if overlap_x < overlap_y {
                    let correction = overlap_x * 0.5;

                    if a.aabb.center().x < b.aabb.center().x {
                        a.aabb.position.x -= correction;
                        b.aabb.position.x += correction;
                    } else {
                        a.aabb.position.x += correction;
                        b.aabb.position.x -= correction;
                    }

                    a.velocity.x = 0.0;
                    b.velocity.x = 0.0;
                }
                // Vertical collision
                else {
                    let correction = overlap_y * 0.5;

                    if a.aabb.center().y < b.aabb.center().y {
                        a.aabb.position.y -= correction;
                        b.aabb.position.y += correction;

                        if a.velocity.y > 0.0 {
                            a.grounded = true;
                        }

                        if b.velocity.y < 0.0 {
                            b.grounded = true;
                        }
                    } else {
                        a.aabb.position.y += correction;
                        b.aabb.position.y -= correction;

                        if b.velocity.y > 0.0 {
                            b.grounded = true;
                        }

                        if a.velocity.y < 0.0 {
                            a.grounded = true;
                        }
                    }

                    a.velocity.y = 0.0;
                    b.velocity.y = 0.0;
                }
            }
        }
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}
