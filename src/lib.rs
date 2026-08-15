use macroquad::math::Vec2;

const DEFAULT_CELL_SIZE: f32 = 64.0;
const DEFAULT_GRID_WIDTH: usize = 64;
const DEFAULT_GRID_HEIGHT: usize = 32;

#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    pub position: Vec2,
    pub size: Vec2,
}

impl Aabb {
    #[inline(always)]
    pub fn new(position: Vec2, size: Vec2) -> Self {
        Self { position, size }
    }

    #[inline(always)]
    pub fn min(&self) -> Vec2 {
        self.position
    }

    #[inline(always)]
    pub fn max(&self) -> Vec2 {
        self.position + self.size
    }

    #[inline(always)]
    pub fn center(&self) -> Vec2 {
        self.position + self.size * 0.5
    }

    #[inline(always)]
    pub fn intersects(&self, other: &Aabb) -> bool {
        self.position.x < other.position.x + other.size.x
            && self.position.x + self.size.x > other.position.x
            && self.position.y < other.position.y + other.size.y
            && self.position.y + self.size.y > other.position.y
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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
    #[inline]
    pub fn new(position: Vec2, size: Vec2) -> Self {
        Self {
            id: BodyId(0),

            aabb: Aabb::new(position, size),

            velocity: Vec2::ZERO,

            gravity: Vec2::new(0.0, 980.0),

            grounded: false,
        }
    }

    #[inline(always)]
    pub fn position(&self) -> Vec2 {
        self.aabb.position
    }

    #[inline(always)]
    pub fn size(&self) -> Vec2 {
        self.aabb.size
    }

    #[inline(always)]
    pub fn set_position(&mut self, position: Vec2) {
        self.aabb.position = position;
    }
}

struct Grid {
    cell_size: f32,
    inv_cell_size: f32,

    width: usize,
    height: usize,

    cells: Vec<Vec<usize>>,
}

impl Grid {
    fn new(width: usize, height: usize, cell_size: f32) -> Self {
        let mut cells = Vec::with_capacity(width * height);

        for _ in 0..width * height {
            cells.push(Vec::with_capacity(16));
        }

        Self {
            cell_size,
            inv_cell_size: 1.0 / cell_size,

            width,
            height,

            cells,
        }
    }

    #[inline(always)]
    fn clear(&mut self) {
        for cell in &mut self.cells {
            cell.clear();
        }
    }

    #[inline(always)]
    fn cell_x(&self, x: f32) -> usize {
        let x = (x * self.inv_cell_size) as i32;

        x.clamp(0, self.width as i32 - 1) as usize
    }

    #[inline(always)]
    fn cell_y(&self, y: f32) -> usize {
        let y = (y * self.inv_cell_size) as i32;

        y.clamp(0, self.height as i32 - 1) as usize
    }

    fn insert(&mut self, index: usize, aabb: &Aabb) {
        let min_x = self.cell_x(aabb.position.x);

        let min_y = self.cell_y(aabb.position.y);

        let max_x = self.cell_x(aabb.position.x + aabb.size.x);

        let max_y = self.cell_y(aabb.position.y + aabb.size.y);

        for y in min_y..=max_y {
            let row = y * self.width;

            for x in min_x..=max_x {
                self.cells[row + x].push(index);
            }
        }
    }
}

pub struct World {
    statics: Vec<StaticBody>,
    kinematics: Vec<KinematicBody>,

    next_id: usize,

    grid: Grid,
}

impl World {
    pub fn new() -> Self {
        Self::with_grid(DEFAULT_GRID_WIDTH, DEFAULT_GRID_HEIGHT, DEFAULT_CELL_SIZE)
    }

    pub fn with_grid(width: usize, height: usize, cell_size: f32) -> Self {
        Self {
            statics: Vec::new(),
            kinematics: Vec::new(),

            next_id: 0,

            grid: Grid::new(width, height, cell_size),
        }
    }

    #[inline]
    pub fn add_static(&mut self, position: Vec2, size: Vec2) -> BodyId {
        let id = BodyId(self.next_id);

        self.next_id += 1;

        self.statics.push(StaticBody {
            id,

            aabb: Aabb::new(position, size),
        });

        id
    }

    #[inline]
    pub fn add_kinematic(&mut self, position: Vec2, size: Vec2) -> BodyId {
        let id = BodyId(self.next_id);

        self.next_id += 1;

        let mut body = KinematicBody::new(position, size);

        body.id = id;

        self.kinematics.push(body);

        id
    }

    #[inline]
    pub fn kinematic(&self, id: BodyId) -> Option<&KinematicBody> {
        self.kinematics.iter().find(|body| body.id == id)
    }

    #[inline]
    pub fn kinematic_mut(&mut self, id: BodyId) -> Option<&mut KinematicBody> {
        self.kinematics.iter_mut().find(|body| body.id == id)
    }

    #[inline(always)]
    pub fn statics(&self) -> &[StaticBody] {
        &self.statics
    }

    #[inline(always)]
    pub fn kinematics(&self) -> &[KinematicBody] {
        &self.kinematics
    }

    pub fn step(&mut self, dt: f32) {
        // Prevent huge physics jumps when the game
        // is paused or the window gets frozen.
        let dt = dt.min(0.05);

        // Sub-stepping prevents fast bodies from
        // tunneling through thin colliders.
        const MAX_STEP: f32 = 1.0 / 120.0;

        let steps = (dt / MAX_STEP).ceil() as usize;

        let steps = steps.max(1);

        let sub_dt = dt / steps as f32;

        for _ in 0..steps {
            self.physics_step(sub_dt);
        }
    }

    fn physics_step(&mut self, dt: f32) {
        // =========================================================
        // Integration
        // =========================================================

        for body in &mut self.kinematics {
            body.grounded = false;

            body.velocity += body.gravity * dt;

            body.aabb.position.x += body.velocity.x * dt;

            body.aabb.position.y += body.velocity.y * dt;
        }

        // =========================================================
        // Kinematic vs Static
        // =========================================================

        for body in &mut self.kinematics {
            for collider in &self.statics {
                if body.aabb.intersects(&collider.aabb) {
                    resolve_static(body, collider);
                }
            }
        }

        // =========================================================
        // Build spatial grid
        // =========================================================

        self.grid.clear();

        for i in 0..self.kinematics.len() {
            let aabb = self.kinematics[i].aabb;

            self.grid.insert(i, &aabb);
        }

        // =========================================================
        // Kinematic vs Kinematic
        // =========================================================

        let width = self.grid.width;

        let height = self.grid.height;

        for cell_y in 0..height {
            for cell_x in 0..width {
                let cell_index = cell_y * width + cell_x;

                let cell = &self.grid.cells[cell_index];

                if cell.len() < 2 {
                    continue;
                }

                for a in 0..cell.len() {
                    let i = cell[a];

                    for b in (a + 1)..cell.len() {
                        let j = cell[b];

                        // Pair is handled only once.
                        if i >= j {
                            continue;
                        }

                        let (left, right) = self.kinematics.split_at_mut(j);

                        let body_a = &mut left[i];

                        let body_b = &mut right[0];

                        if !body_a.aabb.intersects(&body_b.aabb) {
                            continue;
                        }

                        resolve_dynamic(body_a, body_b);
                    }
                }
            }
        }
    }
}

#[inline(always)]
fn resolve_static(body: &mut KinematicBody, collider: &StaticBody) {
    let ax = body.aabb.position.x;

    let ay = body.aabb.position.y;

    let aw = body.aabb.size.x;

    let ah = body.aabb.size.y;

    let bx = collider.aabb.position.x;

    let by = collider.aabb.position.y;

    let bw = collider.aabb.size.x;

    let bh = collider.aabb.size.y;

    let overlap_x = (ax + aw).min(bx + bw) - ax.max(bx);

    let overlap_y = (ay + ah).min(by + bh) - ay.max(by);

    if overlap_x <= 0.0 || overlap_y <= 0.0 {
        return;
    }

    // Resolve along the smallest penetration axis.
    if overlap_x < overlap_y {
        if ax < bx {
            body.aabb.position.x -= overlap_x;
        } else {
            body.aabb.position.x += overlap_x;
        }

        body.velocity.x = 0.0;
    } else {
        if ay < by {
            body.aabb.position.y -= overlap_y;

            if body.velocity.y >= 0.0 {
                body.velocity.y = 0.0;
                body.grounded = true;
            }
        } else {
            body.aabb.position.y += overlap_y;

            if body.velocity.y < 0.0 {
                body.velocity.y = 0.0;
            }
        }
    }
}

#[inline(always)]
fn resolve_dynamic(a: &mut KinematicBody, b: &mut KinematicBody) {
    let ax = a.aabb.position.x;

    let ay = a.aabb.position.y;

    let aw = a.aabb.size.x;

    let ah = a.aabb.size.y;

    let bx = b.aabb.position.x;

    let by = b.aabb.position.y;

    let bw = b.aabb.size.x;

    let bh = b.aabb.size.y;

    let overlap_x = (ax + aw).min(bx + bw) - ax.max(bx);

    let overlap_y = (ay + ah).min(by + bh) - ay.max(by);

    if overlap_x <= 0.0 || overlap_y <= 0.0 {
        return;
    }

    // Horizontal collision
    if overlap_x < overlap_y {
        let correction = overlap_x * 0.5;

        if ax < bx {
            a.aabb.position.x -= correction;

            b.aabb.position.x += correction;
        } else {
            a.aabb.position.x += correction;

            b.aabb.position.x -= correction;
        }

        a.velocity.x = 0.0;
        b.velocity.x = 0.0;

        return;
    }

    // Vertical collision
    let correction = overlap_y * 0.5;

    if ay < by {
        a.aabb.position.y -= correction;

        b.aabb.position.y += correction;

        if a.velocity.y > 0.0 {
            a.velocity.y = 0.0;
            a.grounded = true;
        }

        if b.velocity.y < 0.0 {
            b.velocity.y = 0.0;
        }
    } else {
        a.aabb.position.y += correction;

        b.aabb.position.y -= correction;

        if a.velocity.y < 0.0 {
            a.velocity.y = 0.0;
        }

        if b.velocity.y > 0.0 {
            b.velocity.y = 0.0;
            b.grounded = true;
        }
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}
