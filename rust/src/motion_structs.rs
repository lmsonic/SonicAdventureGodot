use godot::{
    engine::{CollisionObject3D, PhysicsTestMotionResult3D},
    prelude::*,
};

#[derive(Clone, Debug)]
pub struct MotionCollision {
    pub position: Vector3,
    pub normal: Vector3,
    pub collider_velocity: Vector3,
    pub collider_angular_velocity: Vector3,
    pub depth: real,
    pub local_shape: i32,
    pub collider_id: u64,
    pub collider: Option<Gd<Object>>,
    pub collider_shape: i32,
}

impl MotionCollision {
    pub fn get_angle(&self, up_direction: Vector3) -> real {
        real::acos(self.normal.dot(up_direction))
    }
}

impl Default for MotionCollision {
    fn default() -> Self {
        Self {
            position: Vector3::default(),
            normal: Vector3::default(),
            collider_velocity: Vector3::default(),
            collider_angular_velocity: Vector3::default(),
            depth: Default::default(),
            local_shape: Default::default(),
            collider_id: Default::default(),
            collider: None,
            collider_shape: Default::default(),
        }
    }
}

const MAX_COLLISIONS: usize = 32;
pub struct MotionResult {
    pub travel: Vector3,
    pub remainder: Vector3,
    pub collision_depth: real,
    pub collision_safe_fraction: real,
    pub collision_unsafe_fraction: real,
    pub collisions: Vec<MotionCollision>,
    pub collision_count: usize,
}

impl From<Gd<PhysicsTestMotionResult3D>> for MotionResult {
    fn from(result: Gd<PhysicsTestMotionResult3D>) -> Self {
        let collisions: Vec<MotionCollision> = vec![];
        for i in 0..result.get_collision_count() {
            let collision = MotionCollision {
                position: result.get_collision_point_ex().collision_index(i).done(),
                normal: result.get_collision_normal_ex().collision_index(i).done(),
                collider_velocity: result.get_collider_velocity_ex().collision_index(i).done(),
                collider_angular_velocity: Vector3::ZERO,
                // TODO: wait for exposure of platform_angular_velocity from godot
                // collider_angular_velocity: result
                //     .get_collider_angular_velocity_ex()
                //     .collision_index(i)
                //     .done(),
                depth: result.get_collision_depth_ex().collision_index(i).done(),
                local_shape: result
                    .get_collision_local_shape_ex()
                    .collision_index(i)
                    .done(),
                collider_id: result.get_collider_id_ex().collision_index(i).done(),
                collider: result.get_collider_ex().collision_index(i).done(),
                collider_shape: result.get_collider_shape_ex().collision_index(i).done(),
            };
        }
        MotionResult {
            travel: result.get_travel(),
            remainder: result.get_remainder(),
            collision_depth: result.get_collision_depth(),
            collision_safe_fraction: result.get_collision_safe_fraction(),
            collision_unsafe_fraction: result.get_collision_unsafe_fraction(),
            collisions,
            collision_count: result.get_collision_count() as usize,
        }
    }
}
