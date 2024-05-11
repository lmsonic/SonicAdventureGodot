use godot::{
    engine::{
        notify::Node3DNotification, physics_server_3d::BodyAxis, CharacterBody3D,
        CollisionObject3D, IPhysicsBody3D, KinematicCollision3D, PhysicsBody3D, PhysicsServer3D,
        PhysicsTestMotionParameters3D, PhysicsTestMotionResult3D,
    },
    prelude::*,
};

use crate::motion_structs::{MotionCollision, MotionResult};

#[derive(Debug, Default, PartialEq, Eq, GodotConvert, Var, Export)]
#[godot(via = i64)]
enum MotionMode {
    #[default]
    Grounded,
    Floating,
}
#[derive(Debug, Default, PartialEq, Eq, GodotConvert, Var, Export)]
#[godot(via = i64)]
enum PlatformOnLeave {
    #[default]
    AddVelocity,
    AddUpwardVelocity,
    DoNothing,
}

#[derive(Debug, Default, PartialEq, Eq, Clone, Copy)]
struct CollisionState {
    floor: bool,
    wall: bool,
    ceiling: bool,
}

impl CollisionState {
    const fn new(floor: bool, wall: bool, ceiling: bool) -> Self {
        Self {
            floor,
            wall,
            ceiling,
        }
    }
}

const FLOOR_ANGLE_THRESHOLD: f32 = 0.01;

#[derive(GodotClass)]
#[class(base=PhysicsBody3D,init)]
#[allow(clippy::struct_excessive_bools)]
struct CustomCharacterBody3D {
    #[export]
    margin: real,
    #[export]
    motion_mode: MotionMode,
    #[export]
    platform_on_leave: PlatformOnLeave,

    #[export]
    floor_constant_speed: bool,
    #[export]
    #[init(default = true)]
    floor_stop_on_slope: bool,
    #[export]
    #[init(default = true)]
    floor_block_on_wall: bool,
    #[export]
    #[init(default = true)]
    slide_on_ceiling: bool,
    #[var(set=set_max_slides)]
    #[init(default = 6)]
    max_slides: i32,
    #[export]
    platform_layer: u32,

    #[export]
    #[init(default=u32::MAX)]
    platform_floor_layers: u32,
    #[export]
    platform_wall_layers: u32,
    #[export]
    #[init(default = 0.1)]
    floor_snap_length: real,
    #[export]
    #[init(default=f32::to_radians(45.0))]
    floor_max_angle: real,
    #[export]
    wall_min_slide_angle: real,
    #[export]
    #[var(set=set_up_direction)]
    #[init(default=Vector3::UP)]
    up_direction: Vector3,

    #[var]
    velocity: Vector3,

    floor_normal: Vector3,
    wall_normal: Vector3,
    ceiling_normal: Vector3,
    last_motion: Vector3,
    platform_velocity: Vector3,
    platform_angular_velocity: Vector3,
    platform_ceiling_velocity: Vector3,
    previous_position: Vector3,
    real_velocity: Vector3,
    motion_results: Vec<MotionResult>,
    slide_colliders: Vec<Gd<KinematicCollision3D>>,
    collision_state: CollisionState,
    #[init(default = Rid::Invalid)]
    platform_rid: Rid,
    platform_object_id: u64,

    base: Base<PhysicsBody3D>,
}
#[godot_api]
impl IPhysicsBody3D for CustomCharacterBody3D {
    fn on_notification(&mut self, what: Node3DNotification) {
        if let Node3DNotification::EnterTree = what {
            self.collision_state = CollisionState::default();
            self.platform_rid = Rid::Invalid;
            self.platform_object_id = 0;
            self.motion_results.clear();
            self.platform_velocity = Vector3::ZERO;
            self.platform_angular_velocity = Vector3::ZERO;
        }
    }
}
const CMP_EPSILON: real = 0.00001;
fn move_and_collide(
    body: &mut Gd<PhysicsBody3D>,
    parameters: &Gd<PhysicsTestMotionParameters3D>,
    test_only: bool,
    mut cancel_sliding: bool,
) -> Option<MotionResult> {
    let result = PhysicsTestMotionResult3D::new_gd();
    let colliding = PhysicsServer3D::singleton()
        .body_test_motion_ex(body.get_rid(), parameters.clone())
        .result(result.clone())
        .done();
    let mut result: MotionResult = result.into();

    // Restore direction of motion to be along original motion,
    // in order to avoid sliding due to recovery,
    // but only if collision depth is low enough to avoid tunneling.
    if cancel_sliding {
        let motion_length: real = parameters.get_motion().length();
        let mut precision: real = 0.001;

        if colliding {
            // Can't just use margin as a threshold because collision depth is calculated on unsafe motion,
            // so even in normal resting cases the depth can be a bit more than the margin.
            precision +=
                motion_length * (result.collision_unsafe_fraction - result.collision_safe_fraction);
            let depth = result.collision_depth;
            if depth > parameters.get_margin() + precision {
                cancel_sliding = false;
            }
        }

        if cancel_sliding {
            // When motion is null, recovery is the resulting motion.
            let mut motion_normal = Vector3::ZERO;
            if motion_length > CMP_EPSILON {
                motion_normal = parameters.get_motion() / motion_length;
            }

            // Check depth of recovery.
            let projected_length: real = result.travel.dot(motion_normal);
            let recovery = result.travel - motion_normal * projected_length;
            let recovery_length = recovery.length();
            // Fixes cases where canceling slide causes the motion to go too deep into the ground,
            // because we're only taking rest information into account and not general recovery.
            if recovery_length < parameters.get_margin() + precision {
                // Apply adjustment to motion.
                result.travel = motion_normal * projected_length;
                result.remainder = parameters.get_motion() - result.travel;
            }
        }
    }
    if body.get_axis_lock(BodyAxis::LINEAR_X) {
        result.travel.x = 0.0;
    }
    if body.get_axis_lock(BodyAxis::LINEAR_Y) {
        result.travel.y = 0.0;
    }
    if body.get_axis_lock(BodyAxis::LINEAR_Z) {
        result.travel.z = 0.0;
    }

    if test_only {
        let mut gt = parameters.get_from();
        gt.origin += result.travel;
        body.set_global_transform(gt);
    }
    if colliding {
        Some(result)
    } else {
        None
    }
}
#[godot_api]
impl CustomCharacterBody3D {
    #[func]
    fn move_and_slide(&self) -> bool {
        unimplemented!()
    }
    #[func]
    fn apply_floor_snap(&mut self) {
        if self.collision_state.floor {
            return;
        }

        // Snap by at least collision margin to keep floor state consistent.
        let length: real = real::max(self.floor_snap_length, self.margin);
        let mut params = PhysicsTestMotionParameters3D::new_gd();
        params.set_margin(self.margin);
        params.set_from(self.base().get_global_transform());
        params.set_motion(-self.up_direction * length);
        params.set_max_collisions(4);
        params.set_collide_separation_ray_enabled(true);
        params.set_recovery_as_collision_enabled(true);

        let result = move_and_collide(&mut self.base_mut().clone().upcast(), &params, true, false);

        if let Some(result) = result {
            // Apply direction for floor only.
            let result_state =
                self.set_collision_direction_ex(&result, CollisionState::new(true, false, false));

            if result_state.floor {
                let mut travel = result.travel;
                if self.floor_stop_on_slope {
                    // move and collide may stray the object a bit because of pre un-stucking,
                    // so only ensure that motion happens on floor direction in this case.
                    // Setters not exposed
                    if travel.length() > params.get_margin() {
                        travel = self.up_direction * self.up_direction.dot(result.travel);
                    } else {
                        travel = Vector3::default();
                    }
                }
                let mut from = params.get_from();
                from.origin += travel;
                params.set_from(from);
                self.base_mut().set_global_transform(from);
            }
        }
    }

    #[func]
    fn is_on_floor(&self) -> bool {
        self.collision_state.floor
    }
    #[func]
    fn is_on_floor_only(&self) -> bool {
        self.collision_state.floor && !self.collision_state.wall && !self.collision_state.ceiling
    }
    #[func]
    fn is_on_wall(&self) -> bool {
        self.collision_state.wall
    }
    #[func]
    fn is_on_wall_only(&self) -> bool {
        self.collision_state.wall && !self.collision_state.floor && !self.collision_state.ceiling
    }
    #[func]
    fn is_on_ceiling(&self) -> bool {
        self.collision_state.ceiling
    }
    #[func]
    fn is_on_ceiling_only(&self) -> bool {
        self.collision_state.ceiling && !self.collision_state.floor && !self.collision_state.wall
    }
    #[func]
    fn get_last_motion(&self) -> Vector3 {
        self.last_motion
    }
    #[func]
    fn get_position_delta(&self) -> Vector3 {
        self.base().get_global_transform().origin - self.previous_position
    }
    #[func]
    fn get_floor_normal(&self) -> Vector3 {
        self.floor_normal
    }
    #[func]
    fn get_wall_normal(&self) -> Vector3 {
        self.wall_normal
    }
    #[func]
    fn get_real_velocity(&self) -> Vector3 {
        self.real_velocity
    }
    #[func]
    fn get_floor_angle(&self) -> real {
        const UP_DIRECTION: Vector3 = Vector3::UP;
        self.get_floor_angle_ex(UP_DIRECTION)
    }
    #[func]
    fn get_floor_angle_ex(&self, up_direction: Vector3) -> real {
        real::acos(self.floor_normal.dot(up_direction))
    }
    #[func]
    fn get_platform_velocity(&self) -> Vector3 {
        self.platform_velocity
    }
    #[func]
    fn get_platform_angular_velocity(&self) -> Vector3 {
        self.platform_angular_velocity
    }
    #[func]
    fn get_slide_collision_count(&self) -> u32 {
        self.slide_colliders.len() as u32
    }
    #[func]
    fn get_slide_collision(&self, index: u32) -> Option<Gd<KinematicCollision3D>> {
        self.slide_colliders.get(index as usize).cloned()
    }
    #[func]
    fn get_last_slide_collision(&mut self) -> Option<Gd<KinematicCollision3D>> {
        if self.motion_results.is_empty() {
            return None;
        }
        self.get_slide_collision((self.motion_results.len() - 1) as u32)
    }
    #[func]
    fn set_up_direction(&mut self, up_direction: Vector3) {
        if up_direction == Vector3::ZERO {
            godot_error!("up_direction can't be equal to Vector3.ZERO, consider using Floating motion mode instead.");
            return;
        }
        self.up_direction = up_direction;
    }
    #[func]
    fn set_max_slides(&mut self, max_slides: i32) {
        if max_slides < 1 {
            godot_error!("max slides must be >= 1");
            return;
        }
        self.max_slides = max_slides;
    }
}

impl CustomCharacterBody3D {
    fn move_and_slide_floating(&mut self, delta: real) {
        let mut motion = self.get_velocity() * delta;

        self.platform_rid = Rid::Invalid;
        self.platform_object_id = 0;
        self.floor_normal = Vector3::default();
        self.platform_velocity = Vector3::default();
        self.platform_angular_velocity = Vector3::default();

        let mut first_slide = true;
        for iteration in 0..self.max_slides {
            let mut params = PhysicsTestMotionParameters3D::new_gd();
            params.set_from(self.base().get_global_transform());
            params.set_motion(motion);
            params.set_margin(self.margin);
            params.set_recovery_as_collision_enabled(true); // Also report collisions generated only from recovery.

            let result =
                move_and_collide(&mut self.base_mut().clone().upcast(), &params, false, false);

            self.last_motion = Vector3::ZERO;

            if let Some(result) = result {
                self.last_motion = result.travel;
                self.motion_results.push(result.clone());
                self.set_collision_direction(&result);
                if result.remainder.is_zero_approx() {
                    motion = Vector3::ZERO;
                    break;
                }
                if self.wall_min_slide_angle != 0.0
                    && real::acos(self.wall_normal.dot(-self.velocity.normalized()))
                        < self.wall_min_slide_angle + FLOOR_ANGLE_THRESHOLD
                {
                    motion = Vector3::ZERO;
                    if result.travel.length() < self.margin + CMP_EPSILON {
                        let mut gt = self.base().get_global_transform();
                        gt.origin -= result.travel;
                        self.base_mut().set_global_transform(gt);
                    }
                } else if first_slide {
                    let motion_slide_norm = result.remainder.slide(self.wall_normal).normalized();
                    motion = motion_slide_norm * (motion.length() - result.travel.length());
                } else {
                    motion = result.remainder.slide(self.wall_normal);
                }

                if motion.dot(self.velocity) <= 0.0 {
                    motion = Vector3::ZERO;
                }
            } else {
                break;
            }
            if motion.is_zero_approx() {
                break;
            }

            first_slide = false;
        }
    }
    fn move_and_slide_grounded(&self, delta: real, was_on_floor: bool) {
        unimplemented!()
    }

    fn on_floor_if_snapped(&mut self, was_on_floor: bool, vel_dir_facing_up: bool) -> bool {
        if self.up_direction == Vector3::ZERO
            || self.collision_state.floor
            || was_on_floor
            || vel_dir_facing_up
        {
            return false;
        }
        let length: real = real::max(self.floor_snap_length, self.margin);

        let mut params = PhysicsTestMotionParameters3D::new_gd();
        params.set_margin(self.margin);
        params.set_from(self.base().get_global_transform());
        params.set_motion(-self.up_direction * length);
        params.set_max_collisions(4);
        params.set_collide_separation_ray_enabled(true);
        params.set_recovery_as_collision_enabled(true);

        let result = move_and_collide(&mut self.base_mut().clone().upcast(), &params, true, false);

        if let Some(result) = result {
            // Don't apply direction for any type.
            let result_state = self.set_collision_direction_ex(&result, CollisionState::default());

            return result_state.floor;
        }

        false
    }
    fn set_collision_direction(&mut self, result: &MotionResult) {
        const APPLY_STATE: CollisionState = CollisionState::new(true, true, true);
        self.set_collision_direction_ex(result, APPLY_STATE);
    }
    fn set_collision_direction_ex(
        &mut self,
        result: &MotionResult,
        apply_state: CollisionState,
    ) -> CollisionState {
        let mut state = CollisionState::default();

        let mut wall_depth: real = -1.0;
        let mut floor_depth: real = -1.0;

        let was_on_wall = self.collision_state.wall;
        let prev_wall_normal = self.wall_normal;
        let mut wall_collision_count = 0;
        let mut combined_wall_normal = Vector3::default();
        let mut tmp_wall_col = Vector3::default(); // Avoid duplicate on average calculation.

        for i in result.collision_count - 1..=0 {
            let collision = &result.collisions[i];
            let normal = collision.normal;
            let depth = collision.depth;
            let collider_velocity = collision.collider_velocity;
            if self.motion_mode == MotionMode::Grounded {
                // Check if any collision is floor.
                let floor_angle = real::acos(normal.dot(self.up_direction));
                if floor_angle <= self.floor_max_angle + FLOOR_ANGLE_THRESHOLD {
                    state.floor = true;
                    if apply_state.floor && depth > floor_depth {
                        self.collision_state.floor = true;
                        self.floor_normal = normal;
                        floor_depth = depth;
                        self.set_platform_data(collision);
                    }
                }
                // Check if any collision is ceiling.
                let ceiling_angle = real::acos(normal.dot(-self.up_direction));
                if ceiling_angle <= self.floor_max_angle + FLOOR_ANGLE_THRESHOLD {
                    state.ceiling = true;
                    if apply_state.ceiling {
                        self.platform_ceiling_velocity = collider_velocity;
                        self.ceiling_normal = normal;
                        self.collision_state.ceiling = true;
                    }
                    continue;
                }
            }
            // Collision is wall by default.
            if apply_state.wall && depth > wall_depth {
                self.collision_state.wall = true;
                wall_depth = depth;
                self.wall_normal = normal;

                // Don't apply wall velocity when the collider is a CharacterBody3D.
                if let Some(ref collider) = collision.collider {
                    if collider.clone().try_cast::<CharacterBody3D>().is_err() {
                        self.set_platform_data(collision);
                    }
                }
            }

            // Collect normal for calculating average.
            if normal != (tmp_wall_col) {
                tmp_wall_col = normal;
                combined_wall_normal += normal;
                wall_collision_count += 1;
            }
        }
        // Check if wall normals cancel out to floor support.
        if state.wall
            && wall_collision_count > 1
            && !state.floor
            // && !state.floor // Godot source code was checking the same thing twice here lol
            && self.motion_mode == MotionMode::Grounded
        {
            combined_wall_normal = combined_wall_normal.normalized();
            let floor_angle = real::acos(combined_wall_normal.dot(self.up_direction));
            if floor_angle <= self.floor_max_angle + FLOOR_ANGLE_THRESHOLD {
                state.floor = true;
                state.wall = false;
                if apply_state.floor {
                    self.collision_state.floor = true;
                    self.floor_normal = combined_wall_normal;
                }
                if apply_state.wall {
                    self.collision_state.wall = was_on_wall;
                    self.wall_normal = prev_wall_normal;
                }
            }
        }
        state
    }
    fn set_platform_data(&mut self, collision: &MotionCollision) {
        self.platform_rid = Rid::Invalid;
        if let Some(collider) = collision.collider.clone() {
            if let Ok(collision_object) = collider.try_cast::<CollisionObject3D>() {
                self.platform_rid = collision_object.get_rid();
            }
        }
        self.platform_object_id = collision.collider_id;
        self.platform_velocity = collision.collider_velocity;
        self.platform_angular_velocity = collision.collider_angular_velocity;
        self.platform_layer =
            PhysicsServer3D::singleton().body_get_collision_layer(self.platform_rid);
    }
    fn snap_on_floor(&mut self, was_on_floor: bool, vel_dir_facing_up: bool) {
        if self.collision_state.floor || !was_on_floor || vel_dir_facing_up {
            return;
        }

        self.apply_floor_snap();
    }
}
