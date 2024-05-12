use godot::{
    engine::{
        notify::Node3DNotification, physics_server_3d::BodyAxis, AnimatableBody3D, CharacterBody3D,
        CollisionObject3D, Engine, IAnimatableBody3D, IRigidBody3D, IStaticBody3D,
        KinematicCollision3D, PhysicsBody3D, PhysicsServer3D, PhysicsTestMotionParameters3D,
        PhysicsTestMotionResult3D, RigidBody3D, StaticBody3D,
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
#[class(base=RigidBody3D,init)]
#[allow(clippy::struct_excessive_bools)]
struct CustomCharacterBody3D {
    #[export]
    motion_mode: MotionMode,
    #[export]
    #[init(default=Vector3::UP)]
    up_direction: Vector3,
    #[init(default = true)]
    slide_on_ceiling: bool,
    #[export(range=(0.0,180.0,radians))]
    #[init(default=f32::to_radians(15.0))]
    wall_min_slide_angle: real,

    // Floor
    #[export]
    #[init(default = true)]
    floor_stop_on_slope: bool,
    #[export]
    floor_constant_speed: bool,
    #[export]
    #[init(default = true)]
    floor_block_on_wall: bool,
    #[export(range=(0.0,180.0,radians))]
    #[init(default=f32::to_radians(45.0))]
    floor_max_angle: real,
    #[export(range=(0.0, 1.0))]
    #[init(default = 0.1)]
    floor_snap_length: real,
    // Platform
    #[export]
    platform_on_leave: PlatformOnLeave,
    #[export(flags_3d_physics)]
    #[init(default=u32::MAX)]
    platform_floor_layers: u32,
    #[export(flags_3d_physics)]
    platform_wall_layers: u32,
    // Collision
    #[export(range=(0.001,256.0))]
    #[init(default = 0.001)]
    margin: real,

    #[var(set=set_max_slides)]
    #[init(default = 6)]
    max_slides: i32,

    #[var]
    velocity: Vector3,

    floor_normal: Vector3,
    wall_normal: Vector3,
    ceiling_normal: Vector3,
    last_motion: Vector3,
    platform_velocity: Vector3,
    platform_angular_velocity: Vector3,
    platform_ceiling_velocity: Vector3,
    platform_layer: u32,
    previous_position: Vector3,
    real_velocity: Vector3,
    motion_results: Vec<MotionResult>,
    slide_colliders: Vec<Gd<KinematicCollision3D>>,
    collision_state: CollisionState,
    #[init(default = Rid::Invalid)]
    platform_rid: Rid,
    platform_object_id: i64,

    base: Base<RigidBody3D>,
}

#[godot_api]
impl IRigidBody3D for CustomCharacterBody3D {
    fn on_notification(&mut self, what: Node3DNotification) {
        if let Node3DNotification::EnterTree = what {
            self.collision_state = CollisionState::default();
            self.platform_rid = Rid::Invalid;
            self.platform_object_id = 0;
            self.motion_results.clear();
            self.platform_velocity = Vector3::ZERO;
            self.platform_angular_velocity = Vector3::ZERO;
            self.base_mut().set_use_custom_integrator(true);
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
    let colliding = PhysicsServer3D::singleton().call(
        c"body_test_motion".into(),
        &[
            body.get_rid().to_variant(),
            parameters.clone().to_variant(),
            result.clone().to_variant(),
        ],
    );
    let colliding = colliding.booleanize();
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

    if !test_only {
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
    fn move_and_slide(&mut self) -> bool {
        // Hack in order to work with calling from _process as well as from _physics_process; calling from thread is risky
        let delta = if Engine::singleton().is_in_physics_frame() {
            self.base()
                .clone()
                .upcast::<Node>()
                .get_physics_process_delta_time()
        } else {
            self.base()
                .clone()
                .upcast::<Node>()
                .get_process_delta_time()
        };
        if self.base().get_axis_lock(BodyAxis::LINEAR_X) {
            self.velocity.x = 0.0;
        }
        if self.base().get_axis_lock(BodyAxis::LINEAR_Y) {
            self.velocity.y = 0.0;
        }
        if self.base().get_axis_lock(BodyAxis::LINEAR_Z) {
            self.velocity.z = 0.0;
        }

        let gt = self.base().get_global_transform();
        self.previous_position = gt.origin;

        let mut current_platform_velocity = self.platform_velocity;

        if (self.collision_state.floor || self.collision_state.wall) && self.platform_rid.is_valid()
        {
            let mut excluded = false;
            if self.collision_state.floor {
                excluded = (self.platform_floor_layers & self.platform_layer) == 0;
            } else if self.collision_state.wall {
                excluded = (self.platform_wall_layers & self.platform_layer) == 0;
            }
            if excluded {
                current_platform_velocity = Vector3::ZERO;
            } else if let Some(bs) =
                PhysicsServer3D::singleton().body_get_direct_state(self.platform_rid)
            {
                let local_position = gt.origin - bs.get_transform().origin;
                current_platform_velocity = bs.get_velocity_at_local_position(local_position);
            } else {
                self.platform_rid = Rid::Invalid;
            }
        }

        self.motion_results.clear();

        let was_on_floor = self.collision_state.floor;
        self.collision_state = CollisionState::default();

        self.last_motion = Vector3::ZERO;

        if !current_platform_velocity.is_zero_approx() {
            let mut params = PhysicsTestMotionParameters3D::new_gd();
            params.set_from(self.base().get_global_transform());
            params.set_motion(current_platform_velocity * delta as real);
            params.set_margin(self.margin);
            // Also report collisions generated only from recovery.
            params.set_recovery_as_collision_enabled(true);
            let mut bodies = params.get_exclude_bodies();
            bodies.push(self.platform_rid);
            params.set_exclude_bodies(bodies);

            if self.platform_object_id != 0 {
                let mut objects = params.get_exclude_objects();
                objects.push(self.platform_object_id);
                params.set_exclude_objects(objects);
            }

            let mut body = self.base_mut().clone().upcast();
            if let Some(floor_result) = move_and_collide(&mut body, &params, false, false) {
                self.update_collision_state(&floor_result);
                self.motion_results.push(floor_result);
            }
        }

        if self.motion_mode == MotionMode::Grounded {
            self.move_and_slide_grounded(delta as real, was_on_floor);
        } else {
            self.move_and_slide_floating(delta as real);
        }

        // Compute real velocity.
        self.real_velocity = self.get_position_delta() / delta as real;

        if self.platform_on_leave != PlatformOnLeave::DoNothing {
            // Add last platform velocity when just left a moving platform.
            if !self.collision_state.floor && !self.collision_state.wall {
                if self.platform_on_leave == PlatformOnLeave::AddUpwardVelocity
                    && current_platform_velocity.dot(self.up_direction) < 0.0
                {
                    current_platform_velocity = current_platform_velocity.slide(self.up_direction);
                }
                self.velocity += current_platform_velocity;
            }
        }

        !self.motion_results.is_empty()
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
                self.update_collision_state_ex(&result, CollisionState::new(true, false, false));

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
        for _ in 0..self.max_slides {
            let mut params = PhysicsTestMotionParameters3D::new_gd();
            params.set_from(self.base().get_global_transform());
            params.set_motion(motion);
            params.set_margin(self.margin);
            params.set_recovery_as_collision_enabled(true); // Also report collisions generated only from recovery.

            let result =
                move_and_collide(&mut self.base_mut().clone().upcast(), &params, false, false);

            if let Some(result) = result {
                self.last_motion = result.travel;
                self.motion_results.push(result.clone());
                self.update_collision_state(&result);
                if result.remainder.is_zero_approx() {
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
    #[allow(clippy::too_many_lines)]
    fn move_and_slide_grounded(&mut self, delta: real, was_on_floor: bool) {
        let mut motion = self.velocity * delta;
        let motion_slide_up = motion.slide(self.up_direction);
        let prev_floor_normal = self.floor_normal;

        self.platform_rid = Rid::Invalid;
        self.platform_object_id = 0;
        self.platform_velocity = Vector3::ZERO;
        self.platform_angular_velocity = Vector3::ZERO;
        self.platform_ceiling_velocity = Vector3::ZERO;
        self.floor_normal = Vector3::ZERO;
        self.wall_normal = Vector3::ZERO;
        self.ceiling_normal = Vector3::ZERO;
        // No sliding on first attempt to keep floor motion stable when possible,
        // When stop on slope is enabled or when there is no up direction.
        let mut sliding_enabled = !self.floor_stop_on_slope;
        // Constant speed can be applied only the first time sliding is enabled.
        let mut can_apply_constant_speed = sliding_enabled;
        // If the platform's ceiling push down the body.
        let mut apply_ceiling_velocity = false;
        let mut first_slide = true;
        let vel_dir_facing_up = self.velocity.dot(self.up_direction) > 0.0;
        let mut total_travel = Vector3::ZERO;
        for i in 0..self.max_slides {
            let mut params = PhysicsTestMotionParameters3D::new_gd();
            params.set_from(self.base().get_global_transform());
            params.set_motion(motion);
            params.set_margin(self.margin);
            // There can be 4 collisions between 2 walls + 2 more for the floor.
            params.set_max_collisions(6);
            // Also report collisions generated only from recovery.
            params.set_recovery_as_collision_enabled(true);
            let result = move_and_collide(
                &mut self.base_mut().clone().upcast(),
                &params,
                false,
                !sliding_enabled,
            );
            let mut collided = result.is_some();
            self.last_motion = Vector3::ZERO;

            if let Some(mut result) = result {
                self.last_motion = result.travel;
                self.motion_results.push(result.clone());
                let previous_state = self.collision_state;
                let result_state = self.update_collision_state(&result);
                // If we hit a ceiling platform, we set the vertical velocity to at least the platform one.
                if self.collision_state.ceiling
                    && self.platform_ceiling_velocity != Vector3::ZERO
                    && self.platform_ceiling_velocity.dot(self.up_direction) < 0.0
                // If ceiling sliding is on, only apply when the ceiling is flat or when the motion is upward.
                    && (!self.slide_on_ceiling
                        || motion.dot(self.up_direction) < 0.0
                        || (self.ceiling_normal + self.up_direction).length_squared() < 0.01)
                {
                    apply_ceiling_velocity = true;
                    let ceiling_vertical_velocity =
                        self.up_direction * self.up_direction.dot(self.platform_ceiling_velocity);
                    let motion_vertical_velocity =
                        self.up_direction * self.up_direction.dot(self.velocity);
                    if motion_vertical_velocity.dot(self.up_direction) > 0.0
                        || ceiling_vertical_velocity.length_squared()
                            > motion_vertical_velocity.length_squared()
                    {
                        self.velocity =
                            ceiling_vertical_velocity + self.velocity.slide(self.up_direction);
                    }
                }
                if self.collision_state.floor
                    && self.floor_stop_on_slope
                    && (self.velocity.normalized() + self.up_direction).length() < 0.01
                {
                    // Velocity pointing down when on floor
                    let mut gt = self.base().get_global_transform();
                    if result.travel.length() <= self.margin + CMP_EPSILON {
                        gt.origin -= result.travel;
                    }
                    self.base_mut().set_global_transform(gt);
                    self.velocity = Vector3::ZERO;
                    self.last_motion = Vector3::ZERO;
                    godot_print!("stop on slope");
                    break;
                }

                if result.remainder.is_zero_approx() {
                    break;
                }

                // Apply regular sliding by default.
                let mut apply_default_sliding = true;

                // Wall collision checks.
                if result_state.wall && motion_slide_up.dot(self.wall_normal) <= 0.0 {
                    // Move on floor only checks.
                    if self.floor_block_on_wall {
                        // Needs horizontal motion from current motion instead of motion_slide_up
                        // to properly test the angle and avoid standing on slopes
                        let horizontal_motion = motion.slide(self.up_direction);
                        let horizontal_normal =
                            self.wall_normal.slide(self.up_direction).normalized();
                        let motion_angle = real::abs(real::acos(
                            -horizontal_normal.dot(horizontal_motion.normalized()),
                        ));
                        // Avoid to move forward on a wall if floor_block_on_wall is true.
                        // Applies only when the motion angle is under 90 degrees,
                        // in order to avoid blocking lateral motion along a wall.
                        if motion_angle < 0.5 * std::f32::consts::PI {
                            apply_default_sliding = false;
                            if was_on_floor && !vel_dir_facing_up {
                                // Cancel the motion.
                                let mut gt = self.base().get_global_transform();
                                let travel_total = result.travel.length();
                                let cancel_dist_max = real::min(0.1, self.margin * 20.0);
                                if travel_total <= self.margin + CMP_EPSILON {
                                    gt.origin -= result.travel;
                                    result.travel = Vector3::ZERO; // Cancel for constant speed computation.
                                } else if travel_total < cancel_dist_max {
                                    // If the movement is large the body can be prevented from reaching the walls.
                                    gt.origin -= result.travel.slide(self.up_direction);
                                    // Keep remaining motion in sync with amount canceled.
                                    motion = motion.slide(self.up_direction);
                                    result.travel = Vector3::ZERO;
                                } else {
                                    // Travel is too high to be safely canceled, we take it into account.
                                    result.travel = result.travel.slide(self.up_direction);
                                    motion = result.remainder;
                                }
                                self.base_mut().set_global_transform(gt);
                                // Determines if you are on the ground, and limits the possibility of climbing on the walls because of the approximations.
                                self.snap_on_floor(true, false);
                            } else {
                                // If the movement is not canceled we only keep the remaining.
                                motion = result.remainder;
                            }
                            // Apply slide on forward in order to allow only lateral motion on next step.
                            let forward = self.wall_normal.slide(self.up_direction).normalized();
                            motion = motion.slide(forward);

                            // Scales the horizontal velocity according to the wall slope.
                            if vel_dir_facing_up {
                                let slide_motion = self.velocity.slide(result.collisions[0].normal);
                                // Keeps the vertical motion from velocity and add the horizontal motion of the projection.
                                self.velocity = self.up_direction
                                    * self.up_direction.dot(self.velocity)
                                    + slide_motion.slide(self.up_direction);
                            } else {
                                self.velocity = self.velocity.slide(forward);
                            }
                            // Allow only lateral motion along previous floor when already on floor.
                            // Fixes slowing down when moving in diagonal against an inclined wall.
                            if was_on_floor
                                && !vel_dir_facing_up
                                && motion.dot(self.up_direction) > 0.0
                            {
                                // Slide along the corner between the wall and previous floor.
                                let floor_side = prev_floor_normal.cross(self.wall_normal);
                                if floor_side != Vector3::ZERO {
                                    motion = floor_side * motion.dot(floor_side);
                                }
                            }

                            // Stop all motion when a second wall is hit (unless sliding down or jumping),
                            // in order to avoid jittering in corner cases.
                            let mut stop_all_motion = previous_state.wall && !vel_dir_facing_up;

                            // Allow sliding when the body falls.
                            if !self.collision_state.floor && motion.dot(self.up_direction) < 0.0 {
                                let slide_motion = motion.slide(self.wall_normal);
                                // Test again to allow sliding only if the result goes downwards.
                                // Fixes jittering issues at the bottom of inclined walls.
                                if slide_motion.dot(self.up_direction) < 0.0 {
                                    stop_all_motion = false;
                                    motion = slide_motion;
                                }
                            }

                            if stop_all_motion {
                                motion = Vector3::ZERO;
                                self.velocity = Vector3::ZERO;
                            }
                        }
                    }
                    // Stop horizontal motion when under wall slide threshold.
                    if was_on_floor && self.wall_min_slide_angle > 0.0 && result_state.wall {
                        let horizontal_normal =
                            self.wall_normal.slide(self.up_direction).normalized();
                        let motion_angle = real::abs(real::acos(
                            -horizontal_normal.dot(motion_slide_up.normalized()),
                        ));
                        if motion_angle < self.wall_min_slide_angle {
                            motion = self.up_direction * motion.dot(self.up_direction);
                            self.velocity =
                                self.up_direction * self.velocity.dot(self.up_direction);
                            apply_default_sliding = false;
                        }
                    }
                }

                if apply_default_sliding {
                    // Regular sliding, the last part of the test handle the case when you don't want to slide on the ceiling.
                    if (sliding_enabled || !self.collision_state.floor)
                        && (!self.collision_state.ceiling
                            || self.slide_on_ceiling
                            || !vel_dir_facing_up)
                        && !apply_ceiling_velocity
                    {
                        let collision = &result.collisions[0];

                        let mut slide_motion = result.remainder.slide(collision.normal);
                        if self.collision_state.floor
                            && !self.collision_state.wall
                            && !motion_slide_up.is_zero_approx()
                        {
                            // Slide using the intersection between the motion plane and the floor plane,
                            // in order to keep the direction intact.
                            let motion_length = slide_motion.length();
                            slide_motion = self
                                .up_direction
                                .cross(result.remainder)
                                .cross(self.floor_normal);

                            // Keep the length from default slide to change speed in slopes by default,
                            // when constant speed is not enabled.
                            slide_motion = slide_motion.normalized();
                            slide_motion *= motion_length;
                        }

                        if slide_motion.dot(self.velocity) > 0.0 {
                            motion = slide_motion;
                        } else {
                            motion = Vector3::ZERO;
                        }

                        if self.slide_on_ceiling && result_state.ceiling {
                            // Apply slide only in the direction of the input motion, otherwise just stop to avoid jittering when moving against a wall.
                            if vel_dir_facing_up {
                                self.velocity = self.velocity.slide(collision.normal);
                            } else {
                                // Avoid acceleration in slope when falling.
                                self.velocity =
                                    self.up_direction * self.up_direction.dot(self.velocity);
                            }
                        }
                    }
                    // No sliding on first attempt to keep floor motion stable when possible.
                    else {
                        motion = result.remainder;
                        if result_state.ceiling && !self.slide_on_ceiling && vel_dir_facing_up {
                            self.velocity = self.velocity.slide(self.up_direction);
                            motion = motion.slide(self.up_direction);
                        }
                    }
                }
                total_travel += result.travel;

                // Apply Constant Speed.
                if was_on_floor
                    && self.floor_constant_speed
                    && can_apply_constant_speed
                    && self.collision_state.floor
                    && !motion.is_zero_approx()
                {
                    let travel_slide_up = total_travel.slide(self.up_direction);
                    motion = motion.normalized()
                        * real::max(0.0, motion_slide_up.length() - travel_slide_up.length());
                }
            }
            // When you move forward in a downward slope you don’t collide because you will be in the air.
            // This test ensures that constant speed is applied, only if the player is still on the ground after the snap is applied.
            else if self.floor_constant_speed
                && first_slide
                && self.on_floor_if_snapped(was_on_floor, vel_dir_facing_up)
            {
                godot_print!("on slope in air");
                can_apply_constant_speed = false;
                sliding_enabled = true;
                // WTF: result is not valid here!
                // let gt = self.base().get_global_transform();
                // gt.origin = gt.origin - result.travel;
                // self.base_mut().set_global_transform(gt);

                // Slide using the intersection between the motion plane and the floor plane,
                // in order to keep the direction intact.
                let motion_slide_norm = self.up_direction.cross(motion).cross(prev_floor_normal);
                let motion_slide_norm = motion_slide_norm.normalized();

                motion = motion_slide_norm * motion_slide_up.length();
                collided = true;
            }

            if !collided || motion.is_zero_approx() {
                break;
            }

            can_apply_constant_speed = !can_apply_constant_speed && !sliding_enabled;
            sliding_enabled = true;
            first_slide = false;
        }

        self.snap_on_floor(was_on_floor, vel_dir_facing_up);

        // Reset the gravity accumulation when touching the ground.
        if self.collision_state.floor && !vel_dir_facing_up {
            self.velocity = self.velocity.slide(self.up_direction);
        }
    }

    fn on_floor_if_snapped(&mut self, was_on_floor: bool, vel_dir_facing_up: bool) -> bool {
        if self.up_direction == Vector3::ZERO
            || self.collision_state.floor
            || !was_on_floor
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
            let result_state = self.update_collision_state_ex(&result, CollisionState::default());

            return result_state.floor;
        }

        false
    }
    fn update_collision_state(&mut self, result: &MotionResult) -> CollisionState {
        const APPLY_STATE: CollisionState = CollisionState::new(true, true, true);
        self.update_collision_state_ex(result, APPLY_STATE)
    }
    fn update_collision_state_ex(
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
        let mut previous_normal = Vector3::default(); // Avoid duplicate on average calculation.

        for collision in result.collisions.iter().rev() {
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
                    continue;
                }
                // Check if any collision is ceiling.
                let ceiling_angle = real::acos(normal.dot(-self.up_direction));
                if ceiling_angle <= self.floor_max_angle + FLOOR_ANGLE_THRESHOLD {
                    state.ceiling = true;
                    if apply_state.ceiling {
                        self.collision_state.ceiling = true;
                        self.platform_ceiling_velocity = collider_velocity;
                        self.ceiling_normal = normal;
                    }
                    continue;
                }
            }
            // Collision is wall by default.
            state.wall = true;
            if apply_state.wall && depth > wall_depth {
                state.wall = true;
                self.collision_state.wall = true;
                wall_depth = depth;
                self.wall_normal = normal;

                // Don't apply wall velocity when the collider is a CharacterBody3D.
                if let Some(ref collider) = collision.collider {
                    if collider.clone().try_cast::<CharacterBody3D>().is_err()
                        || collider
                            .clone()
                            .try_cast::<CustomCharacterBody3D>()
                            .is_err()
                    {
                        self.set_platform_data(collision);
                    }
                }
            }

            // Collect normal for calculating average.
            if normal != previous_normal {
                previous_normal = normal;
                combined_wall_normal += normal;
                wall_collision_count += 1;
            }
        }
        // Check if wall normals cancel out to floor support.
        if state.wall
            && wall_collision_count > 1
            && !state.floor
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
        if self.platform_rid == Rid::Invalid {
            self.platform_layer = 0;
        } else {
            self.platform_layer =
                PhysicsServer3D::singleton().body_get_collision_layer(self.platform_rid);
        }
    }
    fn snap_on_floor(&mut self, was_on_floor: bool, vel_dir_facing_up: bool) {
        if self.collision_state.floor || !was_on_floor || vel_dir_facing_up {
            return;
        }
        self.apply_floor_snap();
    }
}
