use godot::{
    engine::{
        native::ObjectId, notify::Node3DNotification, IPhysicsBody3D, KinematicCollision3D,
        PhysicsBody3D, PhysicsTestMotionResult3D,
    },
    prelude::*,
};

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

#[derive(Debug, Default, PartialEq, Eq)]
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

#[derive(GodotClass)]
#[class(base=PhysicsBody3D,init)]

struct CustomCharacterBody3D {
    #[export]
    safe_margin: real,
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
    #[var]
    #[init(default = 6)]
    max_slides: i32,
    #[export]
    platform_layer: i32,

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
    motion_results: Vec<Gd<PhysicsTestMotionResult3D>>,
    slide_colliders: Vec<Gd<KinematicCollision3D>>,
    collision_state: CollisionState,
    #[init(default = Rid::Invalid)]
    platform_rid: Rid,
    #[init(default = ObjectId{id:0})]
    platform_object_id: ObjectId,

    base: Base<PhysicsBody3D>,
}
#[godot_api]
impl IPhysicsBody3D for CustomCharacterBody3D {
    fn on_notification(&mut self, what: Node3DNotification) {
        if let Node3DNotification::EnterTree = what {
            self.collision_state = CollisionState::default();
            self.platform_rid = Rid::Invalid;
            self.platform_object_id = ObjectId { id: 0 };
            self.motion_results.clear();
            self.platform_velocity = Vector3::ZERO;
            self.platform_angular_velocity = Vector3::ZERO;
        }
    }
}

#[godot_api]
impl CustomCharacterBody3D {
    #[func]
    fn move_and_slide(&self) -> bool {
        unimplemented!()
    }
    #[func]
    fn apply_floor_snap(&self) {
        unimplemented!()
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
}

impl CustomCharacterBody3D {
    fn _move_and_slide_floating(&self, delta: real) {
        unimplemented!()
    }
    fn _move_and_slide_grounded(&self, delta: real, was_on_floor: bool) {
        unimplemented!()
    }

    fn _on_floor_if_snapped(was_on_floor: bool, vel_dir_facing_up: bool) -> bool {
        unimplemented!()
    }
    fn _set_collision_direction(
        &self,
        result: Gd<PhysicsTestMotionResult3D>,
        state: &CollisionState,
        apply_state: CollisionState,
    ) {
        const apply_state: CollisionState = CollisionState::new(true, true, true);
        self._set_collision_direction_ex(result, state, apply_state);
    }
    fn _set_collision_direction_ex(
        &self,
        result: Gd<PhysicsTestMotionResult3D>,
        state: &CollisionState,
        apply_state: CollisionState,
    ) {
        unimplemented!()
    }
    fn _set_platform_data(collision_index: u32) {
        unimplemented!()
    }
    fn _snap_on_floor(p_was_on_floor: bool, p_vel_dir_facing_up: bool) {
        unimplemented!()
    }
}
