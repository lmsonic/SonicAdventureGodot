use godot::{
    engine::{AnimationPlayer, CharacterBody3D, GpuParticles3D, ICharacterBody3D, InputEvent},
    prelude::*,
};

use crate::state_machine::State;

#[derive(GodotClass)]
#[class(base=CharacterBody3D,init)]
pub(crate) struct Player {
    // Components
    #[export]
    view: Option<Gd<Node3D>>,
    #[export]
    pub particles: Option<Gd<GpuParticles3D>>,
    #[export]
    pub trail: Option<Gd<GpuParticles3D>>,
    #[export]
    pub footsteps: Option<Gd<AudioStreamPlayer>>,
    #[export]
    pub model: Option<Gd<Node3D>>,
    #[export]
    pub animation: Option<Gd<AnimationPlayer>>,

    // Grounded variables

    // Jump variables
    #[init(default = 10.0)]
    #[export]
    pub gravity: f32,
    #[init(default = 5.0)]
    #[export]
    jump_height: f32,
    // Homing attack variables
    #[init(default = 5.0)]
    #[export]
    pub homing_attack_force: f32,

    #[init(default = 10.0)]
    #[export]
    pub acceleration: f32,
    #[init(default = 10.0)]
    #[export]
    pub deceleration: f32,
    #[init(default = 50.0)]
    #[export]
    pub max_speed: f32,

    #[init(default = 10.0)]
    #[export]
    pub air_acceleration: f32,
    #[init(default = 10.0)]
    #[export]
    pub air_deceleration: f32,
    #[init(default = 150.0)]
    #[export]
    pub max_air_speed: f32,

    #[init(default = 1.0)]
    #[export]
    pub spindash_acceleration: f32,
    #[init(default = 1.0)]
    #[export]
    pub spindash_deceleration: f32,
    #[init(default = 200.0)]
    #[export]
    pub max_spindash_speed: f32,

    #[init(default = 10.0)]
    #[export]
    rotation_speed: f32,
    #[init(default = 20.0)]
    #[export]
    pub spin_speed: f32,

    #[init(default = 200.0)]
    #[export]
    pub spindash_speed: f32,
    rotation_y: f32,
    pub spindash_timer: f32,
    #[init(default = true)]
    pub has_homing_attack: bool,

    #[export]
    #[init(default = State::Grounded)]
    current_state: State,

    base: Base<CharacterBody3D>,
}

impl Player {
    pub fn camera_relative_input(&self) -> Vector3 {
        let input = Input::singleton();
        let input = input.get_vector(
            c"move_left".into(),
            c"move_right".into(),
            "move_forward".into(),
            c"move_back".into(),
        );
        let input = Vector3::new(input.x, 0.0, input.y);
        match &self.view {
            Some(camera) => input.rotated(Vector3::UP, camera.get_rotation().y),
            None => {
                godot_warn!("Camera in player is not set");
                input
            }
        }
    }

    pub fn get_forward(&self) -> Vector3 {
        -Vector3::FORWARD.rotated(Vector3::UP, self.rotation_y)
    }

    pub fn jump_velocity(&self) -> f32 {
        f32::sqrt(2.0 * self.gravity * self.jump_height)
    }

    pub fn play_audio(&self, path: &str) {
        let audio = self.base().get_node("/root/Audio".into());
        if let Some(mut audio) = audio {
            audio.call(c"play".into(), &[path.to_variant()]);
        } else {
            godot_error!("Audio autoload not fetched")
        }
    }

    pub fn rotate_on_input(&mut self, delta: f64) {
        let planar_input = self.camera_relative_input().normalized();
        let delta = delta as f32;
        let planar_input = Vector2::new(planar_input.z, planar_input.x);
        let mut rotation = self.base().get_rotation();
        if planar_input.length_squared() > 0.0 {
            self.rotation_y = planar_input.angle();
        }
        rotation.y = rotation
            .y
            .lerp_angle(self.rotation_y, delta * self.rotation_speed);
        self.base_mut().set_rotation(rotation);
    }
    pub fn change_state(&mut self, new_state: State) {
        let current_state = self.current_state;
        current_state.exit(self, new_state);
        new_state.enter(self, self.current_state);
        self.current_state = new_state;
    }
}
#[godot_api]
impl ICharacterBody3D for Player {
    fn ready(&mut self) {
        self.change_state(self.current_state);
    }
    fn input(&mut self, event: Gd<InputEvent>) {
        let current_state = self.current_state;

        if let Some(new_state) = current_state.input(self, event) {
            self.change_state(new_state);
        }
    }
    fn process(&mut self, delta: f64) {
        let current_state = self.current_state;

        if let Some(new_state) = current_state.process(self, delta) {
            self.change_state(new_state);
        }
    }
    fn physics_process(&mut self, delta: f64) {
        let current_state = self.current_state;

        if let Some(new_state) = current_state.process_physics(self, delta) {
            self.change_state(new_state);
        }
    }
}
