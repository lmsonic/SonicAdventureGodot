use godot::{
    engine::{AnimationPlayer, CharacterBody3D, GpuParticles3D, ICharacterBody3D},
    prelude::*,
};

#[derive(GodotClass)]
#[class(base=CharacterBody3D,init)]
pub(crate) struct Player {
    #[export]
    view: Option<Gd<Node3D>>,
    #[export]
    particles: Option<Gd<GpuParticles3D>>,
    #[export]
    footsteps: Option<Gd<AudioStreamPlayer>>,
    #[export]
    model: Option<Gd<Node3D>>,
    #[export]
    animation: Option<Gd<AnimationPlayer>>,

    #[init(default = 50.0)]
    #[export]
    max_speed: f32,
    #[init(default = 150.0)]
    #[export]
    max_air_speed: f32,

    #[init(default = 200.0)]
    #[export]
    max_spindash_speed: f32,

    #[init(default = 10.0)]
    #[export]
    gravity: f32,

    #[init(default = 10.0)]
    #[export]
    acceleration: f32,
    #[init(default = 10.0)]
    #[export]
    deceleration: f32,

    #[init(default = 10.0)]
    #[export]
    air_acceleration: f32,
    #[init(default = 10.0)]
    #[export]
    air_deceleration: f32,

    #[init(default = 0.0)]
    #[export]
    spindash_deceleration: f32,

    #[init(default = 10.0)]
    #[export]
    rotation_speed: f32,
    #[init(default = 20.0)]
    #[export]
    spin_speed: f32,

    #[init(default = 5.0)]
    #[export]
    jump_height: f32,
    #[init(default = 5.0)]
    #[export]
    homing_attack_force: f32,

    #[init(default = 200.0)]
    #[export]
    spindash_speed: f32,
    rotation_y: f32,
    spindash_timer: f32,
    #[init(default = true)]
    has_homing_attack: bool,

    #[init(default = State::Grounded)]
    state: State,

    base: Base<CharacterBody3D>,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum State {
    Grounded,
    Airborne,
    SpindashCharge,
    SpindashRelease,
}

impl Player {
    fn camera_relative_input(&self, input: Vector3) -> Vector3 {
        match &self.view {
            Some(camera) => input.rotated(Vector3::UP, camera.get_rotation().y),
            None => {
                godot_warn!("Camera in player is not set");
                input
            }
        }
    }

    fn get_grounded_movement_input(&self) -> Vector3 {
        let input = Input::singleton();
        let vector_input = input.get_vector(
            c"move_left".into(),
            c"move_right".into(),
            "move_forward".into(),
            c"move_back".into(),
        );
        self.camera_relative_input(Vector3::new(vector_input.x, 0.0, vector_input.y))
    }

    fn get_forward(&self) -> Vector3 {
        -Vector3::FORWARD.rotated(Vector3::UP, self.rotation_y)
    }

    fn jump_velocity(&self) -> f32 {
        f32::sqrt(2.0 * self.gravity * self.jump_height)
    }
    fn animation(&mut self, delta: f64) {
        let mut model = self.model.clone().unwrap();
        let mut particles = self.particles.clone().unwrap();
        let mut footsteps = self.footsteps.clone().unwrap();
        let mut animation = self.animation.clone().unwrap();

        match self.state {
            State::Grounded => {
                let velocity = self.base().get_velocity();
                if velocity.x.abs() > 1.0 || velocity.z.abs() > 1.0 {
                    animation
                        .play_ex()
                        .name(c"walk".into())
                        .custom_blend(0.5)
                        .done();
                    particles.set_emitting(true);
                    footsteps.set_stream_paused(false);
                } else {
                    animation
                        .play_ex()
                        .name(c"idle".into())
                        .custom_blend(0.5)
                        .done();
                }
            }
            State::Airborne => {
                particles.set_emitting(false);
                footsteps.set_stream_paused(true);
                let rot = self.spin_speed * delta as f32;
                model.rotate_object_local(Vector3::RIGHT, rot);
            }
            State::SpindashCharge => {
                let rot = self.spin_speed * delta as f32;
                model.rotate_object_local(Vector3::RIGHT, rot);
            }
            State::SpindashRelease => {
                let rot = 2.0 * self.base().get_velocity().length() * delta as f32;
                model.rotate_object_local(Vector3::RIGHT, rot);
            }
        }
    }

    fn play_audio(&self, path: &str) {
        let audio = self.base().get_node("/root/Audio".into());
        if let Some(mut audio) = audio {
            audio.call(c"play".into(), &[path.to_variant()]);
        } else {
            godot_error!("Audio autoload not fetched")
        }
    }

    fn rotate_on_input(&mut self, delta: f64) {
        let planar_input = self.get_grounded_movement_input().normalized();
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

    fn process_grounded(&mut self, delta: f64) -> State {
        let planar_input = self.get_grounded_movement_input();
        let delta = delta as f32;

        let mut velocity = self.base().get_velocity();
        velocity.y = 0.0;

        // Planar movement
        let forward = self.get_forward();
        if planar_input.length_squared() > 0.0 {
            velocity += forward * self.acceleration * delta;
        } else if velocity.length_squared() > 0.0 {
            velocity = velocity.lerp(Vector3::ZERO, self.deceleration * delta)
        }

        velocity = velocity.limit_length(Some(self.max_speed));

        let input = Input::singleton();
        if input.is_action_just_pressed(c"jump".into()) {
            velocity.y = self.jump_velocity();
            self.play_audio("res://sounds/jump.ogg");
        }

        self.base_mut().set_velocity(velocity);
        self.base_mut().move_and_slide();

        if input.is_action_just_pressed(c"spindash".into()) {
            State::SpindashCharge
        } else if self.base().is_on_floor() {
            State::Grounded
        } else {
            State::Airborne
        }
    }
    fn process_spindash(&mut self, delta: f64) -> State {
        let planar_input = self.get_grounded_movement_input();
        let delta = delta as f32;

        let mut velocity = self.base().get_velocity();
        velocity.y = 0.0;

        // Planar movement
        let forward = self.get_forward();
        if planar_input.length_squared() > 0.0 {
            velocity += forward * self.acceleration * delta;
        } else if velocity.length_squared() > 0.0 {
            velocity = velocity.lerp(Vector3::ZERO, self.spindash_deceleration * delta)
        }

        velocity = velocity.limit_length(Some(self.max_spindash_speed));

        let input = Input::singleton();
        if input.is_action_just_pressed(c"jump".into()) {
            velocity.y = self.jump_velocity();
            self.play_audio("res://sounds/jump.ogg");
        }

        self.base_mut().set_velocity(velocity);
        self.base_mut().move_and_slide();

        if input.is_action_just_pressed(c"spindash".into()) {
            State::Grounded
        } else if self.base().is_on_floor() {
            if velocity.length() < 2.0 {
                State::Grounded
            } else {
                State::SpindashRelease
            }
        } else {
            State::Airborne
        }
    }

    fn process_airborne(&mut self, delta: f64) -> State {
        let planar_input = self.get_grounded_movement_input();
        let delta = delta as f32;

        let mut velocity = self.base().get_velocity();
        let y_speed = velocity.y;
        velocity.y = 0.0;

        // Acceleration based movement
        let forward = self.get_forward();
        if planar_input.length_squared() > 0.0 {
            velocity += forward * self.air_acceleration * delta;
        } else if velocity.length_squared() > 0.0 {
            velocity = velocity.lerp(Vector3::ZERO, self.air_deceleration * delta)
        }

        let input = Input::singleton();
        // Homing Attack
        if self.has_homing_attack && input.is_action_just_pressed(c"jump".into()) {
            velocity += forward * self.homing_attack_force;
            self.has_homing_attack = false;
        }
        velocity = velocity.limit_length(Some(self.max_air_speed));

        // Vertical velocity
        velocity.y = y_speed - self.gravity * delta;

        if velocity.y >= 0.0 && input.is_action_just_released(c"jump".into()) {
            velocity.y *= 0.5;
        }

        self.base_mut().set_velocity(velocity);
        self.base_mut().move_and_slide();

        if self.base().is_on_floor() {
            State::Grounded
        } else {
            State::Airborne
        }
    }

    fn transition(&mut self, new_state: State) {
        if self.state != new_state {
            match new_state {
                State::Grounded => {
                    if self.state == State::Airborne {
                        let model = self.model.clone().unwrap();
                        let mut tween = self.base_mut().create_tween().unwrap();

                        tween.tween_property(
                            model.clone().upcast(),
                            "scale".into(),
                            Vector3::new(1.25, 0.75, 1.25).to_variant(),
                            0.05,
                        );
                        tween.tween_property(
                            model.clone().upcast(),
                            "scale".into(),
                            Vector3::ONE.to_variant(),
                            0.2,
                        );
                        self.play_audio("res://sounds/land.ogg");
                        self.has_homing_attack = true;
                    }
                    let mut tween = self.base_mut().create_tween().unwrap();
                    let model = self.model.clone().unwrap();
                    tween.tween_property(
                        model.upcast(),
                        "rotation".into(),
                        Vector3::ZERO.to_variant(),
                        0.1,
                    );
                }
                State::Airborne => {}
                State::SpindashCharge => {
                    self.spindash_timer = 0.0;
                    self.base_mut().set_velocity(Vector3::ZERO)
                }
                State::SpindashRelease => {
                    let mut velocity = self.base().get_velocity();
                    velocity += self.get_forward() * self.spindash_timer * self.spindash_speed;
                    godot_print!("{velocity}");
                    self.base_mut().set_velocity(velocity);
                }
            }
            self.state = new_state;
            godot_print!("{:?}", new_state);
        }
    }

    fn process_spindash_charge(&mut self, delta: f64) -> State {
        let delta = delta as f32;
        let max_spindash_timer = 1.0;
        self.spindash_timer = (self.spindash_timer + delta).clamp(0.0, max_spindash_timer);
        let input = Input::singleton();
        if input.is_action_just_released(c"spindash".into()) {
            State::SpindashRelease
        } else {
            State::SpindashCharge
        }
    }
}
#[godot_api]
impl ICharacterBody3D for Player {
    fn process(&mut self, delta: f64) {
        self.rotate_on_input(delta);
    }
    fn physics_process(&mut self, delta: f64) {
        self.animation(delta);
        let new_state = match self.state {
            State::Grounded => self.process_grounded(delta),
            State::Airborne => self.process_airborne(delta),
            State::SpindashCharge => self.process_spindash_charge(delta),
            State::SpindashRelease => self.process_spindash(delta),
        };
        self.transition(new_state);

        // Reload when falling
        if self.base().get_position().y < -20.0 {
            self.base().get_tree().unwrap().reload_current_scene();
        }
    }
}
