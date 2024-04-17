use godot::{
    engine::{AnimationPlayer, CharacterBody3D, GpuParticles3D, ICharacterBody3D},
    prelude::*,
};

#[derive(GodotClass)]
#[class(base=CharacterBody3D,init)]
struct Player {
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
    drag: f32,

    #[init(default = 10.0)]
    #[export]
    rotation_speed: f32,

    #[init(default = 5.0)]
    #[export]
    jump_height: f32,

    rotation_y: f32,
    was_on_floor: bool,

    base: Base<CharacterBody3D>,
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

        self.camera_relative_input(Vector3::new(
            input.get_axis(c"move_left".into(), c"move_right".into()),
            0.0,
            input.get_axis(c"move_forward".into(), c"move_back".into()),
        ))
    }

    fn get_forward(&self) -> Vector3 {
        -Vector3::FORWARD.rotated(Vector3::UP, self.rotation_y)
    }

    fn jump_velocity(&self) -> f32 {
        f32::sqrt(2.0 * self.gravity * self.jump_height)
    }
    fn handle_effects(&mut self) {
        let mut particles = self.particles.clone().unwrap();
        let mut footsteps = self.footsteps.clone().unwrap();
        let mut animation = self.animation.clone().unwrap();
        particles.set_emitting(false);
        footsteps.set_stream_paused(true);

        if self.base().is_on_floor() {
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
        } else {
            animation
                .play_ex()
                .name(c"jump".into())
                .custom_blend(0.5)
                .done();
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
}

#[godot_api]
impl ICharacterBody3D for Player {
    fn process(&mut self, delta: f64) {
        // Rotation
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
    fn physics_process(&mut self, delta: f64) {
        let planar_input = self.get_grounded_movement_input();
        let delta = delta as f32;

        self.handle_effects();

        let mut velocity = self.base().get_velocity();
        let y_speed = velocity.y;
        velocity.y = 0.0;

        // Acceleration based movement
        if planar_input.length_squared() > 0.0 {
            velocity += self.get_forward() * self.acceleration * delta;
            velocity = velocity.limit_length(Some(self.max_speed));
        } else if velocity.length_squared() > 0.0 {
            velocity = velocity.lerp(Vector3::ZERO, self.deceleration * delta)
        } else {
            velocity = Vector3::ZERO;
        }

        let input = Input::singleton();

        let on_floor = self.base().is_on_floor();
        // Jump
        if on_floor && input.is_action_just_pressed(c"jump".into()) {
            velocity.y = self.jump_velocity();
            self.play_audio("res://sounds/jump.ogg");
        }
        // Gravity
        if !on_floor {
            velocity.y = y_speed - self.gravity * delta;

            if velocity.y >= 0.0 && input.is_action_just_released(c"jump".into()) {
                velocity.y *= 0.5;
            }
        };

        self.base_mut().set_velocity(velocity);
        self.base_mut().move_and_slide();

        // Reload when falling
        if self.base().get_position().y < -20.0 {
            self.base().get_tree().unwrap().reload_current_scene();
        }

        let mut model = self.model.clone().unwrap();
        let model_scale = model.get_scale();
        model.set_scale(model_scale.lerp(Vector3::ONE, delta * 10.0));

        if on_floor && !self.was_on_floor {
            model.set_scale(Vector3::new(1.25, 0.75, 1.25));
            self.play_audio("res://sounds/land.ogg");
            godot_print!("land");
        }
        self.was_on_floor = on_floor;
    }
}
