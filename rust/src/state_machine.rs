use godot::{engine::InputEvent, prelude::*};

use crate::player::Player;

#[derive(Clone, Copy, PartialEq, Eq, Debug, GodotConvert, Var, Export)]
#[godot(via = GString)]
pub enum State {
    Grounded,
    AirBall,
    SpindashCharge,
    GroundBall,
    HomingAttack,
}

impl State {
    pub fn enter(&self, player: &mut Player, previous_state: State) {
        let mut trail = player.trail.clone().unwrap();
        let mut particles = player.particles.clone().unwrap();
        let mut footsteps = player.footsteps.clone().unwrap();

        match self {
            State::Grounded => {
                let model = player.model.clone().unwrap();
                if previous_state == State::AirBall {
                    let mut tween = player.to_gd().create_tween().unwrap();

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

                    player.play_audio("res://sounds/land.ogg");
                }
                player.has_homing_attack = true;
                let mut tween = player.to_gd().create_tween().unwrap();
                tween.tween_property(
                    model.upcast(),
                    "rotation".into(),
                    Vector3::ZERO.to_variant(),
                    0.1,
                );
                let velocity = player.base().get_velocity();
                player
                    .base_mut()
                    .set_velocity(Vector3::new(velocity.x, 0.0, velocity.z));

                trail.set_emitting(false);
                player.base_mut().set_floor_snap_length(0.5);
                player.base_mut().set_floor_stop_on_slope_enabled(true);
            }
            State::SpindashCharge => {
                player.spindash_timer = 0.0;
                let velocity = player.base().get_velocity();
                player.base_mut().set_velocity(Vector3::ZERO);

                trail.set_emitting(true);
                particles.set_emitting(false);
                footsteps.set_stream_paused(true);
                player.base_mut().set_floor_snap_length(0.3);
                player.base_mut().set_floor_stop_on_slope_enabled(false);
            }
            State::AirBall | State::GroundBall => {
                trail.set_emitting(true);
                particles.set_emitting(false);
                footsteps.set_stream_paused(true);
                player.base_mut().set_floor_snap_length(0.3);
                player.base_mut().set_floor_stop_on_slope_enabled(false);
            }
            State::HomingAttack => {
                match player.closest_target_in_front() {
                    Some(target) => {
                        player.homing_attack_target = Some(target);
                    }
                    None => {
                        let mut velocity = player.base().get_velocity();
                        let forward = player.get_forward();
                        let y_speed = velocity.y;
                        velocity.y = 0.0;
                        velocity += forward * player.homing_attack_force;
                        velocity = velocity.limit_length(Some(player.max_air_speed));
                        velocity.y = y_speed;
                        player.base_mut().set_velocity(velocity)
                    }
                }
                trail.set_emitting(true);
                particles.set_emitting(false);
                footsteps.set_stream_paused(true);
                // Slope changes
                player.base_mut().set_floor_snap_length(0.3);
                player.base_mut().set_floor_stop_on_slope_enabled(false);
            }
        }
    }
    pub fn exit(&self, player: &mut Player, new_state: State) {
        match self {
            State::SpindashCharge => {
                let mut velocity = player.base().get_velocity();
                velocity +=
                    player.get_forward() * player.spindash_timer.max(0.3) * player.spindash_speed;
                player.base_mut().set_velocity(velocity);
            }
            State::AirBall => {}
            _ => {}
        }
    }
    pub fn input(&self, player: &mut Player, event: Gd<InputEvent>) -> Option<State> {
        None
    }
    pub fn process_physics(&self, player: &mut Player, delta: f64) -> Option<State> {
        let planar_input = player.camera_relative_input();
        let delta = delta as f32;
        let mut velocity = player.base().get_velocity();
        let forward = player.get_forward();

        match self {
            State::Grounded => {
                velocity.y = 0.0;
                let acceleration = player.acceleration;
                let deceleration = player.deceleration;
                let max_speed = player.max_speed;

                // Planar movement
                let normal = player.base().get_floor_normal();
                let angle = normal.dot(forward) * 0.5 + 1.0;
                if planar_input.length_squared() > 0.0 {
                    velocity += forward * acceleration * delta * angle;
                } else if velocity.length_squared() > 0.0 {
                    velocity = velocity.lerp(Vector3::ZERO, deceleration * delta)
                }

                velocity = velocity.limit_length(Some(max_speed * angle));

                let input = Input::singleton();
                if input.is_action_just_pressed(c"jump".into()) {
                    velocity.y = player.jump_velocity();
                    player.play_audio("res://sounds/jump.ogg");
                }

                player.base_mut().set_velocity(velocity);
                player.base_mut().move_and_slide();

                if input.is_action_just_pressed(c"spindash".into()) {
                    Some(State::SpindashCharge)
                } else if !player.base().is_on_floor() {
                    Some(State::AirBall)
                } else {
                    None
                }
            }
            State::AirBall => {
                let acceleration = player.air_acceleration;
                let deceleration = player.air_deceleration;
                let max_speed = player.max_air_speed;
                let y_speed = velocity.y;

                // Planar movement
                velocity.y = 0.0;
                if planar_input.length_squared() > 0.0 {
                    velocity += forward * acceleration * delta;
                } else if velocity.length_squared() > 0.0 {
                    velocity = velocity.lerp(Vector3::ZERO, deceleration * delta)
                }
                velocity = velocity.limit_length(Some(max_speed));

                let input = Input::singleton();

                // Vertical velocity
                velocity.y = y_speed - player.gravity * delta;

                if velocity.y >= 0.0 && !input.is_action_pressed(c"jump".into()) {
                    velocity.y *= 0.9;
                }

                player.base_mut().set_velocity(velocity);
                player.base_mut().move_and_slide();

                if player.base().is_on_floor() {
                    Some(State::Grounded)
                } else if player.has_homing_attack && input.is_action_just_pressed(c"jump".into()) {
                    player.has_homing_attack = false;
                    Some(State::HomingAttack)
                } else {
                    None
                }
            }
            State::SpindashCharge => {
                let max_spindash_timer = 1.0;
                player.spindash_timer =
                    (player.spindash_timer + delta).clamp(0.0, max_spindash_timer);
                player.base_mut().move_and_slide();
                let input = Input::singleton();

                if input.is_action_just_released(c"spindash".into()) {
                    Some(State::GroundBall)
                } else {
                    None
                }
            }
            State::GroundBall => {
                velocity.y = 0.0;
                let acceleration = player.spindash_acceleration;
                let deceleration = player.spindash_deceleration;
                let max_speed = player.max_spindash_speed;

                // Planar movement
                let normal = player.base().get_floor_normal();
                // Remap -1,1 to 0,2
                let angle = normal.dot(forward) * 0.5 + 1.0;
                if planar_input.length_squared() > 0.0 {
                    velocity += forward * acceleration * delta * angle;
                } else if velocity.length_squared() > 0.0 {
                    velocity = velocity.lerp(Vector3::ZERO, deceleration * delta)
                }

                velocity = velocity.limit_length(Some(max_speed * angle));

                let input = Input::singleton();
                if input.is_action_just_pressed(c"jump".into()) {
                    velocity.y = player.jump_velocity();
                    player.play_audio("res://sounds/jump.ogg");
                }

                player.base_mut().set_velocity(velocity);
                player.base_mut().move_and_slide();

                if input.is_action_just_pressed(c"spindash".into()) {
                    Some(State::SpindashCharge)
                } else if player.base().is_on_floor() {
                    if velocity.length() < 2.0 {
                        Some(State::Grounded)
                    } else {
                        None
                    }
                } else {
                    Some(State::AirBall)
                }
            }
            State::HomingAttack => {
                // If there is a target, continue until target
                // Else become airborne
                match player.homing_attack_target.clone() {
                    Some(target) => {
                        let mut velocity = player.base().get_velocity();
                        let player_pos = player.base().get_global_position();
                        if player_pos.distance_squared_to(target.get_global_position()) < 0.1 {
                            velocity.x *= 0.5;
                            velocity.z *= 0.5;
                            velocity.y = player.jump_velocity();
                            player.base_mut().set_velocity(velocity);
                            player.base_mut().move_and_slide();
                            player.has_homing_attack = true;
                            player.homing_attack_target = None;
                            return Some(Self::AirBall);
                        }
                        let direction_to_target =
                            player_pos.direction_to(target.get_global_position());

                        velocity = direction_to_target * player.homing_attack_force;
                        velocity = velocity.limit_length(Some(player.max_air_speed));
                        godot_print!("{velocity}");
                        player.base_mut().set_velocity(velocity);
                        player.base_mut().move_and_slide();

                        None
                    }
                    None => Some(State::AirBall),
                }
            }
        }
    }
    pub fn process(&self, mut player: &mut Player, delta: f64) -> Option<State> {
        // Animation
        let mut model = player.model.clone().unwrap();
        match self {
            State::Grounded => {
                let mut animation = player.animation.clone().unwrap();
                let mut particles = player.particles.clone().unwrap();
                let mut footsteps = player.footsteps.clone().unwrap();
                let velocity = player.base().get_velocity();
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
                    particles.set_emitting(false);
                    footsteps.set_stream_paused(true);
                };
                player.rotate_on_input(delta);
            }
            State::AirBall | State::SpindashCharge | State::HomingAttack => {
                let rot = player.spin_speed * delta as f32;
                model.rotate_object_local(Vector3::RIGHT, rot);
                player.rotate_on_input(delta);
            }
            State::GroundBall => {
                let rot = 2.0 * player.base().get_velocity().length() * delta as f32;
                model.rotate_object_local(Vector3::RIGHT, rot);
                player.rotate_on_input(delta);
            }
        }
        None
    }
}
