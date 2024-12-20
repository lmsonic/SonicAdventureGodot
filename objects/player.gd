class_name Player extends CharacterBody3D
@export_group("Components")
@export var view: Node3D
@onready var model: Node3D = $Model
@onready var footsteps: AudioStreamPlayer = $SoundFootsteps
@onready var particles_trail: GPUParticles3D = $ParticlesTrail
@onready var trail: GPUParticles3D = $Trail
@onready var animation: AnimationPlayer = $Model/Character/AnimationPlayer
@onready var raycast_group: RaycastGroup = $RaycastGroup

@export_group("Jump")
@export var gravity := 20.0
@export var jump_height := 4.0
@export_group("Homing Attack")
@export var homing_attack_force := 15.0
@export var homing_attack_radius := 7.0
@export_group("Running")
@export var acceleration := 7.0
@export var deceleration := 15.0
@export var max_speed := 10.0
@export var slope_assistance := 10.0
@export var slope_drag := 10.0

@export_group("Spindash")
@export var spindash_acceleration := 0.0
@export var spindash_deceleration := 0.0
@export var spindash_max_speed := 30.0
@export var spindash_speed := 30.0
@export var fully_charged_spindash_time := 0.5
@export var spindash_slope_assistance := 10.0
@export var spindash_slope_drag := 10.0
@export var spindash_flat_drag := 2.0

@export_group("Airborne")
@export var air_acceleration := 5.0
@export var air_deceleration := 5.0
@export var max_air_speed := 15.0

@export_group("Rotation")
@export var rotation_speed := 70.0
@export var air_rotation_speed := 50.0
@export var align_rotation_speed := 10.0
@export var spin_speed := 30.0

var spindash_timer := 0.0
var has_homing_attack := true
var homing_attack_target: Node3D = null
var rotation_y := 0.0
var disable_input := false
var hitbox_active := false

@onready var state_chart: StateChart = $StateChart

func get_forward() -> Vector3:
	return global_basis.z

func camera_relative_input() -> Vector3:
	var input_2d := Input.get_vector("move_left", "move_right", "move_forward", "move_back")
	var input_3d := Vector3(input_2d.x, 0.0, input_2d.y)
	return input_3d.rotated(Vector3.UP, view.rotation.y)

func get_jump_velocity() -> float:
	return sqrt(2.0 * gravity * jump_height)

func handle_acceleration(acceleration: float, deceleration: float, max_speed: float, delta: float) -> void:
	var forward := get_forward()
	var input := camera_relative_input()
	var dot := input.dot(forward)
	var planar_velocity := velocity - velocity.project(global_basis.y)
	var normal_velocity := velocity.project(global_basis.y)

	var speed := planar_velocity.length()
	if not disable_input and input != Vector3.ZERO:
		speed += acceleration * delta * dot
	elif speed > 0.5:
		speed -= deceleration * delta
	else:
		speed = 0.0
	speed = clampf(speed, 0.0, max_speed)
	planar_velocity = forward * speed
	velocity = planar_velocity + normal_velocity
	handle_gravity(delta)
	move_and_slide()

	var center := global_position + global_basis.y * 0.5

	DebugDraw3D.draw_arrow(center, center + velocity, Color.BLUE, 0.1)
	DebugDraw3D.draw_arrow(center, center + normal_velocity, Color.RED, 0.1)
	DebugDraw3D.draw_arrow(center, center + planar_velocity, Color.GREEN, 0.1)
	DebugDraw3D.draw_arrow(center, center + get_real_velocity(), Color.PURPLE, 0.1)

func is_on_flat_ground() -> bool:
	var angle := get_floor_angle()
	return angle < deg_to_rad(10.0)

func is_downhill() -> bool:
	var normal := raycast_group.get_floor_normal()
	var forward := get_forward()
	forward.y = 0.0
	forward = forward.normalized()
	return normal.dot(forward) > 0.0

func is_uphill() -> bool:
	var normal := raycast_group.get_floor_normal()
	var forward := get_forward()
	forward.y = 0.0
	forward = forward.normalized()
	return normal.dot(forward) < 0.0

func handle_slopes(delta: float, slope_assistance: float, slope_drag: float, flat_drag := 0.0) -> void:
	if not is_on_floor():
		return
	var angle := get_floor_angle()
	var forward := get_forward()
	var speed := velocity.length()
	var normal := raycast_group.get_floor_normal()

	if is_on_flat_ground():
		velocity -= forward * flat_drag * delta * speed
	elif is_uphill():
		velocity -= forward * slope_drag * delta * speed * angle
	elif is_downhill():
		velocity += forward * slope_assistance * delta * speed * angle

func handle_gravity(delta: float) -> void:
	velocity.y -= gravity * delta

func handle_variable_jump() -> void:
	if velocity.y > 0.0 and Input.is_action_just_released("jump"):
		velocity.y *= 0.8

func handle_rotation(delta: float, rotation_speed: float) -> void:
	var input := camera_relative_input()
	if not disable_input and input.length_squared() > 0.0:
		var target_angle := Vector2(input.z, input.x).angle()
		var speed := velocity.length()
		rotation_speed = rotation_speed / (speed if speed * speed >= 1.0 else 1.0)
		rotation_y = lerp_angle(rotation_y, target_angle, delta * rotation_speed)

	var normal := raycast_group.get_floor_normal()
	if not is_on_floor() || normal == Vector3.ZERO:
		var target_rotation := Quaternion(Vector3.UP, rotation_y).normalized()
		quaternion = quaternion.slerp(target_rotation, delta * align_rotation_speed).normalized()
	else:
		var right := normal.cross(Vector3.BACK)
		var target_rotation := Basis(right, normal, Vector3.BACK).rotated(normal, rotation_y).orthonormalized()
		global_basis = global_basis.slerp(target_rotation, align_rotation_speed * delta).orthonormalized()

func _on_grounded_state_entered() -> void:
	has_homing_attack = true

func _on_jump_state_entered() -> void:
	hitbox_active = true
	var normal := raycast_group.get_floor_normal()
	var jump_velocity := get_jump_velocity() + velocity.y * velocity.length()
	velocity += normal * jump_velocity
	floor_snap_length = 0.0
	Audio.play("res://sounds/jump.ogg")
	var timer := get_tree().create_timer(0.2)
	await timer.timeout
	floor_snap_length = 0.5

func _on_running_state_physics_processing(delta: float) -> void:
	handle_slopes(delta, slope_assistance, slope_drag)
	handle_acceleration(acceleration, deceleration, max_speed, delta)
	handle_rotation(delta, air_rotation_speed)

	if not is_on_floor():
		state_chart.send_event("airborne")
	if Input.is_action_just_pressed("spindash"):
		state_chart.send_event("charge_spindash")

func _on_spindash_state_physics_processing(delta: float) -> void:
	handle_slopes(delta, spindash_slope_assistance, spindash_slope_drag, spindash_flat_drag)
	handle_acceleration(spindash_acceleration, spindash_deceleration, spindash_max_speed, delta)
	handle_rotation(delta, rotation_speed)

	if not is_on_floor():
		state_chart.send_event("airborne")
	if velocity.length() < 2.0:
		state_chart.send_event("running")
	if Input.is_action_just_pressed("spindash"):
		state_chart.send_event("running")

func _on_airball_state_physics_processing(delta: float) -> void:
	handle_variable_jump()
	handle_acceleration(air_acceleration, air_deceleration, max_air_speed, delta)
	handle_rotation(delta, air_rotation_speed)
	if is_on_floor():
		state_chart.send_event("grounded")
	if has_homing_attack and Input.is_action_just_pressed("jump"):
		state_chart.send_event("homing_attack")
		has_homing_attack = false

func _on_airborne_event_received(event: StringName) -> void:
	if event == "grounded":
		var tween := create_tween()
		tween.tween_property(model, "scale", Vector3(1.25, 0.75, 1.25), 0.05)
		tween.tween_property(model, "scale", Vector3.ONE, 0.2)

func _on_running_state_entered() -> void:
	trail.emitting = false
	hitbox_active = false
	var tween := create_tween()
	tween.tween_property(model, "rotation", Vector3.ZERO, 0.1)
	floor_snap_length = 0.5

func _on_airball_state_entered() -> void:
	hitbox_active = true
	trail.emitting = true
	particles_trail.emitting = false
	footsteps.stream_paused = true

func _on_running_state_processing(_delta: float) -> void:
	var input := camera_relative_input()
	if not disable_input and input != Vector3.ZERO and input.dot(get_forward()) < -0.5:
		velocity.x *= 0.5
		velocity.z *= 0.5
	if velocity.length_squared() > 1.0:
		animation.play("walk", 0.5)
		particles_trail.emitting = true
		footsteps.stream_paused = false
	else:
		animation.play("idle", 0.5)
		particles_trail.emitting = false
		footsteps.stream_paused = true

func _on_airball_state_processing(delta: float) -> void:
	model.rotate_x(spin_speed * delta)

func _on_spindash_charge_state_processing(delta: float) -> void:
	handle_rotation(delta, rotation_speed)
	model.rotate_x(spin_speed * delta)
	velocity = velocity.lerp(Vector3.ZERO, delta * deceleration)
	spindash_timer += delta
	spindash_timer = clampf(spindash_timer, 0.0, fully_charged_spindash_time)
	if Input.is_action_just_released("spindash"):
			state_chart.send_event("spindash")

func _on_spindash_charge_state_entered() -> void:
	hitbox_active = true
	spindash_timer = 0.0
	trail.emitting = true
	particles_trail.emitting = false
	footsteps.stream_paused = true

func _on_spindash_charge_state_exited() -> void:
	var remapped_timer := remap(spindash_timer, 0.0, fully_charged_spindash_time, 0.3, 1.0)
	velocity += get_forward() * spindash_speed * remapped_timer

func _on_spindash_state_processing(delta: float) -> void:
	model.rotate_x(2.0 * velocity.length() * delta)

func get_closest_targettable() -> Targettable:
	var targettables := get_tree().get_nodes_in_group("targettable")
	if targettables.is_empty():
		return null
	targettables = targettables.filter(func(x: Targettable) -> bool:
		var delta := global_position.direction_to(x.global_position)
		return delta.dot(get_forward()) > 0.0 and delta.y < 0.5)
	targettables = targettables.filter(func(x: Targettable) -> bool:
		return x.global_position.distance_squared_to(global_position) < homing_attack_radius * homing_attack_radius)
	return targettables.reduce(func(closest: Targettable, x: Targettable) -> Targettable: return x \
		if x.global_position.distance_squared_to(global_position) < closest.global_position.distance_squared_to(global_position) \
		else closest)

func _on_homing_attack_state_entered() -> void:
	hitbox_active = true
	var closest := get_closest_targettable()
	if closest:
		homing_attack_target = closest
	else:
		velocity += get_forward() * homing_attack_force
		state_chart.send_event("airball")

func _on_grounded_state_physics_processing(_delta: float) -> void:
	if Input.is_action_just_pressed("jump"):
		state_chart.send_event("jump")

func _on_homing_attack_state_processing(delta: float) -> void:
	model.rotate_x(spin_speed * delta)

func _on_homing_attack_state_physics_processing(_delta: float) -> void:
	if not homing_attack_target:
		state_chart.send_event("airball")
		return
	var direction := global_position.direction_to(homing_attack_target.global_position)
	velocity = direction * homing_attack_force
	move_and_slide()

func _on_spindash_state_entered() -> void:
	floor_snap_length = 0.3
	hitbox_active = true

func entered_targettable() -> void:
	state_chart.send_event("airball")
	has_homing_attack = true
	homing_attack_target = null
