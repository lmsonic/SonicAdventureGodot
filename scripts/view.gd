extends Node3D

@export_group("Properties")
@export var target: Node3D

@export_group("Zoom")
@export var zoom_minimum := 16.0
@export var zoom_maximum := 4.0
@export var zoom_speed := 10.0

@export_group("Rotation")
@export var rotation_speed := 120.0

var camera_rotation: Vector3
var zoom := 10.0

@onready var camera: Camera3D = $Camera

func _ready() -> void:

	camera_rotation = rotation_degrees # Initial rotation

	pass

func _physics_process(delta: float) -> void:

	# Set position and rotation to targets

	position = position.lerp(target.position, delta * 4)

	rotation_degrees = rotation_degrees.lerp(camera_rotation, delta * 6)

	camera.position = camera.position.lerp(Vector3(0, 0, zoom), 8 * delta)

	handle_input(delta)

# Handle input

func handle_input(delta: float) -> void:

	# Rotation

	var input := Vector3.ZERO

	input.y = Input.get_axis("camera_left", "camera_right")
	input.x = Input.get_axis("camera_up", "camera_down")

	camera_rotation += input.limit_length(1.0) * rotation_speed * delta
	camera_rotation.x = clamp(camera_rotation.x, -80, -10)

	# Zooming

	zoom += Input.get_axis("zoom_in", "zoom_out") * zoom_speed * delta
	zoom = clamp(zoom, zoom_maximum, zoom_minimum)
