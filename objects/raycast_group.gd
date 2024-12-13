class_name RaycastGroup extends Node3D
@export var player:Player

func _ready() -> void:
	for child:RayCast3D in get_children():
		child.add_exception(player)

func get_floor_normal() -> Vector3:
	var normal :=Vector3.ZERO
	for child:RayCast3D in get_children():
		normal += child.get_collision_normal()
	return normal.normalized()

func is_on_floor() -> bool:
	for child:RayCast3D in get_children():
		if child.is_colliding():
			var distance := child.get_collision_point().distance_to(child.global_position)
			var ray_length := child.target_position.length()
			if distance/ray_length < 1:
				return true
	return false

func get_floor_angle() -> float:
	return 0.0


func disable_ground_check() -> void:
	for child:RayCast3D in get_children():
		child.enabled = false
	var timer:= get_tree().create_timer(0.5)
	await timer.timeout
	for child:RayCast3D in get_children():
		child.enabled = true
