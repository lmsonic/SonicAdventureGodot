class_name RaycastGroup extends Node3D
@export var player:Player

func _ready() -> void:
	for child:RayCast3D in get_children():
		child.add_exception(player)


func get_floor_normal() -> Vector3:
	var normal :=Vector3.ZERO
	for child:RayCast3D in get_children():
		child.force_raycast_update()
		normal += child.get_collision_normal()
	return normal.normalized()
