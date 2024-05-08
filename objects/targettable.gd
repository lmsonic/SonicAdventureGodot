class_name Targettable extends CSGSphere3D

func _ready() -> void:
	add_to_group("targettable")

func reached(player: Player) -> void:
	player.speed *= 0.4
	player.local_y_momentum.x = 0.0
	player.local_y_momentum.z = 0.0

	player.local_y_momentum.y = player.jump_velocity()
