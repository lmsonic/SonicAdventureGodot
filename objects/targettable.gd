class_name Targettable extends CSGSphere3D

func _ready() -> void:
	add_to_group("targettable")

func reached(player: Player) -> void:
	player.velocity.x *= 0.4
	player.velocity.z *= 0.4
	player.velocity.y = player.jump_velocity()
