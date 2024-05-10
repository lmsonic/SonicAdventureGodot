extends Targettable
@export var spring_strength := 30.0
@export_range(0, 1) var momentum_loss_ratio := 0.5
@export var disable_input_time := 0.1


func on_player_entered(body: Node3D) -> void:
	super(body)
	if body is Player:
		var player := body as Player
		Audio.play("res://sounds/jump.ogg")
		# Keep some momentum not in the spring direction
		var velocity_planar_to_spring := player.velocity - player.velocity.project(global_basis.y)
		var velocity_normal_to_spring := player.velocity.dot(global_basis.y)
		player.velocity -= velocity_planar_to_spring * momentum_loss_ratio + global_basis.y * velocity_normal_to_spring
		player.velocity += global_basis.y * spring_strength
		var velocity_xz := Vector2(player.velocity.z, player.velocity.x)
		# Rotate player towards spring direction
		if velocity_xz.length_squared() > 0.5:
			var angle := velocity_xz.angle()
			player.rotation_y = angle
			player.quaternion = Quaternion(Vector3.UP, player.rotation_y).normalized()

		# Disable player movement for a small time
		player.disable_input = true
		var timer := get_tree().create_timer(disable_input_time)
		await timer.timeout
		player.disable_input = false
