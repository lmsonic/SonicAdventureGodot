extends Targettable
func on_player_entered(body: Node3D) -> void:
	super(body)
	if body is Player:
		var player := body as Player

		if player.hitbox_active:
			Audio.play("res://sounds/jump.ogg")
			player.velocity = player.velocity.bounce(global_basis.y)
			queue_free()
