extends Targettable
@export var spring_strength := 30.0
@export var disable_input_time := 0.1
@onready var animation_player: AnimationPlayer = $AnimationPlayer

func on_targettable_entered(body: Node3D) -> void:
	super(body)
	if body is Player:
		Audio.play("res://sounds/jump.ogg")
		animation_player.play("activate")
		var player := body as Player
		player.velocity = global_basis.y * spring_strength
		var velocity_xz := Vector2(player.velocity.z, player.velocity.x)
		if velocity_xz.length_squared() > 0.5:
			var angle := velocity_xz.angle()
			player.rotation_y = angle

			player.quaternion = Quaternion(Vector3.UP, player.rotation_y).normalized()

		player.disable_input = true
		var timer := get_tree().create_timer(disable_input_time)
		await timer.timeout
		player.disable_input = false
