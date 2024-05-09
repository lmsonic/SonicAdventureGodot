class_name Targettable extends Area3D

func _ready() -> void:
	add_to_group("targettable")
	body_entered.connect(on_targettable_entered)

func on_targettable_entered(body: Node3D) -> void:
	if body is Player:
		var player := body as Player
		player.entered_targettable()
