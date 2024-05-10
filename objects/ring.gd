extends Area3D
@export var rotation_speed := 10.0
@onready var model: MeshInstance3D = $Model


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	model.rotate_y(rotation_speed * delta)


func _on_body_entered(body: Node3D) -> void:
	if body is Player:
		Audio.play("res://sounds/coin.ogg")
		queue_free()
