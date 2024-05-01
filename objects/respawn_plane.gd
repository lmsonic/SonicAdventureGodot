extends Area3D


func _on_body_entered(body: Node3D) -> void:
	await get_tree().process_frame
	await get_tree().reload_current_scene()
