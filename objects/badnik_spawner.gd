extends Node3D

@export var respawn_time:= 0.1
const BADNIK = preload("res://objects/badnik.tscn")
@onready var badnik: Area3D = $Badnik

func _ready() -> void:
	badnik.tree_exited.connect(respawn_badnik)

func respawn_badnik() -> void:
	print("respawn")
	badnik = null
	if not get_tree():
		return
	await get_tree().create_timer(respawn_time).timeout
	badnik = BADNIK.instantiate()
	add_child(badnik)
	badnik.tree_exited.connect(respawn_badnik)
