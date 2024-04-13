extends Area3D

var time := 0.0
var grabbed := false

# Collecting coins
@onready var particles: GPUParticles3D = $Particles

func _on_body_entered(body:PhysicsBody3D) -> void:
	if body.has_method("collect_coin") and !grabbed:
		# allow
		body.call("collect_coin")

		Audio.play("res://sounds/coin.ogg") # Play sound

		$Mesh.queue_free() # Make invisible
		particles.emitting = false # Stop emitting stars

		grabbed = true

# Rotating, animating up and down

func _process(delta:float)-> void:

	rotate_y(2 * delta) # Rotation
	position.y += (cos(time * 5) * 1) * delta # Sine movement

	time += delta
