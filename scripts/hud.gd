extends CanvasLayer
@onready var coins_label: Label = $Coins
func _on_coin_collected(coins: int) -> void:
	coins_label.text = str(coins)
