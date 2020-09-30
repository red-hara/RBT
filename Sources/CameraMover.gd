extends Spatial

var velocity: float = PI / 8
var alpha: float = PI / 2
var beta: float = -PI / 4
var d_alpha: float = 0
var d_beta: float = 0

func _ready():
	pass

func _process(delta):
	alpha += d_alpha * delta
	beta += d_beta * delta
	transform.basis = Basis(Vector3(beta, alpha, 0))

func _unhandled_input(event):
	if event is InputEventKey:
		if event.pressed:
			if event.scancode == KEY_LEFT:
				d_alpha = -velocity
			if event.scancode == KEY_RIGHT:
				d_alpha = velocity
			if event.scancode == KEY_UP:
				d_beta = -velocity
			if event.scancode == KEY_DOWN:
				d_beta = velocity
		else:
			if event.scancode == KEY_LEFT:
				d_alpha = 0
			if event.scancode == KEY_RIGHT:
				d_alpha = 0
			if event.scancode == KEY_UP:
				d_beta = 0
			if event.scancode == KEY_DOWN:
				d_beta = 0

