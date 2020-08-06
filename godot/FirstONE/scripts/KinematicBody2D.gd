extends KinematicBody2D

var motion = Vector2()
const UP = Vector2(0,-1)
const GRAVITY = 23
const VELOCITY = 125

func _physics_process(_delta):
	motion.x = 0
	motion.y = 0
	
	#motion.y += GRAVITY
	
	if Input.is_action_pressed("ui_right"):
		motion.x = VELOCITY
		$Sprite.flip_h = false
		$Sprite.play("idle")
	elif Input.is_action_pressed("ui_left"):
		motion.x = -VELOCITY
		$Sprite.flip_h = true
		$Sprite.play("idle")
	if(Input.is_action_pressed("ui_up")):
		motion.y = -VELOCITY
		$Sprite.play("idle")
	elif(Input.is_action_pressed("ui_down")):
		motion.y = VELOCITY
		$Sprite.play("idle")
	else:
		$Sprite.play("idle")
		
	#if is_on_floor():
		#motion.y = GRAVITY
	#else:
		#if(motion.y < 0):
			#$Sprite.play("jump_up")
		#else: 
			#$Sprite.play("jump_down")
# warning-ignore:return_value_discarded
	move_and_slide(motion,UP)


func _on_Server_bla():
	#print("esse veio do server")
	motion.x = VELOCITY
	move_and_slide(motion,UP)
	pass # Replace with function body.
