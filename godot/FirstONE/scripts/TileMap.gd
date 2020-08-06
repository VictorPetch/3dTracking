#_This code creates rectangular shapes for chosen autotiles_
tool
extends TileMap
export (bool) var to_run = false
var autotiles_id = [0,1,2,3,4,5]
var tile_size = 32

func _physics_process(_delta):
	if Engine.editor_hint:
		if to_run:
			to_run = false
			_generate_tile_collision()
			
func _generate_tile_collision():
	
	for id in autotiles_id: 
		print("Creating collision for autotile ", 
				tile_set.tile_get_name(id))
		var bla = tile_set.tile_get_region(id)
		var sprite_sheet_size = Vector2(bla.size/tile_size)
		if tile_set.tile_get_shape_count(id) == 0:
			for x in sprite_sheet_size.x:
				for y in sprite_sheet_size.y:
					var shape = ConvexPolygonShape2D.new()
					shape.points = [Vector2(0,0),Vector2(0,tile_size),
					Vector2(tile_size,tile_size),Vector2(tile_size,0)]
					tile_set.tile_add_shape(id,
											shape,
											Transform2D(0,Vector2(0,0)),
											false,
											Vector2(x,y))
		else: 
			print("There's already a shape on autotile ",
					tile_set.tile_get_name(id))

