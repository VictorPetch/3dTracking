tool
extends EditorScript
export (bool) var to_run = false
var done = false
var loops = 0
var socket = PacketPeerUDP.new()

func _run():
	
	if(socket.listen(4242) != OK):
		print("deu ruim")
		socket.close()
		print("socked closed")
	else:
		print("listening on port 4242 on localhost")
			
	
func  _physics_process(_delta):
	print("asd")
	if(socket.get_available_packet_count() > 0):
		var data = socket.get_packet().get_string_from_ascii()
		if(data == "quit"):
			done = true
		else: 
			print("data received: " + data)
	

