extends Node2D
var socket = PacketPeerUDP.new()
var bla = 0
func _init():
	socket.set_dest_address("192.168.1.39",4242)
	socket.put_packet("quit".to_ascii())
	print("Exiting application")  
func _process(delta):     
	if socket.get_available_packet_count() > 0:
		var array_bytes = socket.get_packet()
		#print("msg server: " + array_bytes.get_string_from_ascii())
		print(bla)
		bla +=1
