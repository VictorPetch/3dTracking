extends Node2D

var PORT_CLIENT = 4242
var PORT_SERVER = 4242
var IP_SERVER = "127.0.0.1"
var IP_CLIENT = "192.168.1.39"
var socketUDP = PacketPeerUDP.new()

func _ready():
	if (socketUDP.listen(PORT_SERVER,IP_CLIENT) != OK):
		printt("Error listening on port: " + str(PORT_SERVER))
	else:
		printt("Listening on port: " + str(PORT_SERVER))

func _process(delta):   

	if socketUDP.get_available_packet_count() > 0:
		socketUDP.set_dest_address(IP_CLIENT, PORT_CLIENT)
		var array_bytes = socketUDP.get_packet()
		#IP_CLIENT = socketUDP.get_packet_ip()
		PORT_CLIENT = socketUDP.get_packet_port()
		print("msg server: " + array_bytes.get_string_from_ascii())
		socketUDP.set_dest_address(IP_CLIENT, PORT_CLIENT)
		var stg = "hi coco!"
		var pac = stg.to_ascii()
		socketUDP.put_packet(pac)
func _exit_tree():
	socketUDP.close()
