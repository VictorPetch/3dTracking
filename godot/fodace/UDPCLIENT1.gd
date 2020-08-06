extends Node2D

var udp_connection = PacketPeerUDP.new()

func _ready():
	udp_connection.set_dest_address("127.0.0.1", 4555)
	var pool_byte = "This is the message.".to_utf8()
	udp_connection.put_packet(pool_byte)
	set_process(true)
	
func _process(delta):
	if(udp_connection.get_available_packet_count() > 0):
		print(udp_connection.get_packet().get_string_from_utf8())
