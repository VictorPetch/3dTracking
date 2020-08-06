extends Node

var connection = null
var peerstream = null
var values = [
	"test1", "test2", "test3"
]
var test = [null,null,null,null,null,null] 
var error = null
func _ready():
	print("Start client TCP")
	# Connect
	connection = StreamPeerTCP.new()
	print('Connecting to host')
	while error != 0:
		error = connection.connect_to_host("192.168.1.39", 80)
		#OS.delay_msec(500)
	print('Connected')
	peerstream = PacketPeerStream.new()
	peerstream.set_stream_peer(connection)

func _process(delta):
	if connection.is_connected_to_host():
		var number_of_bytes = connection.get_available_bytes()
		if number_of_bytes > 0 :
			print("number of bytes:",number_of_bytes)
			test = connection.get_string(number_of_bytes)
			#test[1] = connection.get_string(5)
			#test[2] = connection.get_string(5)
			##test[3] = connection.get_string(4)
			#test[4] = connection.get_string(4)
			#test[5] = connection.get_string(4)
			print(test)
		#if values.size() > 0 :
			#print(values.pop_front())
			#peerstream.put_var(values.pop_front())
