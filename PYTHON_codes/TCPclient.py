import socket

HOST = socket.gethostname()
print(HOST)
PORT = 4555        # The port used by the server
#SOCK_STREAM IS TCP
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    while(True):
        print("connecting")
        s.connect((HOST, PORT))
        print('Connected to ', HOST)
        while(True):
            #data = input()
            #s.sendall(data.encode())
            data = s.recv(33)
            #data = data.decode()
            print(data)