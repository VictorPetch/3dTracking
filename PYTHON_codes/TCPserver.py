import socket

HOST = ''  # Standard loopback interface address (localhost)
PORT = 4555       # Port to listen on (non-privileged ports are > 1023)
#SOCK_STREAM IS TCP
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

    s.bind((HOST, PORT))
    s.listen()
    print('listening..')
    conn, addr = s.accept()
    print('Connected by', addr)
    while True:
        #input()
        data = conn.recv(1024)
        data = data.decode('utf-8')
        conn.sendall(b'Mensagem recebida')
        print(data)
        
