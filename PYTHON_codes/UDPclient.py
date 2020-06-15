import socket
import time
import simplejson as json
import numpy as np
import matplotlib.pyplot as plt

msgFromClient       = "Hello UDP Server"
bytesToSend         = str.encode(msgFromClient)
serverAddressPort   = ("192.168.1.39", 4242)
bufferSize          = 140
bufferAx,bufferAy,bufferAz = [],[],[]
bufferGx,bufferGy,bufferGz = [],[],[]
buffer = []
iterator = 0

UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPClientSocket.setblocking(0)



while True:
    #___Send First Packet___
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)
    print("sent msg to server")
    starttime = time.time()
    time.sleep(0.1)
    while True:
        try:
            msgFromServer = UDPClientSocket.recvfrom(bufferSize)
            #msg = "Message from Server {}".format(str(msgFromServer[0]))
            biscoito = json.loads(str(msgFromServer[0], 'utf-8'))
            #print(len(buffer))
            #print(biscoito['A'])  
            buffer.append(biscoito['A'])
            time.sleep(0.01)
            print(buffer[0][6])

        except KeyboardInterrupt:
            UDPClientSocket.sendto(bytesToSend, serverAddressPort)
            exit()
        except:
            pass

        #___Populating the buffer to plot___
        num_points = 10
        if(len(buffer) == num_points):

            plt.figure()
            plt.subplot(211)
            plt.plot(np.linspace(0,num_points,num_points), buffer[:][0], 'r')
            plt.plot(np.linspace(0,num_points,num_points), buffer[:][1], 'g')
            plt.plot(np.linspace(0,num_points,num_points), buffer[:][2], 'b')
            plt.axis([0, 200, -200,200 ])
            #plt.subplot(212)
            #plt.plot(np.linspace(0,num_points,num_points),buffer[3],'r')
            #plt.plot(np.linspace(0,num_points,num_points),buffer[4],'g')
            #plt.plot(np.linspace(0,num_points,num_points),buffer[5][5],'b')
            #plt.axis([0,200,0,260])
            plt.savefig('../plots/bla' + str(iterator))

            bufferAx,bufferAy,bufferAz = [],[],[]
            bufferGx,bufferGy,bufferGz = [],[],[]
            buffer = []
            iterator +=1

            UDPClientSocket.sendto(bytesToSend, serverAddressPort)
            time.sleep(50)
            break

    

