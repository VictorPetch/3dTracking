import socket
import time
import simplejson as json
import numpy as np
import matplotlib.pyplot as plt
import scipy
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

msgFromClient       = "Hello UDP Server"
bytesToSend         = str.encode(msgFromClient)
serverAddressPort   = ("192.168.1.39", 4242)
bufferSize          = 140
Ax,Ay,Az = [],[],[]
K_Ax,K_Ay,K_Az = [],[],[]
Gx,Gy,Gz = [],[],[]
Vx,Vy,Vz = [0],[0],[0]
Sx,Sy,Sz = [0],[0],[0]
img_iter = 0
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPClientSocket.setblocking(0)

def plot_data(num_points,x,y,z,subplot,axisrange):
    ''' x,y,z are lists of size num_points
        subplot is a integer
        axisrange is a tuple
    '''
    if(subplot > 0):
        plt.subplot(subplot)
    #print(np.shape(x))
    plt.plot(np.linspace(0,num_points,num_points), x, 'r')
    plt.plot(np.linspace(0,num_points,num_points), y, 'g')
    plt.plot(np.linspace(0,num_points,num_points), z, 'b')
    plt.axis([0, num_points, axisrange[0],axisrange[1]])

try:
    while True: 
        #___Send First Packet___
        UDPClientSocket.sendto(bytesToSend, serverAddressPort)
        print("sent msg to server")
        starttime = time.time()
        time.sleep(0.1)
        while True:
            try:
                #__Receive data___
                msgFromServer = UDPClientSocket.recvfrom(bufferSize)
                biscoito = json.loads(str(msgFromServer[0], 'utf-8'))
                #print(biscoito['A']) 
                Ax.append(biscoito['A'][0]-4) 
                Ay.append(biscoito['A'][1]-1)
                Az.append(biscoito['A'][2]-93)
                Gx.append(biscoito['A'][3])
                Gy.append(biscoito['A'][4])
                Gz.append(biscoito['A'][5])
                #print(Ax[-1],' | ',Ay[-1])
                
                #__Kalman___
                f = KalmanFilter (dim_x=3,dim_z=3)
                f.x = np.array([[0.],    # Ax
                                [0.],    # Ay
                                [0.]])   # Az
                f.F = np.array([[1.,0.,0.], #F = A
                                [0.,1.,0.],
                                [0.,0.,1.]])
                f.H = np.array([[1.,0.,0.], #H = C
                                [0.,1.,0.],
                                [0.,0.,1.]])
                f.P = np.array([[0.05,0.,0.],
                                [0.,0.05,0.],
                                [0.,0.,0.05]])
                f.R = np.array([[0.05,0.,0.],
                                [0.,0.05,0.],
                                [0.,0.,0.05]])
                f.predict() #x,P
                f.update([[Ax[-1]], #X,P
                        [Ay[-1]],
                        [Az[-1]]])
                K_Ax.append(f.x[0])
                K_Ay.append(f.x[1])
                K_Az.append(f.x[2])
                
                #__Integrate data__
                Vx.append(Vx[-1] + (Ax[-1] + Ax[-2])/2.0) 
                Vy.append(Vy[-1] + (Ay[-1] + Ay[-2])/2.0)
                Vz.append(Vz[-1] + (Az[-1] + Az[-2])/2.0)
                Sx.append(Sx[-1] + Vx[-2] + (Ax[-2] + Ax[-1]/4.0))
                Sy.append(Sy[-1] + Vy[-2] + (Ay[-2] + Ay[-1]/4.0))
                Sz.append(Sz[-1] + Vz[-2] + (Az[-2] + Az[-1]/4.0))
                time.sleep(0.01)
            
            except:
                pass

            #___Populating the plot_____
            num_points = 100
            if(len(Ax) == num_points):
                #__Saving the figure____
                plt.figure()
                plot_data(num_points,Ax,Ay,Az,311,(-200,200))
                plot_data(num_points,Gx,Gy,Gz,312,(0,260))
                plot_data(num_points,Sx,Sy,Sz,313,(-100,100))
                plt.figure()
                plot_data(num_points,K_Ax,K_Ay,K_Az,0,(-200,200))
                print('Saving sensor data')
                plt.savefig('../plots/SensorData' + str(img_iter))

                #__FFT data__
                #N = num_points
                #yf = np.abs(scipy.fft.fft(np.asarray(Ay)))
                #xf = np.linspace(0.0, int(100/2), num_points//2)
                #plt.figure()
                #plt.plot(xf,2.0/num_points*yf[0:N//2])
                plt.show()

                #__Emptying the lists___
                Ax,Ay,Az = [],[],[]
                Gx,Gy,Gz = [],[],[]
                img_iter +=1

                #__Send another packet to stop__
                UDPClientSocket.sendto(bytesToSend, serverAddressPort)
                #time.sleep(50)
                break
        break
except Exception as e:
    print(e)
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)
    exit()   

