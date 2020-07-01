import socket
import time
import simplejson as json
import numpy as np
import matplotlib.pyplot as plt
import scipy
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import csv
import pandas as pd

msgFromClient       = "Hello UDP Server"
bytesToSend         = str.encode(msgFromClient)
serverAddressPort   = ("192.168.1.39", 4242)
bufferSize          = 140
Ax,Ay,Az = [],[],[]
K_Ax,K_Ay,K_Az = [],[],[]
Gx,Gy,Gz = [],[],[]
Vx,Vy,Vz = [0],[0],[0]
Sx,Sy,Sz = [0],[0],[0]
v_antx,v_anty,v_antz = 0,0,0
img_iter = 0
#____UDP Socket____
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPClientSocket.setblocking(0)

def kalman_param():
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
    f.P = np.array([[10,0.,0.],
                    [0.,10,0.],
                    [0.,0.,10]])
    f.R = np.array([[4,0.,0.],
                    [0.,4,0.],
                    [0.,0.,4]])
    return f
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

f = kalman_param()

try:
    while True: 
        input('Press enter do calibrate')
        #____Send First Packet____
        bytesToSend = str.encode('GO')
        UDPClientSocket.sendto(bytesToSend, serverAddressPort)
        print("sent msg to server")
        starttime = time.time()

        #____Calibrate sensor____
        print('calibrating')
        
        while True:
            try:
                #____Receive data____
                msgFromServer = UDPClientSocket.recvfrom(bufferSize)
                biscoito = json.loads(str(msgFromServer[0], 'utf-8'))
                Ax.append(biscoito['A'][0]/1000.0) 
                Ay.append(biscoito['A'][1]/1000.0)
                Az.append(biscoito['A'][2]/1000.0)
                Gx.append(biscoito['A'][3])
                Gy.append(biscoito['A'][4])
                Gz.append(biscoito['A'][5])
                
                if(len(Ax) == 100):
                    media_X = np.mean(Ax)
                    media_Y = np.mean(Ay)
                    media_Z = np.mean(Az)
                    Ax,Ay,Az = [],[],[]
                    Gx,Gy,Gz = [],[],[]
                    break
            except:
                pass

        #____Stop sending____
        bytesToSend = str.encode('STOP')
        UDPClientSocket.sendto(bytesToSend, serverAddressPort)
        deltaT = 0.01
        print('done')
        time.sleep(1)
        
        input('Press enter to collect data')
        print('collecting data')
        bytesToSend = str.encode('GO')
        UDPClientSocket.sendto(bytesToSend, serverAddressPort)
        while True:
            try:
                    #____Receive data____
                msgFromServer = UDPClientSocket.recvfrom(bufferSize)
                biscoito = json.loads(str(msgFromServer[0], 'utf-8'))
                #print(biscoito['A']) 
                Ax.append(biscoito['A'][0]/1000.0) 
                Ay.append(biscoito['A'][1]/1000.0)
                Az.append(biscoito['A'][2]/1000.0)
                Gx.append(biscoito['A'][3])
                Gy.append(biscoito['A'][4])
                Gz.append(biscoito['A'][5])
                print(Ax[-1],' | ',Ay[-1], ' | ', Az[-1])
                
                #____Kalman____
                f.predict() #x,P
                f.update([[Ax[-1]], #X,P
                        [Ay[-1]],
                        [Az[-1]]])
                K_Ax.append(f.x[0]*9.8)
                K_Ay.append(f.x[1]*9.8)
                K_Az.append(f.x[2]*9.8)
                #time.sleep(deltaT)
            except:
                pass
            
            

            #____Populating the plot____
            num_points = 100
            anterior = 0
            
            if(len(Ax) == num_points):
                for point in range(num_points-1):
                    #____Integrate data____
                    Vx.append(Vx[-1] + (K_Ax[point+1] + K_Ax[point]) * deltaT/2.0)
                    Vy.append(Vy[-1] + (K_Ay[point] + K_Ay[anterior]) * deltaT/2.0)# 
                    Vz.append(Vz[-1] + (K_Az[point] + K_Az[anterior]) * deltaT/2.0)# 
                    Sx.append(Sx[-1] + Vx[-2]*deltaT + (K_Ax[-2] + K_Ax[-1])/4.0 *(deltaT**2))
                    Sy.append(Sy[-1] + Vy[-2]*deltaT + (K_Ay[-2] + K_Ay[-1])/4.0 *(deltaT**2))
                    Sz.append(Sz[-1] + Vz[-2]*deltaT + (K_Az[-2] + K_Az[-1])/4.0 *(deltaT**2))
                #print('Vx: ', Vx)
                '''
                #____Saving the figure____
                plt.figure()
                plot_data(num_points,Ax,Ay,Az,0,(-2,2))
                plt.plot(np.linspace(0,num_points,num_points),K_Ax)
                plt.axis([0,num_points,-2*9.8,2*9.8])
                plot_data(num_points,K_Ax,K_Ay,K_Az,211,(-2*9.8,2*9.8))
                plt.plot(np.linspace(0,num_points,num_points),Vx)
                plt.axis([0,num_points,-1,1])
                plt.figure()
                plt.plot(np.linspace(0,num_points,num_points),Sx)
                plt.axis([0,num_points,-0.1,0.1])
                plot_data(num_points,Gx,Gy,Gz,412,(0,260))
                plot_data(num_points,Vx,Vy,Vz,212,(-1,1))
                plot_data(num_points,Sx,Sy,Sz,414,(-0.1,0.1))
                plt.figure()
                plt.savefig('../plots/SensorData' + str(img_iter))
                '''
                print('Saving sensor data')
                #____Saving csv____
                K_Ax = np.asarray(K_Ax)
                K_Ay = np.asarray(K_Ay)
                K_Az = np.asarray(K_Az)
                sensorData = np.empty((len(K_Ax),3))
                sensorData = np.concatenate ((K_Ax,K_Ay,K_Az),axis = 1)
                print(sensorData.shape)
                df = pd.DataFrame(sensorData,mode='a',
                                  columns=['K_Ax', 'K_Ay', 'K_Az'])
                df.to_csv(df.to_csv('../CSV/sensor.csv', index=False))
                '''
                #____FFT data____
                N = num_points
                yf = np.abs(scipy.fft.fft(np.asarray(Ay)))
                xf = np.linspace(0.0, int(100/2), num_points//2)
                plt.figure()
                plt.plot(xf,2.0/num_points*yf[0:N//2])
                plt.show()
                '''
                #____Emptying the lists____
                Ax,Ay,Az = [],[],[]
                Gx,Gy,Gz = [],[],[]
                img_iter +=1

                #____Send another packet to stop____
                bytesToSend = str.encode('STOP')
                UDPClientSocket.sendto(bytesToSend, serverAddressPort)

                break
        break
except Exception as e:
    print('Raised exception:')
    print(e)
    bytesToSend = str.encode('STOP')
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)
    exit()   

