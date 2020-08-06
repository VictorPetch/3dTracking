#____Acquires the data to be used in training____
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
import os
import traceback

#____Variables____
serverAddressPort   = ("192.168.1.39", 4242)
bufferSize          = 140
Ax,Ay,Az = [],[],[]
K_Ax,K_Ay,K_Az = [],[],[]
Gx,Gy,Gz = [],[],[]
Vx,Vy,Vz = [0],[0],[0]
Sx,Sy,Sz = [0],[0],[0]
img_iter = 0

#____Writing csv header____
destination = "../CSV/sensor.csv"
filesize = os.path.getsize(destination)
if filesize == 0:
    df = pd.DataFrame(columns=['Ax', 'Ay', 'Az','Gx','Gy','Gz','class'])
    df.to_csv(df.to_csv(destination,mode='a', index=False))

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
        input('Press enter to collect data')
        print('collecting data')
        bytesToSend = str.encode('GO')
        UDPClientSocket.sendto(bytesToSend, serverAddressPort)
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
                print(Ax[-1],' | ',Ay[-1], ' | ',Az[-1],' | ',Gx[-1],' | ',Gy[-1],' | ',Gz[-1])

                print('Ax:', len(Ax))
            except Exception as e:
                pass

            #____Populating the plot____
            num_points = 100
            classe = 2

            if(len(Ax) == num_points):   
                #____Send another packet to stop____
                bytesToSend = str.encode('STOP')
                UDPClientSocket.sendto(bytesToSend, serverAddressPort)

                print('Saving sensor data')
                #____Plotting data____
                plt.figure()
                plot_data(num_points,Ax,Ay,Az,211,(-2,2))
                plot_data(num_points,Gx,Gy,Gz,212,(-250,250))
                plt.show()

                #____Movement activity detector____
                windows = input('Choose number of windows: ')
                for i in range(windows):
                    
                print('Choose the window to consider in sample number')
                sample_from = int(input('Sample start (int 1): '))
                sample_to = int(input('Sample finish (int 2): '))
                if(sample_from - sample_to == 0):
                    print('Choose a window with more than 0 samples')
                    break

                #____Saving csv____
                Ax_arr = np.asarray(Ax[sample_from:sample_to]).reshape(-1,1)
                Ay_arr = np.asarray(Ay[sample_from:sample_to]).reshape(-1,1)
                Az_arr = np.asarray(Az[sample_from:sample_to]).reshape(-1,1)
                Gx_arr = np.asarray(Gx[sample_from:sample_to]).reshape(-1,1)
                Gy_arr = np.asarray(Gy[sample_from:sample_to]).reshape(-1,1)
                Gz_arr = np.asarray(Gz[sample_from:sample_to]).reshape(-1,1)
                sensorData = np.concatenate ((Ax_arr,Ay_arr,Az_arr,Gx_arr,Gy_arr,Gz_arr,
                                              np.ones((len(Ax_arr),1))*classe),axis = 1)
                print(sensorData.shape)
                df = pd.DataFrame(sensorData)
                df.to_csv(df.to_csv(destination,mode='a', index=False,header=False))

                #____Emptying the lists____
                Ax,Ay,Az = [],[],[]
                Gx,Gy,Gz = [],[],[]
                K_Ax,K_Ay,K_Az = [],[],[] 
                img_iter +=1

                #____Get remaining packets____
                try:
                    msgFromServer = UDPClientSocket.recvfrom(bufferSize)
                except:
                    pass
                break
        
except Exception as e:
    print('Raised exception:')
    print(e)
    print(traceback.format_exc())
    bytesToSend = str.encode('STOP')
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)
    exit()   

#____Anotações____
    '''
    #____Kalman____
    f.predict() #x,P
    f.update([[Ax[-1]], #X,P
            [Ay[-1]],
            [Az[-1]]])
    K_Ax = np.append(K_Ax,f.x[0]*9.8)
    K_Ay = np.append(K_Ay,f.y[0]*9.8)
    K_Az = np.append(K_Az,f.z[0]*9.8)
    
    #____FFT data____
    N = num_points
    yf = np.abs(scipy.fft.fft(np.asarray(Ay)))
    xf = np.linspace(0.0, int(100/2), num_points//2)
    plt.figure()
    plt.plot(xf,2.0/num_points*yf[0:N//2])
    plt.show()

    for point in range(num_points-1):
        deltaT = 0.01
        #____Integrate data____
        Vx.append(Vx[-1] + (K_Ax[point+1] + K_Ax[point]) * deltaT/2.0)
        Vy.append(Vy[-1] + (K_Ay[point] + K_Ay[anterior]) * deltaT/2.0)# 
        Vz.append(Vz[-1] + (K_Az[point] + K_Az[anterior]) * deltaT/2.0)# 
        Sx.append(Sx[-1] + Vx[-2]*deltaT + (K_Ax[-2] + K_Ax[-1])/4.0 *(deltaT**2))
        Sy.append(Sy[-1] + Vy[-2]*deltaT + (K_Ay[-2] + K_Ay[-1])/4.0 *(deltaT**2))
        Sz.append(Sz[-1] + Vz[-2]*deltaT + (K_Az[-2] + K_Az[-1])/4.0 *(deltaT**2))
        
    #____Saving the figure____
    #plt.figure()
    #plot_data(num_points,Ax,Ay,Az,0,(-2,2))
    #plt.plot(np.linspace(0,num_points,num_points),K_Ax)
    #plt.axis([0,num_points,-2*9.8,2*9.8])
    #plot_data(num_points,K_Ax,K_Ay,K_Az,0,(-2*9.8,2*9.8))
    #plt.plot(np.linspace(0,num_points,num_points),Vx)
    #plt.axis([0,num_points,-1,1])
    #plt.figure()
    #plt.plot(np.linspace(0,num_points,num_points),Sx)
    #plt.axis([0,num_points,-0.1,0.1])
    #plot_data(num_points,Gx,Gy,Gz,412,(0,260))
    #plot_data(num_points,Vx,Vy,Vz,212,(-1,1))
    #plot_data(num_points,Sx,Sy,Sz,414,(-0.1,0.1))
    #plt.figure()
    #plt.savefig('../plots/SensorData' + str(img_iter))
    #plt.show()
    
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
    print('done')
    time.sleep(1)
    '''