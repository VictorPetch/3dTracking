#____Uses the data received along with a trained model____
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
import minisom
import pickle
import traceback

#____Initialize lists____
Ax,Ay,Az = [],[],[]
K_Ax,K_Ay,K_Az = [],[],[]
Gx,Gy,Gz = [],[],[]
serverAddressPort   = ("192.168.1.39", 4242)
bufferSize = 140

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
f = kalman_param()

#____Load model and markers____
som = pickle.load( open( "../models/SOM_00.p", "rb" ))
dataset = pd.read_csv('../models/markers_00.csv')
markers = dataset.iloc[:].values
SOM_size = 25
print('Load model OK')

#____UDP Socket____
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPClientSocket.setblocking(0)

try:
    #____send GO packet____
    print('Collecting data')
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
            #print(Ax[-1],' | ',Ay[-1], ' | ', Az[-1])
            
            #____Kalman____
            f.predict() #x,P
            f.update([[Ax[-1]], #X,P
                    [Ay[-1]],
                    [Az[-1]]])
            K_Ax.append(f.x[0]*9.8)
            K_Ay.append(f.x[1]*9.8)
            K_Az.append(f.x[2]*9.8)
        except Exception as e:
            pass

        #____When window is full____
        window_len = 20
        step = 5
        if(len(K_Ax) == window_len):
            data = np.concatenate((K_Ax,K_Ay,K_Az),axis = 1) 
            data = data.reshape(1,-1)
            w = som.winner(data)
            print(w)
            #print('Class: ',markers[w[0]*(SOM_size-1) + w[1] + 1][3])
            if(markers[w[0]*(SOM_size-1) + w[1] + 1][3] == '1'):
                print('ONE PUNCH')

            #___Removing 5 samples____
            K_Ax = K_Ax[step:]
            K_Ay = K_Ay[step:]
            K_Az = K_Az[step:]
            Ax = Ax[step:]
            Ay = Ay[step:]
            Az = Az[step:]
            Gx = Gx[step:]
            Gy = Gy[step:]
            Gz = Gz[step:]

            #analiza janela
            #lembrar do scaler
            #descarta os primeiros 5



except Exception as e:
    print('Raised exception:')
    print(e)
    print(traceback.format_exc())
    bytesToSend = str.encode('STOP')
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)
    exit()      