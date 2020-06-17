from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

f = KalmanFilter (dim_x=2, dim_z=1)
f.x = np.array([[0.],    # position
                [0.]])   # velocity
f.F = np.array([[1.,1.],
                [0.,1.]])
f.H = np.array([[1.,0.]])
f.P = np.array([[1000.,    0.],
                [   0., 1000.] ])
f.R = np.array([[.05,0.,0.],
                [0.,1.,0.],
                [0.,0.,1.]])

while True:
    z, R = read_sensor()
    x, P = predict(x, P, F, Q)
    x, P = update(x, P, z, R, H)