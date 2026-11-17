import numpy as np



# TODO Part 3: Comment the code explaining each part
# EKF with state x = [x, y, th, w, v, vdot] and measurement z = [v, w, ax, ay] (v, w from /odom; ax, ay from /imu).
# predict(): move the estimate with the motion model. update(): correct it with the measurement.
class kalman_filter:

    # TODO Part 3: Initialize the covariances and the states
    def __init__(self, P,Q,R, x, dt):

        self.P=P    # 6x6 state covariance (uncertainty of the estimate)
        self.Q=Q    # 6x6 process noise covariance (trust in the motion model)
        self.R=R    # 4x4 measurement noise covariance (trust in the sensors)
        self.x=x    # state estimate
        self.dt = dt    # time step (s)

    # TODO Part 3: Replace the matrices with Jacobians where needed
    def predict(self):

        # A linearizes f at the previous estimate
        self.A = self.jacobian_A()

        # x = f(x)
        self.motion_model()

        # C linearizes h at the predicted state, used in update()
        self.C = self.jacobian_H()

        # P = A P A' + Q
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Calculate the Kalman gain and correct your estimates
    def update(self, z):

        # residual covariance S = C P C' + R
        S = np.dot(np.dot(self.C, self.P), self.C.T) + self.R

        # gain K = P C' S^-1: how much each residual corrects each state
        kalman_gain= np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))

        # residual z - h(x)
        y_bar = z - self.measurement_model()

        # x = x + K (z - h(x))
        self.x= self.x + np.dot(kalman_gain, y_bar)

        # P = (I - K C) P
        self.P= np.dot(np.eye(len(self.x)) - np.dot(kalman_gain, self.C), self.P)


    # TODO Part 3: Implement here the measurement model
    # h(x): expected measurement. IMU is in the robot frame: ax = vdot (tangential), ay = v*w (centripetal).
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,# v
            w,# w
            vdot, # ax
            v*w, # ay
        ])

    # TODO Part 3: Impelment the motion model (state-transition matrice)
    # f(x): unicycle kinematics, forward Euler over dt, w and vdot held constant.
    def motion_model(self):

        x, y, th, w, v, vdot = self.x
        dt = self.dt

        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])




    # A = df/dx (6x6)
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt

        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0, -v*np.sin(th)*dt, 0, np.cos(th)*dt,  0],
            [0, 1,  v*np.cos(th)*dt, 0, np.sin(th)*dt,  0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])


    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)
    # C = dh/dx (4x6). Only the ay = v*w row depends on the state. x, y, theta columns are zero (not measured).
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0], # ay
        ])

    # TODO Part 3: return the states here
    def get_states(self):
        return self.x
