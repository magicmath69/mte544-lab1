

import numpy as np



# TODO Part 3: Comment the code explaining each part
# EKF:
# state x = [x, y, th, w, v, vdot]: pos (m), heading (rad), yaw rate (rad/s), forward speed (m/s), forward acceleration (m/s^2), in odometry frame.
# measurement z = [v, w, ax, ay]: v and w from /odom, ax and ay from /imu.
# step 1: predict() state updated by motion model
# step 2: update() corrects it with the measurements (weighted by covariance).
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        
        self.P=P    # 6x6 state covariance: uncertainty of current state estimate
        self.Q=Q    # 6x6 process noise covariance: how much the real motion can differ from the model in a step
        self.R=R    # 4x4 measurement noise covariance: noise of each sensor
        self.x=x    # state estimate [x, y, th, w, v, vdot]
        self.dt = dt    # time step between filter updates (s)
        
    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        # motion/measurement models are nonlinear, so the EKF uses Jacobians wherever the linear KF uses its A and C matrices.
        # A is evaluated before the motion step
        self.A = self.jacobian_A()
        
        # predicted state: x = f(x)
        self.motion_model()

        # C is evaluated at the predicted state, after the motion step, because update() linearizes h there.
        self.C = self.jacobian_H()
        
        # predicted covariance: old uncertainty passed through motion model, plus process noise
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Calculate the Kalman gain and correct your estimates
    def update(self, z):
        
        # S = C P C' + R: covariance of the measurement residual (predicted uncertainty plus sensor noise)
        S = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        
        # K = P C' S^-1 (6x4): how much of each measurement residual goes into each state (how much it influences the estimate)
        # large P (prediction uncertainty) or small R (trusted sensor) -> large gain.
        kalman_gain= np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        
        # residual y = z - h(x): measured minus what the prediction says we should measure
        y_bar = z - self.measurement_model()

        # corrected state: prediction plus the gain-weighted residual
        self.x= self.x + np.dot(kalman_gain, y_bar)

        # corrected covariance: P = (I - K C) P
        self.P= np.dot(np.eye(len(self.x)) - np.dot(kalman_gain, self.C), self.P)
        
    
    # TODO Part 3: Implement here the measurement model
    # h(x): predicts measurements based on current state
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,# v
            w,# w
            vdot, # ax
            v*w, # ay
        ])
        
    # TODO Part 3: Impelment the motion model (state-transition matrice)
    # f(x): kinematics over one step dt (forward Euler), assuming w and vdot stay constant during the step. Any real change is accounted for by process noise Q.
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
        


    
    # A = df/dx (6x6), the motion model linearized at current estimate.
    # th column spreads heading uncertainty into x and y; v column spreads speed uncertainty along travel direction.
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
    # C = dh/dx (4x6).
    # ay = v*w row is nonlinear
    # d(ay)/dw = v, d(ay)/dv = w.
    # the x, y and th columns are all zero because no sensor measures position or heading
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
    # current estimate [x, y, th, w, v, vdot]
    def get_states(self):
        return self.x
