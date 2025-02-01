#!/usr/bin/env python3

import numpy as np

class ExtendedKalmanFilter:
    def __init__(self, initState, initCov, processNoise, measurementNoise):
        self.state = initState
        self.P = initCov
        self.Q = processNoise
        self.R = measurementNoise
        self.H = np.zeros((len(initState), len(initState)))
        self.dState = np.zeros(len(initState))
        self.R_matrix = np.array([[np.cos(self.state[7])*np.cos(self.state[8]), np.cos(self.state[7])*np.sin(self.state[8]), -np.sin(self.state[7])], \
                            [np.sin(self.state[6])*np.sin(self.state[7])*np.cos(self.state[8])-np.cos(self.state[6])*np.sin(self.state[8]), np.sin(self.state[6])*np.sin(self.state[7])*np.sin(self.state[8])+np.cos(self.state[6])*np.cos(self.state[8]), np.sin(self.state[6])*np.cos(self.state[7])], \
                            [np.cos(self.state[6])*np.sin(self.state[7])*np.cos(self.state[8])+np.sin(self.state[6])*np.sin(self.state[8]), np.cos(self.state[6])*np.sin(self.state[7])*np.sin(self.state[8])-np.sin(self.state[6])*np.cos(self.state[8]), np.cos(self.state[6])*np.cos(self.state[7])]])

    def computeJacobian(self, imuAccelaration, imuAngularVelocity):
        Jacob_F = np.array(np.zeros((len(self.state), len(self.state))))

        Jacob_F[0, 3] = 1  # ∂x/∂vx
        Jacob_F[1, 4] = 1  # ∂y/∂vy
        Jacob_F[2, 5] = 1  # ∂z/∂vz
        
        Jacob_F[3, 7] = np.array([-np.cos(self.state[8]) * np.sin(self.state[7]), -np.sin(self.state[8]) + np.sin(self.state[7]), -np.cos(self.state[7])]) @ imuAccelaration   # ∂vx/∂theta
        Jacob_F[3, 8] = np.array([-np.sin(self.state[8]) * np.cos(self.state[7]), np.cos(self.state[8]) * np.cos(self.state[7]), 0]) @ imuAccelaration # ∂vx/∂psi
        
        Jacob_F[4, 6] = np.array([(np.cos(self.state[6]) * np.sin(self.state[7]) * np.cos(self.state[8]) + np.sin(self.state[6]) * np.sin(self.state[8])), (np.cos(self.state[6]) * np.sin(self.state[7]) * np.sin(self.state[8]) - np.sin(self.state[6]) * np.cos(self.state[8])), np.cos(self.state[6]) * np.cos(self.state[7])]) @ imuAccelaration  # ∂vy/∂phi
        Jacob_F[4, 7] = np.array([np.sin(self.state[6]) * np.cos(self.state[7]) * np.cos(self.state[8]), np.sin(self.state[6]) * np.cos(self.state[7]) * np.sin(self.state[8]),-np.sin(self.state[6]) * np.sin(self.state[8])]) @ imuAccelaration  # ∂vy/∂theta
        Jacob_F[4, 8] = np.array([(np.sin(self.state[6]) * np.sin(self.state[7]) * np.sin(self.state[8]) + np.cos(self.state[6]) * np.cos(self.state[8])), (np.sin(self.state[6]) * np.sin(self.state[7]) * np.cos(self.state[8]) - np.cos(self.state[6]) * np.sin(self.state[8])),0]) @ imuAccelaration  # ∂vy/∂psi
        
        Jacob_F[5, 6] = np.array([(-np.sin(self.state[6]) * np.sin(self.state[7]) * np.cos(self.state[8])) + (np.cos(self.state[6]) * np.sin(self.state[8])), (-np.sin(self.state[6]) * np.sin(self.state[7]) * np.cos(self.state[8]) - np.cos(self.state[6]) * np.cos(self.state[8])), -np.sin(self.state[6]) * np.cos(self.state[7])]) @ imuAccelaration  # ∂vz/∂phi
        Jacob_F[5, 7] = np.array([np.cos(self.state[6]) * np.cos(self.state[7]) * np.cos(self.state[8]), np.cos(self.state[6]) * np.cos(self.state[7]) * np.sin(self.state[8]), - np.cos(self.state[6]) * np.sin(self.state[7])]) @ imuAccelaration  # ∂vz/∂theta
        Jacob_F[5, 8] = np.array([(np.cos(self.state[6]) * np.sin(self.state[7]) * np.cos(self.state[8]) - np.sin(self.state[6]) * np.cos(self.state[8])),-(np.cos(self.state[6]) * np.sin(self.state[7]) * np.cos(self.state[8]) + np.sin(self.state[6]) * np.sin(self.state[8])),0]) @ imuAccelaration  # ∂vz/∂psi
        
        Jacob_F[6, 6] = imuAngularVelocity[1] * np.cos(self.state[6]) * np.tan(self.state[7]) - imuAngularVelocity[2] * np.sin(self.state[6]) * np.tan(self.state[7]) # ∂phi/∂phi
        Jacob_F[6, 7] = imuAngularVelocity[1] * np.sin(self.state[6]) / np.cos(self.state[7])**2 + imuAngularVelocity[2] * np.cos(self.state[6]) / np.cos(self.state[7])**2 # ∂phi/∂theta
        
        Jacob_F[7, 6] = -imuAngularVelocity[1] * np.sin(self.state[6]) - imuAngularVelocity[2] * np.cos(self.state[6]) # ∂theta/∂phi
        
        Jacob_F[8, 6] = imuAngularVelocity[1] * np.cos(self.state[6]) / np.cos(self.state[7]) - imuAngularVelocity[2] * np.sin(self.state[6]) / np.cos(self.state[7]) # ∂psi/∂phi
        Jacob_F[8, 7] = -imuAngularVelocity[1] * np.sin(self.state[6]) * np.sin(self.state[7]) / np.cos(self.state[7])**2 - imuAngularVelocity[2] * np.cos(self.state[6]) * np.sin(self.state[7]) / np.cos(self.state[7])**2 # ∂psi/∂theta

        return Jacob_F
    
    def predict(self, dt, imuAccelaration, imuAngularVelocity, g):

        self.dState = np.array([self.state[3] , \
                                self.state[4] , \
                                self.state[5] , \
                                self.R_matrix[0] @ imuAccelaration, \
                                self.R_matrix[1] @ imuAccelaration, \
                                self.R_matrix[2] @ imuAccelaration - g, \
                                imuAngularVelocity[0] + (imuAngularVelocity[1] * np.sin(self.state[6]) * np.tan(self.state[7])) + (imuAngularVelocity[2] * np.cos(self.state[6]) * np.tan(self.state[7])), \
                                imuAngularVelocity[1] * np.cos(self.state[6]) - imuAngularVelocity[2] * np.sin(self.state[6]), \
                                imuAngularVelocity[1] * np.sin(self.state[6]) / np.cos(self.state[7]) + imuAngularVelocity[2] * np.cos(self.state[6]) / np.cos(self.state[7]) ])
        
        self.statePrior = self.state + self.dState * dt
        self.Jacob_F = self.computeJacobian(imuAccelaration, imuAngularVelocity)
        self.F_d = np.eye(len(self.statePrior)) + self.Jacob_F * dt
        self.PPrior = self.F_d @ self.P @ self.F_d.T + self.Q

        self.state = self.statePrior
        self.P = self.PPrior

        return self.state, self.P
    
    def update(self, measurement):
        self.H[0, 0] = 1  
        self.H[1, 1] = 1      
        self.H[2, 2] = 1
        self.H[3, 3] = 1
        self.H[4, 4] = 1
        self.H[5, 5] = 1

        #Measurement covariance
        S = self.R + self.H @ self.PPrior @ self.H.T

        #Kalman Gain
        K = self.PPrior @ self.H.T @ np.linalg.inv(S)

        #State update
        self.statePosterior = self.statePrior + K @ (measurement - self.H @ self.statePrior)

        #Covariance update
        self.PPosterior = (np.eye(len(self.statePrior)) - K @ self.H) @ self.PPrior

        self.state = self.statePosterior
        self.P = self.PPosterior

        return self.state, self.P


#Initializations
# initState = np.zeros(9)
# initCov = np.eye(9)
# processNoise = np.diag([3, 3, 3, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01])
# measurementNoise = np.diag([5**2, 5**2, 10**2, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
# imuAccelaration = np.zeros(3)
# imuAngularVelocity = np.zeros(3)
# g = 9.81

# ekf = ExtendedKalmanFilter(initState, initCov, processNoise, measurementNoise)
# ekf.predict(0.1, imuAccelaration, imuAngularVelocity, g)
# ekf.update(np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]))

