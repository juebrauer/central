import numpy as np

class kalman_filter:

    def __init__(self,
                 initial_x,
                 initial_P,
                 F,
                 B,
                 Q,
                 H,
                 R):
        self.x = initial_x
        self.P = initial_P
        self.F = F
        self.B = B
        self.Q = Q
        self.H = H
        self.R = R

        self.dim = len(self.x)

    def predict(self, u):
        self.x = self.F @ self.x + self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q

    def correct_prediction_using_measurement(self, z):
        self.y = z - (self.H @ self.x)
        self.S = self.H @ self.P @ self.H.T + self.R
        self.K = self.P @ self.H.T @ np.linalg.inv( self.S )

        self.x = self.x + self.K @ self.y
        self.P = (np.eye(self.dim) - (self.K @ self.H)) @ self.P

    

        