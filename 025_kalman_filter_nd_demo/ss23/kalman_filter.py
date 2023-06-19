import numpy as np

class kalman_filter:

    def __init__(self,
                 initial_x,
                 initial_P,
                 F, B, Q, H, R):
        self.x = initial_x.copy()
        self.P = initial_P.copy()

        self.F = F.copy()
        self.B = B.copy()
        self.Q = Q.copy()
        self.H = H.copy()
        self.R = R.copy()

        self.dim = len(self.x)

    def predict(self, u):

        self.x = self.F @ self.x + self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        
    
    def correct_prediction_using_measurement(self, z):

        self.y = z - self.H@self.x
        self.S = self.H @ self.P @ self.H.T + self.R
        self.K = self.P @ self.H.T @ np.linalg.inv(self.S)
        
        self.x = self.x + self.K @ self.y
        self.P = (np.eye(self.dim) - self.K@self.H) @ self.P

    
    def get_scalar_measure_of_uncertainty_about_state(self):
        return np.linalg.det( self.P )