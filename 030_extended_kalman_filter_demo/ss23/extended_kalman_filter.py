import numpy as np

class extended_kalman_filter:

    def __init__(self,
                 initial_x,
                 initial_P,
                 Q,
                 R,
                 est_f,
                 est_h,
                 Jacobi_f,
                 Jacobi_h):
        self.x = initial_x.copy()
        self.P = initial_P.copy()

        self.Q = Q.copy()
        self.R = R.copy()

        self.est_f = est_f
        self.est_h = est_h
        self.Jacobi_f = Jacobi_f
        self.Jacobi_h = Jacobi_h

        self.dim = len(self.x)

    def predict(self):

        self.x = self.est_f(self.x)
        F = self.Jacobi_f(self.x)
        self.P = F @ self.P @ F.T + self.Q
        
        
    
    def correct_prediction_using_measurement(self, z):

        self.y = z - self.est_h(self.x)
        H = self.Jacobi_h(self.x)

        self.S = H @ self.P @ H.T + self.R
        self.K = self.P @ H.T @ np.linalg.inv(self.S)
        
        self.x = self.x + self.K @ self.y
        self.P = (np.eye(self.dim) - self.K@H) @ self.P

    
    def get_scalar_measure_of_uncertainty_about_state(self):
        return np.linalg.det( self.P )