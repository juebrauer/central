from re import X
import numpy as np

class kalman_filter:

    def __init__(self, initial_x, initial_P, F, B, Q, H, R):

        # copy initiale state estimate and
        # estimate of uncertainty about this state
        self.x = initial_x.copy()
        self.P = initial_P.copy()

        # copy linear transition and measurement "models"
        self.F = F.copy()
        self.B = B.copy()
        self.Q = Q.copy()
        self.H = H.copy()
        self.R = R.copy()

        # how large is the state space?
        self.dim = len(self.x)


    def predict(self, u):

        """"
        KF prediction step equations:
        x=F*x+B*u
        P=F*P*F^t + Q
        """

        self.x = self.F @ self.x  +  self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q


    def get_scalar_measure_of_uncertainty_about_state(self):

        """
        See:
        What does Determinant of Covariance Matrix give?
        
        https://stats.stackexchange.com/questions/110955/what-does-determinant-of-covariance-matrix-give
        --> "The determinant of the covariance matrix is the generalized variance.
            This means it is like a scalar variance when the dimension is 1. Thus, A is more dispersed."
        and

        https://math.stackexchange.com/questions/889425/what-does-determinant-of-covariance-matrix-give
        --> # "The larger |Σ|, the more are your data points dispersed"

        """
        return np.linalg.det( self.P )


    def correct_prediction_using_measurement(self, z):
        
        """
        KF correction step equations:
        x=x+K*y
        P=(I-KH)*P
        with:
         y=z-H*x
         S=H*P*H^T + R
         K=P*H^T*S^(-1)
        """

        self.y = z - (self.H @ self.x)
        self.S = self.H @ self.P @ self.H.T + self.R
        self.K = self.P @ self.H.T @ np.linalg.inv(self.S)

        self.x = self.x + self.K @ self.y
        self.P = (np.eye(self.dim) - (self.K@self.H)) @ self.P
        
        

    
        
