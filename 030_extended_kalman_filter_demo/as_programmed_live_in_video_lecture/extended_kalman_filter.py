from re import X
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

        # copy initiale state estimate and
        # estimate of uncertainty about this state
        self.x = initial_x.copy()
        self.P = initial_P.copy()

        # copy noise covariance matrices
        self.Q = Q.copy()
        self.R = R.copy()

        self.est_f = est_f
        self.est_h = est_h

        self.Jacobi_f = Jacobi_f
        self.Jacobi_h = Jacobi_h

        # how large is the state space?
        self.dim = len(self.x)


    def predict(self):
        """
        EKF prediction step equations:
        x=f_est(x,u)
        P=F*P*F^t + Q
        """

        self.x = self.est_f( self.x )
        F = self.Jacobi_f(self.x)
        self.P = F @ self.P @ F.T + self.Q


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
         y=z-est_h(x)
         S=H*P*H^T + R
         K=P*H^T*S^(-1)
        """
       
        self.y = z - self.est_h(self.x)

        H = self.Jacobi_h(self.x)

        self.S = H @ self.P @ H.T + self.R
        self.K = self.P @ H.T @ np.linalg.inv(self.S)

        self.x = self.x + self.K @ self.y
        self.P = (np.eye(self.dim) - (self.K@H)) @ self.P
    
        
        

    
        
