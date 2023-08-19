import numpy as np

class Trajectory():
    def __init__(self, dt=1/100) -> None:
        self.dt = dt

    def linear_polynomial(self, ti, tf, qi, qf):
        # linearly interpolate between qi and qf
        dq = qf - qi
        dt = tf - ti
        # Compute the coefficients
        a0 = qi
        a1 = dq / dt
        # Generate the profile
        time = np.linspace(0, dt, dt//self.dt)
        q = a0 + a1*time
        
        return q
    
    def rectilinear_motion_primitive(self, u, qi, qf):
        # Generate a rectilinear motion primitive between qi and qf, parametrized by u
        dq = qf - qi
        q = qi + u[:, np.newaxis]*dq
        
        return q
    
    def circular_motion_primitive(self, u, P, c, r):
        # Generate the circular motion primitive defined by a starting point P, center c and axis of rotation r, parametrized by u
        x = P - c
        y = np.cross(r, x)
        R = np.vstack((x, y, r)).T
        r0 = np.linalg.norm(x)
        q = c.reshape(-1,1) + R @ np.vstack((r0*np.cos(u), r0*np.sin(u), np.zeros_like(u)))

        return q