import numpy as np

HOME_OP = np.array([0.484, 0.128, 0.609, 0.5, 0.5, 0.5, 0.5])
SCAN_START_OP = np.array([-0.129, 0.487, 0.422, 0.0017937315462992396, 0.9999976506446824, 0.00035651772527578343, -0.001163669784592465])

class Trajectory():
    def __init__(self, dt=1/100) -> None:
        self.dt = dt

    def linear_polynomial(self, ti, tf, qi, qf):
        # linearly interpolate between qi and qf
        dq = qf - qi
        dt = tf - ti
        # Compute the coefficients
        a0 = qi
        a1 = (dq / dt)
        # Generate the profile
        t = np.linspace(0, dt, dt//self.dt)
        q = a0 + a1*t
        t += ti
        
        return t, q
    
    def rectilinear_motion_primitive(self, u, qi, qf):
        # Generate a rectilinear motion primitive between qi and qf, parametrized by u
        dq = qf - qi
        q = qi + u[:, np.newaxis]*dq
        
        return q
    
    def circular_motion_primitive(self, u, P, C):
        # Generate the circular motion primitive defined by a starting point P, center c, parametrized by u
        direction_vector = P - C
        radius = np.linalg.norm(direction_vector)
        direction_vector = direction_vector / radius
        q = np.zeros((u.shape[0], P.shape[0]))
        for i, theta in enumerate(u):
            q[i,:] = C + radius * (np.cos(theta + np.pi/2) * direction_vector + np.sin(theta + np.pi/2) * np.cross(direction_vector, [0,-1,0]))
        return q
    
if __name__ == '__main__':
    traj = Trajectory()
    # Scanning trajectory parameters
    starting_point = np.array([0.484, 0.128, 0.609]) # Trajectory start
    center = np.array([0,0.128,0]) # Center for circular primitives
    displacement = 0.2 # Distance to cover during rectilinear motion
    rectlinear_dt = 0.5 # Time duration for rectilinear motion
    circular_dt = 1.5 # Time for circular motion
    # First arc
    t1, u1 = traj.linear_polynomial(0, circular_dt, -np.pi/2, np.pi/2)
    q1 = traj.circular_motion_primitive(u1, starting_point, center)
    # First line
    displacement_vector = displacement * (q1[-1,:] - center) / np.linalg.norm(q1[-1,:] - center)
    t2, u2 = traj.linear_polynomial(t1[-1], t1[-1] + rectlinear_dt, 0, 1)
    q2 = traj.rectilinear_motion_primitive(u2, q1[-1,:], q1[-1,:] + displacement_vector)
    # Second arc
    t3, u3 = traj.linear_polynomial(t2[-1], t2[-1] + circular_dt, 3*np.pi/2, np.pi/2)
    q3 = traj.circular_motion_primitive(u3, q2[-1,:], center)
    # Second line
    displacement_vector = displacement * (q3[-1,:] - center) / np.linalg.norm(q3[-1,:] - center)
    t4, u4 = traj.linear_polynomial(t3[-1], t3[-1] + rectlinear_dt, 0, 1)
    q4 = traj.rectilinear_motion_primitive(u4, q3[-1,:], q3[-1,:] + displacement_vector)
    # Final arc
    _, u5 = traj.linear_polynomial(t4[-1], t4[-1] + circular_dt, -np.pi/2, np.pi/2)
    q5 = traj.circular_motion_primitive(u5, q4[-1,:], center)

    q = np.concatenate((q1,q2,q3,q4,q5))

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    plt.figure()
    ax = plt.subplot(projection='3d')
    ax.plot(q[:,0],q[:,1],q[:,2])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()