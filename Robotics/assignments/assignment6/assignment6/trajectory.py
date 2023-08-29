import copy
import numpy as np

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

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
    
    def rectilinear_motion_primitive(self, u, qi: PoseStamped, qf: PoseStamped):
        # Extract the position and orientation  
        pos_i = np.array([qi.pose.position.x,qi.pose.position.y,qi.pose.position.z])
        rot_i = np.array([qi.pose.orientation.x,qi.pose.orientation.y,qi.pose.orientation.z,qi.pose.orientation.w])
        pos_f = np.array([qf.pose.position.x,qf.pose.position.y,qf.pose.position.z])
        rot_f = np.array([qf.pose.orientation.x,qf.pose.orientation.y,qf.pose.orientation.z,qf.pose.orientation.w])
        # Generate a rectilinear motion primitive between qi and qf, parametrized by u
        dq = pos_f - pos_i
        q = []
        for t in u:
            x, y, z = pos_i + t*dq
            qx, qy, qz, qw = self.slerp(rot_i, rot_f, t)
            q.append(PoseStamped(pose=Pose(position=Point(x=x,y=y,z=z),orientation=Quaternion(x=qx,y=qy,z=qz,w=qw))))
        return q
    
    def geodesic(self, P1, P2, P0, du = np.pi/24):
        # Compute the radius
        R0 = np.linalg.norm(P1 - P0) + 0.075 # Offset to avoid the robot colliding with the spheres
        # Compute the radius vectors
        R1 = self.normalize(P1 - P0)
        R2 = self.normalize(P2 - P0)
        # Normal of the circle through P1 and P2
        R = np.cross(R1, R2)
        R = self.normalize(R)
        # Collect the three direction
        Rt = np.array([R1, np.cross(R, R1), R]).T
        # Compute rotations
        Ri = np.array([np.cross(R, -R1), R, -R1]).T
        Rf = np.array([np.cross(R, -R2), R, -R2]).T
        Rif = Ri.T @ Rf
        # Angle from P1 to P2
        uf = np.arccos((Rif[0,0] + Rif[1,1] + Rif[2,2] - 1)/2)
        # Compute the geodesic
        u = np.arange(0, uf, du)
        G = P0.reshape(-1,1) + Rt @ np.hstack((R0*np.cos(u).reshape(-1,1),R0*np.sin(u).reshape(-1,1), np.zeros((u.shape[0],1)))).T
        G = G.T
        # Compute the Frenet frame of each point
        t = Rt @ np.hstack((-R0*np.sin(u).reshape(-1,1),R0*np.cos(u).reshape(-1,1), np.zeros((u.shape[0],1)))).T
        for i in range(t.shape[1]):
            t[:, i] = self.normalize(t[:,i])
        n = Rt @ np.hstack((-R0*np.cos(u).reshape(-1,1),-R0*np.sin(u).reshape(-1,1), np.zeros((u.shape[0],1)))).T
        for i in range(n.shape[1]):
            n[:, i] = self.normalize(n[:,i])
        b = np.empty_like(t)
        for i in range(t.shape[1]):
            b[:, i] = np.cross(t[:, i], n[:, i])
        # Convert the frames to quaternions and compose the resulting trajectory
        q = []
        for i in range(t.shape[1]):
            x, y, z = G[i,:]
            R = np.array([b[:,i],t[:,i],n[:,i]]).T
            qx, qy, qz, qw = self.rotation_matrix_to_quaternion(R)
            q.append(PoseStamped(pose=Pose(position=Point(x=x,y=y,z=z),
                                      orientation=Quaternion(x=qx,y=qy,z=qz,w=qw))))

        return q

    #-------------------------------------------------------------------------------------------------------------------
    #
    #   Utility functions
    #
    #-------------------------------------------------------------------------------------------------------------------

    def slerp(self, q1, q2, t):
        # Compute spherical linear interpolation between quaternions
        q1 = self.normalize(q1)
        q2 = self.normalize(q2)
        dot_product = np.dot(q1, q2)
        if dot_product < 0.0:
            # Switching the sign ensures we take the shortest path on the quaternion sphere
            q1 = -q1
            dot_product = -dot_product
        if dot_product > 0.95:
            # They are basically the same quaternion
            return q1
        theta = np.arccos(dot_product)
        q_interpolated = (np.sin((1 - t) * theta) / np.sin(theta)) * q1 + (np.sin(t * theta) / np.sin(theta)) * q2

        return self.normalize(q_interpolated)
    
    def normalize(self, array):
        return array / np.linalg.norm(array)
    
    def rotation_matrix_to_quaternion(self, matrix):
        m00, m01, m02 = matrix[0, 0], matrix[0, 1], matrix[0, 2]
        m10, m11, m12 = matrix[1, 0], matrix[1, 1], matrix[1, 2]
        m20, m21, m22 = matrix[2, 0], matrix[2, 1], matrix[2, 2]
        trace = m00 + m11 + m22
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            x = (m21 - m12) * s
            y = (m02 - m20) * s
            z = (m10 - m01) * s
            w = 0.25 / s
        else:
            if m00 > m11 and m00 > m22:
                s = 0.5 / np.sqrt(1.0 + m00 - m11 - m22)
                x = 0.25 / s
                y = (m01 + m10) * s
                z = (m02 + m20) * s
                w = (m21 - m12) * s
            elif m11 > m22:
                s = 0.5 / np.sqrt(1.0 + m11 - m00 - m22)
                x = (m01 + m10) * s
                y = 0.25 / s
                z = (m12 + m21) * s
                w = (m02 - m20) * s
            else:
                s = 0.5 / np.sqrt(1.0 + m22 - m00 - m11)
                x = (m02 + m20) * s
                y = (m12 + m21) * s
                z = 0.25 / s
                w = (m10 - m01) * s
        return x, y, z, w
    

if __name__ == '__main__':
    planner = Trajectory()

    P1 = np.array([8.1216, -0.9333, 3.4974])
    P2 = np.array([9.2748, 1, 1.4590])
    P0 = np.array([8, 1, 3])
    q = planner.geodesic(P1, P2, P0)
    q = np.vstack(q)
    # Define sphere parameters
    radius = 2
    theta = np.linspace(0, 2.*np.pi, 50)
    phi = np.linspace(0, np.pi, 50)

    # Convert to Cartesian coordinates
    x = radius * np.outer(np.cos(theta), np.sin(phi)) + P0[0]
    y = radius * np.outer(np.sin(theta), np.sin(phi)) + P0[1]
    z = radius * np.outer(np.ones(np.size(theta)), np.cos(phi)) + P0[2]
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.plot_surface(x, y, z, color='b',alpha=0.5) 
    ax.plot(q[:,0],q[:,1],q[:,2],color='r')
    
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    
    plt.show()
