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
    
    def circular_motion_primitive(self, u, P: PoseStamped, C):
        # Extract the position and orientation  
        pos = np.array([P.pose.position.x,P.pose.position.y,P.pose.position.z])
        rot = np.array([P.pose.orientation.x,P.pose.orientation.y,P.pose.orientation.z,P.pose.orientation.w])
        rot_as_matrix = self.quaternion_to_rotation_matrix(rot)
        z_axis_init = rot_as_matrix[:,-1]
        # Compute the radius versor 
        direction_vector = pos - C
        radius = np.linalg.norm(direction_vector)
        direction_vector = direction_vector / radius
        # Generate the profile
        q = []
        for theta in u:
            x, y, z = C + radius * (-np.sin(theta) * direction_vector + np.cos(theta) * np.cross(direction_vector, [0,0,1]))
            
            y_axis = -np.sin(theta) * direction_vector + np.cos(theta) * np.cross(direction_vector, [0,0,1])
            y_axis = self.normalize(y_axis)
            z_axis = z_axis_init
            x_axis = np.cross(y_axis, z_axis)
            x_axis = self.normalize(x_axis)
            qx, qy, qz, qw = self.rotation_matrix_to_quaternion(np.array([x_axis, y_axis, z_axis]))
            q.append(PoseStamped(pose=Pose(position=Point(x=x,y=y,z=z),orientation=Quaternion(x=qx,y=qy,z=qz,w=qw))))
        # Compute the perpendicular vector to the last pose (useful to add rectilinear motion afterward)
        d = np.array([q[-1].pose.position.x,q[-1].pose.position.y,q[-1].pose.position.z]) - C
        return q, d
    
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
    
    def quaternion_to_rotation_matrix(self, quaternion):
        quaternion = self.normalize(quaternion)
        x, y, z, w = quaternion
        xx = x * x
        xy = x * y
        xz = x * z
        yy = y * y
        yz = y * z
        zz = z * z
        wx = w * x
        wy = w * y
        wz = w * z
        matrix = np.array([
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]
        ])
        return matrix
