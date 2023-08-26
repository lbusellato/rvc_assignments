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

if __name__ == '__main__':
    planner = Trajectory()
    # Scanning trajectory parameters
    starting_point = np.array([0.484, 0.128, 0.609]) # Trajectory start
    center = np.array([0,0.128,0]) # Center for circular primitives
    displacement = 0.2 # Distance to cover during rectilinear motion
    rectilinear_dt = 1.5 # Time duration for rectilinear motion
    circular_dt = 1.5 # Time for circular motion
    
    scan_orientation = Quaternion(x=0.0017937315462992396, y=0.9999976506446824, z=0.00035651772527578343, w=-0.001163669784592465)
    initial_pose = PoseStamped(pose=Pose(position=Point(x=0.484, y=0.128, z=0.609), orientation=Quaternion(x=0.5, y=0.5, z=0.5, w=0.5)))
    scanning_start = PoseStamped(pose=Pose(position=Point(x=-0.129, y=0.487, z=0.422), 
                                        orientation=scan_orientation))

    center = np.array([0.0, 0.0, scanning_start.pose.position.z])
    # Linear motion to the scanning start
    t, u = planner.linear_polynomial(0, rectilinear_dt, 0, 1)
    q = planner.rectilinear_motion_primitive(u, initial_pose, scanning_start)
    # First arc
    t1, u1 = planner.linear_polynomial(t[-1], t[-1] + circular_dt, -np.pi/2, np.pi/2)
    q1, d1 = planner.circular_motion_primitive(u1, q[-1], center)
    # First line
    displacement_vector = displacement * d1
    q2f = copy.deepcopy(q1[-1])
    q2f.pose.position.x += displacement_vector[0]
    q2f.pose.position.y += displacement_vector[1]
    q2f.pose.position.z += displacement_vector[2]
    t2, u2 = planner.linear_polynomial(t1[-1], t1[-1] + rectilinear_dt, 0, 1)
    q2 = planner.rectilinear_motion_primitive(u2, q1[-1], q2f)
    # Second arc
    t3, u3 = planner.linear_polynomial(t2[-1], t2[-1] + circular_dt, 3*np.pi/2, np.pi/2)
    q3, d3 = planner.circular_motion_primitive(u3, q2[-1], center)
    # Second line
    displacement_vector = displacement * d3
    q4f = copy.deepcopy(q3[-1])
    q4f.pose.position.x += displacement_vector[0]
    q4f.pose.position.y += displacement_vector[1]
    q4f.pose.position.z += displacement_vector[2]
    t4, u4 = planner.linear_polynomial(t3[-1], t3[-1] + rectilinear_dt, 0, 1)
    q4 = planner.rectilinear_motion_primitive(u4, q3[-1], q4f)
    # Final arc
    _, u5 = planner.linear_polynomial(t4[-1], t4[-1] + circular_dt, -np.pi/2, np.pi/2)
    q5, _ = planner.circular_motion_primitive(u5, q4[-1], center)
    q = q + q1 + q2 + q3 + q4 + q5
    for i, posestamped in enumerate(q):
        pose = posestamped.pose
        q[i] = np.array([pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
    q = np.vstack((q))
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    plt.figure()
    ax = plt.subplot(projection='3d')
    ax.plot(q[:,0],q[:,1],q[:,2])
    for i in range(q.shape[0]):
        if i % 10 == 0:
            x, y, z = q[i,:3]
            qx, qy, qz, qw = q[i,3:]
            u = np.array([1 - 2*(qy**2 + qz**2), 2*(qx*qy + qz*qw), 2*(qx*qz - qy*qw)])
            u /= np.linalg.norm(u)
            v = np.array([2*(qx*qy - qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz + qx*qw)])
            v /= np.linalg.norm(v)
            w = np.array([2*(qx*qz + qy*qw), 2*(qy*qz - qx*qw), 1 - 2*(qx**2 + qy**2)])
            w /= np.linalg.norm(w)
            ax.quiver(x, y, z, u[0], u[1], u[2], length=0.05, normalize=True, color='red')
            ax.quiver(x, y, z, v[0], v[1], v[2], length=0.05, normalize=True, color='green')
            ax.quiver(x, y, z, w[0], w[1], w[2], length=0.025, normalize=True, color='blue')
        
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.view_init(azim=60, elev=45)
    plt.show()