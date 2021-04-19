import numpy as np
import control as ctrl
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2
        self.gamma  = self.k_drag / self.k_thrust 

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2
        self.Kd = np.diag(np.array([3.4,3.4,40]))
        self.Kp = np.diag(np.array([4,4,80]))
        # self.Kd = np.diag(np.array([1,1,1]))
        # self.Kp = np.diag(np.array([1,1,1    ]))
        self.g = 9.81 # m/s^2
        self.Kr = np.diag(np.array([80,80,80]))
        self.Kw = np.diag(np.array([7,7,7]))
        self.KI = np.diag(np.array([0.1,0.1,0.1,0.1,0.1,0.1]))

        # STUDENT CODE HERE


    def update(self, t, state, flat_output):
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0  
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        p = state['x']
        v = state['v']
        r = Rotation.from_quat(state['q']).as_euler('xyz')
        w = state['w']

        x_v = np.array([p[0],v[0],p[1],v[1],p[2],v[2],r[0],w[0],r[1],w[1],r[2],w[2]])



        A_mat = np.array([[0,1,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,self.g,0,0,0],
                          [0,0,0,1,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,-self.g,0,0,0,0,0],
                          [0,0,0,0,0,1,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,1,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,1,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,1],
                          [0,0,0,0,0,0,0,0,0,0,0,0]])
        B_mat = np.array([[0,0,0,0],
                          [0,0,0,0],
                          [0,0,0,0],
                          [0,0,0,0],
                          [0,0,0,0],
                          [1/self.mass,0,0,0],
                          [0,0,0,0],
                          [0,1/self.Ixx,0,0],
                          [0,0,0,0],
                          [0,0,1/self.Iyy,0],
                          [0,0,0,0],
                          [0,0,0,1/self.Izz]])
        C_mat = np.array([[1,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,1,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,1,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,1,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,1,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,1,0]])
        D_mat = np.zeros((6,4))
        # rAB = Rotation.from_quat(state['q']).as_matrix()  
        phi, theta, psi =  Rotation.from_quat(state['q']).as_euler('XYZ')
        R = np.eye(4)
        Q = C_mat.T @ C_mat
        Q[0,0] = 15
        Q[2,2] = 15
        Q[4,4] = 15
        K, P ,E = ctrl.lqr(A_mat, B_mat, Q, R)

        y_v = C_mat @ x_v 

        # r_des = np.hstack((flat_output['x'], np.array([0,0,flat_output['yaw']])))
        
        r_des = np.array([flat_output['x'][0], flat_output['x_dot'][0],flat_output['x'][1], flat_output['x_dot'][1], 
        flat_output['x'][2], flat_output['x_dot'][2], 0, 0, 0, 0, flat_output['yaw'], flat_output['yaw_dot']])
        
        # x_v = np.array([p[0],v[0],p[1],v[1],p[2],v[2],r[0],w[0],r[1],w[1],r[2],w[2]])
        
        # e_dot = r_des - y_v
        u_r = np.array([self.mass * self.g, 0,0,0])
        u = -K @ (x_v - r_des) + u_r
        # u

        to_force = np.array([[1,1,1,1],
                             [self.arm_length,0,-self.arm_length,0],
                             [0, self.arm_length, 0, -self.arm_length],
                             [-self.gamma,self.gamma,-self.gamma,self.gamma]])

        Matrix_u = np.array([[1,1,1,1],[0,self.arm_length,0,-self.arm_length],[-self.arm_length,0,self.arm_length,0],[self.gamma,-self.gamma,self.gamma,-self.gamma]])
        
        # for i in range(len(cmd_force)):
        #     if cmd_force[i] < 0:
        #         cmd_force[i] = 0
        cmd_motor_speeds = np.sqrt(np.matmul(np.linalg.inv(Matrix_u), u)/self.k_thrust)
        cmd_thrust  = np.matmul(np.linalg.inv(Matrix_u), u)
        # cmd_q = Rotation.from_matrix(R_des).as_quat()
        # STUDENT CODE HERE 

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
        return control_input

    # def update(self, t, state, flat_output):
    #     """
    #     This function receives the current time, true state, and desired flat
    #     outputs. It returns the command inputs.

    #     Inputs:
    #         t, present time in seconds
    #         state, a dict describing the present state with keys
    #             x, position, m
    #             v, linear velocity, m/s
    #             q, quaternion [i,j,k,w]
    #             w, angular velocity, rad/s
    #         flat_output, a dict describing the present desired flat outputs with keys
    #             x,        position, m
    #             x_dot,    velocity, m/s
    #             x_ddot,   acceleration, m/s**2
    #             x_dddot,  jerk, m/s**3
    #             x_ddddot, snap, m/s**4
    #             yaw,      yaw angle, rad
    #             yaw_dot,  yaw rate, rad/s

    #     Outputs:
    #         control_input, a dict describing the present computed control inputs with keys
    #             cmd_motor_speeds, rad/s
    #             cmd_thrust, N (for debugging and laboratory; not used by simulator)
    #             cmd_moment, N*m (for debugging; not used by simulator)
    #             cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
    #     """
    #     cmd_motor_speeds = np.zeros((4,))
    #     cmd_thrust = 0  
    #     cmd_moment = np.zeros((3,))
    #     cmd_q = np.zeros((4,))
        
    #     rAB = Rotation.from_quat(state['q']).as_matrix()    
    #     x_ddot_des = flat_output['x_ddot'] - np.matmul(self.Kd,state['v']-flat_output['x_dot'])- np.matmul(self.Kp,state['x']-flat_output['x'])
        
    #     # u_1 = self.mass*(x_ddot_des[2]+self.g)
    #     # theta_des = 1/2 * (x_ddot_des[0]+x_ddot_des[1])/self.g/np.sin(flat_output['yaw'])
    #     # phi_des = 1/2 * (x_ddot_des[0] - x_ddot_des[1])/self.g/np.sin(flat_output['yaw'])
    #     # psi_des = flat_output['yaw']
        
    #     # phi, theta, psi =  Rotation.from_quat(state['q']).as_euler('XYZ')
        
        
        
    #     # u_2 = np.matmul(self.inertia,np.array([-self.Kp_phi*(phi-phi_des)-self.Kd_phi*(state['w'][0]),-self.Kp_theta*(theta-theta_des)-self.Kd_theta*(state['w'][1]),-self.Kp_psi*(psi-psi_des)-self.Kd_psi*(state['w'][2])]))
    #     F_des = self.mass * x_ddot_des + np.array([0,0,self.mass * self.g])
    #     u_1 = np.inner(np.matmul(rAB,np.array([0,0,1])),F_des)
    #     b3_des = F_des / np.linalg.norm(F_des)
    #     a_psi = np.array([np.cos(flat_output['yaw']),np.sin(flat_output['yaw']),0])
    #     b2_des = np.cross(b3_des,a_psi)/np.linalg.norm(np.cross(b3_des,a_psi))
        
    #     R_des = np.zeros((3,3))
    #     R_des[:,0] = np.cross(b2_des,b3_des)
    #     R_des[:,1] = b2_des
    #     R_des[:,2] = b3_des
        
        
    #     e_R_Matrix= 1/2 * (np.dot(np.transpose(R_des),rAB)-np.dot(np.transpose(rAB),R_des))
        
    #     e_R = np.zeros((3,))
    #     e_R[0] = e_R_Matrix[2,1]
    #     e_R[1] = e_R_Matrix[0,2]
    #     e_R[2] = e_R_Matrix[1,0]
        
        
    #     w_des = np.zeros((3,))
    #     e_w = state['w'] - w_des
        
    #     u_2 = np.zeros((3,))
    #     u_2 = np.matmul(self.inertia,(-np.matmul(self.Kr,e_R)-np.matmul(self.Kw,e_w)))

        
    #     u = np.append([u_1],u_2)
    #     Matrix_u = np.array([[1,1,1,1],[0,self.arm_length,0,-self.arm_length],[-self.arm_length,0,self.arm_length,0],[self.gamma,-self.gamma,self.gamma,-self.gamma]])
        
    #     # for i in range(len(cmd_force)):
    #     #     if cmd_force[i] < 0:
    #     #         cmd_force[i] = 0
    #     cmd_motor_speeds = np.sqrt(np.matmul(np.linalg.inv(Matrix_u), u)/self.k_thrust)
    #     cmd_thrust  = np.matmul(np.linalg.inv(Matrix_u), u)
    #     cmd_q = Rotation.from_matrix(R_des).as_quat()
    #     # STUDENT CODE HERE

    #     control_input = {'cmd_motor_speeds':cmd_motor_speeds,
    #                      'cmd_thrust':cmd_thrust,
    #                      'cmd_moment':cmd_moment,
    #                      'cmd_q':cmd_q}
    #     return control_input
