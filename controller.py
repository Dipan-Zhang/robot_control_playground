import numpy as np
import pinocchio as pin

# virtual class of controller
class controller_base:
    def __init__(self, robot, robot_state, q0, dt=1e-3):
        self.robot = robot
        self.robot_state = robot_state
        self.q0 = q0
        self.dt = dt
        self.M = []
        self.g = []

    def update_dynamics_parameters(self):
        # Assuming `crba` returns a NumPy array
        mass = pin.crba(self.robot.model, self.robot.data, self.robot_state.q)
        mass[np.tril_indices_from(mass, k=-1)] = mass.T[np.tril_indices_from(mass, k=-1)]
        self.M = mass
        self.g = pin.computeGeneralizedGravity(self.robot.model, self.robot.data, self.robot_state.q)


# pid controller
class PID_controller(controller_base): 
    # Defining the P,I,D control parameters
    def __init__(self, robot, robot_state, q0, kp=[85, 85, 85, 10, 10, 10], ki=[100, 100, 100, 10, 10, 3], kd=[30, 30, 30, 1, 1, 0.1], 
                 torque_limit =400, i_max = 10, i_min = -10):
        super().__init__(robot,robot_state, q0)
        self.nu = robot_state.DOF
        self.kp = np.array(kp)
        self.kd = np.array(kd)
        self.ki = np.array(ki)
        self.error_p = []
        self.error_p_last = np.zeros(self.kp.shape)
        self.error_i = np.array([0.]*self.nu)
        self.speed = 1
        self.i_max = i_max
        self.i_min = i_min
        self.torque_limit = torque_limit

    
    def print_control_gain(self):
        print("currently using control gain: ")
        print('------------------------------------------------')
        print(f'kp: {self.kp}\n ki: {self.ki}\n kd: {self.kd}\n')

    def set_control_gain(self, gain):
        self.kp = gain["kp"]
        self.ki = gain["ki"]
        self.kd = gain["kd"]
        print("PID control gain set manually")
        self.print_control_gain()

    def clear_error_i(self):
        self.error_i = np.array([0.00]*self.nu)
        print("integration error erased!")
        
    def calc_error(self,robot_state,reference,dt):
        self.error_p = (np.array(reference) - np.array(robot_state))[:self.nu]
        self.error_i += self.error_p*dt 
        self.error_i = np.clip(self.error_i, -100,100)
        self.error_d = (self.error_p - self.error_p_last)/dt
        self.error_p_last = self.error_p
    
    def compute_torque(self,robot_state,reference,dt=1e-3):
        self.calc_error(robot_state,reference,dt)
        self.update_dynamics_parameters()
        kp_term = np.array(self.kp)*np.array(self.error_p) 
        kd_term = np.array(self.kd)*np.array(self.error_d)
        ki_term = np.array(self.ki)*np.array(self.error_i)

        self.torque = kp_term + kd_term + ki_term + self.g
        # print(np.clip(torque, -self.torque_limit,self.torque_limit))
        return np.clip(self.torque, -self.torque_limit,self.torque_limit)