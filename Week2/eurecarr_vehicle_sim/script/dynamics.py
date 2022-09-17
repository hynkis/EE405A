'''
@author : USRG(Unmanned Systems Research Group) @ KAIST
@date   : 2021-03
@brief  : The dynamics class for simple car dynamics simulation.
'''
import numpy as np

class Dynamics(object):
    def __init__(self, stateDim, inputDim, dt):
        self.dt         = dt
        self.max_steer    = 1
        self.throttleRatio = 1

        self.m       	= 0.041
        self.Iz     	= 27.8E-6
        self.wheelbase  = 0.3 
        self.width      = 0.08 
        self.lf         = 0.15/2.0
        self.lr         = self.wheelbase - self.lf

    def forward(self, states, inputs):
        # 0 1 2   3
        # x y yaw vx
        """
        states : [x, y, yaw, vx]

        """

        states_dot = self.simpleBicycleModel(states, inputs)

        return states_dot

    def local2global(self, x, y, yaw):
        x_g = np.cos(yaw)*x - np.sin(yaw)*y
        y_g = np.sin(yaw)*x + np.cos(yaw)*y
        return x_g, y_g

    def global2local(self, x, y, yaw):
        x_l = np.cos(-yaw)*x - np.sin(-yaw)*y
        y_l = np.sin(-yaw)*x + np.cos(-yaw)*y
        return x_l, y_l

    def simpleBicycleModel(self, states, inputs):
        '''
        Body centered at the rear wheel
            states : [x, y, yaw, vx]
            inputs : [steer, accel]
        '''
        yaw   = states[2]
        vx    = states[3]
        steer = inputs[0]
        accel = inputs[1]

        x_dot   = vx * np.cos(yaw)
        y_dot   = vx * np.sin(yaw)
        yaw_dot = vx * np.tan(steer) / self.wheelbase
        vx_dot  = accel

        # states_dot   = np.array([x_dot, y_dot, yaw_dot, roll_dot, vx_dot, vy_dot, yaw_dotdot], dtype=float)
        states_dot   = np.array([x_dot, y_dot, yaw_dot, vx_dot], dtype=float)
        return states_dot

def main():

    dt        = 0.01
    print("dt : "+str(dt) + " seconds")

    # states
    x0        = 0.0
    y0        = 0.0
    yaw0      = 0.0
    roll0     = 0.0
    vx0       = 10.0
    vy0       = 0.0
    yaw_dot0  = 0.0

    # inputs
    steering  = 0.0
    throttle  = 0.5

    states = np.array([x0, y0, yaw0, roll0, vx0, vy0, yaw_dot0])
    inputs = np.array([steering, throttle])

    state_dim = 7
    input_dim = 2
    model = Dynamics(state_dim, input_dim, dt)
    
    num_sim = 1
    for i in range(num_sim):
        states_dot = model.forward(states, inputs)
        new_states = states + states_dot * dt

        print("Sim step  : " + str(i))
        print("Inputs    : " + str(np.around(inputs, 3)))
        print("State     : " + str(np.around(states, 3)))
        print("              x'     y'    yaw'     roll'      vx'    vy'    yaw_dot'")
        print("State_der : " + str(np.around(states_dot, 3)))
        print("State_del : " + str(np.around(states_dot * dt, 3)))
        print("State_new : " + str(np.around(new_states, 3)))

        states = new_states

        print("====================")

if __name__ == "__main__":
    main()
