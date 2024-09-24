import numpy as np
from MPC_take2.quad_index import *
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from scipy.spatial.transform import Rotation as R
# from cflib.positioning.motion_commander import MotionCommander

# from ros_ws.src.crazyswarm.scripts.MPC_take2.crazyflie_control import CrazyflieControl


class Flie(object):

    def __init__(self, dt,cf_cont):
        self.s_dim = 6
        self.a_dim = 3

        self._state = np.zeros(shape=self.s_dim)

        self._actions = np.zeros(shape=self.a_dim)

        self._gz = 9.81
        self._dt = dt
        self._arm_l=0.042   #4.2cm approx

        self.update(cf_cont)


    ### RESET THE STATE TO DEFAULT VALUES - VALUES AFTER TAKE OFF ###
    ### or used for setting self._state to real drone pos vel ###
    def update(self, cf_cont):
        self._state = np.zeros(shape=self.s_dim)

        ###################### Zero Initialization ######################
        #for now/in sim using pred traj for v update
        currp, currv=cf_cont.getInfo()

        self._state[kPosX] = currp[kPosX]
        self._state[kPosY] = currp[kPosY]
        self._state[kPosZ] = currp[kPosZ]
        
        self._state[kVelX] = currv[0]
        self._state[kVelY] = currv[1]
        self._state[kVelZ] = currv[2]
        
        #
        return self._state
    

    # take optimized control commands and perform action and return real state after the action
    def run(self, action, cf_cont):         # action is lin accel

        ax, ay, az = action

        ax=float(ax)
        ay=float(ay)
        az=float(az)

        curr_vx=self._state[3]
        curr_vy=self._state[4]
        curr_vz=self._state[5]

        #vt+1=vt+at*deltat

        vx=curr_vx+ax*0.01
        vy=curr_vy+ay*0.01
        vz=curr_vz+az*0.01

        cf_cont.action(vx,vy,vz)
        print('ACTION:',vx, vy, vz)       # do action
        #cf_control.land()

        # and update state with real data now
        self._state=self.update(cf_cont)

        return self._state