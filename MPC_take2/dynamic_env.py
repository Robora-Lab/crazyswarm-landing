import numpy as np

import csv
import rospy
import math
#
#from UAV.high_mpc.simulation.quadrotor import Quadrotor_v0
from MPC_take2.quad_index import * # Rename
from MPC_take2.CF1 import Flie
#
import MPC_take2.waveFunction as waveFunction
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

flag=0      #once landing conditions are active they must stay active

def quatToYaw(x,y,z,w):
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    x/=norm
    y/=norm
    z/=norm
    w/=norm
    yaw=math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))
    return yaw


class DynamicEnv(object):

    def __init__(self, mpc, plan_T, plan_dt, path_prefix,cf_cont):

        self.path_prefix = path_prefix
        #
        self.mpc = mpc
        self.plan_T = plan_T
        self.plan_dt = plan_dt

        self.goal_thresh = 0.1 #0.4

        self.sim_T = 500.0   # Episode statelength, seconds

        self.sim_dt = 0.01  # simulation time step
        # Simulators, a quadrotor
        #self.quad = Quadrotor_v0(dt=self.sim_dt) #will be replaced, this is only for simulation

        self.crazyf = Flie(self.sim_dt,cf_cont)

        # reset the environment
        self.t = 0
        self.reset(cf_cont)
        self.cf_cont=cf_cont

        self.count=0

        self.opt_goal_pub = rospy.Publisher('/goalPoseCF', PoseStamped, queue_size=10)
        self.extra_pub = rospy.Publisher('/extraPub', Float64, queue_size=10)         #to get z goal without setting opt_goal_pub to anything but 0.0
        
        self.opt_goal_sub = rospy.Subscriber('/goalPose', PoseStamped, self.huskyCallback)
        self.opt_goal_sub = np.zeros(3)

        self.husky_pos_sub = rospy.Subscriber('/vrpn_client_node/WillHusky/pose', PoseStamped, self.huskyPoseCallback)
        self.husky_pos_sub = np.zeros(3)

        self.tilt_sub = rospy.Subscriber('/tilts', PoseStamped, self.tiltCallback)
        self.tilt_sub = np.zeros(2)

        self.time_sub = rospy.Subscriber('/systemTime', Float64, self.timeCallback)
        self.time_sub = 0


    def huskyCallback(self, msg):
        position = msg.pose.position

        x = float(position.x)
        y = float(position.y)
        z = float(position.z)
        z=1.02
        data_float = [x, y, z]
        self.opt_goal_sub=np.array(data_float)
    
    def huskyPoseCallback(self, msg):
        position = msg.pose.position
        ori=msg.pose.orientation
        position_offset = position

        yaw=quatToYaw(ori.x, ori.y, ori.z, ori.w)

        position_offset.x = position.x + (0.1 *math.cos(-yaw))
        position_offset.y = position.y - (0.1*math.sin(-yaw))

        x = float(position_offset.x)
        y = float(position_offset.y)
        z = float(position.z)
        z=1.02
        data_float = [x, y, z]
        self.husky_pos_sub=np.array(data_float)

    def tiltCallback(self, msg):
        position = msg.pose.position

        x = float(position.x)
        y = float(position.y)
        data_float = [x, y]
        self.tilt_sub=np.array(data_float)

    def timeCallback(self,msg):
        self.time_sub=msg.data

    def reset(self,cf_cont):
        self.t = 0
        # state for ODE
        self.cf_state=self.crazyf.update(cf_cont)

    def step(self, t, u=0):
        self.t = t
        opt_t = u

        ###########################  Request Boat Goal  ################################ 
        self.boat_goal=self.opt_goal_sub

        # goal state, position, velocity
        self.boat_goal_full = self.boat_goal.tolist() + [0.0, 0.0, 0.0]

        #self.time_param = np.array([self.t])
        self.time_param = np.array([self.time_sub])

        # Bool to activate tilt term
        if self.t:
            self.eps_1 = abs(self.boat_goal_full[kPosX]-self.husky_pos_sub[kPosX])<self.goal_thresh and abs(self.boat_goal_full[kPosY]-self.husky_pos_sub[kPosY])<self.goal_thresh  
            #H goal X - H pose X

            self.eps_2 = abs(self.boat_goal_full[kPosX]-self.opt_goal[kPosX])<self.goal_thresh and abs(self.boat_goal_full[kPosY]-self.opt_goal[kPosY])<self.goal_thresh          
            #H goal X - CF goal X

            self.eps_3 = abs(self.cf_state[kPosX]-self.opt_goal[kPosX])<self.goal_thresh and abs(self.cf_state[kPosY]-self.opt_goal[kPosY])<self.goal_thresh                
            #CF pose X - CF goal X

            self.eps_4 = abs(self.cf_state[kPosZ]-self.opt_goal[kPosZ])<(2*self.goal_thresh)            
            #CF pose Z - CF goal Z
            
            #self.eps_5 = abs(self.egz - 1.0)<(2*self.goal_thresh)                                       #egz=CF pose Z - wave amp   why -1 ??
            self.eps_5 = abs(self.egz)<(4.0*self.goal_thresh)
            
            J_tilt = int(self.eps_1 and self.eps_2 and self.eps_3 and self.eps_4 and self.eps_5)        #J_tilt=1 means tilt activated
            # J_tilt = int(self.eps_1 and self.eps_2 and self.eps_3 and self.eps_5)
            print("eps_1, eps_2, eps_3, eps_4, eps_5 ", self.eps_1, self.eps_2, self.eps_3, self.eps_4, self.eps_5)
            #J_tilt=0    #for no tilt consideration
        else:
            J_tilt = 0

        #landing conditions stay on once activated for the first time
        if self.t>0.1 and J_tilt==1:
            self.count=self.count+1
            if self.count>=75:
                global flag
                flag=1

        self.tilt_param = np.array([flag])

        #
        flie_s0 = self.cf_state.tolist()
        print('now', flie_s0) 
        ref_traj = flie_s0 + self.boat_goal_full + self.time_param.tolist() + self.tilt_param.tolist()  # [Postion now, boat goal, time, tilt term]
        print("ref_traj")
        print(ref_traj)

        #################### Prepare intial Guess ####################
        nlp_w0 = []                                                     # Hold our guess of all optimized variables
        # initial state and control action
        _quad_s0 = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # Position Guess
        _quad_u0 = [0.0, 0.0, 0.0]                                # Control Guess 
        _quad_d0 = self.boat_goal_full                                         # Drone Goal Guess = Boat Goal

        # add guesses to array in same format the result will be
        nlp_w0 += _quad_s0
        nlp_w0 += _quad_d0
        for k in range(int(self.plan_T/self.plan_dt)):
            nlp_w0 += _quad_u0
            nlp_w0 += _quad_s0


        # run nonliear model predictive control
        quad_act, pred_traj, self.opt_goal = self.mpc.solve(ref_traj, nlp_w0)    ### Solve given current state, and where we want to go (along with a guess of answer)


        # this is where changes will be made for real experiment
        # run the actual control command on the quadrotor
        #self.quad_state = self.quad.run(quad_act)

        print('qa', quad_act)   # quad-act is accelerations

        self.cf_state=self.crazyf.run(quad_act, self.cf_cont)       # run takes action as acelerations and returns new state of drone after action applied

        # publish drone goal to GoalPoseCF
        opt_goal_msg=PoseStamped()
        opt_goal_msg.pose.position.x = self.opt_goal[0]
        opt_goal_msg.pose.position.y = self.opt_goal[1]
        opt_goal_msg.pose.position.z = 0.0                  #STRANGE needs to be 0.0, before was setting to 0.0 and writing to csv from known info
                                                            #in code but now that taking plot from topic, need to get data while still publish 0.0?

        self.extra_pub.publish(self.opt_goal[kPosZ])
        
        #opt_goal_msg = PoseStamped(data=self.opt_goal[0:3])
        self.opt_goal_pub.publish(opt_goal_msg)

        # update the observation.
        #quad_obs = self.crazyf.get_cartesian_state()
        quad_obs=None
        #obs = (quad_obs).tolist()
        obs=None
        #
        info = {
            "quad_obs": quad_obs, 
            "quad_act": quad_act, 
            "quad_axes": None,  #self.crazyf.get_axes(),
            "pred_quad_traj": pred_traj, 
            "opt_t": opt_t, 
            "plan_dt": self.plan_dt}
        

        #self.boat_goal_z = waveFunction._wave.getCurrWave(self.boat_goal[kPosX], t)    #amplitude of wave at x,t
        #self.egz = self.cf_state[kPosZ]-self.boat_goal_z
        self.boat_goal_z = 1.02     #height of platform
        #self.cf_state[kPosZ]=1.3    #pretend takeoff good
        self.egz = self.cf_state[kPosZ] - 1.02   #setting wave/boat amp = 1.0m
        print('egz:',self.egz)
        #self.boatTilt = waveFunction._wave.getCurrThetaVicon(self.boat_goal[kPosX], t)      #tilt of wave at x,t
        self.phi_sqrd = self.tilt_sub[0]**2


        self.boat_list = np.array([self.boat_goal[kPosX], self.boat_goal[kPosY],self.boat_goal[kPosZ],self.boat_goal_z, self.egz, self.tilt_sub[0], self.phi_sqrd, t]).tolist()
        
        done = False
        if (self.cf_state[kPosZ]-self.boat_goal_z) <= 0.05:     #within 15cm done
            #self.cf_state=self.crazyf.run([quad_act[0],quad_act[1],-50], self.cf_cont)
            done=True
            print('DONE')

        return obs, 0, done, info, self.cf_state, self.opt_goal, self.boat_list, self.husky_pos_sub, self.time_sub, self.tilt_sub