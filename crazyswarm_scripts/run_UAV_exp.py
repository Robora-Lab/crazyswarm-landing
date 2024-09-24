import numpy as np
import csv
import time
import matplotlib.pyplot as plt
import rospy


from MPC_take2.quad_index import * # Rename

from MPC_take2.mpc import MPC
from MPC_take2.dynamic_env import DynamicEnv
#from MPC.plots import uav_plot
from crazyflie_control import CrazyflieControl
from std_msgs.msg import Float64


def run_mpcs(env, gp):

    # Initializing CSV writers
    csv_filename = 'MPC_take2/data/mpc_data.csv'
    uav_file = open(csv_filename, 'w', newline='')
    uav_writer = csv.writer(uav_file)
    # Write headers
    uav_writer.writerow(['px', 'py', 'pz','vx', 'vy', 'vz', 'gx', 'gy', 'gz', 'usvgx', 'usvgy', 'usvgz', 'usvx','usvy','usvz','tiltx','global time'])

    #env.reset()                # i think already reset in init in env
    env.time_prev = 0
    t = 0                                    # t is basically the current time
    n = -1                                   # n records the current time step number

    while t < env.sim_T-1:                   # while simulation time is not exceeded
        print('T',t)

        print('START')

        n += 1                                                       # increase time step number
        t = env.sim_dt * n                                           # increase simulation time  (sim_dt = 0.02), time goes 0.0 -> 0.1 -> 0.2 ... -> sim_T
        _, _, done, info, d_state, uav_g, usv_list, usv_pos_sub, time_sub, tilt_sub= env.step(t,gp)  # UPDATE ENVIRONMENT (very important)

        uav_goal = np.reshape(uav_g[:], newshape=(-1, 3))           # Format for display and plotting
        usv_goal = np.reshape(usv_list[0:3], newshape=(-1, 3))

        usv_pos = np.reshape(usv_pos_sub[:], newshape=(-1, 3))
        tilt = np.reshape(tilt_sub, newshape=(-1, 1))

        uav_writer.writerow(d_state.tolist() + uav_goal[0, :].tolist() + usv_goal[0, :].tolist()+ usv_pos[0, :].tolist()+tilt[0, :].tolist()+[time_sub])       # CSV file write

        if done == True:
            break

    uav_file.close()                          # CSV close

    #uav_plot()


def main():

    ################# Absolute Path to Repo #################
    #path_prefix = '/home/nathan'                              # Nathan's machine
    #path_prefix = '/Users/jls1/PycharmProjects'               # Jess's Windows machine
    path_prefix = '/home/jess/crazyswarm/ros_ws/src/crazyswarm/scripts'              # Jess's Linux machine


    # ##################### UAV Setup ########################    

    plan_T = 2.0                                                        # Prediction horizon for MPC and local planner
    plan_dt = 0.1                                                       # Sampling time step for MPC and local planner  
    so_path = "./mpc/saved/mpc_v1.so"                                   # saved mpc model (casadi code generation)      # WHAT????? -> Make code go faster                                          

    cf_cont=CrazyflieControl()
    cf_cont.init_and_takeoff()

    uav_mpc = MPC(T=plan_T, dt=plan_dt, so_path=so_path,gp=None)               # Set up the controller
    env = DynamicEnv(uav_mpc, plan_T, plan_dt, path_prefix,cf_cont)             # Set up the dynamics
    

    # ##################### Run ##############################
    run_mpcs(env,gp=None)

    print('DONE MPC WILL LAND NOW')
    cf_cont.stopping()
    print('final landed position:', cf_cont.getInfo()[0])
    exit(0)                              


if __name__ == "__main__":
    main()