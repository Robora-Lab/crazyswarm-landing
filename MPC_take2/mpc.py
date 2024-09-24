"""
Standard MPC
"""
# to ride or not to ride?
# [3] get rid of goal
# [1] plot goal on heat map time
# tilt at final goal (with and without time term, time to landing
# remove 50 bounds
# keep 1000 weight
# change init position in plots
# [2] do a mean square plot (calm water)

# start goal directly below uav
# plot intit position of uav vs tilt at landing (plots for all 3 cases)
# plot init position of uav vs distance traveld by boat (plots for all 3 cases)
# landing: goals are withing 5 cm

# Jan 29 2023
# Add X goal constraint -50<x<50
import casadi as ca
import numpy as np
import time
from os import system
import math

#import newWaveGP
#
from MPC_take2.quad_index import * # Rename
import MPC_take2.waveFunction as waveFunction

#
class MPC(object):
    """
    Nonlinear MPC
    """
    def __init__(self, gp, T, dt, so_path='./nmpc.so'):  # Initialization
        """
        Nonlinear MPC for quadrotor control        
        """
        self.so_path = so_path                       # Path to the saved mpc "./mpc/saved/mpc_v1.so"  

        # Time constant
        self._T = T                                  # 2 second horizon
        self._dt = dt                                # 0.1 second horizon
        self._N = int(self._T/self._dt)              # 20 total steps
 
        # Gravity
        self._gz = 9.81                              # Gravity in global z direction

        # Quadrotor constant
        self._max_az = 15
        self._max_ay= 40
        self._max_ax = 40      #max accel.....change
        #self._thrust_min = 2.0
        #self._thrust_max = 2E10

        # Hover height
        self.h_d = 0.3

        # state dimension (px, py, pz,              # quadrotor position
        #                  qw, qx, qy, qz,          # quadrotor quaternion
        #                  vx, vy, vz,              # quadrotor linear velocity
        self._s_dim = 6
        # action dimensions (c_thrust, wx, wy, wz)
        self._u_dim = 3
        
        # cost matrix for tracking the goal point (important)
        self._Q_goal = np.diag([
            600, 600, 600,                          # delta_x, delta_y, delta_z
            0, 0, 0])                            # delta_vx, delta_vy, delta_vz
        
        # cost matrix for the action (important)  
        self._Q_u = np.diag([0.005, 0.005, 0.005])   # ax, ay, az

        # cost matrix for the distributed goal
        self._Q_dis = np.diag([
            600, 600, 600,                          # delta_x, delta_y, delta_z 100 10 # 600
            0, 0, 0])                            # delta_vx, delta_vy, delta_vz
        

        self._Q_theta = np.diag([130000000])                # theta (water surface angle) 13000 130000000

        ##############  # *1 # Guess is done in dynamic_env  ###################
        # *1 # # initial state and control action
        # *1 # self._quad_s0 = [1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # *1 # self._quad_u0 = [9.81, 0.0, 0.0, 0.0]
        # *1 # self._quad_d0 = [1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self._initDynamics(gp)

    def _initDynamics(self,gp):
        # # # # # # # # # # # # # # # # # # # 
        # ---------- Input States -----------
        # # # # # # # # # # # # # # # # # # # 
        
        # define variables as symbols
        px, py, pz = ca.SX.sym('px'), ca.SX.sym('py'), ca.SX.sym('pz')
        #
        vx, vy, vz = ca.SX.sym('vx'), ca.SX.sym('vy'), ca.SX.sym('vz')

        # -- conctenated vector 
        self._x = ca.vertcat(px, py, pz, vx, vy, vz) 

        # # # # # # # # # # # # # # # # # # # 
        # --------- Control Command ------------
        # # # # # # # # # # # # # # # # # # #

        ax, ay, az = ca.SX.sym('ax'), ca.SX.sym('ay'), ca.SX.sym('az')
        
        # -- conctenated vector
        self._u = ca.vertcat(ax, ay, az)
        
        # # # # # # # # # # # # # # # # # # # 
        # --------- System Dynamics ---------
        # # # # # # # # # # # # # # # # # # #

        # x_dot = ca.vertcat(                    # derivative of x
        #     vx,                                                # derivative of px
        #     vy,                                                # derivative of py
        #     vz,                                                # derivative of pz
        #     0.5 * ( -wx*qx - wy*qy - wz*qz ),                  # derivatvie of qw
        #     0.5 * (  wx*qw + wz*qy - wy*qz ),                  # derivative of qx
        #     0.5 * (  wy*qw - wz*qx + wx*qz ),                  # derivative of qy
        #     0.5 * (  wz*qw + wy*qx - wx*qy ),                  # derivative of qz
        #     2 * ( qw*qy + qx*qz ) * thrust,                    # derivative of vx
        #     2 * ( qy*qz - qw*qx ) * thrust,                    # derivative of vy
        #     (qw*qw - qx*qx -qy*qy + qz*qz) * thrust - self._gz # derivative of vz
        #     # (1 - 2*qx*qx - 2*qy*qy) * thrust - self._gz
        # )

        self.Ad=  np.array([[1,   0,   0,   0.01, 0,   0],
                    [0,1,0,0,0.01,0],
                    [0,0,1,0,0,0.01],
                    [0,0,0,1,0,0],
                    [0,0,0,0,1,0],
                    [0,0,0,0,0,1]]                         
                    )
        
        self.Bd= np.array([[5.e-05,0,0],
                          [0,5.e-05,0],
                          [0,0,5.e-05],
                          [1.e-02,0,0],
                          [0,1.e-02,0],
                          [0,0,1.e-02]])
        
        F=self.lin_sys_dynamics()

        ### what is this function? -> gives us derivative of current state
        #self.f = ca.Function('f', [self._x, self._u], [x_dot], ['x', 'u'], ['ode'])
                
        # Fold  ### These casAdi functions are talked about in casAdi Documentation
        #F = self.sys_dynamics(self._dt)
        fMap = F.map(self._N, "openmp") # parallel
        ### Up to this point of _initDynamics we are just setting up fMap to give us the next state

        self.getSurface = waveFunction._wave.getThetaVicon()              # Function returns wave slope at X
        #self.getSurface = waveFunction._wave.getThetaGP(gp)              # Function returns wave slope at X
        #self.getAmplitude = waveFunction._wave.getWave()             # Function returns wave height in Z at X
        #self.getMeanAmplitude = waveFunction._wave.getMeanA()
        self.tilt_boundry = self.boundry_func()
        
        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
        # # # # # # # # # # # # # # # # ---- loss function -------- # # # # # # # # # # # # # # # 
        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

        # placeholder for the quadratic cost function
        Delta_s = ca.SX.sym("Delta_s", self._s_dim)                      # I think s is state               (x right now)                                                     
                                                                         # vector with 10 elements [px, py, pz, ... , vy, vz]
        Delta_u = ca.SX.sym("Delta_u", self._u_dim)                      # I think u is control             (u now) 
                                                                         # vector with 10 elements [c_thrust, wx, wy, wz]
        Delta_d = ca.SX.sym("Delta_d", self._s_dim)                      # I think d is distributive (negotiated btwn boat and drone)
                                                                         # vector with 10 elements [px, py, pz, ... , vy, vz]
        Delta_t = ca.SX.sym("Delta_t", 1)                                # I think t is theta        (water surface angle)
                                                                         # 1 dimensional             (we only care about x)
                
        # Cost function Terms
        cost_goal = Delta_s.T @ self._Q_goal @ Delta_s                   # template for (x_k - x_g)^2 * Q       term of cost func.
        cost_u = Delta_u.T @ self._Q_u @ Delta_u                         # template for U_k * Q                 term of cost func.
        cost_dis = Delta_d.T @ self._Q_dis @ Delta_d                     # template for (x^D_g - x^B_g)^2 * Q   term of cost func.
        # cost_theta = self._Q_theta * Delta_t**2                          # template for (wave(x^D_g, t) * Q)    term of cost func.
        cost_theta = Delta_t.T @ self._Q_theta @ Delta_t              

        # Symbolize cost function terms
        f_cost_goal = ca.Function('cost_goal', [Delta_s], [cost_goal])  
        f_cost_u = ca.Function('cost_u', [Delta_u], [cost_u])
        f_cost_d = ca.Function('cost_dis', [Delta_d], [cost_dis])
        f_cost_t = ca.Function('cost_theta', [Delta_t], [cost_theta])

        # # # # # # # # # # # # # # # # # # # # 
        # # ---- Non-linear OpgetMeanimization -----
        # # # # # # # # # # # # # # # # # # # #
        self.nlp_w = []       # nlp variables
        # *1 # self.nlp_w0 = []      # initial guess of nlp variables
        self.lbw = []         # lower bound of the variables, lbw <= nlp_x
        self.ubw = []         # upper bound of the variables, nlp_x <= ubw
        #
        self.mpc_obj = 0      # objective ### The cost function
        self.nlp_g = []       # constraint functions
        self.lbg = []         # lower bound of constrait functions, lbg < g
        self.ubg = []         # upper bound of constrait functions, g < ubg

        u_min=[-self._max_ax, -self._max_ay, -self._max_az]
        u_max = [self._max_ax,  self._max_ay,  self._max_az]
        x_bound = ca.inf
        x_min = [-x_bound for _ in range(self._s_dim)] 
        x_max = [+x_bound for _ in range(self._s_dim)]
 
        #
        g_min = [0 for _ in range(self._s_dim)]        # Why are constraints zero? (Probably so addition has correct dimension later)
        g_max = [0 for _ in range(self._s_dim)]

        #
        z_min = [0 for _ in range(1)]
        z_max = [x_bound for _ in range(1)]

        P = ca.SX.sym('P', self._s_dim + self._s_dim + 2) # coloumn vector for storing initial state and target state (and 2 times: timestamp & tilt term (old comment: compute time)
        X = ca.SX.sym("X", self._s_dim, self._N+1)        # matrix containing all states over all time steps +1 (each column is a state vector)
        U = ca.SX.sym("U", self._u_dim, self._N)          # matrix containing all control actions over all time steps (each column is an action vector)
        G = ca.SX.sym("G", self._s_dim, 1)                # Drone goal (implemented as constant)
        
        #
        X_next = fMap(X[:, :self._N], U)                  # next state based on RK4

        in_thresh = P[13]   #comes from tilt_param in dyn_env, can be 1 or 0, 1 means tilt activated
        #in_thresh = 0.5     #for no tilt
        self.h_ds = ca.if_else(in_thresh==0.0, self.h_d, 0.0)   #h_ds=hover when 0, h_ds=1m platform height when 1 (tilt activated)
        
        # "Lift" initial conditions
        self.nlp_w += [X[:, 0]]                           # Add to optimization variables
        # *1 # self.nlp_w0 += self._quad_s0               # Add a guess of the solution         
        self.lbw += x_min                                 # Add lower bounds
        self.ubw += x_max                                 # Add upper bounds
        
        # Drone Goal
        self.nlp_w += [G[:, 0]]
        # *1 # self.nlp_w0 += self._quad_d0
        self.lbw += x_min
        self.ubw += x_max
        # Add equality constraint
        # self.nlp_g += [G[kPosZ, 0]-self.getAmplitude(G[0,0], P[self._s_dim+self._s_dim:self._s_dim+self._s_dim+1]+(k+1)*self._dt)]                           # Drone goal should be at Z=0 cuz boat can't fly
        # self.lbg += [0]
        # self.ubg += [0]
        
        # # starting point.
        self.nlp_g += [X[:, 0] - P[0:self._s_dim]]        # constraint is X initial state - P initial state = 0 (they should be the same)
        self.lbg += g_min
        self.ubg += g_max

        # # Add inequality constraint.
        # self.nlp_g += [X[kPosZ, 0]]                           # Don't want the drone to go swimming
        # self.lbg += z_min
        # self.ubg += z_max
        # Z_gb = self.getAmplitude(P[self._s_dim:self._s_dim+1], P[self._s_dim+self._s_dim:self._s_dim+self._s_dim+1]) # Z goal of boat
        

        for k in range(self._N):
            # Define e_g,z
            #Z_gb = self.getAmplitude(P[self._s_dim], P[self._s_dim+self._s_dim])                          # Z goal of boat
            add=self.getSurface(P[self._s_dim:self._s_dim+1], P[self._s_dim+self._s_dim:self._s_dim+self._s_dim+1]+(k+1)*self._dt)
            Z_gb = 1.02    #platform height is 1m
            #mean_amp = self.getMeanAmplitude(P[self._s_dim], P[self._s_dim+self._s_dim]+(k+1)*self._dt)
            e_gz = G[kPosZ] - Z_gb                                                                        # Error in Z goals, when hovering 1.3-1.0
            #e_gz=0  #for now no landing
            Z_g_UAV = ca.if_else(in_thresh == 0.0,      #in_thresh=0 means don't activate tilt, 1.3m
                                 self.h_ds,         #mean_amp+self.h_ds, # If drone is not in threshold, hover above waves 1+0.3
                                 add/5)#Z_gb)               # If drone is in threshold, go to the boat, 1.0+0      #set goal of boat to platform height
            
            z_diff = ca.vertcat(0,0,Z_g_UAV,0,0,0)  #when ready to land (0,0,1,0,0,0)

            #z_diff=ca.vertcat(0,0,0,0,0,0) #for now no landing

            # Add Controls to optimization lists
            self.nlp_w += [U[:, k]]
            # *1 # self.nlp_w0 += self._quad_u0
            self.lbw += u_min
            self.ubw += u_max

            ######################  Cost function  ######################
            delta_s_k = (X[:, k+1] - G[:, 0])            # Drone position - Drone Goal
            cost_goal_k = f_cost_goal(delta_s_k)
        
            delta_u_k = U[:, k]-[0, 0, 0]      # Control - Gravity
            cost_u_k = f_cost_u(delta_u_k)

            delta_g_d = (G[:, 0] - (P[self._s_dim:self._s_dim+self._s_dim]+z_diff))      # Drone Goal - (Boat Goal in P term + Z_g_UAV from z_diff)

            # print('delta_g_d')
            # print(delta_g_d)
            cost_dis = f_cost_d(delta_g_d)

            # delta_g_t = self.getSurface(P[self._s_dim:self._s_dim+1], P[self._s_dim+self._s_dim:self._s_dim+self._s_dim+1]+(k+1)*self._dt)   # Surface angle
            # cost_theta = P[self._s_dim+self._s_dim+1:self._s_dim+self._s_dim+2]*self.tilt_boundry(e_gz)*f_cost_t(delta_g_t)
            
            W = self._Q_theta
            f = self.tilt_boundry(e_gz)
            phi_sqrd = self.getSurface(P[self._s_dim:self._s_dim+1], P[self._s_dim+self._s_dim:self._s_dim+self._s_dim+1]+(k+1)*self._dt)#**2
            #phi_sqrd = newWaveGP.predictOne(gp, P[self._s_dim:self._s_dim+1], P[self._s_dim+self._s_dim:self._s_dim+self._s_dim+1]+(k+1)*self._dt)**2

            cost_theta = in_thresh*W*f*phi_sqrd

            self.mpc_obj = self.mpc_obj + cost_goal_k + cost_u_k + cost_dis + cost_theta  # Sum Cost Function

            # New NLP variable for state at end of interval
            self.nlp_w += [X[:, k+1]]
            # *1 # self.nlp_w0 += self._quad_s0
            self.lbw += x_min
            self.ubw += x_max

            # # Add inequality constraint.
            # self.nlp_g += [X[kPosZ, k+1]]                      # Don't want the drone to go swimming
            # self.lbg += z_min
            # self.ubg += z_max

            # Add equality constraint
            # Postion at k when u is applied, drone should move to position at k+1
            self.nlp_g += [X_next[:, k] - X[:, k+1]]        # X(k+1) = f(X(k), u) difference should be zero
            self.lbg += g_min
            self.ubg += g_max
            
            # # Goal of drone always above boat goal height (wave height)
            self.nlp_g += [e_gz]
            self.lbg += z_min
            self.ubg += z_max

            # Drone must always be above goal
            self.nlp_g += [X[kPosZ, k+1]- G[kPosZ]]
            self.lbg += z_min
            self.ubg += z_max

            #print("Z position Constraint")
            #print([X[kPosZ, k+1]- G[kPosZ]])
            
        # nlp objective
        nlp_dict = {
            'f': self.mpc_obj,              # Cost Function
            'x': ca.vertcat(*self.nlp_w),   # Optimization Variables
            'p': P,                         # Initial State and boat goal
            'g': ca.vertcat(*self.nlp_g),   # Constraints?
        }    

        # # # # # # # # # # # # # # # # # # # 
        # -- ipopt
        # # # # # # # # # # # # # # # # # # # 
        ipopt_options = {
            'verbose': False, \
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 1e-4,
            "ipopt.max_iter": 100,
            "ipopt.warm_start_init_point": "yes",
            "ipopt.print_level": 0, 
            "print_time": False
        }
        
        print("Starting UAV Solver")
        ################### Creating an SO file makes code go faster? ######################
        # self.solver = ca.nlpsol("solver", "ipopt", nlp_dict, ipopt_options)
        # # jit (just-in-time compilation)
        # print("Generating shared library........")
        # cname = self.solver.generate_dependencies("mpc_v2.c")  
        # self.so_path = "./mpc/saved/mpc_v2.so"    
        # system('gcc -fPIC -shared -O3 ' + cname + ' -o ' + self.so_path) # -O3
        
        # # reload compiled mpc
        self.solver = ca.nlpsol("solver", "ipopt", nlp_dict, ipopt_options)

    def solve(self, ref_states, guess):
        # # # # # # # # # # # # # # # #
        # -------- solve NLP ---------
        # # # # # # # # # # # # # # # #
        
        # Enter current info into MPC
        self.sol = self.solver(
            x0=guess, 
            lbx=self.lbw, 
            ubx=self.ubw, 
            p=ref_states, 
            lbg=self.lbg, 
            ubg=self.ubg)
        #
        sol_x0 = self.sol['x'].full()
        opt_d = sol_x0[self._s_dim:self._s_dim+self._s_dim]                         # Isolate Drone Goal
        opt_u = sol_x0[self._s_dim+self._s_dim:self._s_dim+self._s_dim+self._u_dim] # Isolate the optimal next control signal
        
        # Warm initialization                               # Don't know what that means
        self.nlp_w0 = list(sol_x0[self._s_dim+self._u_dim:2*(self._s_dim+self._u_dim)]) + list(sol_x0[self._s_dim+self._u_dim:])
        
        #
        x0_array = np.reshape(sol_x0[:-(self._s_dim+self._s_dim)], newshape=(-1, self._s_dim+self._u_dim)) ### Format next steps
        
        # return optimal action, a sequence of predicted optimal trajectory, and optimal drone goal
        return opt_u, x0_array, opt_d
    
    def sys_dynamics(self, dt):
        M = 4       # refinement
        DT = dt/M
        X0 = ca.SX.sym("X", self._s_dim)
        U = ca.SX.sym("U", self._u_dim)
        # #
        X = X0
        for _ in range(M):
            # --------- RK4------------
            k1 =DT*self.f(X, U)
            k2 =DT*self.f(X+0.5*k1, U)
            k3 =DT*self.f(X+0.5*k2, U)
            k4 =DT*self.f(X+k3, U)
            #
            X = X + (k1 + 2*k2 + 2*k3 + k4)/6        
        # Fold
        F = ca.Function('F', [X0, U], [X])
        return F
    
    # find x_next given state and control input
    def lin_sys_dynamics(self):
        X0 = ca.MX.sym("X", self._s_dim,1)
        U = ca.MX.sym("U", self._u_dim,1)
        x_next=self.Ad@X0+self.Bd@U
        F = ca.Function('F', [X0, U], [x_next])
        return F

    
    #egz=0.3 when hover. Goes to 0 for landing
    def boundry_func(self):
        EGZ = ca.SX.sym("E", 1)
        B = ca.if_else(EGZ>=0.16, 1.0/(1+ca.exp(-(EGZ-self.h_d)/(-0.15))), 1.0/(1+ca.exp((EGZ-self.h_d)/(-0.01))))  #0.16 original
        # B = 1.0/(1+ca.exp((EGZ-self.h_d)/(-0.01)))
        # B = 1.0/(1+ca.exp(-(EGZ-self.h_d)/(-0.15)))
        F = ca.Function('F', [EGZ], [B])
        return F