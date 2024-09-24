import numpy as np
from pycrazyswarm import *
import rospy
from std_msgs.msg import Float64



class CrazyflieControl:
    def __init__(self):
        self.position_estimate = [0.0, 0.0, 0.0]
        self.vel_estimate=[0.0,0.0,0.0]
        #self.mc = None
        #self.scf = None
        #self.logconf = None
        #self.logconfv=None

        self.cf=None
        self.timeHelper=None

        self.time_sub = rospy.Subscriber('/systemTime', Float64, self.timeCallback)
        self.time_sub = 0

    def timeCallback(self,msg):
        self.time_sub=msg.data



    def init_and_takeoff(self):
        swarm = Crazyswarm()
        self.timeHelper = swarm.timeHelper
        self.cf = swarm.allcfs.crazyflies[0]

        self.cf.takeoff(1.3, 3.5)    #1m for 2.5s            #CHANGED
        self.timeHelper.sleep(4)     #hover for takeoff time plus 2.5s. stabilize

        self.position_estimate=self.cf.position()

        print("Takeoff completed.")
        print(f"Current position after takeoff: ", self.position_estimate)

        rounded_time = round(self.time_sub, 1)


        while not (rounded_time > 0 and (rounded_time-1.6)% 2 == 0):            #to start at 0.4, 2.4, 4.4 etc. (rounded_time-0.4)%2==0
            self.timeHelper.sleep(0.1)
            print(rounded_time)
            rounded_time = round(self.time_sub, 1)

        
        print("Starting at: ", self.time_sub)


    def action(self, vx,vy,vz):

        #do accel to vel and thrust mapping
        # roll = np.arctan2(ay, az)
        # pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
        yawrate=0
        # thrust=37000
        vel=[vx,vy,vz]

        # Send command to Crazyflie
        try:
            print(f"Executing action: vx={vx}, vy={vy}, vz={vz}")
            self.cf.cmdVelocityWorld(vel, yawrate)         #CHANGED
            self.timeHelper.sleep(0.01)
            #self.mc.stop()
        except Exception as e:
            print(f"Action failed: {e}")


    def land(self):
        self.timeHelper.sleep(2)    #stabilize before landing
        self.cf.land(0.04, 2.5)    #0.04m for 2.5s
        self.timeHelper.sleep(3)

    def stopping(self):
        #self.timeHelper.sleep(2)
        self.cf.cmdStop()

    # landing without using cf.land()
    # platform height is 0.97m
    # could make this more refined -- eg moniter for spike on acelerometer and then stop
    def prepare(self):
        self.timeHelper.sleep(1)    #stabilize
        vel=[0.0,0.0,-0.5]  # moving at half a meter per second
        yawrate=0
        delta_height=self.position_estimate[2]-1     #try
        secs=delta_height/0.5
        self.cf.cmdVelocityWorld(vel,yawrate)
        self.timeHelper.sleep(secs)

    def getInfo(self):
        # from subscription to ROS topics
        self.position_estimate=self.cf.position()
        return self.position_estimate, self.vel_estimate
