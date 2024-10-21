import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import numpy as np

x_pos_h = []
y_pos_h = []
x_pos_cf = []
y_pos_cf = []
x_goal_h = []
y_goal_h = []
x_goal_cf = []
y_goal_cf = []

def compTilt(time):
    x = np.linspace(-3, 3, 250)
    y = np.linspace(-3, 3, 1)
    x, y = np.meshgrid(x, y)
    theta = 8 * (x + 3.5) * np.sin(np.pi * (time + x))

    return theta

def h_goal_callback(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    #rospy.loginfo(f"Received Husky position: x={x}, y={y}")
    x_goal_h.append(x)
    y_goal_h.append(y)

def cf_goal_callback(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    #rospy.loginfo(f"Received Crazyflie position: x={x}, y={y}")
    x_goal_cf.append(x)
    y_goal_cf.append(y)

def time_callback(msg):
    global current_time
    current_time = msg.data
    #rospy.loginfo(f"Received Time: {current_time}")

def h_pos_callback(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    #rospy.loginfo(f"Received Husky position: x={x}, y={y}")
    x_pos_h.append(x)
    y_pos_h.append(y)

def cf_pos_callback(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    #rospy.loginfo(f"Received Crazyflie position: x={x}, y={y}")
    x_pos_cf.append(x)
    y_pos_cf.append(y)

def live_plot():
    rospy.init_node('live_plot', anonymous=False)

    rospy.Subscriber('/goalPose', PoseStamped, h_goal_callback)
    rospy.Subscriber('/goalPoseCF', PoseStamped, cf_goal_callback)
    rospy.Subscriber('/systemTime', Float64, time_callback)
    rospy.Subscriber('/vrpn_client_node/WillHusky/pose', PoseStamped, h_pos_callback)
    rospy.Subscriber('/vrpn_client_node/cf1/pose', PoseStamped, cf_pos_callback)

    plt.ion()  #interactive mode
    fig, ax = plt.subplots(figsize=(12, 12))

    line_h_pos, = ax.plot([], [], 'r', lw=2, label='Husky Position')
    line_h_goal, = ax.plot([], [], 'r--', lw=2, label='Husky Goal Position')
    line_cf_pos, = ax.plot([], [], 'b', lw=2, label='Crazyflie Position')
    line_cf_goal, = ax.plot([], [], 'b--', lw=2, label='Crazyflie Goal Position')

    
    marker_husky, = ax.plot([], [], 'rs', markersize=10)
    marker_crazyflie, = ax.plot([], [], 'b^', markersize=10)
    marker_husky_goal, = ax.plot([], [], 'ro', markersize=7)
    marker_crazyflie_goal, = ax.plot([], [], 'bo', markersize=7)

    ax.set_xlim(3, -3)
    ax.set_ylim(3, -3)
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Crazyflie Husky Landing')
    ax.legend(loc='lower right')

    rate = rospy.Rate(10)  #10 Hz update rate

    colorbar=None
    
    try:
        while not rospy.is_shutdown():
            #plotting waves
            if current_time:
                output = compTilt(current_time)
                if colorbar is None:
                    im = plt.imshow(
                        output,
                        cmap='Blues',
                        extent=[-3, 3, 3, -3],
                        origin='lower')
                    colorbar = plt.colorbar(im, ax=ax, label='Wave Tilt Squared ($^\circ$)')
                else:
                    im.set_array(output)

            #update line - husky goal
            if x_goal_h and y_goal_h:
                line_h_goal.set_xdata(x_goal_h)
                line_h_goal.set_ydata(y_goal_h)
                
                #update marker - husky goal
                marker_husky_goal.set_xdata(x_goal_h[-1])
                marker_husky_goal.set_ydata(y_goal_h[-1])

            #update line - cf goal
            if x_goal_cf and y_goal_cf:
                line_cf_goal.set_xdata(x_goal_cf)
                line_cf_goal.set_ydata(y_goal_cf)
                
                #update marker - cf goal
                marker_crazyflie_goal.set_xdata(x_goal_cf[-1])
                marker_crazyflie_goal.set_ydata(y_goal_cf[-1])

            #update line - husky pos
            if x_pos_h and y_pos_h:
                line_h_pos.set_xdata(x_pos_h)
                line_h_pos.set_ydata(y_pos_h)
                
                #update marker - husky pos
                marker_husky.set_xdata(x_pos_h[-1])
                marker_husky.set_ydata(y_pos_h[-1])
            
            #update line - cf pos
            if x_pos_cf and y_pos_cf:
                line_cf_pos.set_xdata(x_pos_cf)
                line_cf_pos.set_ydata(y_pos_cf)
                
                #update marker - cf pos
                marker_crazyflie.set_xdata(x_pos_cf[-1])
                marker_crazyflie.set_ydata(y_pos_cf[-1])

            ax.relim()
            ax.autoscale_view()

            fig.canvas.draw()
            fig.canvas.flush_events()

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    try:
        live_plot()
    except rospy.ROSInterruptException:
        pass
