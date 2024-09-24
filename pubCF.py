import rospy
from std_msgs.msg import Float32MultiArray

class PubCFNode:
    def __init__(self):
        rospy.init_node('PubCF', anonymous=True)

        # Publishers
        self.opt_goal_pub = rospy.Publisher('/GoalPoseCF', Float32MultiArray, queue_size=10)

        # Subscribers
        self.opt_goal_sub = rospy.Subscriber('/goalPose', Float32MultiArray, self.goal_callback)

        # Initialize other variables if needed
        self.some_variable = None

    def goal_callback(self, msg):
        # Example callback function for /goalPose subscriber
        rospy.loginfo("Received goalPose: %s", msg.data)
        # Process received data as needed
        self.some_variable = msg.data

    def publish_goal(self, goal_data):
        # Example function to publish goal data to /GoalPoseCF
        goal_msg = Float32MultiArray(data=goal_data)
        self.opt_goal_pub.publish(goal_msg)
        rospy.loginfo("Published goal data: %s", goal_data)

    def run(self):
        # Main loop (if needed)
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Perform any continuous processing if necessary
            rate.sleep()

if __name__ == '__main__':
    try:
        pub_node = PubCFNode()
        pub_node.run()
    except rospy.ROSInterruptException:
        pass