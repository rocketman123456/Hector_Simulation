import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

pubJoints = rospy.Publisher('/jointsTorque', Float32MultiArray, queue_size=10)

def main():
    # ros init
    rospy.init_node('pai_joint', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        position = Float32MultiArray()
        position.data = np.zeros(10)
        position.data[9] =0.4
        pubJoints.publish(position)
        rate.sleep()
        

if __name__ == "__main__":
    main()