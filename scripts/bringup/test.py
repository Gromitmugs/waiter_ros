
import rospy
from std_msgs.msg import Float64MultiArray

rospy.init_node('test')
raw_vel = Float64MultiArray()
raw_pos = Float64MultiArray()
vel = [0.0,0.0]
pos = [0.0,0.0]

vel_pub = rospy.Publisher('/raw_vel', Float64MultiArray, queue_size=10)
pos_pub = rospy.Publisher('/raw_pos', Float64MultiArray, queue_size=10)
raw_vel.data = tuple(vel)
raw_pos.data = tuple(pos)
vel_pub.publish(raw_vel)
pos_pub.publish(raw_pos)