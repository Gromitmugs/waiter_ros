from geometry_msgs.msg import Point, Pose, Quaternion
import rospy
# geometry_msgs/Point position
# geometry_msgs/Quaternion orientation


kitchen = Pose()
table = Pose()

# kitchen
# position
kitchen.position.x = 4.063257694244385
kitchen.position.y = 0.2243729829788208
kitchen.position.z = 0.0
# orientation
kitchen.orientation.x = 0.0
kitchen.orientation.y = 0.0
kitchen.orientation.z = 0.2243729829788208
kitchen.orientation.w = 0.027017837864690385

# table
# position
table.position.x = 3.001 - 4
table.position.y = 0.987
table.position.z = 0.0
# orientation
table.orientation.x = 0.0
table.orientation.y = 0.0
table.orientation.z = 1
table.orientation.w = 0

ROBOT_LOCATION = {
    'kitchen': kitchen,
    'table': table,
}
