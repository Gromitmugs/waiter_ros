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
table.position.x = 1.0408189296722412
table.position.y = -0.30611562728881836
table.position.z = 0.0
# orientation
table.orientation.x = 0.0
table.orientation.y = 0.0
table.orientation.z = 0.9999037310001453
table.orientation.w = 0.013875472315893575

ROBOT_LOCATION = {
    'kitchen': kitchen,
    'table': table,
}
