from geometry_msgs.msg import Point, Pose, Quaternion
import rospy
# geometry_msgs/Point position
# geometry_msgs/Quaternion orientation


kitchen = Pose()
table = Pose()

# kitchen
# position
kitchen.position.x = 2.8303239345550537
kitchen.position.y = -4.334967613220215
kitchen.position.z = 0.0
# orientation
kitchen.orientation.x = 0.0
kitchen.orientation.y = 0.0
kitchen.orientation.z = 0.6532101611290281
kitchen.orientation.w = 0.7571766540232135

# table
# position
table.position.x = 1.1120694875717163
table.position.y = -0.19343096017837524
table.position.z = 0.0
# orientation
table.orientation.x = 0.0
table.orientation.y = 0.0
table.orientation.z = -0.9999556849509784
table.orientation.w = 0.009414251654787098

ROBOT_LOCATION = {
    'kitchen': kitchen,
    'table': table,
}
0.028918243631703287