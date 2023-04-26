from geometry_msgs.msg import Point, Pose, Quaternion
import rospy
# geometry_msgs/Point position
# geometry_msgs/Quaternion orientation


kitchen = Pose()
table = Pose()

# kitchen
# position
kitchen.position.x = 0.213
kitchen.position.y = 0.23123
kitchen.position.z = 0.0
# orientation
kitchen.orientation.x = .123123
kitchen.orientation.y = .23123
kitchen.orientation.z = .23123
kitchen.orientation.w = .23123

# table
# position
table.position.x = 0.213
table.position.y = 0.23123
table.position.z = 0.0
# orientation
table.orientation.x = .123123
table.orientation.y = .23123
table.orientation.z = .23123
table.orientation.w = .23123

ROBOT_LOCATION = {
    'kitchen': kitchen,
    'table': table,
}
