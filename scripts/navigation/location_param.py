from geometry_msgs.msg import Point, Pose, Quaternion
import rospy
# geometry_msgs/Point position
# geometry_msgs/Quaternion orientation


kitchen = Pose()
table = Pose()

# kitchen
# position
kitchen.position.x = 1.0071601867675781
kitchen.position.y = -0.328400582075119
kitchen.position.z = 0.0
# orientation
kitchen.orientation.x = 0.0
kitchen.orientation.y = 0.0
kitchen.orientation.z = 0.9999715405924883
kitchen.orientation.w = 0.007544402235145885

# table
# position
table.position.x = 1.0700958967208862
table.position.y = 0.0645441859960556
table.position.z = 0.0
# orientation
table.orientation.x = 0.0
table.orientation.y = 0.0
table.orientation.z = -0.9999999508287969
table.orientation.w = 0.0003135959242781405

ROBOT_LOCATION = {
    'kitchen': kitchen,
    'table': table,
}
