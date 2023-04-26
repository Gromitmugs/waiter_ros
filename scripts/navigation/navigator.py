import actionlib
import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from location_param import ROBOT_LOCATION

# Data structure: receive location command with pos data type. (positions and quarternions)
# Topic nav_location:
#                   data: "KITCHEN"
# Service that stores location
#                   location:
#                           string navLocation
#                           ---
#                           geometry_msgs/Pose navLocation (position and quaternion)
#                           string data.data


class Navigator():

    def __init__(self):

        self.goal_sent = False

        # Receive command from websocket_server.py
        rospy.Subscriber("NAV_CTRL", Pose, self.callback)

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)

        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def callback(self, data):
        rospy.set_param('ROBOT_STATUS', '1')  # assuming 1 is not free
        print("NAVIGATING!")
        if data.data == 'KITCHEN':
            position = {'x': ROBOT_LOCATION['kitchen'].position.x,
                        'y': ROBOT_LOCATION['kitchen'].position.y}
            quaternion = {'r1': ROBOT_LOCATION['kitchen'].orientation.x, 'r2': ROBOT_LOCATION['kitchen'].orientation.y,
                          'r3': ROBOT_LOCATION['kitchen'].orientation.z, 'r4': ROBOT_LOCATION['kitchen'].orientation.w}
            rospy.loginfo("Going to %s at (%s, %s) pose",
                          data.data, position['x'], position['y'])
            success = self.goto(position, quaternion)

            if success:
                rospy.loginfo("Successfully Reached {0}".format(data.data))
            else:
                rospy.loginfo(
                    "The base failed to reach the desired pose at destination")

        elif data.data == 'TABLE':
            position = {'x': ROBOT_LOCATION['table'].position.x,
                        'y': ROBOT_LOCATION['table'].position.y}
            quaternion = {'r1': ROBOT_LOCATION['table'].orientation.x, 'r2': ROBOT_LOCATION['table'].orientation.y,
                          'r3': ROBOT_LOCATION['table'].orientation.z, 'r4': ROBOT_LOCATION['table'].orientation.w}
            rospy.loginfo("Going to %s at (%s, %s) pose",
                          data.data, position['x'], position['y'])
            success = self.goto(position, quaternion)

            if success:
                rospy.loginfo("Successfully Reached {0}".format(data.data))
            else:
                rospy.loginfo(
                    "The base failed to reach the desired pose at destination")

        else:
            print("The base failed to reach the desired pose")

        print("NAVIGATED!")
        rospy.set_param('ROBOT_STATUS', '0')  # assuming 0 is free

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        #rospy.init_node('navigator_listener', anonymous=True)
        rospy.init_node('nav_test', anonymous=False)
        navigator = Navigator()

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
