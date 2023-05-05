#!/usr/bin/env python3

"""
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   
   ----------------------------------
   Portions of this code borrowed from the arbotix_python diff_controller.
   
diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

import rospy
import roslib
roslib.load_manifest('differential_drive')
from math import sin, cos, pi

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Float64MultiArray


#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',50.0)  # the rate at which to publish the transform
        self.ticks_per_rev = float(rospy.get_param('~ticks_meter', 60))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.245)) # The wheel base width in meters
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        self.odom_child_frame_id = rospy.get_param('~odom_child_frame_id', 'base_link')
 
        self.wheel_radius = float(rospy.get_param('~wheel_radius', 0.035))
        self.wheel_dist = float(rospy.get_param('~wheel_distance', 0.503))
        
        self.t_delta = rospy.Duration(1.0/self.rate)
        now = rospy.Time.now()
        self.t_next = now + self.t_delta

        self.joint_states_names = ["left_wheel_joint", "right_wheel_joint"]
        self.joint_states_pos = [0, 0]
        self.joint_states_vel = [0, 0]

        # internal data
        self.odom_pose = [0, 0, 0]
        self.last_theta = 0

        self.prev_pos_left = 0
        self.prev_pos_right = 0

        self.pos_left = 0 # encoder tick
        self.pos_right = 0

        self.d_left = 0 # distance traveled
        self.d_right = 0

        self.v_left = 0 # wheel velocity
        self.v_right = 0

        self.x = 0  # position in xy plane 
        self.y = 0
        
        self.then = now
        
        # subscriptions
        rospy.Subscriber("/raw_vel", Float64MultiArray, self.rawVelCallback)
        rospy.Subscriber("/raw_pos", Float64MultiArray, self.rawPosCallback)

        self.odomPub = rospy.Publisher("/odom_wheel", Odometry, queue_size=10)
        self.jointStatePub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        now = rospy.Time.now() ## + rospy.Duration(5)
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            Vx = (self.v_left + self.v_right) / 2
            Vz = (self.v_right - self.v_left) / self.wheel_dist

            delta_pos_left = self.pos_left - self.prev_pos_left
            delta_pos_right = self.pos_right - self.prev_pos_right

            self.prev_pos_left = self.pos_left
            self.prev_pos_right = self.pos_right

            rad_left = delta_pos_left * 2 * pi / self.ticks_per_rev
            rad_right = delta_pos_right * 2 * pi / self.ticks_per_rev

            delta_s = self.wheel_radius * (rad_left + rad_right) / 2.0
            theta = self.wheel_radius * (rad_right - rad_left) / self.wheel_dist

            delta_theta = theta - self.last_theta
            # self.last_theta = delta_theta

            self.odom_pose[0] += delta_s * cos(self.odom_pose[2] + (delta_theta / 2.0))
            self.odom_pose[1] += delta_s * sin(self.odom_pose[2] + (delta_theta / 2.0))
            self.odom_pose[2] += delta_theta

            self.x = self.odom_pose[0]
            self.y = self.odom_pose[1]
            
            # publish the odom information
            quaternion = quaternion_from_euler(0, 0, self.odom_pose[2])
            # self.odomBroadcaster.sendTransform(
            #     (self.x, self.y, 0), #translation
            #     (quaternion[0], quaternion[1], quaternion[2], quaternion[3]), #rotation x,y,z,w
            #     now, #time
            #     self.odom_child_frame_id, #child
            #     self.odom_frame_id #parent
            # ) #let the efk handle tf
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0

            quaternion_odom = Quaternion()
            quaternion_odom.x = quaternion[0]
            quaternion_odom.y = quaternion[1]
            quaternion_odom.z = quaternion[2]
            quaternion_odom.w = quaternion[3]
            odom.pose.pose.orientation = quaternion_odom

            odom.child_frame_id = self.odom_child_frame_id
            odom.twist.twist.linear.x = Vx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = Vz
            self.odomPub.publish(odom)
            
            # joint states
            self.joint_states_pos[0] += rad_left
            self.joint_states_pos[1] += rad_right

            self.joint_states_vel[0] = self.v_left
            self.joint_states_vel[1] = self.v_right

            joint_states = JointState()
            joint_states.name = self.joint_states_names
            joint_states.header.frame_id = self.base_frame_id
            joint_states.position = self.joint_states_pos
            joint_states.velocity = self.joint_states_vel
            joint_states.header.stamp = now
            
            self.jointStatePub.publish(joint_states)

    #############################################################################
    def rawVelCallback(self, msg):
    #############################################################################
        self.v_left = msg.data[0]
        self.v_right = msg.data[1]
        
    #############################################################################
    def rawPosCallback(self, msg):
    #############################################################################
        tick_left = msg.data[0]
        tick_right = msg.data[1]
        self.pos_left = tick_left
        self.pos_right = tick_right
        self.d_left = 2 * pi * self.wheel_radius * (tick_left / self.ticks_per_rev)
        self.d_right = 2 * pi * self.wheel_radius * (tick_right / self.ticks_per_rev)

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
