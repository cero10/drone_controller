#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$


import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Float64

class object:
	def __init__(self):
		self.altitude = 0.0
		self.target_altitude = 1.25
		self.pos = Twist()
		self.status = 0

	def altitude_control(self):
		if abs(self.target_altitude - self.altitude)> 0.05:
			self.pos.linear.z = self.target_altitude - self.altitude
		else:
			self.pos.linear.z = 0.0

	def get_altitude(self, msg):
		self.altitude = msg.pose.pose.position.z

	def get_status(self, msg):
		self.status = msg.status_list[0].status if len(msg.status_list) else 0
		#the above (if...) is there in case the array is zero, not to 			publish indexing errors
		#print(self.status)

	def get_new_alt(self, data):
		self.target_altitude = data.data
		#print(target_altitude)


def target_altitude_controller():
	rospy.init_node('target_altitude_controller')
	drone = object()
	rospy.Subscriber('ground_truth/state', Odometry, drone.get_altitude)
	rospy.Subscriber('move_base/status', GoalStatusArray, drone.get_status)
	rospy.Subscriber('alt_control', Float64, drone.get_new_alt)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=20)
	rate = rospy.Rate(5) # 5hz
	while not rospy.is_shutdown():
		if (drone.status == 3 or drone.status == 4 or drone.status == 5) :
			drone.altitude_control()
			#print(drone.pos)
			pub.publish(drone.pos)
		rate.sleep()

if __name__ == '__main__':
    try:
        target_altitude_controller()
    except rospy.ROSInterruptException:
        pass
