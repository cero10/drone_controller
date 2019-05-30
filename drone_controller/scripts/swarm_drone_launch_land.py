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
import time
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback



#def launch_land(self):
drone_status1 = Navdata
drone_status2 = Navdata
drone_status3 = Navdata
pub_launch1 = rospy.Publisher('quadrotor1/ardrone/takeoff', Empty, queue_size=1)
pub_land1 = rospy.Publisher('quadrotor1/ardrone/land', Empty, queue_size=1)
pub_launch2 = rospy.Publisher('quadrotor2/ardrone/takeoff', Empty, queue_size=1)
pub_land2 = rospy.Publisher('quadrotor2/ardrone/land', Empty, queue_size=1)
pub_launch3 = rospy.Publisher('quadrotor3/ardrone/takeoff', Empty, queue_size=1)
pub_land3 = rospy.Publisher('quadrotor3/ardrone/land', Empty, queue_size=1)


def status_update1(msg):
	global drone_status1
	drone_status1=msg.state

def status_update2(msg):
	global drone_status1
	drone_status2=msg.state

def status_update3(msg):
	global drone_status3
	drone_status3=msg.state

def takeoff():
	pub_launch1.publish(Empty())
	pub_launch2.publish(Empty())
	pub_launch3.publish(Empty())

def land():
	pub_land1.publish(Empty())
	pub_land2.publish(Empty())
	pub_land3.publish(Empty())


def swarm_drone_launch_land():
	rospy.init_node('swarm_drone_launch_land')
	rate = rospy.Rate(20) # 20hz

	rospy.Subscriber('quadrotor1/ardrone/navdata', Navdata, status_update1)
	#rospy.Subscriber('quadrotor2/ardrone/navdata', Navdata, status_update2) #they dont update on the first main loop, and there are no subsequent loops.
	#rospy.Subscriber('quadrotor3/ardrone/navdata', Navdata, status_update3)

	time.sleep(0.25)
	if drone_status1 == 2:
		takeoff()
		print('takeoff')
	else:
		land()
		print('land')
		

if __name__ == '__main__':
    swarm_drone_launch_land()
