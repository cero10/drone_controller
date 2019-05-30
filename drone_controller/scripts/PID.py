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
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PID_controller:
	def __init__(self):
		self.iRotZ = 0
		self.iVelX = 0
		self.iVelY = 0
		self.iVelZ = 0
		self.oRotZ = 0
		self.oVelX = 0
		self.oVelY = 0
		self.oVelZ = 0
		self.velocities = Twist()
		self.position = Odometry()
		self.target_pos = Twist()
		self.posX = 0.0
		self.posY = 0.0
		self.posZ = 0.0
		

	def callback(self, msg):
		self.RotZ = msg.rotZ
		self.VelX = msg.vx
		self.VelY = msg.vy
		self.VelZ = msg.vz
	
	def GPosition(self, msg):
		self.posX = msg.pose.pose.position.x
		self.posY = msg.pose.pose.position.y
		self.posZ = msg.pose.pose.position.z

	def velocity_update(self):
		self.velocities.linear.x = self.target_pos.linear.x - self.posX + 0.5
		self.velocities.linear.y = self.target_pos.linear.y - self.posY + 0.5
		self.velocities.linear.z = self.target_pos.linear.z - self.posZ + 0.5
		self.velocities.angular.z = self.target_pos.angular.z - self.posZ + 0.5

	def TPosition(self, msg):
		self.target_pos.linear.x = msg.linear.x
		self.target_pos.linear.y = msg.linear.y
		self.target_pos.linear.z = msg.linear.z
		self.target_pos.angular.z = msg.angular.z
		#rospy.loginfo('\n{},\n{},\n{},\n{}'.format(self.target_pos.linear.x,self.target_pos.linear.y,self.target_pos.linear.z,self.target_pos.angular.z))

	def compute(self):
	
	   #How long since we last calculated
	   now = millis();
	   Tdelta = now - lastime;
	  
	   #Compute all the working error variables
	   error = setpoint - cinput;
	   derror = (error - lasterror) / Tdelta;
	  
	   #Compute PID Output
	   coutput = kp * error + kd * derror;
	  
	   #Remember some variables for next time
	   lasterror = error;
	   lastTime = now;
	
	  
	def SetTunings(Kp, Kd):
	
		global kp, kd
		kp = Kp;
		kd = Kd;
	

def PID():

	rospy.init_node('PID')
	drone = PID_controller()
	rospy.Subscriber('ardrone/navdata', Navdata, drone.callback)
	rospy.Subscriber('ground_truth/state', Odometry, drone.GPosition)
	rospy.Subscriber('target_position', Twist, drone.TPosition)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=20)
	rate = rospy.Rate(100) # 100hz
    	while not rospy.is_shutdown():
		drone.velocity_update()
		print(drone.velocities)
		pub.publish(drone.velocities)
	# spin() simply keeps python from exiting until this node is stopped
		rate.sleep()		
		
if __name__ == '__main__':
	    PID()
