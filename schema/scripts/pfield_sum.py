#!/usr/bin/env python

# Sums all pfields that it is listining to and publishes result at 10 hz
# TODO make it smarter what if something publishes at 1 Hz... the Kinect cough cough

import rospy
from geometry_msgs.msg import Twist
from percept_generators.msg import pfield

class node:
	def __init__(self):
		self.pfields = []
		self.motor_pub = rospy.Publisher("motor_cmd", Twist, queue_size = 10)
		rospy.Subscriber("/percepts/sonar_pfield", pfield, self.newPfieldCb)

		rospy.Timer(rospy.Duration(0.05), self.updateCb)

	def updateCb(self, event):
		now = event.current_real.to_sec()

		# Get rid of out of date pfields
		self.pfields = filter(lambda p: now - p.header.stamp.to_sec() < p.decay_time, self.pfields)

		# Vector Sum 
		vector_sum = [0, 0]
		for p in self.pfields:
			vector_sum[0] = vector_sum[0] + p.x
			vector_sum[1] = vector_sum[1] + p.y

		print vector_sum

		# Publish motor cmd
		#	Any Y component gets maped to rotation
		#	Any X component gets maped to linear motion
		msg = Twist()
		msg.linear.x = vector_sum[0]
		msg.angular.z = vector_sum[1]
		self.motor_pub.publish(msg)

	def newPfieldCb(self, field):
		self.pfields.append(field)



def main():
	# Ros initilization
	#Init ros
	rospy.init_node('pfield_sum', anonymous=False)

	this_node = node()

	rospy.spin()

if __name__ == '__main__':
	main()