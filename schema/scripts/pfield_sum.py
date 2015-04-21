#!/usr/bin/env python

# Sums all pfields that it is listining to and publishes result at 10 hz
# TODO make it smarter what if something publishes at 1 Hz... the Kinect cough cough

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from schema.msg import pfield
import tf

class node:
	def __init__(self):
		self.pfields = []
		self.motor_pub = rospy.Publisher("motor_cmd", Twist, queue_size = 10)

		rospy.Subscriber("/schema/avoid_pfield", pfield, self.newPfieldCb)
		rospy.Subscriber("/percepts/kinect_pfield", pfield, self.newPfieldCb)	# TODO move from percepts to SCHEMA

		rospy.Timer(rospy.Duration(0.05), self.updateCb)

		self.listener = tf.TransformListener()

	def updateCb(self, event):
		now = event.current_real.to_sec()

		# Get rid of out of date pfields
		self.pfields = filter(lambda p: now - p.header.stamp.to_sec() < p.decay_time, self.pfields)

		# Vector Sum 
		vector_sum = [0, 0]
		for p in self.pfields:
			vec_in = Vector3Stamped()
			vec_out = Vector3Stamped()
			vec_in.header = p.header
			vec_in.vector = p.vector

			#print 'Vec in:\n',p
			# Convert to base link
			try:
				vec_out = self.listener.transformVector3('/base_link', vec_in)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				rospy.logerr("Can\'t sum vector with frame {0} because tf error: {1}".format(p.header.frame_id, e))
				continue			

			#print 'Vec out:\n', vec_out
			# Sum
			vector_sum[0] = vector_sum[0] + vec_out.vector.x
			vector_sum[1] = vector_sum[1] + vec_out.vector.y

		#print 'Vector sum', vector_sum

		# Publish motor cmd
		#	Any Y component gets maped to rotation
		#	Any X component gets maped to linear motion
		msg = Twist()
		msg.linear.x = vector_sum[0]
		msg.angular.z = vector_sum[1]
		self.motor_pub.publish(msg)

	def newPfieldCb(self, field):
		#print 'newField'
		self.pfields.append(field)



def main():
	# Ros initilization
	#Init ros
	rospy.init_node('pfield_sum', anonymous=False)

	this_node = node()

	rospy.spin()

if __name__ == '__main__':
	main()