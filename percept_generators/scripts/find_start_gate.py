#!/usr/bin/env python
import rospy
from percept_generators.msg import object as object_msg
import numpy as np

# All cylinders are assumed to be in the same reference frame so no transforms are required

# This mission is the demo day FSM
class mission_demo_day:
	def __init__(self):
		rospy.init_node('find_start_gate', anonymous=False)
		# Publishers
		self.object_pub = rospy.Publisher('/percept_generators/objects', object_msg, queue_size = 10)
		# Subscribers
		self.object_sub = rospy.Subscriber('/percept_generators/objects', object_msg, self.objectCb)
		# Parameters
		self.same_cylinder_distance = rospy.get_param('~same_cylinder_distance', 0.1) # to be to different cylinders they must be this far apart


		self.cylinders = []
		


	# Check the distance the cylinders are apart
	def sameCylinder(self, cylinder1, cylinder2):
		distance = np.linalg.norm(np.array(
			[cylinder1.centroid.x - cylinder2.centroid.x, 
			 cylinder1.centroid.y - cylinder2.centroid.y,
			 cylinder1.centroid.z - cylinder2.centroid.z]))

		if distance < self.same_cylinder_distance:
			return True
		else:
			return False

	def objectCb(self, obj):
		if obj.type == object_msg.CYLINDER:
			# Put the cylinder in the clinders list
			for i, cyl in enumerate(self.cylinders):
				if self.sameCylinder(obj, cyl):
					# Replace the current cylinder with its newer self
					self.cylinders[i] = obj
					return

			# If we are this far then its a new cylinder yay
			self.cylinders.append(obj)
			print self.cylinders


if __name__ == '__main__':
	node = mission_demo_day()
	rospy.spin()
