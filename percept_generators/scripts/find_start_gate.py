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
		self.object_pub = rospy.Publisher('/percepts/objects', object_msg, queue_size = 10)
		# Subscribers
		self.object_sub = rospy.Subscriber('/percepts/objects', object_msg, self.objectCb)
		# Parameters
		self.same_cylinder_distance = rospy.get_param('~same_cylinder_distance', 0.1) # to be to different cylinders they must be this far apart
		self.cylinder_lifetime = rospy.get_param('~cylinder_lifetime', 5.0)
		self.start_gate_min_width = rospy.get_param('~start_gate_min_width', 0.25)
		self.start_gate_max_width = rospy.get_param('~start_gate_max_width', 1)

		self.cylinders = []		# Do the callbacks run in different threads? Will this cause a problem???

		rospy.Timer(rospy.Duration(0.1), self.updateCb)
		
	def getDistanceBetweenCylinders(self, cyl1, cyl2):
		return np.linalg.norm(np.array(
						[cyl1.centroid.x - cyl2.centroid.x, 
						 cyl1.centroid.y - cyl2.centroid.y,
						 cyl1.centroid.z - cyl2.centroid.z]))

	def getMidpointOfStartGate(self, cyl1, cyl2):
		return np.array(
			[(cyl1.centroid.x + cyl2.centroid.x) / 2,
			 (cyl1.centroid.y + cyl2.centroid.y) / 2,
			 (cyl1.centroid.z + cyl2.centroid.z) / 2])

	# Check the distance the cylinders are apart
	def isSameCylinder(self, cyl1, cyl2):
		distance = self.getDistanceBetweenCylinders(cyl1, cyl2)

		if distance < self.same_cylinder_distance:
			return True
		else:
			return False

	def isOldCylinder(self, cyl):
		now = rospy.get_time()
		if now - cyl.header.stamp.to_sec() > self.cylinder_lifetime:
			return True
		else:
			return False

	def objectCb(self, obj):
		if obj.type == object_msg.CYLINDER:
			# Put the cylinder in the clinders list
			for i, cyl in enumerate(self.cylinders):
				if self.isSameCylinder(obj, cyl):
					# Replace the current cylinder with its newer self
					self.cylinders[i] = obj
					return

			# If we are this far then its a new cylinder yay
			self.cylinders.append(obj)
			#print self.cylinders

	def updateCb(self, event):
		# Remove old cylinders
		self.cylinders = filter(lambda cyl: not self.isOldCylinder(cyl), self.cylinders)

		# Find a start gate
		for i, cyl1 in enumerate(self.cylinders):
			for j, cyl2 in enumerate(self.cylinders):
				if i == j:
					continue
				distance = self.getDistanceBetweenCylinders(cyl1, cyl2)
				if distance <= self.start_gate_max_width and distance >= self.start_gate_min_width:
					# publish a start gate
					gate = object_msg()
					gate.header.stamp = rospy.get_rostime()
					gate.header.frame_id = cyl1.header.frame_id
					gate.type = object_msg.START_GATE
					centroid = self.getMidpointOfStartGate(cyl1, cyl2)
					gate.centroid.x = centroid[0]
					gate.centroid.y = centroid[1]
					gate.centroid.z = centroid[2]
					self.object_pub.publish(gate)
					print gate



if __name__ == '__main__':
	node = mission_demo_day()
	rospy.spin()
