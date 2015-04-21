#!/usr/bin/env python
import rospy
from percept_generators.msg import object as object_msg
from percept_generators.msg import objectArray as objectArray_msg
import numpy as np

# All cylinders are assumed to be in the same reference frame so no transforms are required

# This mission is the demo day FSM
class mission_demo_day:
	def __init__(self):
		rospy.init_node('find_start_gate', anonymous=False)

		# Parameters
		self.start_gate_min_width = rospy.get_param('~start_gate_min_width', 0.25)
		self.start_gate_max_width = rospy.get_param('~start_gate_max_width', 1)

		# Publishers
		self.object_pub = rospy.Publisher('/percepts/raw_objects', object_msg, queue_size = 10)
		# Subscribers
		self.object_sub = rospy.Subscriber('/percepts/objects', objectArray_msg, self.objectsCb)
		
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

	def objectsCb(self, objects):
		if objects.type == objectArray_msg.CYLINDER:
			# Find a start gate
			for i, cyl1 in enumerate(objects.objects):
				for j, cyl2 in enumerate(objects.objects):
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


if __name__ == '__main__':
	node = mission_demo_day()
	rospy.spin()
