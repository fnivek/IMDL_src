#!/usr/bin/env python

import rospy
from schema_base import schema_base
from schema.msg import pfield
import numpy as np

class schema(schema_base):
	def __init__(self, name):
		schema_base.__init__(self, name)
		self.max_pfield = rospy.get_param('wander_max_pfield', 3)
		rospy.Timer(rospy.Duration(0.1), self.updateCb)

	def updateCb(self, event):
		# Generate random pfield
		mag = np.random.random_sample() * self.max_pfield
		angle = np.random.random_sample() * np.pi - np.pi / 2
		field = pfield()
		field.header.stamp = rospy.get_rostime()
		field.header.frame_id = 'base_link'
		field.vector.x = np.cos(angle) * mag
		field.vector.y = np.sin(angle) * mag
		field.vector.z = 0
		field.decay_time = 0.1
		
		self.publishPfield(field)

if __name__ == '__main__':
	node = schema('wander')
	rospy.spin()