#!/usr/bin/env python

import rospy
from schema_base import schema_base
from schema.msg import pfield

class schema(schema_base):
	def __init__(self, name):
		schema_base.__init__(self, name)

		self.max_pfield = rospy.get_param('~spin_max_pfield', 1.0)
		self.timer = rospy.Timer(rospy.Duration(0.1), self.updateCb)

	def updateCb(self, event):
		spin = pfield()
		spin.header.stamp = rospy.get_rostime()
		spin.header.frame_id = '/base_link'
		spin.vector.x = 0
		spin.vector.y = self.max_pfield
		spin.vector.z = 0
		spin.decay_time = 0.1

		self.publishPfield(spin)

if __name__ == '__main__':
	node = schema('spin')
	rospy.spin()