#!/usr/bin/env python

import rospy
from schema_base import schema_base
from schema.msg import pfield

class schema(schema_base):
	def __init__(self, name):
		schema_base.__init__(self, name)
		self.max_pfield = rospy.get_param('~forward_max_pfield', 1.0)
		rospy.Timer(rospy.Duration(0.1), self.updateCb)
	

	def updateCb(self, event):
		fwd = pfield()
		fwd.header.stamp = rospy.get_rostime()
		fwd.header.frame_id = '/base_link'
		fwd.vector.x = self.max_pfield
		fwd.vector.y = 0
		fwd.vector.z = 0
		fwd.decay_time = 0.1

		self.publishPfield(fwd)

if __name__ == '__main__':
	node = schema('forward')
	rospy.spin()