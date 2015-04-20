#!/usr/bin/env python

import rospy
from schema_base import schema_base
from percept_generators.msg import pfield

class schema(schema_base):
	def __init__(self, name):
		schema_base.__init__(self, name)

		self.timer = rospy.Timer(rospy.Duration(0.1), self.updateCb)

	def updateCb(self, event):
		pass

if __name__ == '__main__':
	node = schema('spin')
	rospy.spin()