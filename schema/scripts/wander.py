#!/usr/bin/env python

import rospy
from schema_base import schema_base
from percept_generators.msg import pfield

class spin(schema_base):
	def __init__(self, name, update_rate):
		schema_base.__init__(self, name, update_rate)

	def updateCb(self, event):
		pass

if __name__ == '__main__':
	node = spin('wander', 0.1)
	rospy.spin()