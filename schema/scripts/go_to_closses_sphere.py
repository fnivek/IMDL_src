#!/usr/bin/env python

import rospy
from schema_base import schema_base
from schema.msg import pfield

class schema(schema_base):
	def __init__(self, name):
		schema_base.__init__(self, name)

	def updateCb(self, event):
		pass

if __name__ == '__main__':
	node = schema('go_to_closses_sphere')
	rospy.spin()