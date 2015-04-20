#!/usr/bin/env python

import rospy
from percept_generators.msg import pfield
from schema.msg import schema_state

class schema_base:
	def __init__(self, name, update_rate):
		rospy.init_node(name, anonymous=False)

		self.name = name
		self.update_rate = update_rate
		self.state = 0			# 0 is inhibited 1 is active

		self.pfield_pub = rospy.Publisher(name + '_pfield', pfield, queue_size = 10)
		self.state_sub = rospy.Subscriber('schema_state', schema_state, self.schemaStateCb)

		self.timer = rospy.Timer(rospy.Duration(update_rate), self.updateCb)

	def schemaStateCb(self, state):
		if state.schema == self.name:
			self.state = state.state

	def publishPfield(self, field):
		if self.state != schema_state.INHIBIT:
			self.pfield_pub.publish(field)

	def updateCb(self, event):
		pass