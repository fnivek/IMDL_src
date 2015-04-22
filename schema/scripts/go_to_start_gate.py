#!/usr/bin/env python

import rospy
from schema_base import schema_base
from schema.msg import pfield
from percept_generators.msg import object as object_msg
from percept_generators.msg import objectArray as objectArray_msg
from geometry_msgs.msg import PointStamped
import numpy as np
import tf

# assuming all start_gates are in the same reference frame

class schema(schema_base):
	def __init__(self, name):
		schema_base.__init__(self, name)

		self.listener = tf.TransformListener()
		self.field_strength = rospy.get_param('go_to_start_gate_pfield_strength', 3)

		# Subscribers
		self.object_sub = rospy.Subscriber('/percepts/objects', objectArray_msg, self.objectCb)

	def getDistanceToObjAndChangeFrame(self, obj):
		# Check if we are in range
		P = PointStamped()
		P.point = obj.centroid
		P.header = obj.header
		Pbase_link = PointStamped()
		try:
			Pbase_link = self.listener.transformPoint('/base_link', P)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr('TF error: %s' % e)
			return

		obj.centroid = Pbase_link.point
		obj.header.frame_id = '/base_link'

		x = np.array([Pbase_link.point.x, Pbase_link.point.y])
		return np.linalg.norm(x)

	def getClossesObj(self, objs):
		distances = map(self.getDistanceToObjAndChangeFrame, objs)
		i = distances.index(min(distances))
		return objs[i]

	def objectCb(self, objs):
		if objs.objects:
			if objs.type == objectArray_msg.START_GATE:
				# Produce pfield for closses start_gate only
				sg = self.getClossesObj(objs.objects)

				# Get the 
				field_vec = np.array([sg.centroid.x, sg.centroid.y])
				norm = np.linalg.norm(field_vec)
				field_vec = field_vec / norm
				field_vec = field_vec * self.field_strength

				field = pfield()
				field.header.stamp = rospy.get_rostime()
				field.header.frame_id = 'base_link'
				field.vector.x = field_vec[0]
				field.vector.y = field_vec[1]
				field.vector.z = 0
				field.decay_time = 0.1


			
if __name__ == '__main__':
	node = schema('go_to_start_gate')
	rospy.spin()