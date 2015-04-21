#!/usr/bin/env python
import rospy
from schema.msg import schema_state
from percept_generators.msg import object as object_msg
from geometry_msgs.msg import PointStamped
import tf
import numpy as np

# This mission is the demo day FSM
class mission_demo_day:
	def __init__(self):
		rospy.init_node('mission_demo_day', anonymous=False)

		self.listener = tf.TransformListener()

		#int8 UNKOWN=0
		#int8 CYLINDER=1
		#int8 SPHERE=2
		#int8 START_GATE=3

		self.last_detected_object = {}
		old_time = rospy.Time.from_sec(rospy.get_time() - 60)  # One minute ago
		for x in range(object_msg.START_GATE + 1):
			obj = object_msg()
			obj.header.stamp = old_time
			obj.type = x
			self.last_detected_object[x] = obj

		self.schema_state_pub = rospy.Publisher('/schema/schema_state', schema_state, queue_size = 10)
		self.object_sub = rospy.Subscriber('/percept_generators/objects', object_msg, self.objectCb)

		self.state = 'start'

		self.timeout = rospy.Duration.from_sec(5)
		self.in_range_distance = 0.5

		self.schema_names = ('avoid', 'wander', 'go_to_closses_sphere', 'go_to_start_gate', 'spin')

		rospy.Timer(rospy.Duration(0.1), self.updateCb)

	def updateCb(self, even):
		now = rospy.get_rostime()
		start_gate = self.last_detected_object[object_msg.START_GATE]
		sphere = self.last_detected_object[object_msg.SPHERE]

		## Start State
		if self.state == 'start':
			self.publishSchemaStates('')

			## Just go to next state!
			self.state = 'search_for_start_gate'
			print 'Starting in search_for_start_gate'

		## Search for the start gate
		elif self.state == 'search_for_start_gate':
			self.publishSchemaStates(['avoid', 'wander'])

			if now - start_gate.header.stamp < self.timeout:
				self.state = 'go_to_start_gate'
				print 'Found start gate switching to go_to_start_gate'

		## Go to the start gate
		elif self.state == 'go_to_start_gate':
			self.publishSchemaStates(['avoid', 'go_to_start_gate'])

			# Did we loose sight of it?
			if now - start_gate.header.stamp.to_sec() > self.timeout:
				self.state = 'search_for_start_gate'
				print 'Lost sight of start gate switching to go_to_start_gate'
				return

			# Check if we are in range
			P = PointStamped()
			P.point = start_gate.point
			P.header = start_gate.header
			Pbase_link = PointStamped()
			try:
				Pbase_link = self.listener.transformPoint('/base_link', P)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				rospy.logerr('TF error: %s' % e)
				return

			x = np.array([Pbase_link.point.x, Pbase_link.point.y])
			d = np.linalg.norm(x)
			if d < self.in_range_distance:
				self.state = 'search_for_sphere'
				print 'Made it to start gate now search_for_sphere'

		## Search for a sphere
		elif self.state == 'search_for_sphere':
			self.publishSchemaStates(['avoid', 'wander'])

			if now - sphere.header.stamp.to_sec() < self.timeout:
				self.state = 'go_to_closses_sphere'
				print 'Found a sphere switching to go_to_closses_sphere'

		## Go to closses sphere
		elif self.state == 'go_to_closses_sphere':
			self.publishSchemaStates(['avoid', 'go_to_closses_sphere'])

			# Did we loose sight of it?
			if now - sphere.header.stamp.to_sec() > self.timeout:
				self.state = 'search_for_sphere'
				print 'Lost sight of sphere switching to go_to_sphere'
				return

			# Check if we are in range
			P = PointStamped()
			P.point = sphere.point
			P.header = sphere.header
			Pbase_link = PointStamped()
			try:
				Pbase_link = self.listener.transformPoint('/base_link', P)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				rospy.logerr('TF error: %s' % e)
				return

			x = np.array([Pbase_link.point.x, Pbase_link.point.y])
			d = np.linalg.norm(x)
			if d < self.in_range_distance:
				self.state = 'victory'
				print 'Made it to sphere Victory!'
				self.victory_time = now

		## Victory
		elif self.state == 'victory':
			self.publishSchemaStates(['avoid', 'spin'])

			if now - self.victory_time > 5:
				self.state = 'search_for_start_gate'
				print 'Time for a victory lap; Starting again'

	def publishSchemaStates(self, active_states):
		for state in self.schema_names:
			msg = schema_state()
			msg.schema = state
			if state in active_states:
				msg.state = schema_state.ACTIVE
			else:
				msg.state = schema_state.INHIBIT

			self.schema_state_pub.publish(msg)

	def objectCb(self, obj):
		print obj
		self.last_detected_object[obj.type] = obj



if __name__ == '__main__':
	mission = mission_demo_day()
	rospy.spin()