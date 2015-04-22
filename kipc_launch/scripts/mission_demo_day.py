#!/usr/bin/env python
import rospy
from schema.msg import schema_state
from percept_generators.msg import object as object_msg
from percept_generators.msg import objectArray as objectArray_msg
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

		old_time = rospy.Time.from_sec(rospy.get_time() - 60)  # One minute ago
		self.last_sphere = object_msg()
		self.last_start_gate = object_msg()
		self.last_sphere.header.stamp = old_time
		self.last_start_gate.header.stamp = old_time

		self.state = 'start'

		self.timeout = rospy.Duration.from_sec(5)
		self.in_range_of_start_gate_distance = 0.7
		self.in_range_of_sphere_distance = 0.55

		self.schema_names = ('avoid', 'wander', 'go_to_closses_sphere', 'go_to_start_gate', 'spin', 'forward')



		self.schema_state_pub = rospy.Publisher('/schema/schema_state', schema_state, queue_size = 10)
		rospy.Timer(rospy.Duration(0.1), self.updateCb)
		self.object_sub = rospy.Subscriber('/percepts/objects', objectArray_msg, self.objectCb)

	def updateCb(self, even):
		now = rospy.get_rostime()

		## Start State
		if self.state == 'start':
			self.publishSchemaStates('')

			## Just go to next state!
			self.state = 'search_for_start_gate'
			print 'Starting in search_for_start_gate'

		## Search for the start gate
		elif self.state == 'search_for_start_gate':
			self.publishSchemaStates(['avoid', 'wander'])

			if now - self.last_start_gate.header.stamp < self.timeout:
				self.state = 'go_to_start_gate'
				print 'Found start gate switching to go_to_start_gate'

		## Go to the start gate
		elif self.state == 'go_to_start_gate':
			self.publishSchemaStates(['avoid', 'go_to_start_gate'])

			# Did we loose sight of it?
			if now - self.last_start_gate.header.stamp > self.timeout:
				self.state = 'search_for_start_gate'
				print 'Lost sight of start gate switching to go_to_start_gate'
				return

			# Check if we are in range
			d = self.getDistanceToObj(self.last_start_gate)
			if d < self.in_range_of_start_gate_distance:
				self.state = 'search_for_sphere'
				print 'Made it to start gate now search_for_sphere'

		## Search for a sphere
		elif self.state == 'search_for_sphere':
			self.publishSchemaStates(['avoid', 'wander'])

			if now - self.last_sphere.header.stamp < self.timeout:
				self.state = 'go_to_closses_sphere'
				print 'Found a sphere switching to go_to_closses_sphere'

		## Go to closses sphere
		elif self.state == 'go_to_closses_sphere':
			self.publishSchemaStates(['avoid', 'go_to_closses_sphere'])

			# Did we loose sight of it?
			if now - self.last_sphere.header.stamp > self.timeout:
				self.state = 'search_for_sphere'
				print 'Lost sight of sphere switching to search_to_sphere'
				return

			# Check if we are in range
			d = self.getDistanceToObj(self.last_sphere)
			#print d
			#print self.last_sphere
			if d < self.in_range_of_sphere_distance:
				self.state = 'victory'
				print 'Made it to sphere Victory!'
				self.victory_time = now

		## Victory
		elif self.state == 'victory':
			self.publishSchemaStates(['avoid', 'spin'])

			if now - self.victory_time > self.timeout:
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

	def getClossesObj(self, objs):
		distances = map(self.getDistanceToObj, objs)
		i = distances.index(min(distances))
		return objs[i]

	def getMostRecentObj(self, objs):
		now = rospy.get_rostime()
		times = map(lambda obj: now - obj.header.stamp, objs)
		i = times.index(min(times))
		return objs[i]

	def getDistanceToObj(self, obj):
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

		x = np.array([Pbase_link.point.x, Pbase_link.point.y])
		return np.linalg.norm(x)

	def objectCb(self, objs):
		if objs.objects:	# Make sure its not empty
			if objs.type == objectArray_msg.SPHERE:
				self.last_sphere = self.getClossesObj(objs.objects)
			elif objs.type == objectArray_msg.START_GATE:
				self.last_start_gate = self.getMostRecentObj(objs.objects)



if __name__ == '__main__':
	mission = mission_demo_day()
	rospy.spin()