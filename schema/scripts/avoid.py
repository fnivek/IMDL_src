#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from percept_generators.msg import pfield
from sensor_msgs.msg import Range
import numpy as np
import math
from schema_base import schema_base

from collections import deque

"""
	This node produces potential fields from sonar data
		The x component of the pfield is converted into a linear x motion while 
		the y component is converted into a torque 
"""

class schema(schema_base):
	range_tf_frames = ('front_sonar_link', 'back_sonar_link', 'front_right_sonar_link', 'front_left_sonar_link')

	def __init__(self, name):
		schema_base.__init__(self, name)

		self.sizeOfDeques = 3
		self.sonar_data = [deque() for x in range(4)]
		self.range_pub = rospy.Publisher("sonar_range", Range, queue_size = 10)

		# Keep in mind max ticks produces the min pfield and min ticks produces max pfield
		self.max_ticks = rospy.get_param('~max_ticks', 500000)
		self.mid_ticks = rospy.get_param('~mid_ticks', 120000)
		self.min_ticks = rospy.get_param('~min_ticks', 100000)
		self.max_pfield = rospy.get_param('~max_pfield', 20)
		self.mid_pfield = rospy.get_param('~mid_pfield', 3)

		# Piece wise linear approximation of exponintial function
		m1 = 1.0 * self.mid_pfield / (self.mid_ticks - self.max_ticks)
		b1 = -1.0 * m1 * self.max_ticks
		self.close_line = np.matrix([m1, b1])
		m2 = 1.0 * (self.mid_pfield - self.max_pfield) / (self.mid_ticks - self.min_ticks)
		b2 = self.max_pfield - (1.0 * m2 * self.min_ticks) 
		self.far_line = np.matrix([m2, b2])

		rospy.Subscriber("/hardware_interface/sonar_data", Int32MultiArray, self.sonar_data_cb)


	def ticksToMeters(self, ticks):
		return 0.00000272602 * ticks + -0.08904109589

	def avg(self, seq):
		return reduce(lambda x, y: x + y, seq) / len(seq)

	def scaleSonarData(self, ticks):
		if ticks <= 40000:
			return 40001
		elif ticks >= 1500000:
			return 1500000
		else:
			return ticks

	def generatePfieldMagnitude(self, ticks):
		if ticks > self.max_ticks:
			return 0
		elif ticks > self.mid_ticks:
			return (self.close_line * np.matrix([[ticks], [1]])).item(0)
		elif ticks > self.min_ticks:
			return (self.far_line * np.matrix([[ticks], [1]])).item(0)
		else:
			return self.max_pfield
			

	def sonar_data_cb(self, msg):
		data = msg.data
		index = 0

		if len(self.sonar_data[0]) >= self.sizeOfDeques:
			for sonar in data:
				self.sonar_data[index].popleft()
				self.sonar_data[index].append(sonar)
				index += 1
		else:
			for sonar in data:
				self.sonar_data[index].append(sonar)
				index += 1

		# Get avg
		sonar_avg = map(self.avg, self.sonar_data)
		sonar_avg = map(self.scaleSonarData, sonar_avg)

		# Publish avg data to a Range msg to view in Rviz
		range_msg = [Range() for x in range(4)]
		now = rospy.Time.now()
		for i, r in enumerate(range_msg):
			r.header.stamp = now
			r.header.frame_id = schema.range_tf_frames[i]
			r.radiation_type = Range.ULTRASOUND
			r.field_of_view = 15 * math.pi / 180
			r.min_range = 0.02
			r.max_range = 4
			r.range = self.ticksToMeters(sonar_avg[i])
			self.range_pub.publish(r)

		# Generate potential fields
			# Sonars are orderd front, back, front right, front left
			# 					0*,    180*, 30*,		  -30*
			# An inverse tangent function is used to calculate a potential field
			# K * cot((pi/2920000)*(x - 40000))
		pfield_mags = map(self.generatePfieldMagnitude, sonar_avg)
		#print pfield_mags

		#pfield_mags = map( lambda avg: 0.7 / math.tan(1.07588789506499e-6 * avg + 0.0430355158025),
		#						sonar_avg)
		#pfield_mags = map( lambda avg: .1 * ((-1/146000) * (avg - 40000) + 10), sonar_avg)

		pfield_msg = pfield()
		pfield_msg.vector.x = pfield_mags[0] * -1 + pfield_mags[1] + pfield_mags[2] * -math.cos(math.pi/6) + pfield_mags[3] * -math.cos(-math.pi/6)
		pfield_msg.vector.y = 										 pfield_mags[2] * -math.sin(math.pi/6) + pfield_mags[3] * -math.sin(-math.pi/6)
		pfield_msg.vector.z = 0
		pfield_msg.header.stamp = rospy.Time.now()
		pfield_msg.header.frame_id = "/base_link"
		pfield_msg.decay_time = 0.1

		self.publishPfield(pfield_msg)
	

def main():
	# Ros initilization
	#Init ros
	node = schema('avoid')

	rospy.spin()



if __name__ == '__main__':
	main()