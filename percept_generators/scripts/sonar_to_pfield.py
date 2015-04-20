#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from percept_generators.msg import pfield
from sensor_msgs.msg import Range
import numpy as np
import math

from collections import deque

"""
	This node produces potential fields from sonar data
		The x component of the pfield is converted into a linear x motion while 
		the y component is converted into a torque 
"""

class node:
	range_tf_frames = ('front_sonar_link', 'back_sonar_link', 'front_right_sonar_link', 'front_left_sonar_link')

	def __init__(self):
		rospy.init_node('sonar_to_pfield', anonymous=False)
		rospy.Subscriber("/hardware_interface/sonar_data", Int32MultiArray, self.sonar_data_cb)

		self.sizeOfDeques = 3
		self.sonar_data = [deque() for x in range(4)]
		self.pfield_pub = rospy.Publisher("sonar_pfield", pfield, queue_size = 10)
		self.range_pub = rospy.Publisher("sonar_range", Range, queue_size = 10)

		# Keep in mind max ticks produces the min pfield and min ticks produces max pfield
		self.max_ticks = rospy.get_param('max_ticks', 500000)
		self.mid_ticks = rospy.get_param('mid_ticks', 250000)
		self.min_ticks = rospy.get_param('min_ticks', 150000)
		self.max_pfield = rospy.get_param('max_pfield', 10)
		self.mid_pfield = rospy.get_param('mid_pfield', 1)

		# Generate coeficients for hyperbolic interpolation
		#	To generate the equation we set min ticks to be the vertex
		#	and the other two points as points to be passed through
		A = np.matrix([[2.0 * self.max_ticks, 			1.0, 						0.0],		# Vertex constraint
					   [(1.0 * self.mid_ticks) ** 2,	(1.0 * self.mid_ticks),		1.0],		# Mid point constraint
					   [(1.0 * self.min_ticks) ** 2,	(1.0 * self.min_ticks),		1.0]]		# Endpoint constraint
			)
		print 'A:\n', A, '\n'
		A_inv = A.getI()	# Invert A
		print 'A_inv:\n', A_inv, '\n'
		Y = np.matrix( [[0.0], [self.mid_pfield], [self.max_pfield]])			# Y column vector
		print 'Y:\n', Y, '\n'
		self.parabolic_coeffs = A_inv * Y
		print 'Coeffs:\n', self.parabolic_coeffs, '\n'


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
		elif ticks < self.min_ticks:
			return self.max_pfield
		else:
			return (np.matrix([ticks ** 2, ticks, 1]) * self.parabolic_coeffs).item(0)
			

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
			r.header.frame_id = node.range_tf_frames[i]
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
		print pfield_mags

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

		self.pfield_pub.publish(pfield_msg)
	

def main():
	# Ros initilization
	#Init ros
	this_node = node()

	rospy.spin()



if __name__ == '__main__':
	main()