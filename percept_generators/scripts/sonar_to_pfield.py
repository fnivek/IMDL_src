#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from percept_generators.msg import pfield
from sensor_msgs.msg import Range
import numpy
import math

from collections import deque

sizeOfDeques = 3
sonar_data = [deque() for x in range(4)]
pfield_pub = rospy.Publisher("sonar_pfield", pfield, queue_size = 10)
range_pub = rospy.Publisher("sonar_range", Range, queue_size = 10)
range_tf_frames = ('front_sonar_link', 'back_sonar_link', 'front_right_sonar_link', 'front_left_sonar_link')

def ticksToMeters(ticks):
	return 0.00000272602 * ticks + -0.08904109589

def avg(seq):
	return reduce(lambda x, y: x + y, seq) / len(seq)

def scaleSonarData(sonar):
	if sonar <= 40000:
		return 40001
	elif sonar >= 1500000:
		return 1500000
	else:
		return sonar


def sonar_data_cb(msg):
	global sonar_data 
	global pfield_pub

	data = msg.data
	index = 0

	if len(sonar_data[0]) >= sizeOfDeques:
		for sonar in data:
			sonar_data[index].popleft()
			sonar_data[index].append(sonar)
			index += 1
	else:
		for sonar in data:
			sonar_data[index].append(sonar)
			index += 1

	# Get avg
	sonar_avg = map(avg, sonar_data)
	sonar_avg = map(scaleSonarData, sonar_avg)

	# Publish avg data to a Range msg to view in Rviz
	range_msg = [Range() for x in range(4)]
	now = rospy.Time.now()
	for i, r in enumerate(range_msg):
		r.header.stamp = now
		r.header.frame_id = range_tf_frames[i]
		r.radiation_type = Range.ULTRASOUND
		r.field_of_view = 15 * math.pi / 180
		r.min_range = 0.02
		r.max_range = 4
		r.range = ticksToMeters(sonar_avg[i])
		range_pub.publish(r)

	# Generate potential fields
		# Sonars are orderd front, back, front right, front left
		# 					0*,    180*, 30*,		  -30*
		# An inverse tangent function is used to calculate a potential field
		# K * cot((pi/2920000)*(x - 40000))
	scalar_pfields = map( lambda avg: 0.7 / math.tan(1.07588789506499e-6 * avg + 0.0430355158025),
							sonar_avg)
	#scalar_pfields = map( lambda avg: .1 * ((-1/146000) * (avg - 40000) + 10), sonar_avg)

	pfield_msg = pfield()
	pfield_msg.vector.x = scalar_pfields[0] * -1 + scalar_pfields[1] + scalar_pfields[2] * -math.cos(math.pi/6) + scalar_pfields[3] * -math.cos(-math.pi/6)
	pfield_msg.vector.y = 											   scalar_pfields[2] * -math.sin(math.pi/6) + scalar_pfields[3] * -math.sin(-math.pi/6)
	pfield_msg.vector.z = 0
	pfield_msg.header.stamp = rospy.Time.now()
	pfield_msg.header.frame_id = "/base_link"
	pfield_msg.decay_time = 0.1

	pfield_pub.publish(pfield_msg)
	

def main():
	# Ros initilization
	#Init ros
	rospy.init_node('sonar_to_pfield', anonymous=False)

	rospy.Subscriber("/hardware_interface/sonar_data", Int32MultiArray, sonar_data_cb)

	rospy.spin()



if __name__ == '__main__':
	main()