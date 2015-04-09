#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

from collections import deque

sizeOfDeques = 10
sonar_data = [deque() for x in range(4)]
sonar_avg = [0 for x in range(4)]

def avg(seq):
	return reduce(lambda x, y: x + y, seq) / len(seq)
	pass

def sonar_data_cb(msg):
	global sonar_data 
	global sonar_avg

	data = msg.data
	index = 0

	if len(sonar_data[0]) >= sizeOfDeques:
		for sonar in data:
			sonar_data[index].popleft()
			sonar_data[index].append(sonar)
			index += 1
	else:
		for sonar in data:
			print 'what'
			sonar_data[index].append(sonar)
			index += 1

	sonar_avg = map(avg, sonar_data)
	


def main():
	# Ros initilization
	#Init ros
	rospy.init_node('sonar_to_pfield', anonymous=False)

	rospy.Subscriber("sonar_data", Int32MultiArray, sonar_data_cb)

	rospy.spin()



if __name__ == '__main__':
	main()