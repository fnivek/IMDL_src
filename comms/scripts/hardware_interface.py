#!/usr/bin/env python
import rospy
import serial
import time
from struct import *
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray

sonar_id = 'sonar_data'
iface = serial.Serial()


def left_pwm_cb(pwm):
	global iface
	paked_data = pack('<f', pwm.data)
	iface.write('left_duty%s' % paked_data)

def right_pwm_cb(pwm):
	global iface
	paked_data = pack('<f', pwm.data)
	iface.write('right_duty%s' % paked_data)

def main():
	global iface
	#Init ros
	rospy.init_node('hardware_interface', anonymous=False)

	port = rospy.get_param('port', '/dev/serial/by-id/usb-KipCDroid_HardwareInterface-if00')
	rospy.loginfo('Attempting to connect to %s' % port)

	while not rospy.is_shutdown():
		try:
			iface = serial.Serial(port, 115200, timeout=1)
			rospy.loginfo('Connected to %s' % port)
			break;
		except:
			rospy.logerr('Failed to connect to %s' % port)
			time.sleep(1)

	# Send anything to hardware interface to wake it up
	iface.write('WakeUp')

	# Initilize publishers and subscribers
	rospy.Subscriber("left_duty", Float32, left_pwm_cb)
	rospy.Subscriber("right_duty", Float32, right_pwm_cb)

	sonar_pub = rospy.Publisher("sonar_data", Int32MultiArray, queue_size = 10)

	# Main loop
	while not rospy.is_shutdown():
		# See if anythings in the in buffer
		in_buf = iface.inWaiting()


		if in_buf:
			# We got something!
			input = iface.read(in_buf)

			# Is it sonar data?
			sonar_index = input.find(sonar_id)
			if(sonar_index != -1):
				# TODO: Check if all the data is here
				# Grab data (four 32 bit values is 25 chars plus 10 Chars for sonar_data)
				data = input[sonar_index + 10 : len(input)]
				# Unpack 4 little-endian uint32's
				ticks = unpack('<LLLL', data)

				# publish the data
				msg = Int32MultiArray()
				msg.data = [ticks[0], ticks[1], ticks[2], ticks[3]]
				sonar_pub.publish(msg)




	iface.close();




	rospy.spin()

if __name__ == '__main__':
	main()