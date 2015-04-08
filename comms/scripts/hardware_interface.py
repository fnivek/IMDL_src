#!/usr/bin/env python
import rospy
import serial
import time
from struct import *

sonar_id = 'sonar_data'

def main():
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
				print ticks




	iface.close();




	rospy.spin()

if __name__ == '__main__':
	main()