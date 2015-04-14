#!/usr/bin/env python
import rospy
import serial
import time
from struct import *
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
#from std_msgs.msg import Int16MultiArray


sonar_id = 'sonar_data'
motor_id = 'motor_pos'
iface = serial.Serial()


def left_pwm_cb(pwm):
	global iface
	paked_data = pack('<f', pwm.data)
	iface.write('left_duty%s' % paked_data)

def right_pwm_cb(pwm):
	global iface
	paked_data = pack('<f', pwm.data)
	iface.write('right_duty%s' % paked_data)

def heartbeat_cb(event):
	iface.write("beat")

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
	iface.write('beat')

	# Initilize publishers and subscribers
	rospy.Subscriber("left_duty", Float32, left_pwm_cb)
	rospy.Subscriber("right_duty", Float32, right_pwm_cb)

	sonar_pub = rospy.Publisher("sonar_data", Int32MultiArray, queue_size = 10)
	motor_pub = rospy.Publisher("motor_position", Int32MultiArray, queue_size = 10)

	# Heartbeat timer
	rospy.Timer(rospy.Duration(1), heartbeat_cb, oneshot=False)

	# Main loop
	input = ''
	while not rospy.is_shutdown():
		# See if anythings in the in buffer
		in_waiting = iface.inWaiting()


		if in_waiting:
			# We got something!
			input = input + iface.read(in_waiting)

			# What type of data do we have
			sonar_index = input.find(sonar_id)

			if(sonar_index != -1):
				# TODO: Check if all the data is here
				# Grab data (four 32 bit values is 25 chars plus 10 Chars for sonar_data)
				start_index = sonar_index + len(sonar_id)
				end_index = start_index + 16
				if len(input) > end_index:
					data = input[start_index : end_index]
					# Remove the string
					input = input.replace(input[sonar_index : end_index], '')

					# Unpack 4 little-endian uint32's
					ticks = unpack('<LLLL', data)

					# publish the data
					msg = Int32MultiArray()
					msg.data = [ticks[0], ticks[1], ticks[2], ticks[3]]
					sonar_pub.publish(msg)


			motor_index = input.find(motor_id)
			if(motor_index != -1):
				# TODO: Check if all the data is here
				# Grab data (four 32 bit values is 25 chars plus 10 Chars for sonar_data)
				start_index = motor_index + len(motor_id)
				end_index = start_index + 4
				if len(input) > end_index:
					data = input[start_index : end_index]
					# Remove the string
					input = input.replace(input[motor_index : end_index], '')

					# Unpack 2 little-endian uint16's
					motor_pos = unpack('<HH', data)
					temp0 = motor_pos[0]
					temp1 = motor_pos[1]
					print temp1

					# publish the data
					msg = Int32MultiArray()
					print msg
					msg.data = [temp0, temp1]
					print msg.data
					motor_pub.publish(msg)
					print motor_pub

if __name__ == '__main__':
	try:
		main()
	except Exception, e:
		print e
	finally:		
		paked_data = pack('<f', 0)
		iface.write('right_duty%s' % paked_data)
		iface.write('left_duty%s' % paked_data)
		iface.close();