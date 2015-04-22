#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray

class node:
	def __init__(self):
		self.left_duty_pub = rospy.Publisher("left_duty", Float32, queue_size = 10)
		self.right_duty_pub = rospy.Publisher("right_duty", Float32, queue_size = 10)

		self.desired_right_velocity = 0
		self.desired_left_velocity = 0
		self.last_err_l = 0
		self.last_err_r = 0
		self.i_err_l = 0
		self.i_err_r = 0

		self.kp = rospy.get_param('~gain_p', 0.009)
		self.kd = rospy.get_param('~gain_d', 0.00001)
		self.ki = rospy.get_param('~gain_i', 0.0)	# Integral scares me...

		self.wheel_base = rospy.get_param('/wheel_base', 0.3048) # 0.3048 m = 12 in
		self.wheel_radius = rospy.get_param('/wheel_radius', 0.06)

		self.ramp_coef = rospy.get_param('~motor_max_ramp_rate', 5)  # Duty / s
		self.current_duty = [0, 0]
		self.desired_duty = [0, 0]

		self.last_time = rospy.get_time()

		rospy.Subscriber("/schema/motor_cmd", Twist, self.motorCmdCb)
		rospy.Subscriber("/percepts/motor_vels", Float64MultiArray, self.motor_vels_cb)

	#Callback for motor command
	# 
	# Variables:
	#	wr := right wheel angular velocity (rad/s)
	#	wl := left wheel angular velocity (rad/s)
	#	r_wheel := radius of the wheel (m)
	#	Vr := velocity of right wheel along the ground (m/s)
	#	Vl := velocity of left wheel along the ground (m/s)
	#	Vx := Linear velocity of robot (m/s)
	#	w := angular velocity of robot about instantaneous center of curvature (ICC) (rad/s)
	#	L := length between the wheels (m)
	#
	# Equation:
	#	Vr = wr * r_wheel
	#	Vl = wl * r_wheel
	#	Vx = (Vr + Vl) / 2
	#	w = (Vr - Vl) / L
	# ------------------------
	#	Vr = Vx + L * w / 2
	#	Vl = Vx - L * w / 2
	# ------------------------
	#	wr = (1 / r_wheel) * (Vx + L * w / 2)
	# 	wl = (1 / r_wheel) * (Vx - L * w / 2)
	#
	def motorCmdCb(self, twist):
		Vx = twist.linear.x
		w = twist.angular.z

		self.desired_left_velocity = (Vx - self.wheel_base * w / 2) / self.wheel_radius
		self.desired_right_velocity = (Vx + self.wheel_base * w / 2) / self.wheel_radius

		#print 'Desired_Vl: %f, Desired_Vr: %f' % (self.desired_left_velocity, self.desired_right_velocity)

	# PID controllers for velocity
	def motor_vels_cb(self, msg):
		now = rospy.get_time()
		time_step = now - self.last_time
		self.last_time = now

		if(time_step > 5):
			rospy.logerr('Haven\'t had a velocity reading in over 5 seconds stoping motors until update rate restored')
			# Set the duty
			self.left_duty_pub.publish(Float32(0))
			self.right_duty_pub.publish(Float32(0))
			# Reset the integral error cause I think we should...
			self.i_err_l = 0
			self.i_err_r = 0
			return


		data = msg.data
		print '\n----------------\n'
		# Calculate errors
			# Proportional
		err_l = self.desired_left_velocity - data[0]
		err_r = self.desired_right_velocity - data[1]
		print 'Proportional: %f, %f' % (err_l, err_r)
			# Derivative
		d_err_l = (err_l - self.last_err_l) / time_step
		d_err_r = (err_r - self.last_err_r) / time_step
		print 'Derivative: %f, %f' % (d_err_l, d_err_r)
			# Integral
		self.i_err_l = self.i_err_l + time_step * (err_l + self.last_err_l) / 2
		self.i_err_r = self.i_err_r + time_step * (err_r + self.last_err_r) / 2
		print 'Integral: %f, %f' % (self.i_err_l, self.i_err_r)

		print 'Multiplied errors: \np:%f,\t\t %f, \ni:%f,\t\t %f,\nd:%f, \t\t%f' % (
			self.kp * err_l, self.kp * err_r,
			self.ki * self.i_err_l, self.ki * self.i_err_r,
			self.kd * d_err_l, self.kd * d_err_r)


		# Reset last errors
		self.last_err_r = err_r
		self.last_err_l = err_l

		# Calculate duty setting
		self.desired_duty = [ self.current_duty[0] + self.kp * err_l + self.kd * d_err_l + self.ki * self.i_err_l ,
				 			  self.current_duty[1] + self.kp * err_r + self.kd * d_err_r + self.ki * self.i_err_r	]

		# Clip values
		for index in range(2):
			if self.desired_duty[index] > 1:
				rospy.logwarn("Cliping duty cycle to 1")
				self.desired_duty[index] = 1

				debug = [self.kp * err_l, self.kd * d_err_l, self.ki * self.i_err_l]
				debug = map(abs, debug)
				m = max(debug)
				for i, v in enumerate(debug):
					if v == m:
						print i, v
			elif self.desired_duty[index] < -1:
				rospy.logwarn("Cliping duty cycle to -1")

				debug = [self.kp * err_l, self.kd * d_err_l, self.ki * self.i_err_l]
				debug = map(abs, debug)
				m = max(debug)
				for i, v in enumerate(debug):
					if v == m:
						print i, v
				self.desired_duty[index] = -1 

		print 'self.desired_Duty_l: %f, Duty_r: %f' % (self.desired_duty[0], self.desired_duty[1])

		# Ramp function
		for i, (d, c) in enumerate(zip(self.desired_duty, self.current_duty)):
			if(d < c):
				self.current_duty[i] = max([c - self.ramp_coef * time_step, d])
			elif (d > c):
				self.current_duty[i] = min([c + self.ramp_coef * time_step, d])

		print self.current_duty

		print '\n----------------\n'
		# Set the duty
		self.left_duty_pub.publish(Float32(self.current_duty[0]))
		self.right_duty_pub.publish(Float32(self.current_duty[1]))


def main():
	# Ros initilization
	#Init ros
	rospy.init_node('primative_driver', anonymous=False)

	this_node = node()

	rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except e:
		print e
	finally:		
		print 'Warning: motors left in final state before node was shutdown'