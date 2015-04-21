#!/usr/bin/env python
import rospy
from percept_generators.msg import object as object_msg
from percept_generators.msg import objectArray as objectArray_msg
import numpy as np

# This node takes in raw object data and removes all duplicates then republishes an array of data

# All objects are assumed to be in the same reference frame so no transforms are required

# This mission is the demo day FSM
class refine_objects:
	def __init__(self):
		rospy.init_node('refine_objects', anonymous=False)
		# Publishers
		self.object_pub = rospy.Publisher('/percepts/objects', objectArray_msg, queue_size = 10)
		# Subscribers
		self.object_sub = rospy.Subscriber('/percepts/raw_objects', object_msg, self.objectCb)
		# Parameters
		self.same_object_distance = [ # to be to different objects they must be this far apart
			rospy.get_param('~same_unkown_distance', 0.1),
			rospy.get_param('~same_cylinder_distance', 0.1),
			rospy.get_param('~same_sphere_distance', 0.1),
			rospy.get_param('~same_start_gate_distance', 0.1)]

		self.object_lifetime = [# to be to different objects they must be this far apart
			rospy.get_param('~unkown_lifetime', 5.0),
			rospy.get_param('~cylinder_lifetime', 5.0),
			rospy.get_param('~sphere_lifetime', 5.0),
			rospy.get_param('~start_gate_lifetime', 5.0)]

		self.num_types = 4
		self.objects = [list() for x in range(self.num_types)]		# Do the callbacks run in different threads? Will this cause a problem???

		rospy.Timer(rospy.Duration(0.1), self.updateCb)
		
	def getDistanceBetweenObjects(self, obj1, obj2):
		return np.linalg.norm(np.array(
						[obj1.centroid.x - obj2.centroid.x, 
						 obj1.centroid.y - obj2.centroid.y,
						 obj1.centroid.z - obj2.centroid.z]))

	# Check the distance the cylinders are apart
	def isSameObject(self, obj1, obj2):
		if obj1.type != obj2.type:
			return False

		distance = self.getDistanceBetweenObjects(obj1, obj2)

		if distance < self.same_object_distance[obj1.type]:
			return True
		else:
			return False

	def isNewObject(self, obj):
		now = rospy.get_time()
		if now - obj.header.stamp.to_sec() < self.object_lifetime[obj.type]:
			return True
		else:
			return False

	def objectCb(self, new_obj):
		# Put the object in the object list
		for i, old_obj in enumerate(self.objects[new_obj.type]):
			if self.isSameObject(new_obj, old_obj):
				# Replace the current object with its newer self
				self.objects[new_obj.type][i] = new_obj
				return

		# If we are this far then its a new object yay
		self.objects[new_obj.type].append(new_obj)
		#print self.cylinders

	def updateCb(self, event):
		for i in range(self.num_types):
			# Remove old Objects
			self.objects[i] = filter(self.isNewObject, self.objects[i])

			# Publish objects
			objects = objectArray_msg()
			objects.header.stamp = rospy.get_rostime()
			objects.type = i
			objects.objects = self.objects[i]
			self.object_pub.publish(objects)




if __name__ == '__main__':
	node = refine_objects()
	rospy.spin()
