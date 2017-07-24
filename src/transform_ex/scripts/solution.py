#!/usr/bin/env python  
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg

def msg_from_Transform(T):
	msg=geometry_msgs.msg.Transform()
	quaternion=tf.transformations.quaternion_from_matrix(T)
	translation=tf.transformations.translation_from_matrix(T)
	msg.translation.x=translation[0]
	msg.translation.y=translation[1]
	msg.translation.z=translation[2]
	msg.rotation.x=quaternion[0]
	msg.rotation.y=quaternion[1]
	msg.rotation.z=quaternion[2]
	msg.rotation.w=quaternion[3]
	return msg


def publish_transforms():
	#The transform from the 'base' coordinate frame to the 'object' coordinate frame 
	#consists of a rotation expressed as (roll, pitch, yaw) of (0.79, 0.0, 0.79) followed 
	#by a translation of 1.0m along the resulting y-axis and 1.0m along the resulting z-axis. 
	T1=tf.transformations.concatenate_matrices(tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)),tf.transformations.translation_matrix((0.0,1.0,1.0)))
	
	#The transform from the 'base' coordinate frame to the 'robot' coordinate 
	#frame consists of a rotation around the z-axis by 1.5 radians followed by a 
	#translation along the resulting y-axis of -1.0m. 
	T2=tf.transformations.concatenate_matrices(tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(1.5, (0,0,1))),tf.transformations.translation_matrix((0.0,-1.0,0.0)))
	
	T2_inverse = tf.transformations.inverse_matrix(T2)
	
	# rTo_trans is a translation vector from robot frame to object origin
	rTo_trans=tf.transformations.translation_from_matrix(tf.transformations.concatenate_matrices(T2_inverse,T1))
	
	# cTo is a vector from camera frame to object origin
	# It is obtained by adding rTo_trans with a vector from robot to camera origin
	cTo=rTo_trans+numpy.array([0.0,0.1,0.1])
	
	# normalising the cTo vector
	cTo = cTo/numpy.sqrt(numpy.dot(cTo,cTo))
	
	# rot is the angle the camera frame needs to be rotated to make its x axis point to the object origin 
	# It is found by calculating the angle between x axis and cTo vector
	rot=numpy.arccos(cTo[0])
	
	# The frame is rotated along this 'axis' with angle 'rot'
	# This 'axis' is prependicular to both x axis and cTo so that while the camera frame rotates the
	# x axis remains in the the plane containing both x axis and cTo 
	axis=numpy.cross(numpy.array([1.0,0.0,0.0]),cTo)
	
	# The transform from the 'robot' coordinate frame to the 'camera' coordinate frame must be defined as follows:
	
	# The translation component of this transform is (0.0, 0.1, 0.1)
	# The rotation component of this transform must be set such that the camera is pointing directly 
	# at the object. In other words, the x-axis of the 'camera' coordinate frame must be pointing directly 
	# at the origin of the 'object' coordinate frame.
	T3=tf.transformations.concatenate_matrices(tf.transformations.translation_matrix((0.0,0.1,0.1)),tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(rot,(axis[0],axis[1],axis[2]))))

	
	object_transform = geometry_msgs.msg.TransformStamped()
	object_transform.header.stamp = rospy.Time.now()
	object_transform.header.frame_id = "base_frame"
	object_transform.child_frame_id = "object_frame"
	object_transform.transform=msg_from_Transform(T1)
	br.sendTransform(object_transform)
	
	robot_transform = geometry_msgs.msg.TransformStamped()
	robot_transform.header.stamp = rospy.Time.now()
	robot_transform.header.frame_id = "base_frame"
	robot_transform.child_frame_id = "robot_frame"
	robot_transform.transform=msg_from_Transform(T2)
	br.sendTransform(robot_transform)
	
	camera_transform = geometry_msgs.msg.TransformStamped()
	camera_transform.header.stamp = rospy.Time.now()
	camera_transform.header.frame_id = "robot_frame"
	camera_transform.child_frame_id = "camera_frame"
	camera_transform.transform=msg_from_Transform(T3)
	br.sendTransform(camera_transform)
	
if __name__ == '__main__':
	rospy.init_node('project2_solution')
	
	br = tf2_ros.TransformBroadcaster()
	rospy.sleep(0.5)
	while not rospy.is_shutdown():
		publish_transforms()
		rospy.sleep(0.05)
