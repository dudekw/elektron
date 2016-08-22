#!/usr/bin/env python
######################
## written by Wojciech Dudek
######################
__author__ = "Wojciech Dudek"

from tf import transformations
import tf
import rospy
import sys
import signal
from sensor_msgs.msg import Imu
from elektron_msgs.srv import GetRobotPose,GetRobotPoseResponse
from elektron_msgs.srv import SetGlobalPose, SetGlobalPoseResponse 
from elektron_msgs.srv import GetTransform,GetTransformResponse

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped
import numpy as np

# Constants
# class Constants:


class ElektronEstimator():
	def __init__(self,name):
		rospy.init_node('acore_estimator')
		self.moduleName = name
		self.setVariables()
		self.startSubscribers()
		self.openServices()

	def setVariables(self):
		self.tl = tf.TransformListener(True, rospy.Duration(20.0))

		self.tf_br = tf.TransformBroadcaster()
		###
		# constPose - definition of object used in SubCall. Is used to operate on obtained data
		###
		self.constPose = PoseWithCovarianceStamped()
		self.constPose.pose.pose.position.x=0
		self.constPose.pose.pose.position.y=0
		self.constPose.pose.pose.orientation.z=0
		self.constPose.pose.pose.orientation.w=1
		###
		# odom_transformation - definition of object used in SubCall. Contains data that will be published.
		###
		self.odom_transformation = Pose()
		self.odom_transformation.position = [0,0,0]
		self.odom_transformation.orientation = [0,0,0,1]


	def handle_getRobotPose(self,req):

		actual_pose = PoseStamped()
		try:
			if self.tl.canTransform("world","base_link",rospy.Time()):
				ekf_pose = self.tl.lookupTransform("world","base_link",rospy.Time())
				actual_pose.pose.position.x = ekf_pose[0][0]
				actual_pose.pose.position.y = ekf_pose[0][1]
				actual_pose.pose.position.z = ekf_pose[0][2]
				actual_pose.pose.orientation.x = ekf_pose[1][0]
				actual_pose.pose.orientation.y = ekf_pose[1][1]
				actual_pose.pose.orientation.z = ekf_pose[1][2]
				actual_pose.pose.orientation.w = ekf_pose[1][3]

				actual_pose.header.seq = 1
				actual_pose.header.stamp= rospy.Time.now()
				actual_pose.header.frame_id = "world"
			else:
				status = True
			status = False
		except Exception, ex:
			print "[Estimator server] - Exception %s" % str(ex)
			status = True
		return GetRobotPoseResponse(actual_pose)

	def handle_getTransform(self,req):
		actual_pose = PoseStamped()
		if req.space == 0:
			space = "base_link"
		elif req.space == 1:
			space = "world"
		else:
			status = True
			return GetTransformResponse(actual_pose)		
		try:
			if self.tl.canTransform(req.chainName,space,rospy.Time()):
				ekf_pose = self.tl.lookupTransform(req.chainName,space,rospy.Time())
				actual_pose.pose.position.x = ekf_pose[0][0]
				actual_pose.pose.position.y = ekf_pose[0][1]
				actual_pose.pose.position.z = ekf_pose[0][2]
				actual_pose.pose.orientation.x = ekf_pose[1][0]
				actual_pose.pose.orientation.y = ekf_pose[1][1]
				actual_pose.pose.orientation.z = ekf_pose[1][2]
				actual_pose.pose.orientation.w = ekf_pose[1][3]

				actual_pose.header.seq = 1
				actual_pose.header.stamp= rospy.Time.now()
				actual_pose.header.frame_id = space
			else:
				status = True
			status = False
		except Exception, ex:
			print "[Estimator server] - Exception %s" % str(ex)
			status = True
		return GetTransformResponse(actual_pose)

	def handle_setGlobalPose(self,req):
		try:
			self.SubCall(req)
			#
			# check if actual state equals to request, define status
			#
			status = False
		except Exception, ex:
			print "[Estimator server] - Exception %s" % str(ex)
			status = True
		return SetGlobalPoseResponse(status)
	def startSubscribers(self):
		rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.SubCall)

	def SubCall(self,data):

		self.constPose.pose.pose.position = data.pose.pose.position
		self.constPose.pose.pose.orientation = data.pose.pose.orientation
		#
		# handle inirial pose
		#
		self.euler_initial = tf.transformations.euler_from_quaternion((0,0,self.constPose.pose.pose.orientation.z,self.constPose.pose.pose.orientation.w))
		#
		# find fransformation from odom to Nao_T_odom
		#
		if self.tl.canTransform("odom","base_link",rospy.Time()):
			transform_Nao_odom = self.tl.lookupTransform("base_link","odom", rospy.Time())
			euler_transform_Nao_odom =  tf.transformations.euler_from_quaternion(transform_Nao_odom[1])
			#
			# calculate new odom position, so Nao_T_odom will be in pointed position
			#
			matrix_Nao_odom= np.linalg.pinv(np.array([[np.cos(euler_transform_Nao_odom[2]),-np.sin(euler_transform_Nao_odom[2]),0,transform_Nao_odom[0][0]],
												[np.sin(euler_transform_Nao_odom[2]),np.cos(euler_transform_Nao_odom[2]),0,transform_Nao_odom[0][1]],
												[0,0,0,transform_Nao_odom[0][2]],
												[0,0,0,1]]))

			self.odom_transformation.position = [self.constPose.pose.pose.position.x+np.cos(self.euler_initial[2])*transform_Nao_odom[0][0]-np.sin(self.euler_initial[2])*transform_Nao_odom[0][1],
												self.constPose.pose.pose.position.y+np.sin(self.euler_initial[2])*transform_Nao_odom[0][0]+np.cos(self.euler_initial[2])*transform_Nao_odom[0][1],
												0]
			self.odom_transformation.orientation = tf.transformations.quaternion_from_euler(0,0,euler_transform_Nao_odom[2]+self.euler_initial[2])#matrix_Nao_odom[0][1]/matrix_Nao_odom[0][0])#+self.euler_initial[2])

	def openServices(self):
		try:
			print "[Estimator server] - service - [rapp_setGlobalPose]"
			self.service_set = rospy.Service('rapp_setGlobalPose', SetGlobalPose, self.handle_setGlobalPose)
		except Exception, ex:
			print "[Estimator server] - Exception %s" % str(ex)
		try:
			print "[Estimator server] - service - [rapp_getRobotPose]"

			self.service_get = rospy.Service('rapp_getRobotPose', GetRobotPose, self.handle_getRobotPose)
		except Exception, ex:
			print "[Estimator server] - Exception %s" % str(ex)
		try:
			print "[Estimator server] - service - [rapp_getTransforme]"

			self.service_getTr = rospy.Service('rapp_getTransform', GetTransform, self.handle_getTransform)
		except Exception, ex:
			print "[Estimator server] - Exception %s" % str(ex)
	def publishOdom(self):

		self.tf_br.sendTransform(self.odom_transformation.position, self.odom_transformation.orientation,
                                         rospy.Time.now(), "odom", "world")
def signal_handler(signal, frame):
	print "[Estimator server] - signal SIGINT caught"
	print "[Estimator server] - system exits"
	sys.exit(0)

if __name__ == '__main__':
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Estimator server] - Press Ctrl + C to exit system correctly"
				
		estimator = ElektronEstimator("acore_estimator")
		TfRate = rospy.Rate(10)

		while not rospy.is_shutdown():
			estimator.publishOdom()
			TfRate.sleep()

		
	except (KeyboardInterrupt, SystemExit):
		print "[Estimator server] - SystemExit Exception caught"
		
		sys.exit(0)
		
	except Exception, ex:
		print "[Estimator server] - Exception caught %s" % str(ex)

		sys.exit(0)
