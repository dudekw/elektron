#!/usr/bin/env python
######################
## written by Wojciech Dudek
######################
__author__ = "Wojciech Dudek"

########################
# Imports
########################

# Importing services
from elektron_msgs.srv import *
from sensor_msgs.msg import Image
# Importing core system functionality
import signal
import sys, os
import rospy
import numpy 


#from geometry_msgs import PoseStamped
#######################################


# Constants
# class Constants:

#	NAO_IP = "nao.local"
#	PORT = 9559

	

#######################################

class CamerasModule():
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of MoveNaoModule
	def __init__(self,name,enable_head,enable_top_kinect,enable_bottom_kinect):
		print "[Cameras VR] - Virtual receptor <<Cameras>> initialization"
		
		# Initialization of ROS node
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.checkSubsystems()
		self.setVariables(enable_head,enable_top_kinect,enable_bottom_kinect)

		self.openServices()


		print "[Cameras VR] - Waits for clients ..."
				
	# Checking existance of real effectors
	def checkSubsystems(self):
		print "[Cameras VR] - Checking existance of real receptors -> not implemented"

	# Setting variables
	def setVariables(self,enable_head,enable_top_kinect,enable_bottom_kinect):
		try:
			print "[Cameras VR] - Setting variables"

			if (enable_head=="1"):
				topic_name_head = "/head_camera_rgb/image_raw"
				rospy.wait_for_message(topic_name_head,Image,timeout=10)
				image_sub_head = rospy.Subscriber(topic_name_head, Image, self.writeImage_head)
			if (enable_top_kinect=="1"):
				topic_name_top_kinect = "/tower_top_kinect/rgb/image_raw"
				rospy.wait_for_message(topic_name_top_kinect,Image,timeout=10)
				image_sub_top_kinect = rospy.Subscriber(topic_name_top_kinect, Image, self.writeImage_top_kinect)
			if (enable_bottom_kinect=="1"):
				topic_name_bottom_kinect = "/bottom_kinect/rgb/image_raw"
				rospy.wait_for_message(topic_name_bottom_kinect,Image,timeout=10)
				image_sub_bottom_kinect = rospy.Subscriber(topic_name_bottom_kinect, Image, self.writeImage_bottom_kinect)
		except(rospy.ROSException), e:
			print "Camete image topic not available, aborting..."
			print "Error message: ", e
			exit()
	def openServices(self):
		try:
			print "[Cameras VR] - setting services"
			print "[Cameras VR] - service - [captureImage]"
			self.service_map = rospy.Service('rapp_capture_image', GetImage, self.rapp_captrure_image_interface)
		except Exception, ex_map:
			print "[Cameras VR] - Exception %s" % str(ex_map)
	def writeImage_head(self,msg):
		self.image_head = msg
	def writeImage_top_kinect(self,msg):
		self.image_top_kinect = msg
	def writeImage_bottom_kinect(self,msg):
		self.image_bottom_kinect = msg
	def getch(self):
		import sys, tty, termios
		fd = sys.stdin.fileno()
		old = termios.tcgetattr(fd)
		try:
			tty.setraw(fd)
			return sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old)
	def velPublisher(self):
		self.pub_vel.publish(self.set_vel)
####
##  SERVECE HANDLERS
####

	def rapp_captrure_image_interface(self,req):
		if (req.camera_id==0):		
			pub_img = self.image_head
		if (req.camera_id==1):		
			pub_img = self.image_bottom_kinect
		if (req.camera_id==2):		
			pub_img = self.image_top_kinect
		return GetImageResponse(pub_img)

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Cameras VR] - signal SIGINT caught"
	print "[Cameras VR] - system exits"
	sys.exit(0)

def main(node_name,enable_head,enable_top_kinect,enable_bottom_kinect):
	""" Main entry point
		"""
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Cameras VR] - Press Ctrl + C to exit system correctly"
		#self.use_sim = use_sim
		rospy.init_node("acore_cameras_receptor_"+node_name)
		global ElektronCameras
		ElektronCameras = CamerasModule("ElektronCameras",enable_head,enable_top_kinect,enable_bottom_kinect)
		rospy.spin()
	
	except (KeyboardInterrupt, SystemExit):
		print "[Cameras VR] - SystemExit Exception caught"
		#unsubscribe()

		sys.exit(0)
		
	except Exception, ex:
		print "[Cameras VR] - Exception caught %s" % str(ex)
		#unsubscribe()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		if len(sys.argv) < 5:
			print("usage: cameras_controller <<node_name>> <<enable_head>> <<enable_top_kinect>> <<enable_bottom_kinect>>")
		else:
			main(sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4])
	except Exception,e:
		print "__name__ - Error %s" % str(e)
