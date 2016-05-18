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
	def __init__(self,name,topic_name):
		print "[Cameras VR] - Virtual receptor <<Cameras>> initialization"
		
		# Initialization of ROS node
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.checkSubsystems()
		self.setVariables(topic_name)

		self.openServices()


		print "[Cameras VR] - Waits for clients ..."
				
	# Checking existance of real effectors
	def checkSubsystems(self):
		print "[Cameras VR] - Checking existance of real receptors -> not implemented"

	# Setting variables
	def setVariables(self,topic_name):
		try:
			rospy.wait_for_message(topic_name,Image,timeout=4)
			print "[Cameras VR] - Setting variables"
			self.topic_name = topic_name
			image_sub = rospy.Subscriber(topic_name, Image, self.writeImage)
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
	def writeImage(self,msg):
		self.image = msg
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
		
		return GetImageResponse(self.image)

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Cameras VR] - signal SIGINT caught"
	print "[Cameras VR] - system exits"
	sys.exit(0)

def main(node_name,topic_name):
	""" Main entry point
		"""
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Cameras VR] - Press Ctrl + C to exit system correctly"
		#self.use_sim = use_sim
		rospy.init_node("acore_cameras_receptor_"+node_name)
		global ElektronCameras
		ElektronCameras = CamerasModule("ElektronCameras",topic_name)
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
		if len(sys.argv) < 3:
			print("usage: cameras_controller <<node_name>> <<camera_topic_name>>")
		else:
			main(sys.argv[1],sys.argv[2])
	except Exception,e:
		print "__name__ - Error %s" % str(e)
