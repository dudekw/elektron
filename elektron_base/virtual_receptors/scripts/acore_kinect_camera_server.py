#!/usr/bin/env python

__author__ = "Jan Figat"

########################
# Imports
########################

# Importing services
from rapp_ros_naoqi_wrappings.srv import *

# Importing core system functionality
import signal
import sys, os
import rospy
import time
import smtplib


# Importing others
import numpy as np
import math
import Image
#import time #for time measurement
import freenect

from std_msgs.msg import String
from sensor_msgs.msg import Image as Image_ros
from cv_bridge import CvBridge, CvBridgeError


#######################################
class CameraModuleKinect:
	
	# Constructor of CameraModule
	def __init__(self):
		rospy.loginfo("[Camera server] - Acore Camera Server initialization")
		
		# Initialization of ROS node
		rospy.init_node('acore_camera_server')
		#self.moduleName = name
		
		self.bridge = CvBridge()
		
		self.openServices()
		
		rospy.loginfo("[Camera server] - Waiting for clients ...")
		
		
		
	# Initialization of ROS services
	def openServices(self):
		try:
			rospy.loginfo("[Camera server] - setting services")
			rospy.loginfo("[Camera server] - service - [rapp_capture_image]")
			self.service_rdqr = rospy.Service('rapp_capture_image', GetImage, self.handle_rapp_capture_image)
		except Exception, ex:
			rospy.logerr("[Camera server] - Exception (services) %s", str(ex))

		
	#######################################
	# Core functionality methods 
	#######################################
	#function to get RGB image from kinect
	def get_video(self):
		array,_ = freenect.sync_get_video()
		#array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR) ##
		array = array[:, :, ::-1]  # RGB -> BGR
		return array
 
	#########################
	# Handling methods - methods that used handling services
	#########################
		
	def handle_rapp_capture_image(self,req):
		rospy.loginfo("[Camera server receives]: camera: %s resolution: %d", req.request, req.resolution)
		
		# Get Frame from Camera
		try:
			# Capture image from selected camera
			#get a frame from RGB camera
			frame = self.get_video() ## numpy array #BGR
			#print "test"			
			#print frame.shape # shape
			#self.frame_img=Image.fromstring("RGB", (frame.shape[0], frame.shape[1]), frame) ## tuple
			#self.frame_img= np.array(self.frame_img)##conversion from tuple to numpy array
			#self.image_message = self.bridge.cv2_to_imgmsg(self.frame_img,"bgr8")#, encoding="rbg") # form numpy.array to imgmsg for ROS communication
			self.image_message = self.bridge.cv2_to_imgmsg(frame,"bgr8")#, encoding="rbg") # form numpy.array to imgmsg for ROS communication
			
			return self.image_message

		except AttributeError, ex:
			print "[Camera server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[Camera server] - Unnamed exception = %s" % str(ex)

		return None
	
#######################################
# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Camera server] - signal SIGINT caught"
	print "[Camera server] - system exits"
	sys.exit(0)
def main():
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Camera server] - Press Ctrl + C to exit system correctly"
		CameraServer = CameraModuleKinect()
		rospy.spin()
	
	except AttributeError:
		print "[Camera server] -  - AttributeError"
		#unsubscribe from camera
		
		sys.exit(0)
		
	except (KeyboardInterrupt, SystemExit):
		print "[Camera server] - SystemExit Exception caught"
		#unsubscribe from camera
		
		sys.exit(0)
		
	except Exception, ex:
		print "[Camera server] - Exception caught %s" % str(ex)
		#unsubscribe from camera
		
		sys.exit(0)
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
