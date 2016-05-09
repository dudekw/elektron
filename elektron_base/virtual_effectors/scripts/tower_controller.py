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
from elektron_msgs.msg import obstacleData

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

class TowerEffectorModule():
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of MoveNaoModule
	def __init__(self,name):
		print "[Tower effector] - Virtual effector <<Tower>> initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_tower_effector')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.checkSubsystems()
		self.setVariables()

		self.openServices()


		print "[Tower effector] - Waits for clients ..."
				
	# Checking existance of real effectors
	def checkSubsystems(self):
		print "[Tower effector] - Checking existance of real effectors -> not implemented"

	# Setting variables
	def setVariables(self):
		print "[Tower effector] - Setting variables"
		self.orientation_pitch = 0
		self.orientation_yaw = 0
		self.moveJoint = rospy.ServiceProxy('re_moveTowerJoint', MoveTower)
	def openServices(self):
		try:
			print "[Tower effector] - setting services"
			print "[Tower effector] - service - [moveJoint]"
			self.service_map = rospy.Service('moveTowerJoint', MoveTower, self.rapp_move_joint_interface)
		except Exception, ex_map:
			print "[Tower effector] - Exception %s" % str(ex_map)

	# Event subscribtion
	# def subscribeToEvents(self):
		#print "subscribing to events"

	def getch(self):
		import sys, tty, termios
		fd = sys.stdin.fileno()
		old = termios.tcgetattr(fd)
		try:
			tty.setraw(fd)
			return sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old)

####
##  SERVECE HANDLERS
####

	def rapp_move_joint_interface(self,req):
		#self.unsubscribeToObstacle()
		
		if (len(req.moveJoints)==2):
			resp1 = self.moveJoint(req.yaw, req.pitch, ["yaw","pitch"])
			self.orientation_yaw = req.yaw		
			self.orientation_pitch = req.pitch		
		if (req.moveJoints[0]=="yaw"):
			resp1 = self.moveJoint(req.yaw, self.orientation_pitch,["yaw"])
			self.orientation_yaw = req.yaw		
		
		if (req.moveJoints[0]=="pitch"):
			resp1 = self.moveJoint(self.orientation_yaw, req.pitch,["pitch"])
			self.orientation_pitch = req.pitch
		return MoveTowerResponse(resp1.status)

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Tower effector] - signal SIGINT caught"
	print "[Tower effector] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
		"""
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Tower effector] - Press Ctrl + C to exit system correctly"
		global TowerEffectorModule
		ElektronMove = TowerEffectorModule("ElektronMove")

		rospy.spin()
	
	except (KeyboardInterrupt, SystemExit):
		print "[Tower effector] - SystemExit Exception caught"
		#unsubscribe()

		sys.exit(0)
		
	except Exception, ex:
		print "[Tower effector] - Exception caught %s" % str(ex)
		#unsubscribe()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)