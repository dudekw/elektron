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
from std_msgs.msg import Float64


class TowerEffectorModule():
	
	# Constructor of MoveNaoModule
	def __init__(self,name):
		print "[Tower real effector] - <<Tower>> Real effector for Gazebo initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_tower_real_effector')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		# self.checkSubsystems()
		self.setVariables()

		self.openServices()
		print "[Tower real effector] - Waits for clients ..."
				
	# Checking existance of real effectors
	# def checkSubsystems(self):
	# 	print "[Tower real effector] - Checking existance of real effectors -> not implemented"

	# Setting variables
	def setVariables(self):
		print "[Tower real effector] - Setting variables"
		self.pub_pitch = rospy.Publisher('/tower/head_pitch_position_controller/command', Float64, queue_size=10)
		self.pub_yaw = rospy.Publisher('/tower/head_yaw_position_controller/command', Float64, queue_size=10)
	def openServices(self):
		try:
			print "[Tower real effector] - setting services"
			print "[Tower real effector] - service - [moveJoint]"
			self.service_map = rospy.Service('re_moveTowerJoint', MoveTower, self.rapp_move_tower)
		except Exception, ex_map:
			print "[Tower real effector] - Exception %s" % str(ex_map)

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

	def rapp_move_tower(self,req):
		if (len(req.moveJoints)==2):
			self.pub_pitch.publish(req.pitch)
			self.pub_yaw.publish(req.yaw)
		
		if (req.moveJoints[0]=="yaw"):
			self.pub_yaw.publish(req.yaw)
	
		if (req.moveJoints[0]=="pitch"):
			self.pub_pitch.publish(req.pitch)

		return MoveTowerResponse(False)

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Tower real effector] - signal SIGINT caught"
	print "[Tower real effector] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
		"""
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Tower real effector] - Press Ctrl + C to exit system correctly"
		global TowerEffectorModule
		ElektronMove = TowerEffectorModule("ElektronMove")

		rospy.spin()
	
	except (KeyboardInterrupt, SystemExit):
		print "[Tower real effector] - SystemExit Exception caught"
		#unsubscribe()

		sys.exit(0)
		
	except Exception, ex:
		print "[Tower real effector] - Exception caught %s" % str(ex)
		#unsubscribe()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)