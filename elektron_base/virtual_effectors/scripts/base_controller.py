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
from geometry_msgs.msg import Twist
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

class BaseEffectorModule():
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of MoveNaoModule
	def __init__(self,name):
		print "[Base effector] - Virtual effector <<Base>> initialization"
		
		# Initialization of ROS node
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.checkSubsystems()
		self.setVariables()

		self.openServices()


		print "[Base effector] - Waits for clients ..."
				
	# Checking existance of real effectors
	def checkSubsystems(self):
		print "[Base effector] - Checking existance of real effectors -> not implemented"

	# Setting variables
	def setVariables(self):
		print "[Base effector] - Setting variables"
		self.moveJoint = rospy.ServiceProxy('moveVel', MoveTower)
		topic = "/mux_vel_raw/cmd_vel"
		self.pub_vel = rospy.Publisher(topic, Twist, queue_size=10)
		self.set_vel = Twist()
		self.vel_pub = Twist()
		self.trigger_publishing = False
	def openServices(self):
		try:
			print "[Base effector] - setting services"
			print "[Base effector] - service - [moveVel]"
			self.service_map = rospy.Service('moveVel', MoveVel, self.rapp_set_vel_interface)
			print "[Base effector] - service - [moveStop]"
			self.service_map2 = rospy.Service('moveStop', MoveStop, self.rapp_stop_vel_interface)

		except Exception, ex_map:
			print "[Base effector] - Exception %s" % str(ex_map)
			self.service_map2 = rospy.Service('moveStop', MoveStop, self.rapp_stop_vel_interface)
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
	def velPublisher(self):
		if (self.trigger_publishing):
			self.pub_vel.publish(self.set_vel)


####
##  SERVECE HANDLERS
####

	def rapp_set_vel_interface(self,req):
#		if (req.velocity_x == 0 and req.velocity_theta == 0):
#			req_stop = MoveStopRequest()
#			status = self.rapp_stop_vel_interface(req_stop)
#
#			return MoveVelResponse(status.status)
#		else:
		self.set_vel.linear.x = req.velocity_x
		self.set_vel.angular.z = req.velocity_theta
		self.trigger_publishing = True

		status = False
		return MoveVelResponse(status)
	def rapp_stop_vel_interface(self,req):
		self.set_vel.linear.x = 0
		self.set_vel.angular.z = 0
		self.triger_publishing = True
		self.velPublisher()
		self.trigger_publishing = False

		status = False
		return MoveStopResponse(status)

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Base effector] - signal SIGINT caught"
	print "[Base effector] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
		"""
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Base effector] - Press Ctrl + C to exit system correctly"
		#self.use_sim = use_sim
		rospy.init_node('acore_base_effector')
		global ElektronBaseMove
		ElektronBaseMove = BaseEffectorModule("ElektronBaseMove")
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			ElektronBaseMove.velPublisher()
			rate.sleep()
	
	except (KeyboardInterrupt, SystemExit):
		print "[Base effector] - SystemExit Exception caught"
		#unsubscribe()

		sys.exit(0)
		
	except Exception, ex:
		print "[Base effector] - Exception caught %s" % str(ex)
		#unsubscribe()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
