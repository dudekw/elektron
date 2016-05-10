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
	def __init__(self,name,use_sim):
		print "[Base effector] - Virtual effector <<Base>> initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_base_effector')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.checkSubsystems()
		self.setVariables(use_sim)

		self.openServices()


		print "[Base effector] - Waits for clients ..."
				
	# Checking existance of real effectors
	def checkSubsystems(self):
		print "[Base effector] - Checking existance of real effectors -> not implemented"

	# Setting variables
	def setVariables(self,use_sim):
		print "[Base effector] - Setting variables"
		self.moveJoint = rospy.ServiceProxy('moveVel', MoveTower)
		if (use_sim == "0"):
			topic = "/cmd_vel"
		elif (use_sim == "1"):
			topic = "/elektron/mobile_base_controller/cmd_vel"	
		else:
			print("wrong node initialization usage: base_controller <<use_simulation_real_base_effector>> (1 - simulation|| 0 - real elektron)")
		self.pub_vel = rospy.Publisher(topic, Twist, queue_size=10)

	def openServices(self):
		try:
			print "[Base effector] - setting services"
			print "[Base effector] - service - [moveVel]"
			self.service_map = rospy.Service('moveVel', MoveVel, self.rapp_move_vel_interface)
		except Exception, ex_map:
			print "[Base effector] - Exception %s" % str(ex_map)

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

	def rapp_move_vel_interface(self,req):

		set_vel = Twist()
		set_vel.linear.x = req.velocity_x
		set_vel.angular.z = req.velocity_theta
		self.pub_vel.publish(set_vel)
		status = False
		return MoveVelResponse(status)

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Base effector] - signal SIGINT caught"
	print "[Base effector] - system exits"
	sys.exit(0)

def main(use_sim):
	""" Main entry point
		"""
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Base effector] - Press Ctrl + C to exit system correctly"
		#self.use_sim = use_sim
		global ElektronBaseMove
		ElektronBaseMove = BaseEffectorModule("ElektronBaseMove",use_sim)

		rospy.spin()
	
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
		if len(sys.argv) < 2:
			print("usage: base_controller <<use_simulation_real_base_effector>> (1 - simulation|| 0 - real elektron)")
		else:
			main(sys.argv[1])
	except Exception,e:
		print "__name__ - Error %s" % str(e)
