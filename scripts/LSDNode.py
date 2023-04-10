#!/usr/bin/env python
import rospy

import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import time
import yaml
from os.path import expanduser

home = expanduser('~')
filey =  open(home + '/catkin_ws/src/f1tenth_sim/lsdParam.yaml', "r")
yamldata = yaml.safe_load(filey)
desired_velocity = yamldata['desired_velocity']
filey.close()

def key_pressed_call_back(data):
	if (data.data == 'w'):
		print("v")
		# speed 0
		drive_ack_pub(0.0)
		# set inital pos
		set_initial_pose()
		#
		time.sleep(1.)
		# speed 5
		drive_ack_pub(desired_velocity)
		time.sleep(1.)
		rospy.signal_shutdown("dones")

drive_ack = rospy.Publisher('/lsdnode_drive', AckermannDriveStamped, queue_size=10)
        
keysub = rospy.Subscriber('/key', String, key_pressed_call_back, queue_size=10)
        
pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)

#pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
#vel = Twist()


def drive_ack_pub(speed):
	drive = AckermannDriveStamped()
	drive.drive.speed = speed
	#vel.drive.acceleration = 7.
	#vel.jerk = 1.
	#print("publishing:", vel.drive.speed, vel.drive.acceleration, vel.drive.jerk)
	print(drive)
	drive_ack.publish(drive)

def set_initial_pose():
	pose = PoseStamped()
	pose.pose.position.x = -20.0 #-47
	pose.pose.position.y = 30.0
	pose_pub.publish(pose)
    
#def __del__():
	# body of destructor
	#drive_ack_pub(0.0)


def main(args=None):
	rospy.init_node('LSDNode')
	rate = rospy.Rate(10) # 10hz

	#while not rospy.is_shutdown():
		#try:
			#print("trying to publish")
			#drive_ack_pub(5.0)
			#print("should have published")
		#finally:
			#print("failed to publish")
	
	rate.sleep()
	rospy.spin()
		


if __name__ == '__main__':
	print("LSD node running")
	main()
