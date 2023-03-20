#!/usr/bin/env python
import rospy

import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import os
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
import csv
import time
from threading import Thread


home = expanduser('~')

	#file = open(strftime(home+'/sim_ws/our_test_logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
	
doLog = False
simStart = time.time()

file = None

        # body of destructor
        #file.close()

def key_pressed_call_back(data):
	global doLog, file, simStart

	if (data.data == 'w'):
		print("v")
		doLog = False
		#open file
		file = open(home +'/catkin_ws/src/f1tenth_simulator/test_results/longitude-'+ str(gmtime()[1:6]) +'.csv', 'w')
		header = ['x', 'y', 'yaw', 'speed', 'time']
		writer = csv.writer(file)
		writer.writerow(header)

		# sleep
		time.sleep(0.5)
		# log data
		doLog = True

		simStart = time.time()



def save_waypoint_call_back(data):
	global doLog, file, simStart

	#print(rospy.get_rostime())
	if doLog:
		if simStart + 10 < time.time():
			doLog = False
			print(data.pose.pose.position.x)
			file.close()
			return
		
		quaternion = np.array([data.pose.pose.orientation.x, 
							data.pose.pose.orientation.y, 
							data.pose.pose.orientation.z, 
							data.pose.pose.orientation.w])

		euler = euler_from_quaternion(quaternion)
		speed = LA.norm(np.array([data.twist.twist.linear.x, 
								data.twist.twist.linear.y, 
								data.twist.twist.linear.z]),2)
		#if data.twist.twist.linear.x>0.:
			#print (data.twist.twist.linear.x)
		file.write('%f, %f, %f, %f, %f\n' % (data.pose.pose.position.x,
										data.pose.pose.position.y,
										euler[2],
										speed, 
										rospy.get_rostime().to_time()))
    

odom = rospy.Subscriber('/odom', Odometry, save_waypoint_call_back, queue_size=10)

	# Subsriber to read key pressed.
keysub = rospy.Subscriber('/key', String, key_pressed_call_back, queue_size=10)


def main(args=None):
    rospy.init_node('LoggerNode')

    rospy.spin()

    rospy.shutdown()


if __name__ == '__main__':
    print("Logger node running")
    main()