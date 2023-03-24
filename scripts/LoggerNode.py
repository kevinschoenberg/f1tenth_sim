#!/usr/bin/env python
# -*- coding: ascii -*-
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
from threading import Thread, Event, enumerate, current_thread
import yaml

home = expanduser('~')

	#file = open(strftime(home+'/sim_ws/our_test_logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
doLog = False
simStart = time.time()
data_stream = None


filey =  open(home + '/catkin_ws/src/f1tenth_sim/logParam.yaml', "r")
yamldata = yaml.safe_load(filey)
desired_velocity_yaml = yamldata['desired_velocity']
mu = yamldata['friction_coeff']
#filey.close()
file = None

# body of destructor
# file.close()

def key_pressed_call_back(data):
	global doLog, file, simStart, data_stream
	
	if (data.data == 'w'):
		doLog = False
		#open file
		file = open(home +'/catkin_ws/src/f1tenth_sim/test_results/longitude-'+ str(gmtime()[1:6]) +'.csv', 'w')
		header = ['x', 'y', 'yaw', 'speed', 'time', 'mu', 'desired_velocity']
		writer = csv.writer(file)
		writer.writerow(header)

		# reset data
		#data_stream = []
		# sleep
		time.sleep(0.5)
		# log data
		doLog = True
		new = Thread(name="Johny", target=save_log,args=())
		# starter timer thread with logging
		new.start()
		# setting start sim time
		print("Start logging")
		#simStart = time.time()
		# Waiting to be logging done
		#rospy.on_shutdown(new.join())
		new.join()
		#rospy.signal_shutdown("done")
	


# Saving data from odom.
def save_waypoint_call_back(data):
	global doLog, file, simStart, data_stream
	data_stream = data

def save_log():
	global doLog, file, simStart, data_stream
	simStart = time.time()
	# Logging with 0.002 interval in seconds 250 hertz
	interval = 0.004
	lastlogged = None
	lastdatareading = None
	desired_velocity = 0.0
	while doLog:
		if simStart + 10 < time.time():
			print("Time taken for experiment", time.time()-simStart)
			doLog = False
			file.close()
			rospy.signal_shutdown("done")
			return
		if (lastlogged == None or lastlogged + interval < time.time()) :
			print(time.time())
			quaternion = np.array([data_stream.pose.pose.orientation.x, 
								data_stream.pose.pose.orientation.y, 
								data_stream.pose.pose.orientation.z, 
								data_stream.pose.pose.orientation.w])

			euler = euler_from_quaternion(quaternion)
			speed = LA.norm(np.array([data_stream.twist.twist.linear.x, 
									data_stream.twist.twist.linear.y, 
									data_stream.twist.twist.linear.z]),2)
			newdatareading = (data_stream.pose.pose.position.x,
												data_stream.pose.pose.position.y,
												euler[2],
												speed)
			if (speed != 0.0):
				desired_velocity = desired_velocity_yaml
			
			if (lastdatareading == None or lastdatareading != newdatareading):
				file.write('%f, %f, %f, %f, %f, %f, %f\n' % (data_stream.pose.pose.position.x,
												data_stream.pose.pose.position.y,
												euler[2],
												speed, 
												rospy.get_rostime().to_time(),mu,desired_velocity))
			lastdatareading = newdatareading
			lastlogged = time.time()

odom = rospy.Subscriber('/odom', Odometry, save_waypoint_call_back, queue_size=10)

	# Subsriber to read key pressed.
keysub = rospy.Subscriber('/key', String, key_pressed_call_back, queue_size=10)

def main(args=None):
    rospy.init_node('LoggerNode')
    #print(data_stream)
    rospy.spin()

    #rospy.shutdown()


if __name__ == '__main__':
    print("Logger node running")
    main()