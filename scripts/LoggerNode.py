#!/usr/bin/env python
# -*- coding: ascii -*-
import rospy

import numpy as np
from std_msgs.msg import String, Int32
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
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import ast


home = expanduser('~')

	#file = open(strftime(home+'/sim_ws/our_test_logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
doLog = False
simStart = time.time()
data_stream = None


filey =  open(home + '/catkin_ws/src/f1tenth_sim/logParam.yaml', "r")
yamldata = yaml.safe_load(filey)
#desired_velocity_yaml = yamldata['desired_velocity']
mu = yamldata['friction_coeff']
a_max = yamldata['max_accel']
filey.close()
file = None
desired_velocity = 0.0 # Start desire velocity
current_model = 4
expected_velocity = 0.0
# body of destructor
# file.close()

def key_pressed_call_back(data):
	global doLog, file, simStart, data_stream
	
	if (data.data == 'w'):
		doLog = False
		#open file
		file = open(home +'/catkin_ws/src/f1tenth_sim/test_results/longitude-'+ str(gmtime()[1:6]) +'.csv', 'w')
		header = ['x', 'y', 'yaw', 'speed', 'a_max', 'time', 'mu', 'desired_velocity', 'current_model', 'expected_velocity']
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
		#print("Start logging")
		#simStart = time.time()
		# Waiting to be logging done
		#rospy.on_shutdown(new.join())
		new.join()
		#rospy.signal_shutdown("done")
	


# Saving data from odom.
def save_waypoint_call_back(data):
	global doLog, file, simStart, data_stream
	data_stream = data

# Saving desired velocity data from drive.
def save_drive_call_back(data):
	global desired_velocity
	desired_velocity = data.drive.speed

# Saving model data from mode_change.
def event_callback(data):
	global current_model, expected_velocity
	print(data.data)
	my_dict = ast.literal_eval(data.data)
	current_model = my_dict['Current_model']
	expected_velocity = my_dict['expected_velocity']

def save_log():
	global desired_velocity, doLog, file, simStart, data_stream
	simStart = time.time()
	# Logging with 0.002 interval in seconds 250 hertz
	interval = 0.004
	lastlogged = None
	lastdatareading = None
	while doLog:
		sim_time = data_stream.header.stamp.to_sec()
		#print(sim_time)
		if simStart + 20 < sim_time: #time.time()
			print("Time taken for experiment", sim_time-simStart) ##time.time()
			doLog = False
			file.close()
			rospy.signal_shutdown("done")
			return
		if (lastlogged == None or lastlogged + interval < sim_time): #time.time()) :
			quaternion = np.array([data_stream.pose.pose.orientation.x, 
								data_stream.pose.pose.orientation.y, 
								data_stream.pose.pose.orientation.z, 
								data_stream.pose.pose.orientation.w])

			euler = euler_from_quaternion(quaternion)
			speed = LA.norm(np.array([data_stream.twist.twist.linear.x, 
									data_stream.twist.twist.linear.y, 
									data_stream.twist.twist.linear.z]),2)
			#newdatareading = (data_stream.pose.pose.position.x,
			#									data_stream.pose.pose.position.y,
			#									euler[2],
			#									speed,desired_velocity)
			#if (speed != 0.0):
			#	desired_velocity = desired_velocity_yaml
			file.write('%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n' % (data_stream.pose.pose.position.x,
												data_stream.pose.pose.position.y,
												euler[2],
												speed,
												a_max, 
												sim_time,
												mu,
												desired_velocity,
												int(current_model),
												expected_velocity))
			#if (lastdatareading == None or desired_velocity == 0.0): #or lastdatareading != newdatareading
			#	file.write('%f, %f, %f, %f, %f, %f, %f\n' % (data_stream.pose.pose.position.x,
			#									data_stream.pose.pose.position.y,
			#									euler[2],
			#									speed, 
			#									rospy.get_rostime().to_time(),mu,desired_velocity))
			#lastdatareading = newdatareading
			lastlogged = sim_time#time.time()

odom = rospy.Subscriber('/odom', Odometry, save_waypoint_call_back, queue_size=10)

	# Subsriber to read key pressed.
keysub = rospy.Subscriber('/key', String, key_pressed_call_back, queue_size=10)

    # Subsriber to drive
drive = rospy.Subscriber('/lsdnode_drive', AckermannDriveStamped, save_drive_call_back, queue_size=10)

event_sub = rospy.Subscriber('/event_topic', String, event_callback, queue_size=10)

def main(args=None):
    rospy.init_node('LoggerNode')
    #print(data_stream)
    rospy.spin()

    #rospy.shutdown()


if __name__ == '__main__':
    print("Logger node running")
    main()