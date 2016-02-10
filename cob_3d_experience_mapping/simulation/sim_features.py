#!/usr/bin/python

import rospy, math, random
from nav_msgs.msg import Odometry
from ratslam_ros.msg import ViewTemplate
from cob_3d_experience_mapping.msg import SensorInfoArray, SensorInfo
import tf, random

ids={}
pub_si = None
pub_vt = None

NUM_FEATURES=1000
def callback(msg):
	global ids
	global pub_si
	global pub_vt

	global NUM_FEATURES
	
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	(r, p, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
	
	RESOLUTION_metric=1.5
	RESOLUTION_angle=0.75
	
	ix = int(x/RESOLUTION_metric)
	iy = int(y/RESOLUTION_metric)
	iyaw = int(yaw/RESOLUTION_angle)
	
	name = str(ix)+","+str(iy)+","+str(iyaw)
	no=len(ids)
	if NUM_FEATURES<=0:
		no = (ix*1000+iy)*100+iyaw
	if name in ids:
		no = ids[name]
	else:
		ids[name] = no
		
	if NUM_FEATURES>0 and no>NUM_FEATURES:
		random.seed(no)
		no = random.randint(0,NUM_FEATURES)
		
	msg = SensorInfoArray()
	msg.infos.append(SensorInfo(no))
	pub_si.publish(msg)
	
	msg = ViewTemplate()
	msg.header = msg.header
	msg.current_id = no
	pub_vt.publish(msg)
	
def listener():
	global pub_si
	global pub_vt
	global NUM_FEATURES
	
	robot="robot0"
	
	rospy.init_node('sim_features')
	NUM_FEATURES = rospy.get_param('~NUM_FEATURES', NUM_FEATURES)
	
	rospy.Subscriber(robot+"/odom", Odometry, callback)
	pub_si = rospy.Publisher('/sim_barks/sensor_info', SensorInfoArray, queue_size=10)
	pub_vt = rospy.Publisher('/irat_red/LocalView/Template', ViewTemplate, queue_size=10)
	
	rospy.spin()

listener()
