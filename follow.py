import rospy
import numpy as np
import cv2
import time
import json
import requests
import tf.transformations
import subprocess as sp
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist, Vector3
import datetime
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import math
#import chance_constrained_tracker as ct
from numpy import linalg as LA
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import proc_input
import signal
from detecting_msg.msg import *
import string

class Follower(object):
    ROS_TIMEOUT = 600
    ENABLE = 0
    obj_path = '/home/biturtle/AUPAIR/datas/target_recog/target.txt'
    TRACK_SIGNAL = 1
    ACC = 0.05
    timeout = 1.5
    target_x = 0.0
    target_z = 1.2
    x_mult = 1.0
    z_mult = -0.5
    w_min = -0.4
    w_max = 0.4
    v_min = -0.35
    v_max = 0.35
	
    frame_size_x = 640
    frame_size_y = 480
    input_str = ""
    #print input_str
    z= 0
    x = 0
    y = 0
    x_w = 0
    y_h = 0
    
    from_speech = ""

    def __init__(self):
	print 'Initialize follower'
	self.proc = proc_input.proc_input('follower',False,True,False)        
	#self.cctt = ct.chance_constrained_tracker(['var',self.VAR,'sen_len',self.RANGE,'Wmax',self.WMAX,'Wmin',self.WMIN,'Vmax',self.VMAX,'Vmin',self.VMIN])
	self.track_pub = rospy.Publisher('cmd_vel/', Twist, queue_size=1)
	self.sub1 = rospy.Subscriber('/chatter', String, self.callback_pose)
	self.sub2 = rospy.Subscriber('/aupair/stt_en', String, self.callback_speech)
	#self.mode_pub = rospy.Publisher('/AUPAIR/followori', Int32, queue_size=1)
	#self.sub_ = rospy.Subscriber('/AUPAIR/detection_target_without_ori', detection_array, self.callback_detections)
	#self.sub_ = rospy.Subscriber('/AUPAIR/detection_all', detection_array, self.callback_detections)
	#self.sub2_ = rospy.Subscriber('/AUPAIR/followori', Int32, self.callback_mode)
	self.detection_filtered = []
	self.detection_raw = []

    def __del__(self):
	self.track_pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

    def callback_pose(self,data):
	self.frame_size_x = 640
	self.frame_size_y = 480
	self.input_str = str(data)
	print self.input_str
	self.z, self.x,self.y,self.x_w,self.y_h= self.input_str.split(" ")
	self.x = int(self.x)
	self.y = int(self.y)
	self.x_w = int(self.x_w)
	self.y_h = int(self.y_h)
	#self.middle_x = (self.x+self.x+self.x_w)/2
	#self.middle_y = (self.y+self.y+self.y_h)/2
	#self.target_range_x = range(((self.frame_size_x/2)-35) ,((self.frame_size_x/2) +35))
	#self.target_range_y = range(((self.frame_size_y/2)-30) ,((self.frame_size_y/2) +30))
	
    def callback_speech(self, data):
	self.from_speech = str(data)

    def callback_detections(self,data):
	temp = []
	for i in range(len(data.detections)):
		if data.detections[i].classes == 'person' :
			temp.append([data.detections[i].classes,data.detections[i].x1,data.detections[i].y1,data.detections[i].x2,data.detections[i].y2,data.detections[i].prob,data.header.stamp.to_sec()])
	self.detection_filtered = temp

    def track(self):

	print 'Start tracking'
        self.position = np.array([0.0,0.0,0.0])
	self.position_old = np.array([0.0,0.0,0.0])
	self.positions =[]
	self.odom = np.array([0,0,0])
	#self.pred_table = np.zeros((3,5))
	self.velocity = np.zeros(2)
	self.compress_rate = 1
	self.timestamp = 0
	#self.subs3 = rospy.Subscriber(self.ODOM_TOPIC, Odometry, self.odom_callback, queue_size=1)	
	time.sleep(1)
	s = time.time()	
	ss = time.time()
	stop_sign = "stop"
	while 1 :		
		if(self.from_speech.find(stop_sign) >=0 ):
			self.track_pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
		else:
			self.set_velo()
		
	#self.terminate()
	#self.subs2.unregister()    
        

    def set_velo(self):
	    signal.signal(signal.SIGINT, terminate)
	    #self.position = np.array([0,0,0])
	    #chose target (nearest target from previous frame)
	    dist = 999999
		
	    #self.middle_x = (self.x+self.x+self.x_w)/2
		#self.middle_y = (self.y+self.y+self.y_h)/2
		#self.target_range_x = range(((self.frame_size_x/2)-35) ,((self.frame_size_x/2) +35))
		#self.target_range_y = range(((self.frame_size_y/2)-30) ,((self.frame_size_y/2) +30))
		
	    mx = (self.x+self.x+self.x_w)/2
	    my = (self.y+self.y+self.y_h)/2
		#mx = (self.detection_filtered[0][3] + self.detection_filtered[0][1])/2
	    #my = (self.detection_filtered[0][4] + self.detection_filtered[0][2])/2
	    minz = 999999.0
	    print mx, my
	    #print "point cloude :" + self.proc.get_cloud(mx,my)
	    xy = self.proc.get_cloud(mx,my)
            xy = xy[1]
	    #print xy
	    for i in range(self.x, self.x + self.x_w,10):
	    	tt = self.proc.get_cloud(i,my)[1]
		if tt[2] < minz :
			minz = tt[2]
			self.position[2] = float(tt[2])
	    self.position[0] = float(xy[0])
	    self.position[1] = float(xy[1])
	    #print self.position
	    if self.position[0] == 0 and self.position[1] == 0 : return

	    #print np.sum((self.position-self.position_old)*(self.position-self.position_old))

	    #Predict next position
	    #self.pred_table[0] = self.pred_table[1]
	    #self.pred_table[1] = self.pred_table[2]
	    #self.pred_table[2] = np.array([self.position[2]*1000,self.position[0]*1000,self.odom[0]*1000,self.odom[1]*1000,self.odom[2]])
	    next_pos = np.array([self.position[2] * 1000,self.position[0] * 1000])  
	    #self.velocity = np.array(self.cctt.track([next_pos[0],next_pos[1]]))

	    self.velocity[1] = max(min(self.w_max,(self.target_x - self.position[0]) * self.x_mult),self.w_min)
	    self.velocity[0] = max(min(self.v_max,(self.target_z - self.position[2]) * self.z_mult),self.v_min)

	    if abs(self.velocity[1]) < 0.1 : self.velocity[1] = 0.0
	    if abs(self.velocity[0]) < 0.1 : self.velocity[0] = 0.0

	    #self.velocity[0] = max(min((self.velocity[0] + self.ACC),vel_temp[0]),(self.velocity[0] - self.ACC))
	    #self.velocity[1] = max(min((self.velocity[1] + self.ACC),vel_temp[1]),(self.velocity[1] - self.ACC))
	    print "distance and velocities are calculated : "
	    print next_pos
	    print self.velocity
	    print ""
	    self.track_pub.publish(Twist(Vector3(self.velocity[0],0,0),Vector3(0,0,self.velocity[1])))
	    self.position_old = self.position.copy()
	    #time.sleep(0.2)
            
def terminate(what,ever):
	raise KeyboardInterrupt("CTRL-C!")

def main():
    #track_pub = rospy.Publisher('track_enable', Int32, queue_size=1)
    #track_pub.publish(2) #Tracking On
    #time.sleep(60)
    #track_pub.publish(0) #Tracking Off
    signal.signal(signal.SIGINT, terminate)
    #time.sleep(10)
    tracker = Follower()
    tracker.track()
    
if __name__ == '__main__':
    main()


