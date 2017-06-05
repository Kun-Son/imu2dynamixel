#!/usr/bin/env python

import rospy
import math
import wx
import numpy as np

from sensor_msgs.msg import Imu
from beginner_tutorials.msg import Data
from tf.transformations import euler_from_quaternion


rad2degrees = 180.0/math.pi
precision = 2 #round to this number of digits
yaw_offset = 0 #used to align animation upon key press

i=0

def processIMU_message(imuMsg):
    global yaw_offset
   
    quaternion = (
      imuMsg.orientation.x,
      imuMsg.orientation.y,
      imuMsg.orientation.z,
      imuMsg.orientation.w)
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)
    
    	
#    print "Roll : ",roll*rad2degrees, "Pitch : ",pitch*rad2degrees,"Yaw : ",yaw*rad2degrees

    yaw += yaw_offset
    
    data = Data()
    data = [2000,500,500,0,0,0]
#    data[0] = (int)(roll*2048.0/math.pi)+2048
#    data[1] = (int)(pitch*2048.0/math.pi)+2048
#    data[2] = (int)(yaw*2048.0/math.pi)+2048
#    data[3] = (int)(roll*2048.0/math.pi)+2048
#    data[4] = (int)(pitch*2048.0/math.pi)+2048
#    data[5] = (int)(yaw*2048.0/math.pi)+2048

        

#    pub = rospy.Publisher('tester2', Data, queue_size=10)
    rospy.loginfo(data)
    pub.publish(data)
    
def start(): 
        rospy.init_node("talker2", anonymous=True)
	sub = rospy.Subscriber('imu', Imu, processIMU_message)
	rospy.spin()


if __name__ == '__main__':
	pub = rospy.Publisher('tester2', Data, queue_size=10)
	start()

	
