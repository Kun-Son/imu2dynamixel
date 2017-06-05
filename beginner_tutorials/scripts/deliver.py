#!/usr/bin/env python

import rospy
import tensorflow as tf
from std_msgs.msg import String
from std_msgs.msg import Float32

def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + ' talker said %s', data.data)
    m = tensor(data.data)
#    m = data.data*2
    pub.publish(m)
    rospy.loginfo(m)

def tensor(data):
    x = tf.placeholder(tf.float32)
    y = tf.add(x, 2.)
#    y = tf.constant(2.)
    sess = tf.Session()
    z = sess.run(y, {x:data})
    return z

def deliver():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('deliver', anonymous=True)

    rospy.Subscriber('tester', Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('delivertopic', Float32, queue_size=10)
    deliver()
