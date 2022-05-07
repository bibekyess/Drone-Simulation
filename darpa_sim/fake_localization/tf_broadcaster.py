#!/usr/bin/env python  
import roslib
import rospy

import tf
from nav_msgs.msg import Odometry

def broadcast(msg):
    br = tf.TransformBroadcaster()
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     quaternion,
                     rospy.Time.now(),
                     'base_link',
                     "world")

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('/scout/ground_truth/state_raw',
                     Odometry,
                     broadcast)
    rospy.spin()