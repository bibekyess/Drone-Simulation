#!/usr/bin/env python

"""ground_truth_republisher.py

Republishes a corrected version of the raw ground truth state estimate from the simulation in the
same frame as the state estimator.
"""

import numpy as np
import math

import rospy
import tf

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class GroundTruthRepublisher(object):
  
  def __init__(self, namespace='ground_truth_republisher'):
    """Initialize this _ground_truth_republisher"""
    rospy.init_node("ground_truth_republisher", anonymous = True)
    self.prefix = "/scout"
    self.pub = rospy.Publisher(self.prefix + "/ground_truth/state", Odometry, queue_size = 1)
    self.pubPoseStamped = rospy.Publisher(self.prefix + "/mavros/vision_pose/pose", PoseStamped, queue_size = 1)
    self.sub = rospy.Subscriber(self.prefix + "/ground_truth/state_raw", Odometry, self.subCallback)
    # self.sub = rospy.Subscriber("/odom", Odometry, self.subCallback)

    self.use_rotate = False

  def subCallback(self, msg):
    if(self.use_rotate):
      msg = self.handle_pose(msg)

    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.header = msg.header
    pose_stamped_msg.pose   = msg.pose.pose
    self.pub.publish(msg)
    self.pubPoseStamped.publish(pose_stamped_msg)
    self.broadcast(msg)

  
  def handle_pose(self, msg):
    #set frame to be the same as state estimator output
    # msg.header.frame_id='odom'
    # msg.child_frame_id='base_link'

    rotationNeg90Deg = np.array([[0, -1], [1, 0]])

    #print msg.pose.pose.position.x, msg.pose.pose.position.y

    #rotate position 90 deg around z-axis
    pos = np.dot(rotationNeg90Deg,np.array([msg.pose.pose.position.x, msg.pose.pose.position.y]))
    msg.pose.pose.position.x = pos[0]
    msg.pose.pose.position.y = pos[1]

    #rotate orientation 90 deg around z-axis
    q_rot = tf.transformations.quaternion_from_euler(0, 0, 1.57)  
    q_new = tf.transformations.quaternion_multiply(q_rot,[msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    
    msg.pose.pose.orientation.x = q_new[0]
    msg.pose.pose.orientation.y = q_new[1]
    msg.pose.pose.orientation.z = q_new[2]
    msg.pose.pose.orientation.w = q_new[3]

    #rotate linear velocity 90 deg around z-axis
    lin = np.dot(rotationNeg90Deg,np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y]))
    msg.twist.twist.linear.x = lin[0]
    msg.twist.twist.linear.y = lin[1]

    return msg

  def broadcast(self, odom):
    br = tf.TransformBroadcaster()
    quaternion = (
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w)
    br.sendTransform((odom.pose.pose.position.x, odom.pose.pose.position.y, 0),
                     quaternion,
                     rospy.Time.now(),
                     odom.child_frame_id,
                     odom.header.frame_id)

if __name__ == "__main__":
  ground_truth_republisher = GroundTruthRepublisher()
  rospy.spin()
  