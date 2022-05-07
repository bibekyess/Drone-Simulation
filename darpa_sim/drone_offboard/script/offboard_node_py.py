#!/usr/bin/env python

'''
@file       offboard_node_py.py
@author     Seungwook Lee
@copyright  USRG, 2021
@brief      Offboard node written in Python and made into a class.
            Responsible for connecting to the FCU, arming, and changing 
            the vehicle state into the offboard mode.
'''

import rospy

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

import copy

class Offboard():
    def __init__(self):
        # Subscribers
        self.operation_state_sub = rospy.Subscriber('mavros/state', State, self.operation_state_callback)
        self.cmd_pose_sub = rospy.Subscriber('position_command/rqt', PoseStamped, self.command_pose_callback)
        self.cmd_vel_sub = rospy.Subscriber('velocity_command/rqt', Twist, self.command_vel_callback)
        self.ground_truth_odom_sub = rospy.Subscriber('/uav1/ground_truth/state', Odometry, self.ground_truth_odom_callback)

        # Publishers
        self.local_pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.ground_truth_pose_pub = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=1)

        # Services
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # Member variables
        self.rate = rospy.Rate(20) # 20 Hz
        self.last_request = rospy.Time.now()
        self.current_state = State()

        self.init_pose = PoseStamped()
        self.init_pose.pose.position.z    = 2
        self.init_pose.pose.orientation.w = 1
        self.cmd_pose = copy.deepcopy(self.init_pose)
        self.cmd_vel = TwistStamped()
        self.cmd_vel.twist.linear.z = 0.5
        
        self.is_position_control = False
        self.is_velocity_control = False

    def operation_state_callback(self, msg):
        prev_state = self.current_state
        self.current_state = msg
        if self.current_state.mode != prev_state.mode:
            rospy.loginfo("Current mode: %s" % self.current_state.mode)
            rospy.loginfo("Previous mode: %s" % prev_state.mode)
        if self.current_state.armed != prev_state.armed:
            rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)

    def ground_truth_odom_callback(self, msg):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose = msg.pose.pose
        self.ground_truth_pose_pub.publish(pose)

    def check_FCU_connection(self):
        rospy.loginfo("Wait FCU connection")
        while not self.current_state.connected:
            self.rate.sleep()
        rospy.loginfo("FCU connected")

    def initial_cmd_publishing(self):
        # send a few setpoints before starting
        rospy.loginfo("send a few setpoints before starting")
        for _ in range(100):
            self.cmd_vel_pub.publish(self.cmd_vel)

    def set_arm(self):
        rospy.loginfo("Arming the vehicle..")
        while True:
            if self.current_state.armed is not True \
                and rospy.Time.now() - self.last_request > rospy.Duration(3):
                self.arming_client(value = True)
                self.last_request = rospy.Time.now()
            elif self.current_state.armed is True:
                break
            self.rate.sleep()

    def set_mode(self, mode):
        rospy.loginfo("Setting vehicle mode..")
        while True:
            if self.current_state.mode != mode \
                and rospy.Time.now() - self.last_request > rospy.Duration(3):
                self.set_mode_client(base_mode=0, custom_mode=mode)
                self.last_request = rospy.Time.now()
            elif self.current_state.mode != mode:
                break
            self.rate.sleep()

    def run(self):
        if self.is_position_control or \
            (not self.is_position_control and not self.is_velocity_control):
            self.pub_position()
        else:
            self.pub_cmd_vel()

    def command_pose_callback(self, msg):
        self.cmd_pose = msg
        self.is_position_control = True
        self.is_velocity_control = False

    def pub_position(self):
        self.cmd_pose.header.stamp = rospy.Time.now()
        self.local_pose_pub.publish(self.cmd_pose)

    def command_vel_callback(self, msg):
        self.cmd_vel.twist = msg
        self.is_position_control = False
        self.is_velocity_control = True

    def pub_cmd_vel(self):
        self.cmd_vel.header.stamp = rospy.Time.now()
        self.cmd_vel_pub.publish(self.cmd_vel)


if __name__ == '__main__':
    rospy.init_node('offboard_node',anonymous=False)
    offboard = Offboard()

    offboard.rate = rospy.Rate(20)
    offboard.check_FCU_connection()
    offboard.initial_cmd_publishing()
    offboard.set_arm()
    offboard.set_mode("OFFBOARD")

    rospy.loginfo("Offboard node ready")
    rate = rospy.Rate(100) # 100 hz
    while not rospy.is_shutdown():
        offboard.run()
        rate.sleep()

