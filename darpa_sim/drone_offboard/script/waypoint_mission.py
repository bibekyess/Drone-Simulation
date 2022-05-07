#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Mission():
    def __init__(self):
        self.current_odom = Odometry()
        self.odom_sub = rospy.Subscriber("mavros/local_position/odom", Odometry, self.odom_sub_callback)
        self.pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)

        self.waypoints_x = [0.0, 10.0, 18.0, 23.0, 18.0, 10.0, 0.0, 0.0]
        self.waypoints_y = [0.0, 0.0, 0.0, -15.0, 0.0, 0.0, 0.0, 0.0]
        self.waypoints_z = [1.1, 1.5, 2.0, 2.7, 2.0, 1.5, 1.5, -0.7]

        self.waypoint_index = 0

    def publish_pos(self):
        x_cmd, y_cmd, z_cmd = self.get_reference_waypoint()
        pos_cmd = PoseStamped()
        pos_cmd.header.stamp = rospy.Time.now()
        pos_cmd.pose.position.x = x_cmd
        pos_cmd.pose.position.y = y_cmd
        pos_cmd.pose.position.z = z_cmd
        pos_cmd.pose.orientation.w = 1.0

        self.pos_pub.publish(pos_cmd)

    def get_reference_waypoint(self):
        x = self.current_odom.pose.pose.position.x
        y = self.current_odom.pose.pose.position.y
        z = self.current_odom.pose.pose.position.z
        dist = (x-self.waypoints_x[self.waypoint_index])**2 \
               + (y-self.waypoints_y[self.waypoint_index])**2 \
               + (z-self.waypoints_z[self.waypoint_index])**2
        if dist < 1.5:
            self.waypoint_index = min(self.waypoint_index + 1, len(self.waypoints_x) -1)
            print("target waypoint (%.1f, %.1f, %.1f)" \
                %(self.waypoints_x[self.waypoint_index],self.waypoints_y[self.waypoint_index],self.waypoints_z[self.waypoint_index]))

        return self.waypoints_x[self.waypoint_index], self.waypoints_y[self.waypoint_index], self.waypoints_z[self.waypoint_index] 

    def odom_sub_callback(self, msg):
        self.current_odom = msg
        self.publish_pos()

if __name__ == "__main__":
    rospy.init_node("mission", anonymous = False)
    mission = Mission()
    rospy.spin()