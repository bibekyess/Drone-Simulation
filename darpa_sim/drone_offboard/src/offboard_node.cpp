/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL https://docs.px4.io/master/en/ros/mavros_offboard.html
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

nav_msgs::Odometry current_odom;
void odom_sub_callback(const nav_msgs::Odometry::ConstPtr& msg){
    current_odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ROS_INFO("Starting Offboard Node");

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/local_position/odom", 10, odom_sub_callback);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_print = ros::Time::now();

    bool has_taken_off = false;
    while(ros::ok()){
        if (current_state.mode != "AUTO.TAKEOFF" && !has_taken_off &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Takeoff enabled");
            }
            else ROS_INFO("Requesting Takeoff");
            last_request = ros::Time::now();
        }
        else if( current_state.mode != "OFFBOARD" && has_taken_off &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            offb_set_mode.request.custom_mode = "OFFBOARD";
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            else ROS_INFO("Requesting Offboard");
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if (ros::Time::now() - last_print > ros::Duration(5.0))
        {
            ROS_INFO("Current mode: %s, has_taken_off: %s", current_state.mode.c_str(), has_taken_off ? "yes" : "no");
            last_print = ros::Time::now();
        }
            
        if(!has_taken_off && current_odom.pose.pose.position.z > 1){
            has_taken_off = true;
        }
        // local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
