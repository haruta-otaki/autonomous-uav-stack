#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <octomap_msgs/Octomap.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <iostream>

// header file generated from the srv file 
#include <mapping/ScanClearance.h>

octomap::AbstractOcTree* abstract;
octomap::OcTree* tree; 
// global pointer 
DynamicEDTOctomap* distmap = nullptr;

// publish ground_truth/pose as a TF transform 
void ground_truth_pose_cb(const nav_msgs::Odometry::ConstPtr &msg){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;
    transformStamped.transform.rotation = msg->pose.pose.orientation; 

    br.sendTransform(transformStamped);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "mapping_node");

    ros::NodeHandle nh;

    ros::Subscriber ground_truth_pose_sub = nh.subscribe<nav_msgs::Odometry>
        ("ground_truth/pose", 10, ground_truth_pose_cb);

    ros::spin();
}
