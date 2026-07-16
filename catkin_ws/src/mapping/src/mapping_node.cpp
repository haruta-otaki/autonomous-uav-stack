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

void octomap_cb(const octomap_msgs::Octomap::ConstPtr &msg){
    // msgToMap() returns AbstractOcTree* as the message could theoretically contain any tree type. 
    abstract = octomap_msgs::msgToMap(*msg);
    // get concrete OcTree type
    tree = dynamic_cast<octomap::OcTree*>(abstract);
}

// takes in the request and response type defined in the srv file and returns 
bool build_edt(mapping::ScanClearance::Request  &req,
        mapping::ScanClearance::Response &res)
{
    // get minimum and maximum coordinates from octree and create OctoMap point objects
    double x,y,z;
    tree->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);
    //std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
    tree->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);
    //std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;

    // treat unknown voxels as free (aggressive approach)
    bool unknownAsOccupied = false;
    // indoor value
    float maxDist = 2.0;    
    if (distmap) {
        delete distmap;
    }
    // compute an Euclidean Distance Transform (EDT) over an OctoMap to get maximize clearance from obstacles
    // clamp distance computations and restrict the distance map to a subarea through the arguments
    distmap = new DynamicEDTOctomap(maxDist, tree, min, max, unknownAsOccupied);    
    // EDT algorithm: walks through the entire octree and computes, for every free voxel, its distance to nearest occupied voxel
    // repeated calls incrementally updates the distances to reflect the modified occupancy map
    distmap->update(); 
    return true;
}

bool query_edt(mapping::ScanClearance::Request  &req,
        mapping::ScanClearance::Response &res)
{
    if (!distmap) {
        ROS_WARN("edt not built, call build_edt service");
        return false;
    }

    distmap->update(); 
    octomap::point3d p(req.pose.x, req.pose.y, req.pose.z);

    octomap::point3d closestObst;
    float distance;
    geometry_msgs::Point closest_obst_pose; 
    // return the distance of the obstacle from the query point and its location
    distmap->getDistanceAndClosestObstacle(p, distance, closestObst);
    closest_obst_pose.x = closestObst.x();
    closest_obst_pose.y = closestObst.y();
    closest_obst_pose.z = closestObst.z();
    
    res.clearance = distance;
    res.closest_obstacle = closest_obst_pose;
    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "mapping_node");

    ros::NodeHandle nh;

    ros::Subscriber ground_truth_pose_sub = nh.subscribe<nav_msgs::Odometry>
        ("ground_truth/pose", 10, ground_truth_pose_cb);

    ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>
        ("octomap_binary", 10, octomap_cb);

    // check resolution looks right and confirm coordinate frames are aligned: $ octovis simple_tree.bt
    // tree.writeBinary("tree.bt");
    
    // service is created and advertised over ROS 
    ros::ServiceServer build_service = nh.advertiseService("build_edt", build_edt);
    ros::ServiceServer query_service = nh.advertiseService("query_edt", query_edt);

    ros::spin();
}
