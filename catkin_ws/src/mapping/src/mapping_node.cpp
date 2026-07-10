#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace octomap;

// void print_query_info(point3d query, OcTreeNode* node) {
//   if (node != NULL) {
//     cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
//   }
//   else 
//     cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
// }

// publish ground_truth/pose as a TF transform 
void ground_truth_pose_cb(const nav_msgs::Odometry::ConstPtr &msg){
   static tf2_ros::TransformBroadcaster br;
   geometry_msgs::TransformStamped transformStamped;
   
   transformStamped.header = msg->header;
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

    // ros::Publisher supervisor_completion_pub =
    // nh.advertise<std_msgs::Bool>("/supervisor/completion", 1, true);

    // ros::ServiceClient octomap_server_client = nh.serviceClient<>
    // ("");

    ros::Rate rate(20.0);

    // create empty tree with resolution 0.2
    OcTree tree (0.2);  

    // default configurations

    // probability threshold above which a voxel is considered occupied
    tree.setOccupancyThres(0.5);
    // occupancy probability increase when a ray hits a voxel
    tree.setProbHit(0.7);
    // occupancy probability decrease when a ray passes through a voxel
    tree.setProbMiss(0.3);
    // prevent probabilities from reaching exactly 0 or 1 (known obstacles/free space very persistent)
    tree.setClampingThresMin(0.05);
    tree.setClampingThresMax(0.95);

    // insert some measurements of occupied cells

    for (int x=-20; x<20; x++) {
        for (int y=-20; y<20; y++) {
        for (int z=-20; z<20; z++) {
            point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
            tree.updateNode(endpoint, true); // integrate 'occupied' measurement
        }
        }
    }

    // insert some measurements of free cells

    for (int x=-30; x<30; x++) {
        for (int y=-30; y<30; y++) {
        for (int z=-30; z<30; z++) {
            point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
            tree.updateNode(endpoint, false);  // integrate 'free' measurement
        }
        }
    }

    cout << endl;
    cout << "performing some queries:" << endl;
    
    point3d query (0., 0., 0.);
    OcTreeNode* result = tree.search (query);
    // print_query_info(query, result);

    query = point3d(-1.,-1.,-1.);
    result = tree.search (query);
    // print_query_info(query, result);

    query = point3d(1.,1.,1.);
    result = tree.search (query);
    // print_query_info(query, result);


    // check resolution looks right and confirm coordinate frames are aligned: $ octovis simple_tree.bt
    tree.writeBinary("tree.bt");
}
