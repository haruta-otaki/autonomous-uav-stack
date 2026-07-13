#include <ros/ros.h>
// header file generated from the srv file 
#include <mapping/ScanClearance.h>

// takes in the request and response type defined in the srv file and returns 
bool add(mapping::ScanClearance::Request  &req,
        mapping::ScanClearance::Response &res)
{
    res.clearance = req.SAFE;
    // ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    // ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle n;

    // service is created and advertised over ROS 
    ros::ServiceServer service = n.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints.");
    ros::spin();

    return 0;
}