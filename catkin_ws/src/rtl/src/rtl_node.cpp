#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// contains all custom messages required to operate services and topics by mavros
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <supervisor/FailureMode.h>

#include <std_msgs/Bool.h>

#include <string>
#include <vector>
#include <limits>
#include <deque>
#include <random>

// global variables
bool received_pose = false; 
mavros_msgs::State current_state; 
geometry_msgs::PoseStamped current_pose; 
supervisor::FailureMode current_failure_mode; 
double current_distance = 0.0; 
const int QUEUE_SIZE = 500; 
std::deque<geometry_msgs::PoseStamped> trail; 

geometry_msgs::PoseStamped create_pose(double x, double y, double z) {
    geometry_msgs::PoseStamped pose; 
    pose.pose.position.x = x; 
    pose.pose.position.y = y; 
    pose.pose.position.z = z; 
    pose.pose.orientation.w = 1.0; 
    return pose; 
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
    received_pose = true; 
}

void mode_cb(const supervisor::FailureMode::ConstPtr& msg){
    current_failure_mode = *msg;
}

geometry_msgs::PoseStamped interpolation(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target, double step) {
    double dx = target.pose.position.x - current.pose.position.x;
    double dy = target.pose.position.y - current.pose.position.y;
    double dz = target.pose.position.z - current.pose.position.z;
    double d =  std::sqrt(dx * dx + dy * dy + dz * dz);
    geometry_msgs::PoseStamped next_pose;
    // set orientation (w) to 1.0 as, otherwise, it initializes an invalid, zero quaternion (0,0,0,0) for the pose
    next_pose.pose.orientation.w = 1.0; 

    if (d < step) {
        next_pose.pose.position = target.pose.position; 
    } else {
        next_pose.pose.position.x = current.pose.position.x + dx * (step / d); 
        next_pose.pose.position.y = current.pose.position.y + dy * (step / d); 
        next_pose.pose.position.z = current.pose.position.z + dz * (step / d); 
    }
    return next_pose;
}

double find_distance(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped previous) {
     double dx = previous.pose.position.x - current.pose.position.x;
    double dy = previous.pose.position.y - current.pose.position.y;
    double dz = previous.pose.position.z - current.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool at_waypoint(geometry_msgs::PoseStamped target_pose, double tol) {
    double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    double dz = target_pose.pose.position.z - current_pose.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) < tol;
}

bool vehicle_stability() {
    return current_state.connected && current_state.mode == "OFFBOARD" && current_state.armed;
}

std::random_device rd; 
std::mt19937 gen(rd());

// placeholder 
int random_index(int min, int max) {
    std::uniform_int_distribution<int> dist(min, max);
    return dist(gen);
}

// a topic is continuous streaming
// a service is request / response

// asks the px4 to enter offboard mode then arm the drone 
// offboard mode is defined as a mode in which the drone is controlled by an external computer (ros node)
int main(int argc, char **argv) {
    // starts ros node
    ros::init(argc, argv, "rtl_node");
    // node's access point to ros
    ros::NodeHandle nh;

    //topic: mavros/state, queue size: 10, callback: state_cb()
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, pose_cb);

    ros::Subscriber mode_sub = nh.subscribe<supervisor::FailureMode>
        ("supervisor/failure_mode", 10, mode_cb);
    
    // set latch=true such that ROS remembers the last message that was published,
    // and automatically send it to any new subscriber that connects later
    ros::Publisher supervisor_completion_pub =
    nh.advertise<std_msgs::Bool>("/supervisor/completion", 1, true);

    // publishes the commanded local position (relative to local origin)
    ros::Publisher supervisor_pose_pub = nh.advertise<geometry_msgs::PoseStamped> 
        ("supervisor/intermediate_offboard_pose", 10);   

    // setpoint publishing rate must be faster than 2 Hz 
    // PX4 has a timeout of 500 ms between two offboard commands, and fall backs to the last mode if timeout is exceeded 
    // recommended to enter offboard mode from position mode
    ros::Rate rate(20.0);

    std::vector<geometry_msgs::PoseStamped> states = {
        create_pose(0.0, 0.0, 2.0), create_pose(-15.0, 15.0, 2.0)
    };

    double tol = 0.3; 

    bool record = false; 
    // maximum waypoint age
    double record_interval = 5.0; 
    // minimum distance threshold
    double record_distance = 0.5; 
    ros::Time last_record_time = ros::Time::now(); 

    bool new_failure = false; 

    ros::Time last_request = ros::Time::now();
    
    ROS_INFO("[RTL_NODE] initializing...");
    geometry_msgs::PoseStamped command_pose = states[0];
    geometry_msgs::PoseStamped waypoint = states[0];

    // while ros is running normally... 
    while(ros::ok()) {
        if (current_failure_mode.mode == supervisor::FailureMode::SMART_RTL) {
            // retrace
            if (!new_failure) {
                // in consecutive failures, restart mission and find waypoint again
                if (trail.empty()) {
                    ROS_WARN("[RTL_NODE] trail empty, cannot retrace");
                    std_msgs::Bool msg;
                    msg.data = true;
                    supervisor_completion_pub.publish(msg);
                } else {
                    waypoint = trail.back();
                    trail.pop_back();
                    ROS_INFO("[RTL_NODE] navigating to waypoint: (%.2f, %.2f, %.2f)", 
                        waypoint.pose.position.x,
                        waypoint.pose.position.y,
                        waypoint.pose.position.z);
                }
                new_failure = true; 
            }

            if (at_waypoint(waypoint, tol) && vehicle_stability()) {
                ROS_INFO("[RTL_NODE] at waypoint");
                if (trail.size() == 0) {
                    ROS_INFO("[RTL_NODE] landing...");
                    waypoint = states[0];
                    std_msgs::Bool msg; 
                    msg.data = true; 
                    supervisor_completion_pub.publish(msg);   
                } else {
                    waypoint = trail.back();
                    trail.pop_back();
                    ROS_INFO("[RTL_NODE] navigating to waypoint: (%.2f, %.2f, %.2f)", 
                    waypoint.pose.position.x,
                    waypoint.pose.position.y,
                    waypoint.pose.position.z);
                
                }
            }

            // continue sending the requested pose at the appropriate rate 
            // interpolation set at 5cm / tick (1 m/s)
            command_pose = interpolation(command_pose, waypoint, 0.05);
            supervisor_pose_pub.publish(command_pose);
        } else {
            //filter
            if (received_pose) {
                // distance based recording
                if (!trail.empty()) {
                    current_distance = find_distance(current_pose, trail.back());
                    record = (ros::Time::now() - last_record_time).toSec() > record_interval || current_distance > record_distance;
                }
                if (trail.size() >= QUEUE_SIZE) {
                    // remove random pose in trail 
                    trail.erase(trail.begin() + random_index(1, trail.size() - 1));
                }

                if (trail.empty() || record) {
                    trail.push_back(current_pose);
                    ROS_INFO("[OFFB_NODE] trail recorded: size=%zu pose=(%.2f,%.2f,%.2f)",
                        trail.size(),
                        current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z);
                    last_record_time = ros::Time::now();
                }
            }
            new_failure = false; 
        }

        //keeps the loop at 20 Hz
        ros::spinOnce();
        rate.sleep();
    }
    return 0; 
}