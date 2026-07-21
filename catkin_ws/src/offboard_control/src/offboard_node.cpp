#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// contains all custom messages required to operate services and topics by mavros
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <supervisor/FailureMode.h>
#include <planner/plan_path.h>

#include <std_msgs/Bool.h>

#include <string>
#include <vector>
#include <limits>

// global variables
bool received_pose = false; 

mavros_msgs::State current_state; 
geometry_msgs::PoseStamped current_pose; 

// callback that saves the current state of the autopilot 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
    received_pose = true; 
}

supervisor::FailureMode current_failure_mode; 
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

bool at_waypoint(geometry_msgs::PoseStamped target_pose, double tol) {
    double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    double dz = target_pose.pose.position.z - current_pose.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) < tol;
}

bool vehicle_stability() {
    return current_state.connected && current_state.mode == "OFFBOARD" && current_state.armed;
}

// C++ passes by value using copy by default, thus references are made to manipulate the original dwell-related variables
bool dwell(bool& dwelling, ros::Time& dwell_start_time, geometry_msgs::PoseStamped waypoint, double tol) {
    if (!dwelling && at_waypoint(waypoint, tol)) {
        dwelling = true; 
        dwell_start_time = ros::Time::now();
    }
    

    return at_waypoint(waypoint, tol) && vehicle_stability() && dwelling && 
        (ros::Time::now() - dwell_start_time > ros::Duration(2.0));
}

// a topic is continuous streaming
// a service is request / response

// asks the px4 to enter offboard mode then arm the drone 
// offboard mode is defined as a mode in which the drone is controlled by an external computer (ros node)
int main(int argc, char **argv) {
    // starts ros node
    ros::init(argc, argv, "offboard_control_node");
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
    ros::Publisher supervisor_failure_pub =
    nh.advertise<std_msgs::Bool>("/supervisor/failure", 1, true);


    // publishes the commanded local position (relative to local origin)
    ros::Publisher supervisor_pose_pub = nh.advertise<geometry_msgs::PoseStamped> 
        ("supervisor/intermediate_offboard_pose", 10);   

    ros::ServiceClient planner_client = nh.serviceClient<planner::plan_path>("plan_path");
    planner::plan_path planner_srv;

    // setpoint publishing rate must be faster than 2 Hz 
    // PX4 has a timeout of 500 ms between two offboard commands, and fall backs to the last mode if timeout is exceeded 
    // recommended to enter offboard mode from position mode
    ros::Rate rate(20.0);

    std::vector<geometry_msgs::PoseStamped> waypoints;

    int mode_index = 0;
    bool dwelling = false; 
    bool new_failure = false; 
    double tol = 0.3; 

    ros::Time last_request = ros::Time::now();
    ros::Time dwell_start_time = ros::Time::now();
    
    ROS_INFO("[OFFB_NODE] initializing...");
    geometry_msgs::PoseStamped command_pose = current_pose;

    // while ros is running normally... 
    while(ros::ok()) {
        if (current_failure_mode.mode == supervisor::FailureMode::CONTINUE) {
            // in consecutive failures, restart mission and find waypoint again
            if (!new_failure) {
                new_failure = true; 
                mode_index = 0; 
                planner_srv.request.start = current_pose; 
                planner_srv.request.goal = current_failure_mode.destination; 
                if (planner_client.call(planner_srv))
                {
                    if (planner_srv.response.success) {
                        waypoints = planner_srv.response.path.poses; 
                    }
                    else {
                        ROS_WARN("[OFFB_NODE] planner server failed to generate path");
                        std_msgs::Bool msg; 
                        msg.data = true; 
                        supervisor_failure_pub.publish(msg); 
                    }
                } else {
                    ROS_WARN("[OFFB_NODE] planner service call failed");
                    // send supervisor a failure message 
                    std_msgs::Bool msg; 
                    msg.data = true; 
                    supervisor_failure_pub.publish(msg); 
                }
            }

            if (mode_index == waypoints.size() - 1) {
                if (dwell(dwelling, dwell_start_time, waypoints[mode_index], tol)) {
                        ROS_INFO("[OFFB_NODE] dwelling completed at destination: (%.2f, %.2f, %.2f)",
                        waypoints[mode_index].pose.position.x,
                        waypoints[mode_index].pose.position.y,
                        waypoints[mode_index].pose.position.z);
                        ROS_INFO("[OFFB_NODE] landing...");
                        dwelling = false; 
                        // send supervisor a "done" message 
                        std_msgs::Bool msg; 
                        msg.data = true; 
                        supervisor_completion_pub.publish(msg);                        
                        ROS_INFO("[OFFB_NODE] navigating to waypoint %d: (%.2f, %.2f, %.2f)", 
                            mode_index,
                            waypoints[mode_index].pose.position.x,
                            waypoints[mode_index].pose.position.y,
                            waypoints[mode_index].pose.position.z);
                }
            }

            if (mode_index >= 0 && mode_index < waypoints.size()) {
                // continue sending the requested pose at the appropriate rate 
                // interpolation set at 5cm / tick (1 m/s)
                command_pose = interpolation(command_pose, waypoints[mode_index], 0.05);
                supervisor_pose_pub.publish(command_pose);
            }
        } else {
            new_failure = false; 
        }

        //keeps the loop at 20 Hz
        ros::spinOnce();
        rate.sleep();
    }
    return 0; 
}