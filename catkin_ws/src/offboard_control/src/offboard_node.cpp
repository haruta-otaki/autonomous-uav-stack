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

// FSM: INIT WAIT PRESET OFFBOARD ARM WAYPOINT_1 ~ 3 LAND HALT 
enum class Mode {
    Offboard, 
    // Simplify to Waypoint, to make custom messages 
    Waypoint_0,
    Waypoint_1,
    Waypoint_2,
    Waypoint_3,
};

struct MachineState {
    //fields 
    geometry_msgs::PoseStamped pose; 
    Mode mode; 

    // constructor 
    MachineState(double x, double y, double z, Mode m) {
        pose.pose.position.x = x; 
        pose.pose.position.y = y; 
        pose.pose.position.z = z; 
        pose.pose.orientation.w = 1.0; 

        mode = m; 
    }
};

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

int find_waypoint(geometry_msgs::PoseStamped current, std::vector<MachineState> states) {
    int index = 0; 

    double minimum_d = std::numeric_limits<double>::max();
    int n = states.size(); 
    // temporarily hard coded 
    for (int i = 0; i < n; i++) {
        double dx = states[i].pose.pose.position.x - current.pose.position.x;
        double dy = states[i].pose.pose.position.y - current.pose.position.y;
        double dz = states[i].pose.pose.position.z - current.pose.position.z;
        double d =  std::sqrt(dx * dx + dy * dy + dz * dz);
        if (d < minimum_d) {
            index = i;
            minimum_d = d; 
        }
    }
    return index; 
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

    // publishes the commanded local position (relative to local origin)
    ros::Publisher supervisor_pose_pub = nh.advertise<geometry_msgs::PoseStamped> 
        ("supervisor/intermediate_offboard_pose", 10);   

    // setpoint publishing rate must be faster than 2 Hz 
    // PX4 has a timeout of 500 ms between two offboard commands, and fall backs to the last mode if timeout is exceeded 
    // recommended to enter offboard mode from position mode
    ros::Rate rate(20.0);

    std::vector<MachineState> states = {
        MachineState(0.0, 0.0, 2.0, Mode::Offboard), MachineState(0.0, 0.0, 2.0, Mode::Waypoint_0), MachineState(0.0, 9.5, 2.0, Mode::Waypoint_1), 
        MachineState(-15.0, 9.5, 2.0, Mode::Waypoint_2), MachineState(-15.0, 15.0, 2.0, Mode::Waypoint_3)
    };

    int mode_index = 0;
    bool dwelling = false; 
    bool new_failure = false; 
    double tol = 0.3; 

    ros::Time last_request = ros::Time::now();
    ros::Time dwell_start_time = ros::Time::now();
    
    ROS_INFO("[OFFB_NODE] initializing...");
    Mode current_mode;
    geometry_msgs::PoseStamped command_pose = states[0].pose;

    // while ros is running normally... 
    while(ros::ok()) {
        if (current_failure_mode.mode == supervisor::FailureMode::CONTINUE) {
            // in consecutive failures, restart mission and find waypoint again
            if (!new_failure) {
                current_mode = Mode::Offboard;
                new_failure = true; 
            }
            switch (current_mode) {
                case Mode::Offboard:
                    if (current_state.mode == "OFFBOARD") {
                        mode_index = find_waypoint(current_pose, states);
                        current_mode = states[mode_index].mode;
                        ROS_INFO("[OFFB_NODE] navigating to waypoint %d: (%.2f, %.2f, %.2f)", 
                            mode_index,
                            states[mode_index].pose.pose.position.x,
                            states[mode_index].pose.pose.position.y,
                            states[mode_index].pose.pose.position.z);
                    }
                    break;
                case Mode::Waypoint_0:
                    if (dwell(dwelling, dwell_start_time, states[mode_index].pose, tol)) {
                        ROS_INFO("[OFFB_NODE] dwelling completed");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Waypoint_1;
                        ROS_INFO("[OFFB_NODE] navigating to waypoint %d: (%.2f, %.2f, %.2f)", 
                            mode_index,
                            states[mode_index].pose.pose.position.x,
                            states[mode_index].pose.pose.position.y,
                            states[mode_index].pose.pose.position.z);
                        
                    }
                    break;
                case Mode::Waypoint_1: 
                    // handleMode(current_mode);
                    if (dwell(dwelling, dwell_start_time, states[mode_index].pose, tol)) {
                        ROS_INFO("[OFFB_NODE] dwelling completed");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Waypoint_2;
                        ROS_INFO("[OFFB_NODE] navigating to waypoint %d: (%.2f, %.2f, %.2f)", 
                            mode_index,
                            states[mode_index].pose.pose.position.x,
                            states[mode_index].pose.pose.position.y,
                            states[mode_index].pose.pose.position.z);
                    }

                    break;
                case Mode::Waypoint_2: 
                    if (dwell(dwelling, dwell_start_time, states[mode_index].pose, tol)) {
                        ROS_INFO("[OFFB_NODE] dwelling completed");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Waypoint_3;
                        ROS_INFO("[OFFB_NODE] navigating to waypoint %d: (%.2f, %.2f, %.2f)", 
                            mode_index,
                            states[mode_index].pose.pose.position.x,
                            states[mode_index].pose.pose.position.y,
                            states[mode_index].pose.pose.position.z);
                    }
                    break;
                case Mode::Waypoint_3: 
                    if (dwell(dwelling, dwell_start_time, states[mode_index].pose, tol)) {
                            ROS_INFO("[OFFB_NODE] dwelling completed");
                            ROS_INFO("[OFFB_NODE] landing...");
                            dwelling = false; 
                            // send supervisor a "done" message 
                            std_msgs::Bool msg; 
                            msg.data = true; 
                            supervisor_completion_pub.publish(msg);                        
                            ROS_INFO("[OFFB_NODE] navigating to waypoint %d: (%.2f, %.2f, %.2f)", 
                                mode_index,
                                states[mode_index].pose.pose.position.x,
                                states[mode_index].pose.pose.position.y,
                                states[mode_index].pose.pose.position.z);
                    }
                    break;
            }
            if (mode_index >= 0 && mode_index < states.size()) {
                // continue sending the requested pose at the appropriate rate 
                // interpolation set at 5cm / tick (1 m/s)
                command_pose = interpolation(command_pose, states[mode_index].pose, 0.05);
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