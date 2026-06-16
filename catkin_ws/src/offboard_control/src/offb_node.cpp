#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// contains all custom messages required to operate services and topics by mavros
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <string>
#include <array>
#include <vector>

double TOL = 0.3; 

// callback that saves the current state of the autopilot 
mavros_msgs::State current_state; 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose; 
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
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

geometry_msgs::PoseStamped make_pose(double x, double y, double z) {
    // PX4 flight stack operates in NED coordinate frame but mavros translates them to standard ENU frame 
    // target position that is consistently sent to px4 (it expects continous commands)
    // poseStamped: postition+orientation+timeframe
    // NED: x: North, y: East, z: Down
    // ENU: x: East, y: North, z: Up
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x; 
    pose.pose.position.y = y; 
    pose.pose.position.z = z; 
    pose.pose.orientation.w = 1.0; 
    return pose;
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

// FSM: INIT WAIT PRESET OFFBOARD ARM WAYPOINT_1 ~ 3 LAND HALT 
enum class Mode {
    Init, 
    Offboard, 
    Waypoint_1,
    Waypoint_2,
    Waypoint_3,
    Land,
    Halt
};

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
        
    // publishes the commanded local position (relative to local origin)
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> 
        ("mavros/setpoint_position/local", 10);

    // clients that request arming and mode changes
    // mavros prefix depend on the name given to the node in .launch file
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
    
    // setpoint publishing rate must be faster than 2 Hz 
    // PX4 has a timeout of 500 ms between two offboard commands, and fall backs to the last mode if timeout is exceeded 
    // recommended to enter offboard mode from position mode
    ros::Rate rate(20.0);

    std::vector<geometry_msgs::PoseStamped> waypoints = {
        make_pose(0.0, 0.0, 2.0), make_pose(0.0, 0.0, 2.0), make_pose(0.0, 9.5, 2.0), make_pose(-15.0, 9.5, 2.0), 
        make_pose(-15.0, 15.0, 2.0), make_pose(-15.0, 15.0, 0.3), make_pose(-15.0, 15.0, 0.3) 
    };

    int mode_index = 0;

    mavros_msgs::SetMode offboard_set_mode; 
    mavros_msgs::SetMode land_set_mode; 

    mavros_msgs::SetMode manual_set_mode; 

    mavros_msgs::CommandBool arm_cmd; 

    geometry_msgs::PoseStamped command_pose = waypoints[0];

    ros::Time last_request = ros::Time::now();
    ros::Time dwell_start_time = ros::Time::now();


    bool dwelling = false; 
    
    Mode current_mode = Mode::Init;
    ROS_INFO("initializing...");

    // while ros is running normally... 
    while(ros::ok()) {
        switch (current_mode) {
            case Mode::Init: 
                offboard_set_mode.request.custom_mode = "OFFBOARD";
                arm_cmd.request.value = true; 

                // wait for connection to be established between mavros and the autopilot 
                // loop exits when heartbeat message (small mavlink message that tells whether the system is alive) is received
                while(ros::ok() && !current_state.connected) {
                    // lets ros process callbacks once to update current_state
                    ros::spinOnce();
                    rate.sleep();
                }

                // setpoints must be already streamed before entering offboard mode (PX4 rejects it otherwise)
                // 100 messages across 5 seconds before requesting offboard 
                for (int i = 100; ros::ok() && i > 0; --i) {
                    local_pos_pub.publish(waypoints[mode_index]);
                    ros::spinOnce();
                    rate.sleep();
                }

                // handleMode(current_mode);
                mode_index += 1; 
                current_mode = Mode::Offboard;
                ROS_INFO("offboarding...");
                break;
            case Mode::Offboard:
                // space the service calls by 5s (usually shorter) to not flood the autopilot with requests 
                if (current_state.mode != "OFFBOARD" && 
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    // asks to switch to offboard mode and checks if mavros sent mode-change request to px4
                    if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
                        ROS_INFO("offboard enabled");
                    }
                    last_request = ros::Time::now();
                } else {
                    // arm the quad to allow it to fly (spin motors & apply actuator outputs)
                    if (!current_state.armed && 
                        (ros::Time::now() - last_request > ros::Duration(5.0))) {
                        // asks to arm the vehicle && checks mavros' consequential action
                        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                            ROS_INFO("vehicle armed");
                        }
                        last_request = ros::Time::now();
                    }
                    if (current_state.armed && 
                        (ros::Time::now() - last_request > ros::Duration(0.5))) {
                        ROS_INFO("hovering(1)...");
                    }
                }
                if (!dwelling && at_waypoint(waypoints[mode_index], TOL)) {
                    dwelling = true; 
                    dwell_start_time = ros::Time::now();
                }
                if (at_waypoint(waypoints[mode_index], TOL) && vehicle_stability() && dwelling) {
                    if ((ros::Time::now() - dwell_start_time > ros::Duration(2.0))) {
                        ROS_INFO("dwelling completed");
                        ROS_INFO("hovering(1)...");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Waypoint_1;
                    }
                }

                // handleMode(current_mode);
                break;
            case Mode::Waypoint_1: 
                // handleMode(current_mode);
                if (!dwelling && at_waypoint(waypoints[mode_index], TOL)) {
                    dwelling = true; 
                    dwell_start_time = ros::Time::now();
                }
                if (at_waypoint(waypoints[mode_index], TOL) && vehicle_stability() && dwelling) {
                    if ((ros::Time::now() - dwell_start_time > ros::Duration(2.0))) {
                        ROS_INFO("dwelling completed");
                        ROS_INFO("hovering(2)...");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Waypoint_2;
                    }
                }

                break;
            case Mode::Waypoint_2: 
                if (!dwelling && at_waypoint(waypoints[mode_index], TOL)) {
                    dwelling = true; 
                    dwell_start_time = ros::Time::now();
                }
                if (at_waypoint(waypoints[mode_index], TOL) && vehicle_stability() && dwelling) {
                    if ((ros::Time::now() - dwell_start_time > ros::Duration(2.0))) {
                        ROS_INFO("dwelling completed");
                        ROS_INFO("hovering(3)...");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Waypoint_3;
                    }
                }
                break;
            case Mode::Waypoint_3: 
                if (!dwelling && at_waypoint(waypoints[mode_index], TOL)) {
                    dwelling = true; 
                    dwell_start_time = ros::Time::now();
                }
                if (at_waypoint(waypoints[mode_index], TOL) && vehicle_stability() && dwelling) {
                    if ((ros::Time::now() - dwell_start_time > ros::Duration(2.0))) {
                        ROS_INFO("dwelling completed");
                        ROS_INFO("landing...");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Land;
                    }
                }
                break;
            case Mode::Land: 
                if (!dwelling && at_waypoint(waypoints[mode_index], TOL)) {
                    dwelling = true; 
                    dwell_start_time = ros::Time::now();
                }
                if (at_waypoint(waypoints[mode_index], TOL) && vehicle_stability() && dwelling) {
                    if ((ros::Time::now() - dwell_start_time > ros::Duration(2.0))) {
                        ROS_INFO("dwelling completed");
                        ROS_INFO("halting...");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Halt;
                    }
                }
                break;
            case Mode::Halt: 
                land_set_mode.request.custom_mode = "AUTO.LAND";
                if (current_state.mode != "AUTO.LAND" && 
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    // asks to switch to offboard mode and checks if mavros sent mode-change request to px4
                    if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent) {
                        ROS_INFO("landing enabled");
                    }
                    last_request = ros::Time::now();
                } 
                   
                break;
        }

        if (mode_index < waypoints.size()) {
            // continue sending the requested pose at the appropriate rate 
            // interpolation set at 5cm / tick (1 m/s)
            command_pose = interpolation(command_pose, waypoints[mode_index], 0.05);
            local_pos_pub.publish(command_pose);
        }
        //keeps the loop at 20 Hz
        ros::spinOnce();
        rate.sleep();
    }
    return 0; 
}
