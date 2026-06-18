#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// contains all custom messages required to operate services and topics by mavros
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <iostream>
#include <mutex>

std::mutex mtx; 

enum class Mode {
    Normal, 
    Short_Dropout,
    Long_Dropout,
    Burst_Short_Dropout,
    Burst_Long_Dropout,
    Command_Degradation,
    State_Degradation
};

struct FailureRequest {
    Mode mode; 
    double failure_duration; 
    ros::Time last_request; 

    FailureRequest(std::string input, double duration) {
        if (input == "short") {
            mode = Mode::Short_Dropout;
        } else if (input == "long") {
            mode = Mode::Long_Dropout;
        } else if (input == "short_burst") {
            mode = Mode::Burst_Short_Dropout;
        } else if (input == "long_burst") {
            mode = Mode::Burst_Long_Dropout;
        } else if (input == "command") {
            mode = Mode::Command_Degradation;
        } else if (input == "state") {
            mode = Mode::State_Degradation;
        } else {
            mode = Mode::Normal;
        }

        failure_duration = duration; 
        last_request = ros::Time::now();
    }
};

FailureRequest failure_request("normal", 0.0); 
std::atomic<bool> incoming_request(false); 

// callback that saves the current state of the autopilot 
mavros_msgs::State current_state; 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose; 
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

void terminal_thread() {
    std::string input; 
    // safe, incomparison to traditional cin as character input stream (cin), seperates at whitespace
    std::string line; 
    double duration; 
    while (ros::ok()) {
        // character output stream 
        // << : insertion operator 
        // >> : extraction operator
        std::cout << "Command> " << std::flush; 
        std::getline(std::cin, line);
        std::stringstream ss(line);
        
        if (ss >> input) {
            if (input == "quit") {
                ros::shutdown();
            }
            std::lock_guard<std::mutex> lock(mtx);
            if (ss >> duration) {
                failure_request = FailureRequest(input, duration);
                incoming_request = true; 
            }
            else {
                failure_request = FailureRequest(input, 0.0);
                incoming_request = true; 
            }
        } else {
            incoming_request = false; 
        }
    }
}

int main(int argc, char **argv) {
    // starts ros node
    ros::init(argc, argv, "failure_sim_node");
    // node's access point to ros
    ros::NodeHandle nh;

    // runs the terminal input function in a seperate thread to not block the ROS loop
    std::thread input_thread(terminal_thread);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, pose_cb);
    
    ros::Publisher intermediate_state_pub = nh.advertise<mavros_msgs::State> 
        ("offboard_control/intermediate_state_setpoint", 10);

    ros::Publisher intermediate_pose_pub = nh.advertise<geometry_msgs::PoseStamped> 
        ("offboard_control/intermediate_pose_setpoint", 10);
    
    // setpoint publishing rate must be faster than 2 Hz 
    // PX4 has a timeout of 500 ms between two offboard commands, and fall backs to the last mode if timeout is exceeded 
    // recommended to enter offboard mode from position mode
    ros::Rate rate(20.0);

    // std::vector<geometry_msgs::PoseStamped> waypoints = {
    //     make_pose(0.0, 0.0, 2.0), make_pose(0.0, 0.0, 2.0), make_pose(0.0, 9.5, 2.0), make_pose(-15.0, 9.5, 2.0), 
    //     make_pose(-15.0, 15.0, 2.0), make_pose(-15.0, 15.0, 0.3), make_pose(-15.0, 15.0, 0.3) 
    // };

    Mode current_mode;
    ROS_INFO("failure simulation setting up...");

    while(ros::ok()) {
        if (incoming_request) {  
            std::lock_guard<std::mutex> lock(mtx);  
            current_mode = failure_request.mode; 
            incoming_request = false; 
        } 
        else {
            current_mode = Mode::Normal; 
        }
        switch (current_mode) {
            case Mode::Normal: 
                intermediate_pose_pub.publish(current_pose);
                intermediate_state_pub.publish(current_state);
                break;
            case Mode::Short_Dropout: 
                // offboard_set_mode.request.custom_mode = "OFFBOARD";
                // arm_cmd.request.value = true; 

                // // wait for connection to be established between mavros and the autopilot 
                // // loop exits when heartbeat message (small mavlink message that tells whether the system is alive) is received
                // while(ros::ok() && !current_state.connected) {
                //     // lets ros process callbacks once to update current_state
                //     ros::spinOnce();
                //     rate.sleep();
                // }

                // // setpoints must be already streamed before entering offboard mode (PX4 rejects it otherwise)
                // // 100 messages across 5 seconds before requesting offboard 
                // for (int i = 100; ros::ok() && i > 0; --i) {
                //     local_pos_pub.publish(waypoints[mode_index]);
                //     ros::spinOnce();
                //     rate.sleep();
                // }

                // current_mode = Mode::Offboard;
                // ROS_INFO("offboarding...");
                break;
            case Mode::Long_Dropout: 
                intermediate_pose_pub.publish(current_pose);
                intermediate_state_pub.publish(current_state);
                break;
            case Mode::Burst_Short_Dropout: 
                intermediate_pose_pub.publish(current_pose);
                intermediate_state_pub.publish(current_state);
                break;
            case Mode::Burst_Long_Dropout: 
                intermediate_pose_pub.publish(current_pose);
                intermediate_state_pub.publish(current_state);
                break;
            case Mode::Command_Degradation: 
                intermediate_pose_pub.publish(current_pose);
                intermediate_state_pub.publish(current_state);
                break;
            case Mode::State_Degradation: 
                intermediate_pose_pub.publish(current_pose);
                intermediate_state_pub.publish(current_state);
                break;
        }
        // if (mode_index < waypoints.size()) {
        //     // continue sending the requested pose at the appropriate rate 
        //     // interpolation set at 5cm / tick (1 m/s)
        //     command_pose = interpolation(command_pose, waypoints[mode_index], 0.05);
        // }
        //keeps the loop at 20 Hz
        ros::spinOnce();
        rate.sleep();
    }

    // safe shutdown logic 
    if (input_thread.joinable()) {
        // waits for input_thread to finish (a thread may only be joined once)
        input_thread.join();
    }
    return 0; 
}
