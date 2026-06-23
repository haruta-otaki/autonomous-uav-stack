#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <sensor_msgs/BatteryState.h>

#include <string>
#include <vector>
#include <fstream>
#include <sstream>

struct FailureRequest {
    // Mode mode; 
    double failure_duration; 
    ros::Time last_request; 

    FailureRequest(std::string input) {
    }
};

FailureRequest failure_request("normal"); 

mavros_msgs::State current_state; 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose; 
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

sensor_msgs::BatteryState current_battery; 
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg){
    current_battery = *msg;
}

// sensor_msgs::BatteryState current_failure_mode; 
// void mode_cb(const sensor_msgs::BatteryState::ConstPtr& msg){
//     current_failure_mode = *msg;
// }

void log(std::string filename, std::string description, double data) {
    std::ofstream file(filename, std::ios::app); 
    file << description << ": " << data << "\n";
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "metrics_node");
    ros::NodeHandle nh;

    // std::thread input_thread(terminal_thread);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber battery_sub = nh.subscribe<sensor_msgs::BatteryState>
        ("metrics/intermediate_battery", 10, battery_cb);
    // ros::Subscriber mode_sub = nh.subscribe<std::string>
    //     ("metrics/intermediate_mode", 10, mode_cb);

    ros::Publisher intermediate_state_pub = nh.advertise<mavros_msgs::State> 
        ("offboard_control/intermediate_state_setpoint", 10);

    ros::Publisher intermediate_pose_pub = nh.advertise<geometry_msgs::PoseStamped> 
        ("offboard_control/intermediate_pose_setpoint", 10);
    
    ros::Rate rate(20.0);

    //enter number here 
    std::stringstream ss;
    int id = 0;
    ss << "metrics" << id << ".csv";
    std::string filename = ss.str();
    std::ofstream csvFile(filename);

    // std::vector<geometry_msgs::PoseStamped> waypoints = {
    //     make_pose(0.0, 0.0, 2.0), make_pose(0.0, 0.0, 2.0), make_pose(0.0, 9.5, 2.0), make_pose(-15.0, 9.5, 2.0), 
    //     make_pose(-15.0, 15.0, 2.0), make_pose(-15.0, 15.0, 0.3), make_pose(-15.0, 15.0, 0.3) 
    // };

    double current_duration = 0.0; 
    ros::Time last_request = ros::Time::now(); 
    ROS_INFO("failure simulation setting up...");

    ros::Time mission_start_time;
    ros::Time mission_end_time;
    ros::Time failure_start_time;
    ros::Time failure_end_time;
    double failure_duration;

    bool takeoff = false; 
    bool land = false; 
    std::string last_mode; 
            std::string description;

    while(ros::ok()) {
        if (current_state.mode == "AUTO.TAKEOFF" && !takeoff) {
            mission_start_time = ros::Time::now();
            description = "mission start time";
            log(filename, description, mission_start_time.toSec());
        } else if (current_state.mode == "AUTO.Land" && !land) {
            mission_end_time = ros::Time::now();
            description = "mission end time";
            log(filename, description, mission_end_time.toSec());
        }
        else if (current_state.mode == "AUTO.LOITER" || current_state.mode == "POSTCTL") {
            if (last_mode != "AUTO.LOITER" || last_mode != "POSTCTL") {
                failure_end_time = ros::Time::now();
                description = "failure end time";
                log(filename, description, failure_end_time.toSec());
                failure_duration = failure_end_time.toSec() - failure_start_time.toSec();
                description = "failure duration";
                log(filename, description, failure_duration);             
            }
        } else {
            if (last_mode == "AUTO.LOITER" || last_mode == "POSTCTL") {
                failure_start_time = ros::Time::now();
                description = "failure start time";
                log(filename, description, failure_start_time.toSec());
            }
        }
        // if (incoming_request) {  
        //     ROS_INFO("processing request...");
        //     std::lock_guard<std::mutex> lock(mtx);  
        //     current_mode = failure_request.mode; 
        //     current_duration = failure_request.failure_duration; 
        //     failure_request.last_request = ros::Time::now();
        //     last_request = ros::Time::now(); 
        //     incoming_request = false; 
        // }
        // switch (current_mode) {
        //     case Mode::Normal: 
        //         ROS_INFO_THROTTLE(4.0, "manual control...");
        //         intermediate_pose_pub.publish(current_pose);
        //         // intermediate_state_pub.publish(current_state);
        //         break;
        // }
        ros::spinOnce();
        rate.sleep();
    }

    // if (input_thread.joinable()) {
    //     input_thread.join();
    // }
    return 0; 
}
