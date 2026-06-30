#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <sensor_msgs/BatteryState.h>

#include <supervisor/FailureMode.h>

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>

enum class LogType {
    Normal, 
    Short_Dropout,
    Long_Dropout,
    Burst_Short_Dropout,
    Burst_Long_Dropout,
    Command_Degradation,
    State_Degradation
};

class MetricsLogger {
    // make file a member variable (attribute) in MetricLogger class such that it is accessible by any function within that same class
    // unlike local variables that only exist inside a single function, preventing reopening and closing the file at each write
    std::ofstream file; 
    ros::Time creation_time;
    bool creation = false; 

public: 
    MetricsLogger(const std::string& file_path) {
        creation = true;
        creation_time = ros::Time::now();
        file.open(file_path);
        // columns
        file <<  "time,event,mode,x,y,z,battery,details\n";
        ROS_INFO("[METRICS] logging...");
    }

    void write(const std::string& event, const std::string& mode, const geometry_msgs::PoseStamped& pose,
            const sensor_msgs::BatteryState& battery, const std::string& details = "") {
        double elapsed_time = (ros::Time::now() - creation_time).toSec();

        // allows entries of decimals with fixed point notation up to 3 decimal points
        file << std::fixed << std::setprecision(3)
            << elapsed_time                               << ","
            << event                                      << ","
            << mode                                       << ","
            << pose.pose.position.x                       << ","
            << pose.pose.position.y                       << ","
            << pose.pose.position.z                       << ","
            << battery.percentage                         << ","
            << details                                    << "\n";

        file.flush(); 
    }
};

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

supervisor::FailureMode current_failure_mode; 
void mode_cb(const supervisor::FailureMode::ConstPtr& msg){
    current_failure_mode = *msg;
}

std::stringstream log_detail() {
    std::stringstream detail;
    switch(current_failure_mode.mode) {
        case supervisor::FailureMode::HOVER:
            detail << "Fallback: " << "Hover";
            break; 
        case supervisor::FailureMode::LAND:
            detail << "Fallback: " << "Land";
            break; 
        case supervisor::FailureMode::RTL:
            detail << "Fallback: " << "RTL";
            break; 
        case supervisor::FailureMode::CONTINUE:
            detail << "Fallback: " << "Continue";
            break; 
        case supervisor::FailureMode::SMART_HOVER:
            detail << "Fallback: " << "Smart Hover";
            break; 
        case supervisor::FailureMode::SMART_LAND:
            detail << "Fallback: " << "Smart Land";
            break; 
        case supervisor::FailureMode::SMART_RTL:
            detail << "Fallback: " << "Smart RTL";
            break; 
    }
    return detail;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "metrics_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~"); 

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber battery_sub = nh.subscribe<sensor_msgs::BatteryState>
        ("mavros/battery", 10, battery_cb);
    ros::Subscriber mode_sub = nh.subscribe<supervisor::FailureMode>
        ("supervisor/failure_mode", 10, mode_cb);

    // read parameters from launch file 
    int trial_id; 
    std::string failure_mode; 
    std::string fallback_mode; 
    std::string metrics_path; 
    // do not provide default values to avoid silent failures 
    nh_private.param("trial_id", trial_id, -1);
    nh_private.param("failure_mode", failure_mode, std::string(""));
    nh_private.param("fallback_mode", fallback_mode, std::string(""));
    nh_private.param("metrics_path", metrics_path, std::string(""));

    ros::Rate rate(20.0);

    std::stringstream ss;
    ss << metrics_path << "/logs/" 
    << "trial_" << trial_id 
    << "_" << failure_mode 
    << "_" << fallback_mode 
    << ".csv";
    MetricsLogger logger(ss.str());

    ROS_INFO("failure simulation setting up...");

    ros::Time failure_time;
    double failure_duration;

    bool takeoff = false; 
    bool land = false; 
    bool failure_start = false;
    bool failure_end = false;
    std::string current_mode; 
    std::string last_mode; 


    while(ros::ok()) {
        current_mode = current_state.mode; 
        failure_start = current_mode == "OFFBOARD" && last_mode != "OFFBOARD"; 
        failure_end = last_mode == "OFFBOARD" && (current_mode == "AUTO.LOITER" || current_mode == "POSCTL");
        if (current_mode == "AUTO.TAKEOFF" && !takeoff) {
            ROS_INFO("[METRICS] logging takeoff...");
            logger.write("Mission Start", current_mode, current_pose, current_battery, "");
            takeoff = true;
        } else if (current_mode == "AUTO.LAND" && !land) {
            ROS_INFO("[METRICS] logging landing...");
            logger.write("Mission Complete", current_mode, current_pose, current_battery, "");
            land = true; 
        }
        else if (current_mode == "AUTO.LOITER" || current_mode == "POSTCTL") {
            if (failure_end) {
                ROS_INFO("[METRICS] logging failure completion...");
                failure_duration = (ros::Time::now() - failure_time).toSec();
                std::stringstream detail = log_detail();
                detail << std::fixed << std::setprecision(3) 
                << "Duration: " << failure_duration;
                logger.write("Failure End", current_mode, current_pose, current_battery, detail.str());
            }
        } else {
            if (failure_start) {
                ROS_INFO("[METRICS] logging failure start...");
                failure_time = ros::Time::now();
                std::stringstream detail = log_detail();
                logger.write("Failure Start", current_mode, current_pose, current_battery, detail.str());
            }
        }
        last_mode = current_mode;
        ros::spinOnce();
        rate.sleep();
    }

    return 0; 
}
