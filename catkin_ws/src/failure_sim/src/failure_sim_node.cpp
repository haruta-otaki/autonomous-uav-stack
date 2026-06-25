#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <iostream>
#include <mutex>
#include <random>

std::mutex mtx;

std::random_device rd; 
std::mt19937 gen(rd());

double random_duration(double min, double max) {
    std::uniform_real_distribution<double> dist(min, max);
    return dist(gen);
}

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

    FailureRequest(std::string input) {
        if (input == "short") {
            ROS_INFO("[FAIL_SIM] short dropout...");
            failure_duration = random_duration(2.0, 5.0); 
            mode = Mode::Short_Dropout;
        } else if (input == "long") {
            ROS_INFO("[FAIL_SIM] long dropout...");
            failure_duration = random_duration(10.0, 30.0); 
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
    }
};

FailureRequest failure_request("normal"); 
std::atomic<bool> incoming_request(false); 
std::atomic<Mode> current_mode(Mode::Normal);

// callback that saves the current state of the autopilot 
// mavros_msgs::State current_state; 
// void state_cb(const mavros_msgs::State::ConstPtr& msg){
//     current_state = *msg;
// }

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
            } else if (current_mode != Mode::Normal) {
                ROS_ERROR("[FAIL_SIM] previous failure mode is still running");
            } else {
                std::lock_guard<std::mutex> lock(mtx);  
                failure_request = FailureRequest(input);
                incoming_request = true; 
            }
        } else {
            incoming_request = false; 
        }
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "failure_sim_node");
    ros::NodeHandle nh;

    // runs the terminal input function in a seperate thread to not block the ROS loop
    std::thread input_thread(terminal_thread);

    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    //     ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, pose_cb);
    
    // ros::Publisher intermediate_state_pub = nh.advertise<mavros_msgs::State> 
    //     ("supervisor/intermediate_state_setpoint", 10);

    ros::Publisher intermediate_pose_pub = nh.advertise<geometry_msgs::PoseStamped> 
        ("supervisor/intermediate_mavros_pose", 10);
    
    ros::Rate rate(20.0);

    // std::vector<geometry_msgs::PoseStamped> waypoints = {
    //     make_pose(0.0, 0.0, 2.0), make_pose(0.0, 0.0, 2.0), make_pose(0.0, 9.5, 2.0), make_pose(-15.0, 9.5, 2.0), 
    //     make_pose(-15.0, 15.0, 2.0), make_pose(-15.0, 15.0, 0.3), make_pose(-15.0, 15.0, 0.3) 
    // };

    double current_duration = 0.0; 
    ros::Time last_request = ros::Time::now(); 
    ROS_INFO("[FAIL_SIM] failure simulation setting up...");

    while(ros::ok()) {
        if (incoming_request) {  
            ROS_INFO("[FAIL_SIM] processing request...");
            std::lock_guard<std::mutex> lock(mtx);  
            current_mode = failure_request.mode; 
            current_duration = failure_request.failure_duration; 
            failure_request.last_request = ros::Time::now();
            last_request = ros::Time::now(); 
            incoming_request = false; 
        }
        switch (current_mode) {
            case Mode::Normal: 
            // prints every x seconds
                ROS_INFO_THROTTLE(4.0, "[FAIL_SIM] manual control...");
                intermediate_pose_pub.publish(current_pose);
                // intermediate_state_pub.publish(current_state);
                break;
            // in dropouts, intentionally publish nothing
            case Mode::Short_Dropout: 
                ROS_INFO_THROTTLE(1.0, "[FAIL_SIM] performing short dropout...");
                if (ros::Time::now() - last_request > ros::Duration(current_duration)) {
                    ROS_INFO("[FAIL_SIM] short dropout of %fs complete...", current_duration);
                    current_mode = Mode::Normal; 
                }
                break;
            case Mode::Long_Dropout:
                ROS_INFO_THROTTLE(1.0, "[FAIL_SIM] performing long dropout...");
                if (ros::Time::now() - last_request > ros::Duration(current_duration)) {
                    ROS_INFO("[FAIL_SIM] long dropout of %fs complete...", current_duration);
                    current_mode = Mode::Normal; 
                }
                break;
            case Mode::Burst_Short_Dropout: 
                break;
            case Mode::Burst_Long_Dropout: 
                break;
            case Mode::Command_Degradation: 
                break;
            case Mode::State_Degradation: 
                break;
        }
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
