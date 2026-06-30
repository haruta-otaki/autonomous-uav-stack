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
    Latency
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
            mode = Mode::Latency;
        } else {
            mode = Mode::Normal;
        }
    }
};

FailureRequest failure_request("normal"); 
std::atomic<bool> incoming_request(false); 
std::atomic<bool> processed_request(true); 
std::atomic<Mode> current_mode(Mode::Normal);
std::atomic<double> current_duration(0.0);

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

void interruptible_sleep(double duration) {
    ros::Time start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start).toSec() < duration) {
        ros::Duration(0.1).sleep();
    }
}

void dropout(double duration) {
    // cut QGC - PX4 connection
    // installs a queuing discipline (qdisc) or router on the loopback interface (lo) at the root of its traffic control hierarchy
    // with the "prio" qdisc type, which creates multiple priority bands that traffic can be classified into
    if (system("sudo tc qdisc add dev lo root handle 1: prio")) {
        ROS_WARN("[FAIL_SIM] failed adding root qdisc");
    }
    // adds a filter that inspects packets and decides which band (1:) to send them into such that 
    // any packet destined for ip port 14550 gets routed into queue 1:1
    if (system("sudo tc filter add dev lo protocol ip parent 1:0 \
            prio 1 u32 match ip dport 14550 0xffff flowid 1:1")) {
        ROS_WARN("[FAIL_SIM] failed adding filter");
    }
    //  attaches netem (network emulator qdisc, which can drop, delay, duplicate, or corrupt packets) to band 1:1 specifically
    // such that only packets that were filtered into band 1:1 get dropped. Everything else passes through bands 1:2/1:3 untouched.
    if (system("tc qdisc add dev lo parent 1:1 handle 10: netem loss 100%")) {
        ROS_WARN("[FAIL_SIM] failed attaching network emulator");
    }
    
    // wait
    interruptible_sleep(duration);
    
    // revive connection
    // deletes the entire root qdisc you installed (netem and the filter were both children of that root thus, removing them as well)
    if (system("sudo tc qdisc del dev lo root") != 0) {
        ROS_WARN("[FAIL_SIM] failed deleting root qdisc");
    }
    ROS_INFO("[FAIL_SIM] connection restored");
}

void network_thread() {
    while (ros::ok()) {
        if (!processed_request && current_mode != Mode::Normal) {
            {
                std::lock_guard<std::mutex> lock(mtx);  
                current_duration = failure_request.failure_duration; 
            }
            dropout(current_duration);
            processed_request = true; 
        }
        // prevent busy-spin
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "failure_sim_node");
    ros::NodeHandle nh;

    // runs the terminal input function in a seperate thread to not block the ROS loop
    std::thread input_thread(terminal_thread);

    // detach() allows fail_sim_thread to not block the main thread when it calls sleep()
    std::thread fail_sim_thread(network_thread);
    
    ros::Rate rate(20.0);

    double current_duration = 0.0; 
    ROS_INFO("[FAIL_SIM] failure simulation setting up...");

    while(ros::ok()) {
        if (incoming_request) {  
            ROS_INFO("[FAIL_SIM] processing request...");
            std::lock_guard<std::mutex> lock(mtx);  
            current_mode = failure_request.mode; 
            current_duration = failure_request.failure_duration; 
            failure_request.last_request = ros::Time::now();
            incoming_request = false; 
            processed_request = false; 
        }
        switch (current_mode) {
            case Mode::Normal: 
            // prints every x seconds
                ROS_INFO_THROTTLE(4.0, "[FAIL_SIM] manual control...");
                break;
            // in dropouts, intentionally publish nothing
            case Mode::Short_Dropout: 
                ROS_INFO_THROTTLE(1.0, "[FAIL_SIM] performing short dropout...");
                if (processed_request) {
                    ROS_INFO("[FAIL_SIM] short dropout of %fs complete...", current_duration);
                    current_mode = Mode::Normal; 
                }
                break;
            case Mode::Long_Dropout:
                ROS_INFO_THROTTLE(1.0, "[FAIL_SIM] performing long dropout...");
                if (processed_request) {
                    ROS_INFO("[FAIL_SIM] long dropout of %fs complete...", current_duration);
                    current_mode = Mode::Normal; 
                }
                break;
            case Mode::Latency: 
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
    if (fail_sim_thread.joinable()) {
        fail_sim_thread.join();
    }
    system("sudo tc qdisc del dev lo root 2>/dev/null || true");
    return 0; 
}
