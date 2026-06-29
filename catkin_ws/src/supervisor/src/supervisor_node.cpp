#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// contains all custom messages required to operate services and topics by mavros
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <sensor_msgs/BatteryState.h>

// include custom message
#include <supervisor/FailureMode.h>

#include <string>
#include <vector>
#include <limits>

enum class Mode {
    Init, 
    Prestream,
    Hover,
    Land,
    RTL, 
    Continue, 
    Smart_Hover,
    Smart_Land,
    Smart_RTL
};

// tune watchdog parameters based on hardware
// consideration: only tracking pose, as state is necessary for changing modes 
struct Watchdog {
    //fields 
    int consecutive_feeds = 0; 
    ros::Time last_cb; 
    bool warn = false; 
    bool trigger = false; 
    double warn_timeout; 
    double trigger_timeout; 
    int feed_threshold = 4;

    // constructor 
    Watchdog(double warn_time=0.4, double trigger_time=0.8) {
        warn_timeout = warn_time;
        trigger_timeout = trigger_time;
        last_cb = ros::Time(0);
    }

    // methods
    void feed() {
        last_cb = ros::Time::now();
        consecutive_feeds += 1; 
        if (consecutive_feeds > feed_threshold) {
            warn = false; 
            trigger = false; 
        }
    }

    void tick() {
        if (!warn && ros::Time::now() - last_cb > ros::Duration(warn_timeout)) {
            consecutive_feeds = 0; 
            warn = true; 
            ROS_WARN("[SUPERVISOR] Watchdog: no pose received, monitoring...");
        }
        if (!trigger && ros::Time::now() - last_cb > ros::Duration(trigger_timeout)) {
            consecutive_feeds = 0; 
            trigger = true; 
            ROS_ERROR("[SUPERVISOR] Watchdog: communication loss detected, trigerring offboard...");
        }
    }

    bool is_healthy() {
        return !trigger; 
    }
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

supervisor::FailureMode failure_mode_msg; 
mavros_msgs::State current_state; 
geometry_msgs::PoseStamped current_pose; 
geometry_msgs::PoseStamped command_pose; 
sensor_msgs::BatteryState current_battery; 
Watchdog pose_watchdog(0.4, 0.8);

// callback that saves the currfailure_mode_msgent state of the autopilot 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
    received_pose = true; 
    pose_watchdog.feed();
}

void offboard_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    command_pose = *msg;
}

void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg){
    current_battery = *msg;
}

// a topic is continuous streaming
// a service is request / response

// asks the px4 to enter offboard mode then arm the drone 
// offboard mode is defined as a mode in which the drone is controlled by an external computer (ros node)
int main(int argc, char **argv) {
    // starts ros node
    ros::init(argc, argv, "supervisor_node");
    // node's access point to ros
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~"); 

    //topic: mavros/state, queue size: 10, callback: state_cb()
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    // placeholder, reconsider 
    ros::Subscriber mavros_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("supervisor/intermediate_mavros_pose", 10, pose_cb);
    
    ros::Subscriber offboard_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("supervisor/intermediate_offboard_pose", 10, offboard_pose_cb);

    ros::Subscriber battery_sub = nh.subscribe<sensor_msgs::BatteryState>
        ("mavros/battery", 10, battery_cb);
        
    // publishes the commanded local position (relative to local origin)
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> 
        ("mavros/setpoint_position/local", 10);

    ros::Publisher mode_pub = nh.advertise<supervisor::FailureMode> 
        ("supervisor/failure_mode", 10);

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

    mavros_msgs::SetMode manual_set_mode; 
    mavros_msgs::SetMode hover_set_mode; 
    mavros_msgs::SetMode land_set_mode; 
    mavros_msgs::SetMode rtl_set_mode; 
    mavros_msgs::SetMode offboard_set_mode; 

    manual_set_mode.request.custom_mode = "POSCTL";
    hover_set_mode.request.custom_mode = "AUTO.LOITER";
    land_set_mode.request.custom_mode = "AUTO.LAND";
    rtl_set_mode.request.custom_mode = "AUTO.RTL";
    offboard_set_mode.request.custom_mode = "OFFBOARD";
    // consider "AUTO.MISSION"
    
    std::string fallback_mode; 
    nh_private.param("fallback_mode", fallback_mode, std::string(""));
    ROS_INFO_STREAM("[SUPERVISOR] fallback_mode = '" << fallback_mode << "'");

    // update the machine states
    std::vector<MachineState> states = {
        MachineState(0.0, 0.0, 2.0, Mode::Init), MachineState(0.0, 0.0, 2.0, Mode::Prestream), MachineState(-15.0, 15.0, 0.3, Mode::Land)
    };

    int mode_index = 0; 
    bool recovery_eligible; 
    ros::Time last_request = ros::Time::now();
    // currently unused 
    mavros_msgs::CommandBool arm_cmd; 
    
    ROS_INFO("[SUPERVISOR] initializing...");
    Mode current_mode = Mode::Init;
    command_pose = states[0].pose;

    // while ros is running normally... 
    while(ros::ok()) {
        switch (current_mode) {
            case Mode::Init: 
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
                    local_pos_pub.publish(states[mode_index].pose);
                    ros::spinOnce();
                    rate.sleep();
                }

                mode_index += 1; 
                current_mode = Mode::Prestream;
                failure_mode_msg.mode = supervisor::FailureMode::PRESTREAM; 
                ROS_INFO("[SUPERVISOR] prestreaming...");
                break;
            case Mode::Prestream:
                local_pos_pub.publish(current_pose);
                if (received_pose) {
                    pose_watchdog.tick();
                    if (!pose_watchdog.is_healthy()) {
                        ROS_INFO_STREAM("[SUPERVISOR] fallback_mode = '" << fallback_mode << "'");
                        if (fallback_mode == "hover") {
                            ROS_INFO("[SUPERVISOR] hovering...");
                            current_mode = Mode::Hover;
                            failure_mode_msg.mode = supervisor::FailureMode::HOVER; 
                        } else if (fallback_mode == "land") {
                            ROS_INFO("[SUPERVISOR] landing...");
                            current_mode = Mode::Land;
                            failure_mode_msg.mode = supervisor::FailureMode::LAND; 
                        } else if (fallback_mode == "rtl") {
                            ROS_INFO("[SUPERVISOR] returning...");
                            current_mode = Mode::RTL;
                            failure_mode_msg.mode = supervisor::FailureMode::RTL; 
                        } else if (fallback_mode == "continue") {
                            ROS_INFO("[SUPERVISOR] continuing...");
                            current_mode = Mode::Continue;
                            failure_mode_msg.mode = supervisor::FailureMode::CONTINUE; 
                        } else if (fallback_mode == "smart_hover") {
                            ROS_INFO("[SUPERVISOR] hovering(smart)...");
                            current_mode = Mode::Smart_Hover;
                            failure_mode_msg.mode = supervisor::FailureMode::SMART_HOVER; 
                        } else if (fallback_mode == "smart_land") {
                            ROS_INFO("[SUPERVISOR] landing(smart)...");
                            current_mode = Mode::Smart_Land;
                            failure_mode_msg.mode = supervisor::FailureMode::SMART_LAND; 
                        } else {
                            ROS_INFO("[SUPERVISOR] returning(smart)...");
                            current_mode = Mode::Smart_RTL;
                            failure_mode_msg.mode = supervisor::FailureMode::SMART_RTL; 
                        }
                    }
                }
                break;
            case Mode::Hover: 
                // space the service calls by 5s (usually shorter) to not flood the autopilot with requests 
                if (current_state.mode != "AUTO.LOITER" && 
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    // asks to switch to offboard mode and checks if mavros sent mode-change request to px4
                    if (set_mode_client.call(hover_set_mode) && hover_set_mode.response.mode_sent) {
                        ROS_INFO("[SUPERVISOR] hovering enabled");
                    }
                    last_request = ros::Time::now();
                } 
                break;
            case Mode::Land: 
                if (current_state.mode != "AUTO.LAND" && 
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent) {
                        ROS_INFO("[SUPERVISOR] landing enabled");
                    }
                    last_request = ros::Time::now();
                } 
                break;
            case Mode::RTL: 
                if (current_state.mode != "AUTO.RTL" && 
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (set_mode_client.call(rtl_set_mode) && rtl_set_mode.response.mode_sent) {
                        ROS_INFO("[SUPERVISOR] returning enabled");
                    }
                    last_request = ros::Time::now();
                } 
                break;            
            case Mode::Continue:
                if (current_state.mode != "OFFBOARD" && 
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
                        ROS_INFO("[SUPERVISOR] continuing logic enabled");
                    }
                    last_request = ros::Time::now();
                } 
                local_pos_pub.publish(command_pose);
                break;
            case Mode::Smart_Hover:
                if (current_state.mode != "OFFBOARD" && 
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
                        ROS_INFO("[SUPERVISOR] smart hovering enabled");
                    }
                    last_request = ros::Time::now();
                } 
                break;
            case Mode::Smart_Land:
                if (current_state.mode != "OFFBOARD" && 
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
                        ROS_INFO("[SUPERVISOR] smart landing enabled");
                    }
                    last_request = ros::Time::now();
                } 
                break;
            case Mode::Smart_RTL:
                if (current_state.mode != "OFFBOARD" && 
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
                        ROS_INFO("[SUPERVISOR] smart returning enabled");
                    }
                    last_request = ros::Time::now();
                } 
                break;
        }
        recovery_eligible = current_mode != Mode::Land && current_mode != Mode::Smart_Land;

        if (pose_watchdog.is_healthy() && current_mode != Mode::Prestream && recovery_eligible) {
            manual_set_mode.request.custom_mode = "POSCTL";
            if (current_state.mode != "POSCTL" && 
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
                // asks to switch to offboard mode and checks if mavros sent mode-change request to px4
                if (set_mode_client.call(manual_set_mode) && manual_set_mode.response.mode_sent) {
                    ROS_INFO("[SUPERVISOR] manual control enabled");
                }
                last_request = ros::Time::now();
            } 
            current_mode = Mode::Prestream;
            mode_index = 1; 
            ROS_INFO_THROTTLE(4.0, "[SUPERVISOR] prestreaming...");
        }
        mode_pub.publish(failure_mode_msg);
        //keeps the loop at 20 Hz
        ros::spinOnce();
        rate.sleep();
    }
    return 0; 
}
