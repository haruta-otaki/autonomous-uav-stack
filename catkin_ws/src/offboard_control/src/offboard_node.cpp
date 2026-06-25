#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// contains all custom messages required to operate services and topics by mavros
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <sensor_msgs/BatteryState.h>

#include <string>
#include <vector>
#include <limits>

const int FIRST_WAYPOINT_INDEX = 3; // exclude offboard protocols
const int LAST_WAYPOINT_INDEX = 7; // exclude halt
// FSM: INIT WAIT PRESET OFFBOARD ARM WAYPOINT_1 ~ 3 LAND HALT 
enum class Mode {
    Init, 
    Prestream, 
    Offboard, 
    // Simplify to Waypoint, to make custom messages 
    Waypoint_0,
    Waypoint_1,
    Waypoint_2,
    Waypoint_3,
    Land,
    Halt
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
sensor_msgs::BatteryState current_battery; 

// callback that saves the current state of the autopilot 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
    received_pose = true; 
}

void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg){
    current_battery = *msg;
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
    int index = FIRST_WAYPOINT_INDEX; 

    double minimum_d = std::numeric_limits<double>::max();
    int n = LAST_WAYPOINT_INDEX; 
    // temporarily hard coded 
    for (int i = FIRST_WAYPOINT_INDEX; i < n; i++) {
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
        ("offboard_control/intermediate_pose_setpoint", 10, pose_cb);
       
    ros::Subscriber battery_sub = nh.subscribe<sensor_msgs::BatteryState>
        ("mavros/battery", 10, battery_cb);
        
    // publishes the commanded local position (relative to local origin)
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> 
        ("mavros/setpoint_position/local", 10);
    
    ros::Publisher supervisor_pose_pub = nh.advertise<geometry_msgs::PoseStamped> 
        ("supervisor/intermediate_pose_setpoint", 10);   

    ros::Publisher intermediate_battery_pub = nh.advertise<sensor_msgs::BatteryState> 
        ("metrics/intermediate_battery", 10);

    // ros::Publisher intermediate_mode_pub = nh.advertise<std::string> 
    //     ("metrics/intermediate_mode", 10);

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

    std::vector<MachineState> states = {
        MachineState(0.0, 0.0, 2.0, Mode::Init), MachineState(0.0, 0.0, 2.0, Mode::Prestream), MachineState(0.0, 0.0, 2.0, Mode::Offboard), 
        MachineState(0.0, 0.0, 2.0, Mode::Waypoint_0), MachineState(0.0, 9.5, 2.0, Mode::Waypoint_1), MachineState(-15.0, 9.5, 2.0, Mode::Waypoint_2), 
        MachineState(-15.0, 15.0, 2.0, Mode::Waypoint_3), MachineState(-15.0, 15.0, 0.3, Mode::Land), MachineState(-15.0, 15.0, 0.3, Mode::Halt)
    };

    int mode_index = 0;
    int index = 3; 
    bool dwelling = false; 
    double tol = 0.3; 

    mavros_msgs::SetMode offboard_set_mode; 
    mavros_msgs::SetMode land_set_mode; 

    ros::Time last_request = ros::Time::now();
    ros::Time dwell_start_time = ros::Time::now();

    // currently unused 
    mavros_msgs::CommandBool arm_cmd; 
    
    ROS_INFO("[OFFB_NODE] initializing...");
    Mode current_mode = Mode::Init;
    geometry_msgs::PoseStamped command_pose = states[0].pose;

    // while ros is running normally... 
    while(ros::ok()) {
        switch (current_mode) {
            case Mode::Offboard:
                // comment out the arm logic it is assumed the drone is already armed and manually controlled
                if (current_state.mode == "OFFBOARD" && 
                (ros::Time::now() - last_request > ros::Duration(0.5))) {
                    ROS_INFO("[OFFB_NODE] hovering...");
                    index = find_waypoint(current_pose, states);
                    mode_index = index; 
                    current_mode = states[index].mode;
                }
                // else {
                //     // arm the quad to allow it to fly (spin motors & apply actuator outputs)
                //     if (!current_state.armed && 
                //         (ros::Time::now() - last_request > ros::Duration(5.0))) {
                //         // asks to arm the vehicle && checks mavros' consequential action
                //         if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                //             ROS_INFO("[OFFB_NODE] vehicle armed");
                //         }
                //         last_request = ros::Time::now();
                //     }
                //     if (current_state.armed && 
                //         (ros::Time::now() - last_request > ros::Duration(0.5))) {
                //         ROS_INFO("[OFFB_NODE] hovering...");
                //         index = find_waypoint(current_pose, waypoints, modes);
                //         mode_index = index; 
                //         current_mode = states[index].mode;
                //     }
                // }
                break;
                case Mode::Waypoint_0:
                if (dwell(dwelling, dwell_start_time, states[mode_index].pose, tol)) {
                        ROS_INFO("[OFFB_NODE] dwelling completed");
                        ROS_INFO("[OFFB_NODE] hovering(1)...");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Waypoint_1;
                    
                }
                break;
            case Mode::Waypoint_1: 
                // handleMode(current_mode);
                if (dwell(dwelling, dwell_start_time, states[mode_index].pose, tol)) {
                        ROS_INFO("[OFFB_NODE] dwelling completed");
                        ROS_INFO("[OFFB_NODE] hovering(2)...");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Waypoint_2;
                    
                }

                break;
            case Mode::Waypoint_2: 
                if (dwell(dwelling, dwell_start_time, states[mode_index].pose, tol)) {
                        ROS_INFO("[OFFB_NODE] dwelling completed");
                        ROS_INFO("[OFFB_NODE] hovering(3)...");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Waypoint_3;
                    
                }
                break;
            case Mode::Waypoint_3: 
                if (dwell(dwelling, dwell_start_time, states[mode_index].pose, tol)) {
                        ROS_INFO("[OFFB_NODE] dwelling completed");
                        ROS_INFO("[OFFB_NODE] landing...");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Land;
                    
                }
                break;
            case Mode::Land: 
                if (dwell(dwelling, dwell_start_time, states[mode_index].pose, tol)) {
                        ROS_INFO("[OFFB_NODE] dwelling completed");
                        ROS_INFO("[OFFB_NODE] halting...");
                        dwelling = false; 
                        mode_index += 1; 
                        current_mode = Mode::Halt;
                    
                }
                break;
            case Mode::Halt: 
                land_set_mode.request.custom_mode = "AUTO.LAND";
                if (current_state.mode != "AUTO.LAND" && 
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    // asks to switch to offboard mode and checks if mavros sent mode-change request to px4
                    if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent) {
                        ROS_INFO("[OFFB_NODE] landing enabled");
                    }
                    last_request = ros::Time::now();
                } 
                   
                break;
        }

        if (mode_index > 1 && mode_index < states.size()) {
            // continue sending the requested pose at the appropriate rate 
            // interpolation set at 5cm / tick (1 m/s)
            command_pose = interpolation(command_pose, states[mode_index].pose, 0.05);
            supervisor_pose_pub.publish(command_pose);
        }
        // intermediate_mode_pub.publish("OFFBOARD");
        //keeps the loop at 20 Hz
        ros::spinOnce();
        rate.sleep();
    }
    return 0; 
}