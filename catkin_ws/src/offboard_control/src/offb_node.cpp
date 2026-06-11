#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// contains all custom messages required to operate services and topics by mavros
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// callback that saves the current state of the autopilot 
mavros_msgs::State current_state; 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// a topic is continuous streaming
// a service is request / response

// asks the px4 to enter offboard mode then arm the drone 
// offboard mode is defined as a mode in which the drone is controlled by an external computer (ros node)
int main(int argc, char **argv) {
    // starts ros node
    ros::init(argc, argv, "offb_node");
    // node's access point to ros
    ros::NodeHandle nh;

    //topic: mavros/state, queue size: 10, callback: state_cb()
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros_state", 10, state_cb);
        
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

    // wait for connection to be established between mavros and the autopilot 
    // loop exits when heartbeat message (small mavlink message that tells whether the system is alive) is received
    while(ros::ok() && !current_state.connected) {
        // lets ros process callbacks once to update current_state
        ros::spinOnce();
        rate.sleep();
    }

    // PX4 flight stack operates in NED coordinate frame but mavros translates them to standard ENU frame 
    // target position that is consistently sent to px4 (it expects continous commands)
    // poseStamped: postition+orientation+timeframe
    // NED: x: North, y: East, z: Down
    // ENU: x: East, y: North, z: Up
    geometry_msgs::PoseStamped pose; 
    pose.pose.position.x = 0; 
    pose.pose.position.y = 0; 
    pose.pose.position.z = 2; 

    // setpoints must be already streamed before entering offboard mode (PX4 rejects it otherwise)
    // 100 messages across 5 seconds before requesting offboard 
    for (int i = 100; ros::ok() && i > 0; i--) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode; 
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd; 
    arm_cmd.request.value = true; 

    ros::Time last_request = ros::Time::now();

    // while ros is running normally... 
    while(ros::ok()) {
        // space the service calls by 5s (usually shorter) to not flood the autopilot with requests 
        if (current_state.mode != "OFFBOARD" && 
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
            // asks to switch to offboard mode and checks if mavros sent mode-change request to px4
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
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
            }
            last_request = ros::Time::now();
        }
        // continue sending the requested pose at the appropriate rate 
        local_pos_pub.publish(pose);

        //keeps the loop at 20 Hz
        ros::spinOnce();
        rate.sleep();
    }
    return 0; 
}
