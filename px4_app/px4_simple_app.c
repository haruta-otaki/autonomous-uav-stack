#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
// topic definitions for the used data structures (synonymous to ROS messages)
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

// macro that tells the compiler / linker for the symbol (for px4_simple_app_) to be visible to PX4
__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[]) {
    // print statement (from log.h)
    PX4_INFO("Hello Sky!");

    //orb...() are for ros publisher / subscriber functionalities in px4
    // topics: individual message channels between applications
    // topic handle, use the sensor_combined topic that holds synchronized sensor data by performing a blocking wait 
    // ORB_ID(): returns a unique identifier for that topic.
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    // limit update rate to 5 Hz, as sensor data arrives much faster, this reduces CPU usage 
    // do not wake me up more frequently than once every 200 ms
    orb_set_interval(sensor_sub_fd, 200);

    // object to publish vehicle attitude topic in px4
    struct vehicle_attitude_s att;
    // fill a contiguous block of memory with a specific byte value (from string.h)
    // zero out the struct as the initially created memory has garbage
    memset(&att, 0, sizeof(att));
    orb_advert_t att_pub_fd = orb_advertise(ORB_ID(vehicle_attitude), &att);
    
    // technique to wait for multiple topics by having multiple file descriptors 
    // from posix.h, a linux wrapper for px4
    px4_pollfd_struct_t fds[] = {
        // .revents is a bitmask for the bits in the .events
        { .fd = sensor_sub_fd,  .events=POLLIN},
    };

    int error_counter = 0;
    for (int i = 0; i < 5; i++) {
        // current thread goes to sleep and is woken up automatically by the scheduler once new data is available 
        // wait for sensor update of 1 file descriptor for 1 second (1000 ms)
        int poll_ret = px4_poll(fds, 1, 1000);
        if (poll_ret == 0) {
            // no providers giving data to subscriber
            PX4_ERR("received no data");
            continue;
        }
        else if (poll_ret < 0) {
            // entails emergency, use counter to prevent flooding
            if (error_counter < 10 || error_counter % 50 == 0) {
                PX4_ERR("return value from poll(): %d", poll_ret);
            }
            error_counter++; 
            continue; 
        }
        else {
            // revents contains event flag for each file descriptor
            // POLLIN represents data is available for read
            if (fds[0].revents & POLLIN) {
                // obtained data for the first file descriptor 
                struct sensor_combined_s raw; 
                // copy sensors raw data into local buffer (do not overwrite)
                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
                PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
                        (double)raw.accelerometer_m_s2[0],
                        (double)raw.accelerometer_m_s2[1],
                        (double)raw.accelerometer_m_s2[2]);
                // set att, then publish the information for other apps 
                att.q[0] = raw.accelerometer_m_s2[0];
                att.q[1] = raw.accelerometer_m_s2[1];
                att.q[2] = raw.accelerometer_m_s2[2];

                orb_publish(ORB_ID(vehicle_attitude), att_pub_fd, &att);
            }
            // for more file descriptors, create more condition of the above form through fds indexing 
        }
    }
    PX4_INFO("exiting...");
    return OK; 
}


