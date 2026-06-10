#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/sensor_combined.h>


// macro that tells the compiler / linker for the symbol (for px4_simple_app_) to be visible to PX4
__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[]) {
    // print statement 
    PX4_INFO("Hello Sky!");

    // topics: individual message channels between applications
    // topic handle, use the sensor_combined topic that holds synchronized sensor data by performing a blocking wait 
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

    px4_pollfd_struct_t fds[] = {
        { .fd = sensor_sub_fd,  .events=POLLIN},
    };

    while (true) {
        // current thread goes to sleep and is woken up automatically by the scheduler once new data is available 
        // wait for sensor update of 1 file descriptor for 1 second (1000 ms)
        int poll_ret = px4_poll(fds, 1, 1000);
        if (poll_ret == 0) {
            continue;
        }
        else if (poll_ret < 0) {
            PX4_ERR("poll error");
            continue; 
        }
        else {
            if (fds[0].revents & POLLIN) {
                struct sensor_combined_s raw; 
                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
                PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
                        (double)raw.accelerometer_m_s2[0],
                        (double)raw.accelerometer_m_s2[1],
                        (double)raw.accelerometer_m_s2[2]);
            }
        }
        
    }
    return OK; 
}


