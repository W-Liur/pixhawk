/************************************************
* text: test_acc_gyro_xyz
* author :tianshao
* input:
* output:
* function: this is a test,the objective is to measure
           the acceleration and gyroscop of sensor
*************************************************/
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/test_acc_gyro_xyz.h>

__EXPORT int test_acc_gyro_xyz_main(int argc, char *argv[]);

int test_acc_gyro_xyz_main(int argc, char *argv[])
{
	PX4_INFO("Hello tianshao!");
//*************test 订阅数据 acc的xyz  gpyo的xyz×××××××××××××××
	/* subscribe to sensor_gyro topic */
	int sensor_gyro_sub_fd = orb_subscribe(ORB_ID(sensor_gyro));  //
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_gyro_sub_fd, 200);
        
        /* subscribe to sensor_accel topic*/
        int sensor_acc_sub_fd = orb_subscribe(ORB_ID(sensor_accel));
        /* limit the update rate to 5 Hz */
        orb_set_interval(sensor_acc_sub_fd,200);      
	
        /* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {		
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
                { .fd = sensor_gyro_sub_fd,  .events = POLLIN },
                { .fd = sensor_acc_sub_fd,   .events = POLLIN },
	};
//************公告test_acc_gyro_xyz主题**********************
	/* advertise test_acc_gyro_xyz topic */
	struct test_acc_gyro_xyz_s test;
	memset(&test, 0, sizeof(test));
	orb_advert_t test_pub = orb_advertise(ORB_ID(test_acc_gyro_xyz), &test);

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 2, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0 ].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_gyro_s gyro;
                               //  struct sensor_accel_s acc;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_gyro), sensor_gyro_sub_fd, &gyro);
                              //  orb_copy(ORB_ID(sensor_accel), sensor_acc_sub_fd, &acc);  
				PX4_INFO("GYRO is:\t%8.4f\t%8.4f\t%8.4f",
					 (double)gyro.x,
					 (double)gyro.y,
					 (double)gyro.z);
                     
				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example   */
				test.gyro_x = (float32)gyro.x;
				test.gyro_y = (float32)gyro.y;
				test.gyro_z = (float32)gyro.z;

			}  
                        if (fds[1].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_accel_s acc;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_accel), sensor_acc_sub_fd, &acc);              
                                PX4_INFO("accel is:\t%8.4f\t%8.4f\t%8.4f",
                                         (double)acc.x,
					 (double)acc.y,
					 (double)acc.z);
                                test.acc_x = (float32)acc.x;
				test.acc_y = (float32)acc.y;
				test.acc_z = (float32)acc.z;
                        }
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
                      PX4_INFO("test is:\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
                        //!!! must add (double) in the front of variable!!!!
                           (double)test.gyro_x,(double)test.gyro_y ,(double)test.gyro_z,
                           (double)test.acc_x, (double)test.acc_y,  (double)test.acc_z);  
		      orb_publish(ORB_ID(test_acc_gyro_xyz), test_pub, &test);    
      
	        }
                    
        }

	PX4_INFO("exiting");

	return 0;
}
