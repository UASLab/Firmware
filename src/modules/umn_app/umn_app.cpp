/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file umn_app.c
 * Application example for PX4 autopilot interfacing with Sensors
 *
 * @author HMokhtarzadeh <Hamid@OrganicNavigation.com>
 *
 * CHANGE LOG
 * 2016-01-06: Example includes subscribing to accel, gyro and mag
 *             TODO: add publishing capability
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <px4_posix.h> // Defines `px4_pollfd_struct_t` type
//#include <nuttx/sched.h> // 2017-01-05 Hamid removed: causing build error.

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

// Subscrite to uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/umn_output.h>

// Custom data
#include "data_structures.h"
#include "umn_algorithm.h"

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * Parameters (Hamid TODO: Move to separate params file)
 */
#define RC_CHANNEL_SWITCH 5 // zero-based numbering

/**
 * daemon management function.
 */
extern "C" __EXPORT int umn_app_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int umn_sensors_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int umn_app_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("UMN Sensors: app already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("daemon",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 umn_sensors_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int umn_sensors_thread_main(int argc, char *argv[])
{

	PX4_INFO("Starting UMN Sensors App. (use `stop` to end)");

	thread_running = true;


    /* Declare structures to be used for shuttling data */
    struct imu imu_s;
    struct airdata airdata_s;
    struct gps gps_s;
    struct sensordata sensordata_s;
    sensordata_s.imuData_ptr = &imu_s;
    sensordata_s.gpsData_ptr = &gps_s;
    sensordata_s.adData_ptr = &airdata_s;
    struct nav nav_s;
    struct control control_s;

    /* subscribe to topics */
    // Sensors Combined
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    orb_set_interval(sensor_sub_fd, 100);

    // RC Channels
    int rc_channels_fd = orb_subscribe(ORB_ID(rc_channels));
    orb_set_interval(rc_channels_fd, 100);

    // UMN Attitude
    /* publish topics */
    orb_advert_t    _uout_pub = NULL;


    /* one could wait for multiple topics with this technique */
    px4_pollfd_struct_t fds[2] = {};
    fds[0].fd = sensor_sub_fd;
    fds[0].events = POLLIN;
    fds[1].fd = rc_channels_fd;
    fds[1].events = POLLIN;

	while (!thread_should_exit) {

        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
		if (poll_ret < 0) {
			// Poll error, sleep and try again
			PX4_WARN("UMN POLL ERROR");
            usleep(10000);
			continue;

		} else if (poll_ret == 0) {
			// Poll timeout, do nothing
			PX4_WARN("UMN POLL TIMEOUT");
			continue;
		}


        /* When using actual hardware, check RC Switch to decide on execution. */
        #if defined __PX4_NUTTX
        struct rc_channels_s rc_struct;
        // Only proceed further if scaled channel output > 0
        if (!orb_copy(ORB_ID(rc_channels), rc_channels_fd, &rc_struct)){
            //PX4_INFO("RC channel[RC_CHANNEL_SWITCH]: %8.4f", (double)rc_struct.channels[RC_CHANNEL_SWITCH]);
            if(rc_struct.channels[RC_CHANNEL_SWITCH] < 0){
                continue;
            }
        } else {
            continue;
        }
        #endif

		/* update sensor readings */
		struct sensor_combined_s sensors;
        

		if (!orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &sensors)) {

            imu_s.ax = sensors.accelerometer_m_s2[0];
            imu_s.ay = sensors.accelerometer_m_s2[1];
            imu_s.az = sensors.accelerometer_m_s2[2];

            imu_s.wx = sensors.gyro_rad[0];
            imu_s.wy = sensors.gyro_rad[1];
            imu_s.wz = sensors.gyro_rad[2];

            imu_s.hx = sensors.magnetometer_ga[0];
            imu_s.hy = sensors.magnetometer_ga[1];
            imu_s.hz = sensors.magnetometer_ga[2];

            airdata_s.h = sensors.baro_alt_meter;
            // TODO: airdata_s.ias = <airspeed.msg>
            // TODO: GPS
            // TODO: actuators



            // PX4_INFO("Gyro:\t%8.4f\t%8.4f\t%8.4f",
            //          (double)imu_s.wx,
            //          (double)imu_s.wy,
            //          (double)imu_s.wz);
            
            // PX4_INFO("Airdata :\t%8.4f",
            //          (double)airdata_s.h);         

            // PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
            //          (double)sensors.accelerometer_m_s2[0],
            //          (double)sensors.accelerometer_m_s2[1],
            //          (double)sensors.accelerometer_m_s2[2]);

            // PX4_INFO("Gyro:\t%8.4f\t%8.4f\t%8.4f",
            //          (double)sensors.gyro_rad[0],
            //          (double)sensors.gyro_rad[1],
            //          (double)sensors.gyro_rad[2]);

            // PX4_INFO("Mag:\t%8.4f\t%8.4f\t%8.4f",
            //          (double)sensors.magnetometer_ga[0],
            //          (double)sensors.magnetometer_ga[1],
            //          (double)sensors.magnetometer_ga[2]);

		}

        /* Call algorithm and update output*/
        if (update(&sensordata_s, &nav_s, &control_s)) {
            /* publish attitude */
            struct umn_output_s uout = {};
            uout.timestamp = sensors.timestamp; // TODO: Decide if this works even if sensors not updated

            uout.yaw_body = nav_s.psi;
            uout.roll_body = nav_s.phi;
            uout.pitch_body = nav_s.the;

            int uout_inst;
            orb_publish_auto(ORB_ID(umn_output), &_uout_pub, &uout, &uout_inst, ORB_PRIO_HIGH);
        }



	} // while loop

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}
