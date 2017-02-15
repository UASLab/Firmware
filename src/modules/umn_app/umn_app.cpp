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
#include <systemlib/param/param.h>

#include <drivers/drv_hrt.h>


// Include headers for uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/umn_output.h>
#include <uORB/topics/parameter_update.h>
// [Hamid] Debug topic can be added.  
//         Reference: https://dev.px4.io/advanced-debug-values.html
//         Tested successfully 2017-01-31, but commented out.
//#include <uORB/topics/debug_key_value.h>


// Custom data
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
						 5800,
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



    /* subscribe to topics */
    int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
    orb_set_interval(sensor_sub, 10); // Without this, this will run at ~250 Hz.
    int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
    int airspeed_sub = orb_subscribe(ORB_ID(airspeed));
    int vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

    /* subscribe to topics which will be published to selectively so that we can
       copy over the latest version of the message */
    int _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
    int _att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    // RC Channels
    int rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
    orb_set_interval(rc_channels_sub, 100);

    /* publish topics */
    // UMN Attitude
    orb_advert_t    _uout_pub = NULL;
    orb_advert_t    _ctrl_state_pub = nullptr;
    orb_advert_t    _att_pub = nullptr;

    // /* advertise debug value */
    // struct debug_key_value_s dbg = {};
    // //dbg.key = "velx"; // Leads to error in C++
    // dbg.key[0] = 118; // v
    // dbg.key[1] = 101; // e
    // dbg.key[2] = 108; // l
    // dbg.key[3] = 120; // x
    // dbg.key[4] = '\0';// null
    // dbg.value = 0.0f;
    // orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);

    /* one could wait for multiple topics with this technique */
    px4_pollfd_struct_t fds[2] = {};
    fds[0].fd = sensor_sub;
    fds[0].events = POLLIN;
    fds[1].fd = rc_channels_sub;
    fds[1].events = POLLIN;

    // initialize data structures outside of loop
    // because they will else not always be
    // properly populated
    sensor_combined_s sensors = {};
    vehicle_gps_position_s gps = {};
    airspeed_s airspeed = {};
    vehicle_land_detected_s vehicle_land_detected = {};
    //vehicle_status_s vehicle_status = {};
    rc_channels_s rc_struct = {};
    control_state_s ctrl_state = {};
    vehicle_attitude_s att = {};

    umn_output_s uout = {};

    // Local parameter value
    param_t param_umn_control = param_find("EKF2_UMN_CONTROL");
    bool umn_control_b = false;
    // int param_res = PX4_OK;


	while (!thread_should_exit) {

        /* wait for sensor update of file descriptors for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

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

        if (fds[1].revents & POLLIN) {
            orb_copy(ORB_ID(rc_channels), rc_channels_sub, &rc_struct);
            // TODO: handle how to utilize RC channels
            // /* When using actual hardware, check RC Switch to decide on execution. */
            // #if defined __PX4_NUTTX
            // // Only proceed further if scaled channel output > 0
            // if (!orb_copy(ORB_ID(rc_channels), rc_channels_sub, &rc_struct)){
            //     //PX4_INFO("RC channel[RC_CHANNEL_SWITCH]: %8.4f", (double)rc_struct.channels[RC_CHANNEL_SWITCH]);
            //     if(rc_struct.channels[RC_CHANNEL_SWITCH] < 0){
            //         continue;
            //     }
            // } else {
            //     continue;
            // }
            // #endif

            // fetch sensor data in next loop
            continue;

        } else if (!(fds[0].revents & POLLIN)) {
            // no new data
            continue;
        }


        bool gps_updated = false;
        bool airspeed_updated = false;
        bool vehicle_land_detected_updated = false;

        orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensors);
        
        /* update all other topics if they have new data */
        orb_check(gps_sub, &gps_updated);
        if (gps_updated) {
            orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps);
        }

        orb_check(airspeed_sub, &airspeed_updated);
        if (airspeed_updated) {
            orb_copy(ORB_ID(airspeed), airspeed_sub, &airspeed);
        }

        orb_check(vehicle_land_detected_sub, &vehicle_land_detected_updated);
        if (gps_updated) {
            orb_copy(ORB_ID(vehicle_land_detected), vehicle_land_detected_sub, &vehicle_land_detected);
        }

        // Always copy over latest `control_state` and `vehicle_attitude` message
        // regardless of if changed or not.  This is because we will be selectively
        // updating it's parameters.
        orb_copy(ORB_ID(control_state), _ctrl_state_sub, &ctrl_state);
        orb_copy(ORB_ID(vehicle_attitude), _att_sub, &att);

        hrt_abstime now = 0;
        now = hrt_absolute_time();
 

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



        /* generate control state data */
        // note: of all the messages, we will only update the quaternions
       
        /* Call algorithm and update output*/
        if (update(now, &sensors, &airspeed, &gps, &vehicle_land_detected, &uout)) {
            /* publish U of MN Output */
            int uout_inst;
            orb_publish_auto(ORB_ID(umn_output), &_uout_pub, &uout, &uout_inst, ORB_PRIO_HIGH);

            // Optional: force `EKF2_UMN_CONTROL = 1`.  It may make sense to set
            //           this using QGC or via RC.
            /* Set parameter for enabling attitude control */
            // bool temp_true=1;
            // param_res = param_set_no_autosave(param_umn_control, &temp_true);
            // if (param_res != PX4_OK) {
            //     PX4_ERR("unable to set %s", "EKF2_UMN_CONTROL");
            // }

            /* Decide whether t publish modified `veicle_attitude` and 
               `control_state` messages */
            param_get(param_umn_control, &umn_control_b);
            // PX4_INFO("EKF2_UMN_CONTROL:\t%d \n",
            //      (int)umn_control_b);
            if (umn_control_b) {
                /* publish control state */
                ctrl_state.q[0] = uout.q[0];
                ctrl_state.q[1] = uout.q[1];
                ctrl_state.q[2] = uout.q[2];
                ctrl_state.q[3] = uout.q[3];
                if (_ctrl_state_pub == nullptr) {
                    _ctrl_state_pub = orb_advertise(ORB_ID(control_state), &ctrl_state);
                } else {
                    orb_publish(ORB_ID(control_state), _ctrl_state_pub, &ctrl_state);
                }

                /* publish vehicle attitude */
                att.q[0] = uout.q[0];
                att.q[1] = uout.q[1];
                att.q[2] = uout.q[2];
                att.q[3] = uout.q[3];
                if (_att_pub == nullptr) {
                    _att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

                } else {
                    orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
                }
            }

            // /* Publish debug value */
            // dbg.value = 3.14;
            // orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
        }



	} // while loop

	warnx("[daemon] exiting.\n");

    /* Reset default parameter values */
    param_reset(param_umn_control);

	thread_running = false;

	return 0;
}
