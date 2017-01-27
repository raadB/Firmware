/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>


extern "C" __EXPORT int ipcmonitor_main(int argc, char *argv[]);


static int ipcm_daemon_task;					/**< Handle of daemon task / thread */
static volatile bool ipcm_thread_running = false;		/**< daemon status flag */
static volatile bool ipcm_thread_should_exit = false;


/**
 * Print the correct usage.
 */
void ipcm_usage(const char *reason);

void print_ipcm_status();

int ipcmonitor_thread_main (int argc, char *argv[]);








void ipcm_usage(const char *reason)
{
    if (reason && *reason > 0) {
        PX4_INFO("%s", reason);
    }

    PX4_INFO("usage: ipcmonitor {start|stop|status}\n");
}


void print_ipcm_status() {
    warnx("to be implemented ...");
}






int ipcmonitor_main(int argc, char *argv[]) {

    if (argc < 2) {
        ipcm_usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (ipcm_thread_running) {
            warnx("already running");
            /* this is not an error */
            return 0;
        }

        ipcm_thread_should_exit = false;
        ipcm_daemon_task = px4_task_spawn_cmd("ipcmonitor",
                                              SCHED_DEFAULT,
                                              SCHED_PRIORITY_MAX,
                                              3600,
                                              ipcmonitor_thread_main,
                                              (char *const *) &argv[0]);

        unsigned constexpr max_wait_us = 1000000;
        unsigned constexpr max_wait_steps = 2000;

        unsigned i;
        for (i = 0; i < max_wait_steps; i++) {
            usleep(max_wait_us / max_wait_steps);
            if (ipcm_thread_running) {
                break;
            }
        }

        return !(i < max_wait_steps);
    }


    if (!strcmp(argv[1], "stop")) {

        if (!ipcm_thread_running) {
            warnx("ipcmonitor already stopped");
            return 0;
        }

        ipcm_thread_should_exit = true;

        while (ipcm_thread_running) {
            usleep(200000);
            warnx(".");
        }

        warnx("terminated.");

        return 0;
    }

    /* commands needing the app to run below */
    if (! ipcm_thread_running) {
        warnx("\tcommander not started");
        return 1;
    }

    if (!strcmp(argv[1], "status")) {
        print_ipcm_status();
        return 0;
    }


    return 0;

}


int ipcmonitor_thread_main (int argc, char *argv[])
{

    PX4_INFO("[mr] ipc monitoring --------------------------------");
    ipcm_thread_running = true;


    while (! ipcm_thread_should_exit) {

        usleep(20000000);
        PX4_INFO("[mr] do monitoring");
    }


    PX4_INFO("[mr] monitoring stopped");
    ipcm_thread_running = false;


    return 0;
}