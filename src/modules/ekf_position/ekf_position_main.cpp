
/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file uw_att_control_main.cpp
 *
 * HippoCampus Underwater Attitude Controller. Feed Forward RC Input except roll. Balance roll.
 *
 * Based on rover steering control example by Lorenz Meier <lorenz@px4.io>
 *
 * @author Viktor Rausch
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <geo/geo.h>
#include <uORB/topics/ekf_vector.h>



extern "C" __EXPORT int ekf_position_main(int argc, char *argv[]);

class Read_EKF_Data {
public:
    /**
     * Constructor
     */
    Read_EKF_Data();

    /**
     * Destructor, also kills the main task
     */
    ~Read_EKF_Data();

    double n=1; //counter
    /**
     * Start the underwater attitude control task.
     *
     * @return		OK on success.
     */
    int		start();

private:

    bool	_task_should_exit;		/**< if true, task_main() should exit */
    int		_control_task;			/**< task handle */


    int subscribed_ekf_vector_int;

       /**
     * Check for ekf updates from Pi0 and handle it.
     */
    void		ekf_update_poll();

    /**
     * Main attitude control task.
     */
    void		task_main();

      /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);
};

namespace ekf_position
{

 Read_EKF_Data	*g_control;
}


Read_EKF_Data::Read_EKF_Data() :

    _task_should_exit(false),
    _control_task(-1),
    subscribed_ekf_vector_int(-1)
{

}

Read_EKF_Data::~Read_EKF_Data()
{
    if (_control_task != -1) {
        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                px4_task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }


    ekf_position::g_control = nullptr;
}

void Read_EKF_Data::ekf_update_poll()
{
    bool updated;

    /* Check if parameters have changed */
    orb_check(subscribed_ekf_vector_int, &updated);

    if (updated) {
        /* read from param to clear updated flag (uORB API requirement) */
        struct ekf_vector_s subscribed_ekf_vector;
        orb_copy(ORB_ID(ekf_vector), subscribed_ekf_vector_int, &subscribed_ekf_vector);

        /* Print the updatet absolut position and ekf covariance*/
        PX4_INFO("EKF_Position:\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f",
                                       (double)subscribed_ekf_vector.EKF_pos_x,
                                       (double)subscribed_ekf_vector.EKF_pos_x,
                                       (double)subscribed_ekf_vector.EKF_covar_00,
                                       (double)subscribed_ekf_vector.EKF_covar_01,
                                       (double)subscribed_ekf_vector.EKF_covar_10,
                                       (double)subscribed_ekf_vector.EKF_covar_11);

         /* If you need to now how many times the Pixhawk updates the EKF_data*/
         /*
        PX4_INFO("Number of received positions:\t%.4f",
                                       (double)n);
                                       n=n+1;
        */
    }
}





void Read_EKF_Data::task_main()
{

subscribed_ekf_vector_int = orb_subscribe(ORB_ID(ekf_vector));

	while (!_task_should_exit) {

           usleep(20000);
           ekf_update_poll();

     }
}

void Read_EKF_Data::task_main_trampoline(int argc, char *argv[])
{
   ekf_position::g_control->task_main();
}


int Read_EKF_Data::start()
{
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("ekf_position",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       1500,
                       (px4_main_t)&Read_EKF_Data::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}


int ekf_position_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("usage: ekf_position {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (ekf_position::g_control != nullptr) {
            warnx("already running");
            return 1;
        }

        ekf_position::g_control = new Read_EKF_Data;

        if (ekf_position::g_control == nullptr) {
            warnx("alloc failed");
            return 1;
        }

        if (OK != ekf_position::g_control->start()) {
            delete ekf_position::g_control;
           ekf_position::g_control = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (ekf_position::g_control == nullptr) {
            warnx("not running");
            return 1;
        }

        delete ekf_position::g_control;
        ekf_position::g_control = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (ekf_position::g_control) {
            warnx("running");
            return 0;

        } else {
            warnx("not running");
            return 1;
        }
    }

    warnx("unrecognized command");
    return 1;
}
