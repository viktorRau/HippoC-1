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
 * OF USE, DATA, OR PROFITS; OR BUsinfESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARIsinfG IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file circle_main.cpp
 *
 * HippoCampus Drive a circle with Geometric Control
 *
 *
 * @author Eugen Solowjow
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
#include <uORB/topics/control_state.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <geo/geo.h>

#define PI 3.14159265f

/**
 * Water depth control app start / stop handling function
 *
 * @ingroup apps
 */

extern "C" __EXPORT int circle_main(int argc, char *argv[]);

class CircleControl {
public:
    /**
      * Constructor
      */
     CircleControl();

     /**
      * Destructor, also kills the main task
      */
     ~CircleControl();

     int start();


private:

     bool       _task_should_exit;      /**< if true, task_main() should exit */
     int        _control_task;          /**< task handle */
     float      _thrust_sp;             /**< thrust setpoint */
     int        _v_att_sub;             /**< vehicle attitude subscription */
     int        _params_sub;            /**< parameter updates subscription */
     int counter;


     float      _det;
     float      _invdet;
     float time_saved;
     float time_R_sp;

     float p;
     float p_neg;
     float t;
     float t_neg;
     float thrust;


     orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */

     struct control_state_s             _ctrl_state;    /**< control state */
     struct vehicle_rates_setpoint_s    _v_rates_sp;    /**< vehicle rates setpoint */
     struct vehicle_attitude_setpoint_s _v_att_sp;      /**< vehicle attitude setpoint */
     struct actuator_controls_s			_actuators;			/**< actuator controls */
     struct vehicle_attitude_s           _v_att;             /**< vehicle attitude */

     perf_counter_t     _loop_perf;     /**< loop performance counter */
     perf_counter_t     _controller_latency_perf;


     math::Vector<3>    _att_control;   /**< attitude control vector */
     math::Vector<3>      torques;

     math::Matrix<3, 3> _R_sp;              /**< rotation matrix setpoint */
     math::Matrix<3, 3> _Rdot_sp;              /**< rotation matrix setpoint */
     math::Matrix<3, 3> _I;                 /**< intertia tensor */
     math::Matrix<3, 3> _att_p_gain;        /**< Matrix Controller Parameters Stabilization */
     math::Matrix<3, 3> _att_d_gain;

     struct {
         param_t circle_amp;
         param_t period_R_sp;
         param_t surge_damping;

         param_t att_p_gain_xx;
         param_t att_p_gain_yx;
         param_t att_p_gain_zx;
         param_t att_p_gain_xy;
         param_t att_p_gain_yy;
         param_t att_p_gain_zy;
         param_t att_p_gain_xz;
         param_t att_p_gain_yz;
         param_t att_p_gain_zz;

         param_t att_d_gain_xx;
         param_t att_d_gain_yx;
         param_t att_d_gain_zx;
         param_t att_d_gain_xy;
         param_t att_d_gain_yy;
         param_t att_d_gain_zy;
         param_t att_d_gain_xz;
         param_t att_d_gain_yz;
         param_t att_d_gain_zz;

     }		_params_handles;		/**< handles for interesting parameters */

     struct {

         float circle_amp;
         float period_R_sp;
         float surge_damping;

         float att_p_gain_xx;
         float att_p_gain_yx;
         float att_p_gain_zx;
         float att_p_gain_xy;
         float att_p_gain_yy;
         float att_p_gain_zy;
         float att_p_gain_xz;
         float att_p_gain_yz;
         float att_p_gain_zz;

         float att_d_gain_xx;
         float att_d_gain_yx;
         float att_d_gain_zx;
         float att_d_gain_xy;
         float att_d_gain_yy;
         float att_d_gain_zy;
         float att_d_gain_xz;
         float att_d_gain_yz;
         float att_d_gain_zz;

     }		_params;


     int   parameters_update();
     void   parameter_update_poll();




     static void task_main_trampoline(int argc, char *argv[]);

     void control_attitude();


     /**
      * Check for control state updates.
      */

     void task_main();




};

namespace circle
{
    CircleControl	*g_control;
}

//define Constructor
CircleControl::CircleControl() :

    _task_should_exit(false),/**
         * Check for control state updates.
         */
    _control_task(-1),

    //subscriptions
    _v_att_sub(-1),
    _params_sub(-1),

    // publications

    _actuators_0_pub(nullptr),

    _ctrl_state{},

    /* performance counters */
    _loop_perf(perf_alloc(PC_ELAPSED, "circle")),
    _controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

{
    memset(&_v_att_sp, 0, sizeof(_v_att_sp));
    memset(&_actuators, 0, sizeof(_actuators));
    memset(&_v_att, 0, sizeof(_v_att));


    /* inertia tensor */
    _I(0, 0) = 0.1;     /**< _I_xx */
    _I(1, 0) = 0;       /**< _I_yx */
    _I(2, 0) = 0;       /**< _I_zx */
    _I(0, 1) = 0;       /**< _I_xy */
    _I(1, 1) = 0.4;     /**< _I_yy */
    _I(2, 1) = 0;       /**< _I_zy */
    _I(0, 2) = 0;       /**< _I_xz */
    _I(1, 2) = 0;       /**< _I_yz */
    _I(2, 2) = 0.4;     /**< _I_zz */

    time_saved  = 0;
    time_R_sp = 0.0;


    p = 0.2;
    p_neg = -0.2;
    t = 0.1;
    t_neg = -0.1;
    counter = 1;

    _params_handles.circle_amp = param_find("CIRCLE_AMP");
    _params_handles.period_R_sp = param_find("PERIOD_R_SP");
    _params_handles.surge_damping = param_find("SURGE_DAMPING");

    _params_handles.att_p_gain_xx = param_find("ATT_P_GAIN_XX");
    _params_handles.att_p_gain_yx = param_find("ATT_P_GAIN_YX");
    _params_handles.att_p_gain_zx = param_find("ATT_P_GAIN_ZX");
    _params_handles.att_p_gain_xy = param_find("ATT_P_GAIN_XY");
    _params_handles.att_p_gain_yy = param_find("ATT_P_GAIN_YY");
    _params_handles.att_p_gain_zy = param_find("ATT_P_GAIN_ZY");
    _params_handles.att_p_gain_xz = param_find("ATT_P_GAIN_XZ");
    _params_handles.att_p_gain_yz = param_find("ATT_P_GAIN_YZ");
    _params_handles.att_p_gain_zz = param_find("ATT_P_GAIN_ZZ");

    _params_handles.att_d_gain_xx = param_find("ATT_D_GAIN_XX");
    _params_handles.att_d_gain_yx = param_find("ATT_D_GAIN_YX");
    _params_handles.att_d_gain_zx = param_find("ATT_D_GAIN_ZX");
    _params_handles.att_d_gain_xy = param_find("ATT_D_GAIN_XY");
    _params_handles.att_d_gain_yy = param_find("ATT_D_GAIN_YY");
    _params_handles.att_d_gain_zy = param_find("ATT_D_GAIN_ZY");
    _params_handles.att_d_gain_xz = param_find("ATT_D_GAIN_XZ");
    _params_handles.att_d_gain_yz = param_find("ATT_D_GAIN_YZ");
    _params_handles.att_d_gain_zz = param_find("ATT_D_GAIN_ZZ");

    /* fetch initial parameter values */
    parameters_update();

}

//define Destructor
CircleControl::~CircleControl()
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

    circle::g_control = nullptr;
}

int CircleControl::parameters_update()
{
    param_get(_params_handles.circle_amp, &(_params.circle_amp));
    param_get(_params_handles.period_R_sp, &(_params.period_R_sp));
    param_get(_params_handles.surge_damping, &(_params.surge_damping));

    param_get(_params_handles.att_p_gain_xx, &(_params.att_p_gain_xx));
    param_get(_params_handles.att_p_gain_yx, &(_params.att_p_gain_yx));
    param_get(_params_handles.att_p_gain_zx, &(_params.att_p_gain_zx));
    param_get(_params_handles.att_p_gain_xy, &(_params.att_p_gain_xy));
    param_get(_params_handles.att_p_gain_yy, &(_params.att_p_gain_yy));
    param_get(_params_handles.att_p_gain_zy, &(_params.att_p_gain_zy));
    param_get(_params_handles.att_p_gain_xz, &(_params.att_p_gain_xz));
    param_get(_params_handles.att_p_gain_yz, &(_params.att_p_gain_yz));
    param_get(_params_handles.att_p_gain_zz, &(_params.att_p_gain_zz));

    param_get(_params_handles.att_d_gain_xx, &(_params.att_d_gain_xx));
    param_get(_params_handles.att_d_gain_yx, &(_params.att_d_gain_yx));
    param_get(_params_handles.att_d_gain_zx, &(_params.att_d_gain_zx));
    param_get(_params_handles.att_d_gain_xy, &(_params.att_d_gain_xy));
    param_get(_params_handles.att_d_gain_yy, &(_params.att_d_gain_yy));
    param_get(_params_handles.att_d_gain_zy, &(_params.att_d_gain_zy));
    param_get(_params_handles.att_d_gain_xz, &(_params.att_d_gain_xz));
    param_get(_params_handles.att_d_gain_yz, &(_params.att_d_gain_yz));
    param_get(_params_handles.att_d_gain_zz, &(_params.att_d_gain_zz));

    return OK;
}

void CircleControl::parameter_update_poll()
{
    bool updated;

    /* Check if parameters have changed */
    orb_check(_params_sub, &updated);

    if (updated) {
        /* read from param to clear updated flag (uORB API requirement) */
        struct parameter_update_s param_update;
        orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);

        parameters_update();
    }
}


//define start function
int CircleControl::start()
{
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("circle",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       1500,
                       (px4_main_t)&CircleControl::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

void CircleControl::task_main_trampoline(int argc, char *argv[])
{
    circle::g_control->task_main();
}



void CircleControl::control_attitude()
{

    /* geometric control start */
             time_R_sp = hrt_absolute_time() / 1000000.0f;  //get current time

             math::Vector<3> _r_des;
             _r_des(0) = _params.circle_amp * cosf(2.0f * PI * time_R_sp / _params.period_R_sp);
             _r_des(1) = _params.circle_amp * sinf(2.0f * PI * time_R_sp / _params.period_R_sp);
             _r_des(2) = 0.0f;

             // get rotation setpoint of CIRCLE!
             _R_sp(0, 0) = _params.circle_amp * cosf(2.0f * PI * time_R_sp / _params.period_R_sp);
             _R_sp(1, 0) = _params.circle_amp * sinf(2.0f * PI * time_R_sp / _params.period_R_sp);
             _R_sp(2, 0) = 0.0f;
             _R_sp(0, 1) = (-1.0f)*_params.circle_amp * sinf(2.0f * PI * time_R_sp / _params.period_R_sp);
             _R_sp(1, 1) = _params.circle_amp * cosf(2.0f * PI * time_R_sp / _params.period_R_sp);
             _R_sp(0, 2) = 0.0f;
             _R_sp(1, 2) = 0.0f;
             _R_sp(2, 2) = 1.0f;

             // get rotation derivative setpoint of CIRCLE!
             _Rdot_sp(0, 0) = (-1.0f)*_params.circle_amp * sinf(2.0f * PI * time_R_sp / _params.period_R_sp) * (2.0f  * PI * time_R_sp / _params.period_R_sp);
             _Rdot_sp(1, 0) = _params.circle_amp * cosf(2.0f * PI * time_R_sp / _params.period_R_sp)* (2.0f  * PI * time_R_sp / _params.period_R_sp);
             _Rdot_sp(2, 0) = 0.0f;
             _Rdot_sp(0, 1) = (-1.0f)*_params.circle_amp * cosf(2.0f * PI * time_R_sp / _params.period_R_sp)* (2.0f  * PI * time_R_sp / _params.period_R_sp);
             _Rdot_sp(1, 1) = (-1.0f)*_params.circle_amp * sinf(2.0f * PI * time_R_sp / _params.period_R_sp)* (2.0f  * PI * time_R_sp / _params.period_R_sp);
             _Rdot_sp(0, 2) = 0.0f;
             _Rdot_sp(1, 2) = 0.0f;
             _Rdot_sp(2, 2) = 1.0f;

             // get rotation rate setpoint
             math::Matrix <3, 3> omega_sp_mat = _Rdot_sp * _R_sp.transposed() ;
             // vee-map
             math::Vector<3> omega_sp(omega_sp_mat(2,1), omega_sp_mat(0,2), omega_sp_mat(1,0));

            // get current rotation matrix from control state quaternions
            math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
            math::Matrix<3, 3> R = q_att.to_dcm();

            // get current rates
            math::Vector <3> omega;
            omega(0) = _v_att.yawspeed;
            omega(1) = _v_att.pitchspeed;
            omega(2) = _v_att.rollspeed;

    //Output current rotation matrix
/*            PX4_INFO("R_x:\t%8.4f\t%8.4f\t%8.4f",
                                 (double)R(0, 0),
                                 (double)R(1, 0),
                                 (double)R(2, 0));
            PX4_INFO("R_y:\t%8.4f\t%8.4f\t%8.4f",
                                 (double)R(0, 1),
                                 (double)R(1, 1),
                                 (double)R(2, 1));
            PX4_INFO("R_z:\t%8.4f\t%8.4f\t%8.4f",
                                 (double)R(0, 2),
                                 (double)R(1, 2),
                                 (double)R(2, 2));
*/
            // Compute attitude error
            math::Matrix<3, 3> e_R =  (_R_sp.transposed() * R - R.transposed() * _R_sp) * 0.5;

            // vee-map the error to get a vector instead of matrix e_R
            math::Vector<3> e_R_vec(e_R(2,1), e_R(0,2), e_R(1,0));

            math::Vector<3> e_omega = omega - omega_sp;

          //Output attitude error
     /*       PX4_INFO("e_R:\t%8.4f\t%8.4f\t%8.4f",
                                 (double)e_R(2, 1),
                                 (double)e_R(0, 2),
                                 (double)e_R(1, 0));
    */


/*
            math::Vector <3> omega;
            omega(0) = _v_att.rollspeed;
            omega(1) = _v_att.pitchspeed;
            omega(2) = _v_att.yawspeed;
            math::Vector <3> omega2;
            omega2(0) = _I(0, 0) * omega(0);
            omega2(1) = _I(1, 1) * omega(1);
            omega2(2) = _I(2, 2) * omega(2);
            math::Vector<3> cross;
            cross(0) = omega(1) * omega2(2) - omega(2) * omega2(1);
            cross(1) = omega(2) * omega2(0) - omega(0) * omega2(2);
            cross(2) = omega(0) * omega2(1) - omega(1) * omega2(0);
*/
            //torques = - _att_p_gain * e_R_vec  + cross;

                    /* Matrix Controller Parameters Stabilization */
                    _att_p_gain(0, 0) = _params.att_p_gain_xx;       /**< _att_p_gain_xx */
                    _att_p_gain(1, 0) = _params.att_p_gain_yx;       /**< _att_p_gain_yx */
                    _att_p_gain(2, 0) = _params.att_p_gain_zx;       /**< _att_p_gain_zx */
                    _att_p_gain(0, 1) = _params.att_p_gain_xy;       /**< _att_p_gain_xy */
                    _att_p_gain(1, 1) = _params.att_p_gain_yy;       /**< _att_p_gain_yy */
                    _att_p_gain(2, 1) = _params.att_p_gain_zy;       /**< _att_p_gain_zy */
                    _att_p_gain(0, 2) = _params.att_p_gain_xz;       /**< _att_p_gain_xz */
                    _att_p_gain(1, 2) = _params.att_p_gain_yz;       /**< _att_p_gain_yz */
                    _att_p_gain(2, 2) = _params.att_p_gain_zz;       /**< _att_p_gain_zz */

                    /* Matrix Controller Parameters Stabilization */
                    _att_d_gain(0, 0) = _params.att_d_gain_xx;       /**< _att_p_gain_xx */
                    _att_d_gain(1, 0) = _params.att_d_gain_yx;       /**< _att_p_gain_yx */
                    _att_d_gain(2, 0) = _params.att_d_gain_zx;       /**< _att_p_gain_zx */
                    _att_d_gain(0, 1) = _params.att_d_gain_xy;       /**< _att_p_gain_xy */
                    _att_d_gain(1, 1) = _params.att_d_gain_yy;       /**< _att_p_gain_yy */
                    _att_d_gain(2, 1) = _params.att_d_gain_zy;       /**< _att_p_gain_zy */
                    _att_d_gain(0, 2) = _params.att_d_gain_xz;       /**< _att_p_gain_xz */
                    _att_d_gain(1, 2) = _params.att_d_gain_yz;       /**< _att_p_gain_yz */
                    _att_d_gain(2, 2) = _params.att_d_gain_zz;       /**< _att_p_gain_zz */


            //p-control
            //torques(0) = yaw
            //torques(1) = pitch
            //torques(2) = roll
            math::Vector<3> R_prod_ex(_R_sp(0,0), _R_sp(0,1), _R_sp(0,2)); // R*e_x
            thrust = _params.surge_damping * (_r_des(0) * R_prod_ex(0) + _r_des(1) * R_prod_ex(1) + _r_des(2) * R_prod_ex(2));
            torques = - _att_p_gain * e_R_vec - _att_d_gain * e_omega ;
  /*
            PX4_INFO("torques p-control:\t%8.4f\t%8.4f\t%8.4f",
                        (double)torques(0),
                        (double)torques(1),
                        (double)torques(0));
   */

   /*
            PX4_INFO("torques d-control:\t%8.4f\t%8.4f\t%8.4f",
                        (double)torques(0),
                        (double)torques(1),
                        (double)torques(0));
*/

/*
        if (hrt_absolute_time() - time_saved  > 500000){
           PX4_INFO("e_R and torques:\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
                                (double)e_R(2, 1),
                                (double)e_R(0, 2),
                                (double)e_R(1, 0),
                                (double)torques(2),
                                (double)torques(1),
                                (double)torques(0));
           time_saved = hrt_absolute_time();
        }
    */


        /* geometric control end */

    if (hrt_absolute_time() - time_saved  > 500000){
      //  PX4_INFO("Absolute time:\t%8.4f",
      //           (double)hrt_absolute_time());

        PX4_INFO("thrust:\t%8.4f",
                 (double)thrust);

        PX4_INFO("Yaw, Pitch, Roll:\t%8.4f\t%8.4f\t%8.4f",
                 (double)torques(0),
                 (double)torques(1),
                 (double)torques(2));

        time_saved = hrt_absolute_time();
    }

       _att_control(0) = torques(2);    //roll
       _att_control(1) = torques(1);    //pitch
       _att_control(2) = torques(0);    //yaw

       _thrust_sp = thrust;

}

//main task
void CircleControl::task_main()
{
    //do subscriptions
    _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    _params_sub = orb_subscribe(ORB_ID(parameter_update));


    /* initialize parameters cache */
    parameters_update();

    /* advertise actuator controls */
    _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);

    px4_pollfd_struct_t fds[1];

    fds[0].fd = _v_att_sub;
    fds[0].events = POLLIN;


    while (!_task_should_exit) {

        /* wait for up to 100ms for data */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

        /* timed out - periodic check for _task_should_exit */
        if (pret == 0) {
            continue;
        }

        /* this is undesirable but not much we can do - might want to flag unhappy status */
        if (pret < 0) {
            warn("mc att ctrl: poll error %d, %d", pret, errno);
            /* sleep a bit before next try */
            usleep(100000);
            continue;
        }

        perf_begin(_loop_perf);

        /* run controller on attitude changes */
        if (fds[0].revents & POLLIN) {

            /* copy attitude and control state topics */
            orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

            /* check for updates in other topics */
            parameter_update_poll();

            //start controller
            control_attitude();

            /* publish actuator controls */
            _actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
            _actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
            _actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
            _actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
            _actuators.timestamp = hrt_absolute_time();
            _actuators.timestamp_sample = _v_att.timestamp;

            orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

            perf_end(_controller_latency_perf);
       }

        perf_end(_loop_perf);
    }

    _control_task = -1;
    return;

}



int circle_main(int argc, char *argv[])
{
    if (argc < 2) {
            warnx("usage: circle {start|stop|status|mystatus}");
            return 1;
        }

        if (!strcmp(argv[1], "start")) {

            if (circle::g_control != nullptr) {
                warnx("already running");
                return 1;
            }

            circle::g_control = new CircleControl;

            if (circle::g_control == nullptr) {
                warnx("alloc failed");
                return 1;
            }

            if (OK != circle::g_control->start()) {
                delete circle::g_control;
                circle::g_control = nullptr;
                warnx("start failed");
                return 1;
            }

            return 0;
        }

        if (!strcmp(argv[1], "stop")) {
            if (circle::g_control == nullptr) {
                warnx("not running");
                return 1;
            }

            delete circle::g_control;
            circle::g_control = nullptr;
            return 0;
        }

        if (!strcmp(argv[1], "status")) {
            if (circle::g_control) {
                warnx("running");
                return 0;

            } else {
                warnx("not running");
                return 1;
            }
        }

        if (!strcmp(argv[1], "mystatus")) {

            if (circle::g_control) {
                return 0;
            } else {
                warnx("not running");
                return 1;
            }
        }

        warnx("unrecognized command");
        return 1;
}
