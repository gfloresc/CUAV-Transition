/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
* @file tiltrotor.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
*
*/

#ifndef TILTROTOR_H
#define TILTROTOR_H
#include "vtol_type.h"
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
/* gera */
#include <drivers/drv_pwm_output.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
//
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
//
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/debug_array.h>
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::wrap_pi;
/* gera */

class Tiltrotor : public VtolType
{

public:

    Tiltrotor(VtolAttitudeControl *_att_controller);
    ~Tiltrotor() override = default;

    void update_vtol_state() override;
    void update_transition_state() override;
    void fill_actuator_outputs() override;
    void update_mc_state() override;
    void update_fw_state() override;
    void waiting_on_tecs() override;
    float thrust_compensation_for_tilt();

    /* gera */
    //float time_since_trans_start{0.0f}; //OJO , no estaba definido, hay que quitarlo posiblemente

    float saturation(float s_, float L_, float M_);

    float C{0.0f};

    Dcmf _R_nb_gera;
    float _phi_gera{0.0f};
    float _theta_gera{0.0f};
    float _psi_gera{0.0f};

    float airspeed_vector_gera{0.0f};
    float alpha_gera{0.0f};

    float rho{1.2041f};
    float S{0.24f};
    float C_l{4.752798721f};
    float C_d{0.6417112299f};
    float alpha_0{0.05984281113f};
    //float TAS{0.0f};
    float m{1.1f}; // el sdf tiene m = 1.5; yo le puse 1.1
    float a{1/m};
    float L{0.0f};
    float D{0.0f};
    float x1_d{0.0f};
    float Vd_int{0.0f};
    //float x2_d{0.0f}; // la movi a privada

    float delta_time{0.0f};
    float last_time{0.0f};
    float x2_d_dot{0.0f};
    float x2_d_prev{0.0f};

    float z1_d{0.0f};
    float z2_d{0.0f}; //[pues zid es constante]
    float z2_d_dot{0.0f};

    float time_graph{0.0f};

    float f1{0.0f};
    float f2{0.0f};
    float T_gera{0.0f};
    float _tilt_control_gera{0.0f}; //

    float sigma2_x{0.0f};   float sigma1_x{0.0f};
    float sigma2_z{0.0f};   float sigma1_z{0.0f};

    float M2_x{80.5f};       float M1_x{41.5f}; // 40.0f     10.3f  todas estas son con la condicion _tilt_control >= _params_tiltrotor.tilt_fw
    float L2_x{80.0f};       float L1_x{41.0f}; // 30.0f     10.0f

    float M2_z{92.0f};       float M1_z{32.5f}; // 80.0f    30.35f
    float L2_z{90.5f};       float L1_z{30.0f};  // 70.8f    30.3f

    float k1{16.0f};     //30.2f de x todas estas son con la condicion  _tilt_control >= _params_tiltrotor.tilt_fw
    float k2{12.0f};     //24.2f Hace una diferencia sustancial tener 20 a 24; la buena es 24
    float kap1{21.5f};   //48.0  100.3f  de z           6.3 ** 100.3
    float kap2{15.5f};   //25.0   30.0f                 5.0   ** 16





    float M2b_x{60.5f};       float M1b_x{25.5f}; //
    float L2b_x{58.0f};       float L1b_x{24.0f}; //

    float M2b_z{100.0f};       float M1b_z{50.5f}; //
    float L2b_z{90.5f};       float L1b_z{48.0f};  //

    float k1_b{30.2f};      //
    float k2_b{5.2f};     //
    float kap1_b{23.3f};    //23.3      20.3f
    float kap2_b{6.0f};   //16.0       14.0f



    //Estas ganancias son de prueba simulando un simple PD para un facil ajuste de ganancias.
    float kap_p{70.0f};
    float kap_d{16.0f}; // cuando este se le sube, por ejemplo con 35 no controla la altura




    float _THROTTLE_gera{0.0f};

    struct vehicle_attitude_s 			*get_att() {return &_v_att_gera;}
    struct vehicle_local_position_s 		*get_local_pos() {return &_local_pos_gera;}

    /* gera */

private:

    struct {
        float tilt_mc;				/**< actuator value corresponding to mc tilt */
        float tilt_transition;			/**< actuator value corresponding to transition tilt (e.g 45 degrees) */
        float tilt_fw;				/**< actuator value corresponding to fw tilt */
        float tilt_spinup;			/**< actuator value corresponding to spinup tilt */
        float front_trans_dur_p2;
        /* Gera */
        float airspeed_setpoint;
        float airspeed_d;
        float altura_d;
        float fw_pitch_sp_offset;
        /* Gera */
    } _params_tiltrotor;

    struct {
        param_t tilt_mc;
        param_t tilt_transition;
        param_t tilt_fw;
        param_t tilt_spinup;
        param_t front_trans_dur_p2;
        /* Gera */
        param_t airspeed_setpoint;
        param_t airspeed_d;
        param_t altura_d;
        param_t fw_pitch_sp_offset;
        /* Gera */
    } _params_handles_tiltrotor;

    enum class vtol_mode {
        MC_MODE = 0,			/**< vtol is in multicopter mode */
        TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
        TRANSITION_FRONT_P2,	/**< vtol is in front transition part 2 mode */
        TRANSITION_BACK,		/**< vtol is in back transition mode */
        FW_MODE					/**< vtol is in fixed wing mode */
    };

    /**
     * Specific to tiltrotor with vertical aligned rear engine/s.
     * These engines need to be shut down in fw mode. During the back-transition
     * they need to idle otherwise they need too much time to spin up for mc mode.
     */


    struct {
        vtol_mode flight_mode;			/**< vtol flight mode, defined by enum vtol_mode */
        hrt_abstime transition_start;	/**< absoulte time at which front transition started */
    } _vtol_schedule;

    float _tilt_control{0.0f};		/**< actuator value for the tilt servo */
    /* Gera */
    hrt_abstime _last_run_gera{0};
    float x2_d{0.0f}; // estaba en publica
    /* Gera */

    void parameters_update() override;
    hrt_abstime _last_timestamp_disarmed{0}; /**< used for calculating time since arming */
    bool _tilt_motors_for_startup{false};

    /* Gera */
    uORB::Subscription _v_att_sub{ORB_ID(vehicle_attitude)};		//vehicle attitude subscription
    uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};			// sensor subscription

    vehicle_attitude_s              _v_att_gera{};				//vehicle attitude
    vehicle_local_position_s		_local_pos_gera{};
    /* Gera */

};
#endif
