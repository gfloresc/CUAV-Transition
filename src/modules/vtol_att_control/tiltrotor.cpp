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
 * @file tiltrotor.cpp
 *
 * @author Roman Bapst 		<bapstroman@gmail.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
*/

#include "tiltrotor.h"
#include "vtol_att_control_main.h"

using namespace matrix;
using namespace time_literals;

#define ARSP_YAW_CTRL_DISABLE 7.0f	// airspeed at which we stop controlling yaw during a front transition

Tiltrotor::Tiltrotor(VtolAttitudeControl *attc) :
    VtolType(attc)
{
    _vtol_schedule.flight_mode = vtol_mode::MC_MODE;
    _vtol_schedule.transition_start = 0;

    _mc_roll_weight = 1.0f;
    _mc_pitch_weight = 1.0f;
    _mc_yaw_weight = 1.0f;

    _flag_was_in_trans_mode = false;

    _params_handles_tiltrotor.tilt_mc = param_find("VT_TILT_MC");   // 0 (tiltrotor_params)
    _params_handles_tiltrotor.tilt_transition = param_find("VT_TILT_TRANS");    // 0.6 (1042 en init.d); 0.3 (tiltrotor_params)
    _params_handles_tiltrotor.tilt_fw = param_find("VT_TILT_FW");               // 1 (1042 en init.d, tiltrotor_params)
    _params_handles_tiltrotor.tilt_spinup = param_find("VT_TILT_SPINUP"); // 0 (tiltrotor_params, )
    _params_handles_tiltrotor.front_trans_dur_p2 = param_find("VT_TRANS_P2_DUR"); // 0.5 (tiltrotor_params)
    /* Gera */
    _params_handles_tiltrotor.airspeed_setpoint = param_find("FW_AIRSPD_TRIM"); // 19 m/s; yo s elo cambié en 1042_tiltrotor
    _params_handles_tiltrotor.airspeed_d = param_find("FW_AIRSPD_TRIM");
    _params_handles_tiltrotor.altura_d = param_find("MIS_TAKEOFF_ALT");
    _params_handles_tiltrotor.fw_pitch_sp_offset = param_find("FW_PSP_OFF");
    /* Gera */

}

void
Tiltrotor::parameters_update()
{
    float v;

    /* vtol tilt mechanism position in mc mode */
    param_get(_params_handles_tiltrotor.tilt_mc, &v);
    _params_tiltrotor.tilt_mc = v;

    /* vtol tilt mechanism position in transition mode */
    param_get(_params_handles_tiltrotor.tilt_transition, &v);
    _params_tiltrotor.tilt_transition = v;

    /* vtol tilt mechanism position in fw mode */
    param_get(_params_handles_tiltrotor.tilt_fw, &v);
    _params_tiltrotor.tilt_fw = v;

    /* vtol tilt mechanism position during motor spinup */
    param_get(_params_handles_tiltrotor.tilt_spinup, &v);
    _params_tiltrotor.tilt_spinup = v;

    /* vtol front transition phase 2 duration */
    param_get(_params_handles_tiltrotor.front_trans_dur_p2, &v);
    _params_tiltrotor.front_trans_dur_p2 = v;

    /* Gera */
    /* vtol airspeed setpoint in airplane mode [yo se lo puse ]*/
    param_get(_params_handles_tiltrotor.airspeed_setpoint, &v);
    _params_tiltrotor.airspeed_setpoint = v;

    param_get(_params_handles_tiltrotor.airspeed_d, &v);
    _params_tiltrotor.airspeed_d = v;

    param_get(_params_handles_tiltrotor.altura_d, &v);
    _params_tiltrotor.altura_d = v;

    param_get(_params_handles_tiltrotor.fw_pitch_sp_offset, &v); //pitch deseado [en grados] en modo avion
    _params_tiltrotor.fw_pitch_sp_offset = math::radians(v);

    z1_d = - _params_tiltrotor.altura_d; // MIS_TAKEOFF_ALT [modificar en archivo de parametros ROMFS/px4fmu_common/init.d-posix]
    /* Gera */
}

void Tiltrotor::update_vtol_state()
{
    /* simple logic using a two way switch to perform transitions.
     * after flipping the switch the vehicle will start tilting rotors, picking up
     * forward speed. After the vehicle has picked up enough speed the rotors are tilted
     * forward completely. For the backtransition the motors simply rotate back.
    */

    if (!_attc->is_fixed_wing_requested()) { // Es lo que se esta PIDIENO, y se revisa en que ESTADO esta.

        // plane is in multicopter mode
        switch (_vtol_schedule.flight_mode) {
        case vtol_mode::MC_MODE:
            break;

        case vtol_mode::FW_MODE:
            _vtol_schedule.flight_mode = vtol_mode::TRANSITION_BACK;
            _vtol_schedule.transition_start = hrt_absolute_time();
            break;

        case vtol_mode::TRANSITION_FRONT_P1:
            // failsafe into multicopter mode
            _vtol_schedule.flight_mode = vtol_mode::MC_MODE;
            break;

        case vtol_mode::TRANSITION_FRONT_P2:
            // failsafe into multicopter mode
            _vtol_schedule.flight_mode = vtol_mode::MC_MODE;
            break;

        case vtol_mode::TRANSITION_BACK:
//            float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

//            if (_tilt_control <= _params_tiltrotor.tilt_mc && time_since_trans_start > _params->back_trans_duration) {
//                _vtol_schedule.flight_mode = vtol_mode::MC_MODE;
//            }



            //float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

            //ESTA ES LA CLAVE, TENIA 2 CONDICIONES.  Con solo _tilt_control <= _params_tiltrotor.tilt_mc ya no oscila el thrust
            //if (_tilt_control <= _params_tiltrotor.tilt_mc && x2_d <= 0.01f) { // x2_d <= 0.01f
            if ( x2_d <= 0.1f ) {
                _vtol_schedule.flight_mode = vtol_mode::MC_MODE;

            }


//            //COn este funciono:
//            if (_tilt_control <= _params_tiltrotor.tilt_mc && time_since_trans_start > _params->back_trans_duration * 1.5f) { //
//                _vtol_schedule.flight_mode = vtol_mode::MC_MODE;

//            }



            break;
        }

    } else {

        switch (_vtol_schedule.flight_mode) {
        case vtol_mode::MC_MODE:
            // initialise a front transition
            _vtol_schedule.flight_mode = vtol_mode::TRANSITION_FRONT_P1;
            _vtol_schedule.transition_start = hrt_absolute_time();
            break;

        case vtol_mode::FW_MODE:
            break;

        case vtol_mode::TRANSITION_FRONT_P1: {

                float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

                const bool airspeed_triggers_transition = PX4_ISFINITE(_airspeed_validated->equivalent_airspeed_m_s)
                        && !_params->airspeed_disabled;

                bool transition_to_p2 = false;

                if (time_since_trans_start > _params->front_trans_time_min) {
                    if (airspeed_triggers_transition) {
                        transition_to_p2 = _airspeed_validated->equivalent_airspeed_m_s >= _params->transition_airspeed;
                // VT_ARSP_TRANS = 10 [se lo cambie a 5] en vtol_att_control_params

                    } else {
                        transition_to_p2 = _tilt_control >= _params_tiltrotor.tilt_transition &&
                                   time_since_trans_start > _params->front_trans_time_openloop;; //=6sec
                    }
                }

                transition_to_p2 |= can_transition_on_ground();

                // Esta linea hace que pase a P2 sin filtros, y hace que haga la transicion continua
                _vtol_schedule.flight_mode = vtol_mode::TRANSITION_FRONT_P2;


                if (transition_to_p2) {
                    _vtol_schedule.flight_mode = vtol_mode::TRANSITION_FRONT_P2;
                    _vtol_schedule.transition_start = hrt_absolute_time();
                }

                break;
            }

        case vtol_mode::TRANSITION_FRONT_P2:
            // CONDICIONES QUE PODEMOS USAR CON AND
            // _airspeed_validated->true_airspeed_m_s >= x2_d && x2_d >= _params_tiltrotor.airspeed_d
            // _tilt_control >= _params_tiltrotor.tilt_fw && x2_d >= _params_tiltrotor.airspeed_d
            // if the rotors have been tilted completely we switch to fw mode // && x2_d >= _params_tiltrotor.airspeed_d - 0.01f ; _tilt_control >= _params_tiltrotor.tilt_fw
            if ( _tilt_control >= _params_tiltrotor.tilt_fw || _local_pos_gera.vy >= _params_tiltrotor.airspeed_d || x2_d >= _params_tiltrotor.airspeed_d ) {  // ESTA ES LA QUE CAMBIE!!!, solo tenia una condicion: _tilt_control >= _params_tiltrotor.tilt_fw. Agregue x2_d >= _params_tiltrotor.airspeed_d. Luego trate con esta: _airspeed_validated->equivalent_airspeed_m_s >= _params_tiltrotor.airspeed_d
                _vtol_schedule.flight_mode = vtol_mode::FW_MODE;
                _tilt_control = _params_tiltrotor.tilt_fw;          //
            }

            break;

        case vtol_mode::TRANSITION_BACK:
            // failsafe into fixed wing mode
            _vtol_schedule.flight_mode = vtol_mode::FW_MODE;
            break;
        }
    }

    // map tiltrotor specific control phases to simple control modes
    switch (_vtol_schedule.flight_mode) {
    case vtol_mode::MC_MODE:
        _vtol_mode = mode::ROTARY_WING;
        break;

    case vtol_mode::FW_MODE:
        _vtol_mode = mode::FIXED_WING;
        break;

    case vtol_mode::TRANSITION_FRONT_P1:
    case vtol_mode::TRANSITION_FRONT_P2:
        _vtol_mode = mode::TRANSITION_TO_FW;
        break;

    case vtol_mode::TRANSITION_BACK:
        _vtol_mode = mode::TRANSITION_TO_MC;
        break;
    }







    /* gera*/
    // ESTE ES EL LUGAR DONDE DEBEN IR ESTAS LINEAS CUANDO ESTE CORRIENDO EL CONTROL

    _v_att_sub.update(&_v_att_gera);
    _local_pos_sub.update(&_local_pos_gera);

    Dcmf Rotation_gera;
    Vector3f vel_NED_gera = {_local_pos_gera.vx, _local_pos_gera.vy, _local_pos_gera.vz};
    Vector3f vel_body_gera = Rotation_gera * vel_NED_gera;

    // Ground speed [m/s], y es el mismo que aparece en el Qgrpundcontrol, siempre positivo
    //_airspeed_validated.indicated_airspeed_m_s :  equivalent_airspeed_m_s y indicated_airspeed_m_s salen discretas, y son parecidas a Ground speed
    //airspeed_vector_gera = sqrt(vel_body_gera(0) * vel_body_gera(0) + vel_body_gera(1) * vel_body_gera(1) + vel_body_gera(2) * vel_body_gera(2) ); // True airspeed segun Stevens & Lewis. No tenia este termino: + vel_body_gera(2) * vel_body_gera(2)
    //alpha_gera = atan2f(vel_body_gera(2), airspeed_vector_gera) ;               //Angle of attack AoA . Se hizo esta modificacion para que sunfionara  * (float)57.2956  alpha_gera = atan2f(vel_body_gera(2), _airspeed_validated->equivalent_airspeed_m_s) ;
    //alpha_gera = atan2f(vel_body_gera(2), _airspeed_validated->true_airspeed_m_s);
    alpha_gera = atan2f( vel_body_gera(2), _airspeed_validated->true_ground_minus_wind_m_s );

    _R_nb_gera = Quatf(_v_att_gera.q);
    const Eulerf euler_angles(_R_nb_gera);
    _phi_gera   = euler_angles(0);
    _theta_gera = euler_angles(1);
    _psi_gera   = euler_angles(2);

    L = 0.5f * rho * _airspeed_validated->true_ground_minus_wind_m_s * _airspeed_validated->true_ground_minus_wind_m_s * S * C_l * (alpha_gera + alpha_0); // cambia por 2 unidades mas o menos cuando usamos TAS = _airspeed_validated.true_airspeed_m_s. Antes _airspeed_validated->equivalent_airspeed_m_s
    D = 0.5f * rho * _airspeed_validated->true_ground_minus_wind_m_s * _airspeed_validated->true_ground_minus_wind_m_s * S * C_d * (alpha_gera + alpha_0); // tenia alpha_gera



    const hrt_abstime now_gera = hrt_absolute_time();

    // Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
    const float dt_gera = math::constrain(((now_gera - _last_run_gera) / 1e6f), 0.0002f, 0.02f);
    _last_run_gera = now_gera;




    //time_graph = (float)(hrt_absolute_time() - 0.0f) * 1e-6f; //tenia float
    //time_graph = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;
    //delta_time =  float(hrt_absolute_time() - last_time) * 1e-6f;
    //last_time = hrt_absolute_time();

    //x_2 deseada
    //x2_d = 0.5f * _params_tiltrotor.airspeed_d + 0.5f * _params_tiltrotor.airspeed_d * tanh( 0.65f * (time_graph - 20.0f) );
    //if (x2_d >= _params_tiltrotor.airspeed_d){ x2_d = _params_tiltrotor.airspeed_d;}

    //derivada de x_2 deseada
    //x2_d_dot = (x2_d - x2_d_prev) / delta_time;
    //x2_d_prev = x2_d;

    //integral de x_2 deseada
    x1_d += x2_d * dt_gera; //tenia time_graph

//    // Estas deberrian NO IR aqui, sino donde esta la transicion
//    f1 = D * cos( _theta_gera - alpha_gera ) + (m) * x2_d_dot - (m) * ( k2*(_local_pos_gera.vx - x2_d) + k1*( ( _local_pos_gera.x - x1_d ) + (_local_pos_gera.vx - x2_d) ) ) ;
//    f2 = m * CONSTANTS_ONE_G -  L * cos( _theta_gera - alpha_gera ) + (m) * z2_d_dot - (m) * ( kap2 * ( _local_pos_gera.vz - z2_d ) + kap1 * ( (_local_pos_gera.z - z1_d) + ( _local_pos_gera.vz - z2_d ) ) );
//    T_gera = sqrt( (f1*f1) + (f2*f2) );
//    _tilt_control_gera = asinf( f1/T_gera ) - _theta_gera;


    /* gera */

    /* gera PUBLICACION */
    struct debug_value_s dbg_ind;
    struct debug_vect_s dbg_vect;
    strncpy(dbg_vect.name, "T", 10);

    dbg_ind.ind = 42;
    dbg_ind.value = _tilt_control; // _params_tiltrotor.altura_d
    dbg_ind.timestamp = hrt_absolute_time();


    //yo estaba graficando _airspeed_validated->equivalent_airspeed_m_s - x2_d ; airspeed_vector_gera - x2_d
    // esta no es ruidosa, esta suave: _airspeed_validated->true_ground_minus_wind_m_s
    dbg_vect.x = _local_pos_gera.vy - x2_d ; // tenia     _airspeed_validated->true_airspeed_m_s - x2_d ;         x1_d L _local_pos_sp.x, _local_pos_sp.z me salen nan
    dbg_vect.y = _local_pos_gera.z - z1_d ; //  x2_d  D
    dbg_vect.z = x2_d; // _params.airspeed_d        x2_d
    dbg_vect.p = _THROTTLE_gera ;
    dbg_vect.q = T_gera ;
    dbg_vect.r = sigma2_x ; //alpha_gera
    dbg_vect.u = L ; //f1
    dbg_vect.v = D ; //f2 _params_tiltrotor.fw_pitch_sp_offset
    dbg_vect.w = sigma1_x  ;
    dbg_vect.n = sigma1_z  ;
    dbg_vect.o = sigma2_z  ;//  _theta_gera
    dbg_vect.l = _local_pos_gera.y;
    dbg_vect.m = x1_d;
    dbg_vect.k = _theta_gera;
    dbg_vect.j = alpha_gera; //_local_pos_gera.vy es local position vy
    dbg_vect.i = alpha_gera + alpha_0 ;
    dbg_vect.timestamp = hrt_absolute_time(); //hrt_absolute_time() o time_graph

    orb_advert_t pub_dbg_ind = orb_advertise(ORB_ID(debug_value), &dbg_ind);
    orb_advert_t pub_dbg_vect = orb_advertise(ORB_ID(debug_vect), &dbg_vect);

    orb_publish(ORB_ID(debug_value), pub_dbg_ind, &dbg_ind);
    orb_publish(ORB_ID(debug_vect), pub_dbg_vect, &dbg_vect);
    /* gera PUBLICACION */








}

void Tiltrotor::update_mc_state()
{
    VtolType::update_mc_state(); //ESTE ES EL QUE HACE QUE FUNCIONE, SI LO QUITO NO SIRVE.

    /*Motor spin up: define the first second after arming as motor spin up time, during which
    * the tilt is set to the value of VT_TILT_SPINUP. This allowes the user to set a spin up
    * tilt angle in case the propellers don't spin up smootly in full upright (MC mode) position.
    */

    const int spin_up_duration_p1 = 1000_ms; // duration of 1st phase of spinup (at fixed tilt)
    const int spin_up_duration_p2 = 700_ms; // duration of 2nd phase of spinup (transition from spinup tilt to mc tilt)


    //Si quito lo qu sigue y dejo el siguiente bloque sigue funcionando.
    // reset this timestamp while disarmed
    if (!_v_control_mode->flag_armed) {
        _last_timestamp_disarmed = hrt_absolute_time();
        _tilt_motors_for_startup = _params_tiltrotor.tilt_spinup > 0.01f; // spinup phase only required if spinup tilt > 0

    } else if (_tilt_motors_for_startup) {
        // leave motors tilted forward after arming to allow them to spin up easier
        if (hrt_absolute_time() - _last_timestamp_disarmed > (spin_up_duration_p1 + spin_up_duration_p2)) {
            _tilt_motors_for_startup = false;
        }
    }
    //Si quito lo qu sigue y dejo el siguiente bloque sigue funcionando.

// Gera: de aqui hasta abajo si lo quito funciona igual
    if (_tilt_motors_for_startup) {
        if (hrt_absolute_time() - _last_timestamp_disarmed < spin_up_duration_p1) {
            _tilt_control = _params_tiltrotor.tilt_spinup;

        } else {
            // duration phase 2: begin to adapt tilt to multicopter tilt
            float delta_tilt = (_params_tiltrotor.tilt_mc - _params_tiltrotor.tilt_spinup);
            _tilt_control = _params_tiltrotor.tilt_spinup + delta_tilt / spin_up_duration_p2 * (hrt_absolute_time() -
                    (_last_timestamp_disarmed + spin_up_duration_p1));
        }

        _mc_yaw_weight = 0.0f; //disable yaw control during spinup

    } else {
        // normal operation
//        _tilt_control = VtolType::pusher_assist();
        _tilt_control = 0.16f; // 0.15f = 0.03rad, 0.13f = 0.09, 0.17f = +0.03rad, 0.16f =0 rad
        _mc_yaw_weight = 1.0f;
        _v_att_sp->thrust_body[2] = Tiltrotor::thrust_compensation_for_tilt();
    }
// Gera: de aqui hasta abajo si lo quito funciona igual




    /*Gera*/
    _THROTTLE_gera =  _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];
    /*Gera*/




}

void Tiltrotor::update_fw_state()
{
    VtolType::update_fw_state();

    // make sure motors are tilted forward
    _tilt_control = _params_tiltrotor.tilt_fw;






    /*Gera*/
    _THROTTLE_gera =  _actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];
    /*Gera*/





}

void Tiltrotor::update_transition_state()
{
    VtolType::update_transition_state();

    // copy virtual attitude setpoint to real attitude setpoint (we use multicopter att sp)
    memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

    _v_att_sp->roll_body = _fw_virtual_att_sp->roll_body;

    float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;
    /* Gera */
    delta_time =  time_since_trans_start - last_time;
    last_time = time_since_trans_start;
    /* Gera */

    if (!_flag_was_in_trans_mode) {
        // save desired heading for transition and last thrust value
        _flag_was_in_trans_mode = true;
    }

    if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_FRONT_P1) {
        // for the first part of the transition all rotors are enabled
        if (_motor_state != motor_state::ENABLED) {
            _motor_state = set_motor_state(_motor_state, motor_state::ENABLED);
        }








//        /* gera: Valores de prueba */
//        _local_pos_sub.update(&_local_pos_gera);

//        Dcmf Rotation_gera;
//        Vector3f vel_NED_gera = {_local_pos_gera.vx, _local_pos_gera.vy, _local_pos_gera.vz};
//        Vector3f vel_body_gera = Rotation_gera * vel_NED_gera;

//        // Ground speed [m/s], y es el mismo que aparece en el Qgrpundcontrol, siempre positivo
//        //_airspeed_validated.indicated_airspeed_m_s :  equivalent_airspeed_m_s y indicated_airspeed_m_s salen discretas, y son parecidas a Ground speed
//        airspeed_vector_gera = sqrt(vel_body_gera(0) * vel_body_gera(0) + vel_body_gera(1) * vel_body_gera(1) + vel_body_gera(2) * vel_body_gera(2) ); // True airspeed segun Stevens & Lewis. No tenia este termino: + vel_body_gera(2) * vel_body_gera(2)
//        alpha_gera = atan2f(vel_body_gera(2), airspeed_vector_gera) ;               //Angle of attack AoA . Se hizo esta modificacion para que sunfionara  * (float)57.2956
//        //float alpha_ =  atan2f(vel_body_gera(2), abs(vel_body_gera(0))) ; // Asi se parece a alpha_gera y en avion son iguales
//        //float alpha_ = atan2f(vel_body_gera(2), vel_body_gera(0)); // tal como lo define Stevens & Lewis

//        _R_nb_gera = Quatf(_v_att.q);
//        const Eulerf euler_angles(_R_nb_gera);

//        _phi_gera   = euler_angles(0);
//        _theta_gera = euler_angles(1);
//        _psi_gera   = euler_angles(2);

//        L = 0.5f * rho * airspeed_vector_gera*airspeed_vector_gera * S * C_l * (alpha_gera + alpha_0); // cambia por 2 unidades mas o menos cuando usamos TAS = _airspeed_validated.true_airspeed_m_s;
//        D = 0.5f * rho * airspeed_vector_gera*airspeed_vector_gera * S * C_d * alpha_gera;

//        float time_graph = (float)(hrt_absolute_time() - 0.0f) * 1e-6f;
//        delta_time =  float(hrt_absolute_time() - last_time) * 1e-6f;
//        last_time = hrt_absolute_time();

//        //x_2 deseada
//        x2_d = 0.5f * _params.airspeed_d + 0.5f * _params.airspeed_d * tanh( 0.65f * (time_graph - 20.0f) );
//        //if (x2_d >= _params.airspeed_d){ x2_d = _params.airspeed_d;}

//        //derivada de x_2 deseada
//        x2_d_dot = (x2_d - x2_d_prev) / delta_time;
//        x2_d_prev = x2_d;

//        //integral de x_2 deseada
//        x1_d = x2_d*time_graph;

        /* gera: Valores de prueba */


        //x2_d = 0.5f * _params_tiltrotor.airspeed_d + 0.5f * _params_tiltrotor.airspeed_d * tanh( 0.65f * (time_since_trans_start - 0.0f) );


        // tilt rotors forward up to certain angle
        if (_tilt_control <= _params_tiltrotor.tilt_transition) {




//            _tilt_control = _params_tiltrotor.tilt_mc +
//                    fabsf(_params_tiltrotor.tilt_transition*0.5f - _params_tiltrotor.tilt_mc) * time_since_trans_start /
//                    _params->front_trans_duration; // ORIGINAL    front_trans_duration = 5; tilt_transition = 0.6 o 0.3

            /*Gera*/
            x2_d = 0.5f * _params_tiltrotor.airspeed_d + 0.5f * _params_tiltrotor.airspeed_d * tanh( 0.85f * (time_since_trans_start - 0.85f*10.0f) );
            //x2_d = 0.5f * _params_tiltrotor.airspeed_d + 0.5f * _params_tiltrotor.airspeed_d * tanh( 0.65f * (time_since_trans_start - 0.65f*10.0f) );
                    //time_since_trans_start; // empieza a contar desde 0 en el momento donde se da click en transicion

            x2_d_dot = ( x2_d - x2_d_prev ) / delta_time;
            x2_d_prev = x2_d;

            if (x2_d >= _params_tiltrotor.airspeed_d - 0.0f){ x2_d = _params_tiltrotor.airspeed_d;  x2_d_dot = 0.0f; }



            sigma1_x = saturation(   k1*( ( _local_pos_gera.y - x1_d ) + ( _local_pos_gera.vy - x2_d) )  , L1_x, M1_x  ); // tenia _local_pos_gera.x - x1_d, y _local_pos_gera.vx - x2_d
            sigma2_x = saturation(   k2*( _local_pos_gera.vy - x2_d) + sigma1_x                          , L2_x, M2_x  ); // tenia (_local_pos_gera.vx - x2_d)
            f1 = D * cos( _theta_gera - alpha_gera ) + (m) * x2_d_dot - (m) * sigma2_x ;

            sigma1_z = saturation(   kap1 * ( (_local_pos_gera.z - z1_d) + ( _local_pos_gera.vz - z2_d ) )  , L1_z, M1_z  ); // _local_pos_gera.z - z1_d
            sigma2_z = saturation(   kap2 * ( _local_pos_gera.vz - z2_d ) + sigma1_z                        , L2_z, M2_z  );
            f2 =  -( m * CONSTANTS_ONE_G  -  L * cos( _theta_gera - alpha_gera ) + (m) * z2_d_dot) - (m) * sigma2_z ;



//            f1 = D * cos( _theta_gera - alpha_gera ) + (m) * x2_d_dot - (m) * ( k2*(_local_pos_gera.vx - x2_d) + k1*( ( _local_pos_gera.x - x1_d ) + (_local_pos_gera.vx - x2_d) ) ) ;
//            f2 = m * CONSTANTS_ONE_G -  L * cos( _theta_gera - alpha_gera ) + (m) * z2_d_dot - (m) * ( kap2 * ( _local_pos_gera.vz - z2_d ) + kap1 * ( (_local_pos_gera.z - z1_d) + ( _local_pos_gera.vz - z2_d ) ) );


            T_gera = sqrt( (f1*f1) + (f2*f2) );
            //_THROTTLE_gera =  _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];
            _THROTTLE_gera = T_gera/115.0f ; //100.0f 115 ,      tenia T_gera/120.0f
            _THROTTLE_gera = math::constrain(_THROTTLE_gera, 0.0f, 1.0f);




            _tilt_control = asinf( f1/T_gera ) - _theta_gera ; // tenia T_gera
            _tilt_control /= 1.5707963268f ; // _tilt_control = _tilt_control/1.5707963268f + 0.25f ;
            _tilt_control = math::constrain(_tilt_control, -1.0f, 1.3f);
            //_tilt_control = 0.5f * 1.0f + 0.5f * 1.0f * tanh( 0.65f * (time_since_trans_start - 0.65f*10.0f) );
//            if (_tilt_control >= 0.9999f){ _tilt_control = _params_tiltrotor.tilt_fw;  }

            /*Gera*/







        }


        // at low speeds give full weight to MC
        _mc_roll_weight = 1.0f;
        _mc_yaw_weight = 1.0f;

        // reduce MC controls once the plane has picked up speed
        if (!_params->airspeed_disabled && PX4_ISFINITE(_airspeed_validated->equivalent_airspeed_m_s) &&
            _airspeed_validated->equivalent_airspeed_m_s > ARSP_YAW_CTRL_DISABLE) {
            _mc_yaw_weight = 0.0f;
        }

        if (!_params->airspeed_disabled && PX4_ISFINITE(_airspeed_validated->equivalent_airspeed_m_s) &&
            _airspeed_validated->equivalent_airspeed_m_s >= _params->airspeed_blend) {
            _mc_roll_weight = 1.0f - (_airspeed_validated->equivalent_airspeed_m_s - _params->airspeed_blend) /
                      (_params->transition_airspeed - _params->airspeed_blend);
        }

        // without airspeed do timed weight changes
        if ((_params->airspeed_disabled || !PX4_ISFINITE(_airspeed_validated->equivalent_airspeed_m_s)) &&
            time_since_trans_start > _params->front_trans_time_min) {
            _mc_roll_weight = 1.0f - (time_since_trans_start - _params->front_trans_time_min) /
                      (_params->front_trans_time_openloop - _params->front_trans_time_min);
            _mc_yaw_weight = _mc_roll_weight;
        }

        _thrust_transition = -_mc_virtual_att_sp->thrust_body[2];

    } else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_FRONT_P2) {
        // the plane is ready to go into fixed wing mode, tilt the rotors forward completely
//        _tilt_control = _params_tiltrotor.tilt_transition +
//                fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_transition) * time_since_trans_start /
//                _params_tiltrotor.front_trans_dur_p2;


//        _tilt_control = _params_tiltrotor.tilt_mc +
//                fabsf(_params_tiltrotor.tilt_transition*0.5f - _params_tiltrotor.tilt_mc) * time_since_trans_start /
//                _params->front_trans_duration; // ORIGINAL


        /*Gera*/
        x2_d = 0.5f * _params_tiltrotor.airspeed_d + 0.5f * _params_tiltrotor.airspeed_d * tanh( 0.85f * (time_since_trans_start - 0.85f*10.0f) );
        //x2_d = 0.5f * _params_tiltrotor.airspeed_d + 0.5f * _params_tiltrotor.airspeed_d * tanh( 0.65f * (time_since_trans_start - 0.65f*10.0f) );

        x2_d_dot = ( x2_d - x2_d_prev ) / delta_time;
        x2_d_prev = x2_d;

        if (x2_d >= _params_tiltrotor.airspeed_d - 0.0f){ x2_d = _params_tiltrotor.airspeed_d;  x2_d_dot = 0.0f; }



//        f1 = D * cos( _theta_gera - alpha_gera ) + (m) * x2_d_dot - (m) * ( k2*(_local_pos_gera.vx - x2_d) + k1*( ( _local_pos_gera.x - x1_d ) + (_local_pos_gera.vx - x2_d) ) ) ;
//        f2 = m * CONSTANTS_ONE_G -  L * cos( _theta_gera - alpha_gera ) + (m) * z2_d_dot - (m) * ( kap2 * ( _local_pos_gera.vz - z2_d ) + kap1 * ( (_local_pos_gera.z - z1_d) + ( _local_pos_gera.vz - z2_d ) ) );


        sigma1_x = saturation(   k1*( ( _local_pos_gera.y - x1_d ) + ( _local_pos_gera.vy - x2_d) )  , L1_x, M1_x  ); // tenia _local_pos_gera.x - x1_d, y _local_pos_gera.vx - x2_d
        sigma2_x = saturation(   k2*( _local_pos_gera.vy - x2_d) + sigma1_x                          , L2_x, M2_x  ); // tenia (_local_pos_gera.vx - x2_d)
        f1 = D * cos( _theta_gera - alpha_gera ) + (m) * x2_d_dot - (m) * sigma2_x ;

        sigma1_z = saturation(   kap1 * ( (_local_pos_gera.z - z1_d) + ( _local_pos_gera.vz - z2_d ) )  , L1_z, M1_z  );
        sigma2_z = saturation(   kap2 * ( _local_pos_gera.vz - z2_d ) + sigma1_z                        , L2_z, M2_z  );
        f2 =  -( m * CONSTANTS_ONE_G  -  L * cos( _theta_gera - alpha_gera ) + (m) * z2_d_dot) - (m) * sigma2_z ;

        T_gera = sqrt( (f1*f1) + (f2*f2) );
        //_THROTTLE_gera =  _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];
        _THROTTLE_gera = T_gera/115.0f ; //115,    tenia T_gera/120.0f
        _THROTTLE_gera = math::constrain(_THROTTLE_gera, 0.0f, 1.0f);


        _tilt_control = asinf( f1/T_gera ) - _theta_gera ; // tenia T_gera
        _tilt_control /= 1.5707963268f ; // _tilt_control = _tilt_control/1.5707963268f + 0.25f ;
        _tilt_control = math::constrain(_tilt_control, -1.0f, 1.3f);
        //_tilt_control = 0.5f * 1.0f + 0.5f * 1.0f * tanh( 0.65f * (time_since_trans_start - 0.65f*10.0f) );
//        if (_tilt_control >= 0.9999f){ _tilt_control = _params_tiltrotor.tilt_fw;  }







        /*Gera*/



//        _mc_roll_weight = 0.0f;
//        _mc_yaw_weight = 0.0f;

//        // ramp down motors not used in fixed-wing flight (setting MAX_PWM down scales the given output into the new range)
//        int ramp_down_value = (1.0f - time_since_trans_start / _params_tiltrotor.front_trans_dur_p2) *
//                      (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) + PWM_DEFAULT_MIN;


//        _motor_state = set_motor_state(_motor_state, motor_state::VALUE, ramp_down_value);


//        _thrust_transition = -_mc_virtual_att_sp->thrust_body[2];

    } else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_BACK) {
//        // turn on all MC motors
        if (_motor_state != motor_state::ENABLED) {
            _motor_state = set_motor_state(_motor_state, motor_state::ENABLED);
        }

        // set idle speed for rotary wing mode
        if (!flag_idle_mc) {
            flag_idle_mc = set_idle_mc();
        }

        // tilt rotors back
//        if (_tilt_control > _params_tiltrotor.tilt_mc) {

            /* Gera */
            x2_d = 0.5f * _params_tiltrotor.airspeed_d - 0.5f * _params_tiltrotor.airspeed_d * tanh( 0.85f * (time_since_trans_start - 0.85f*10.0f) );

            x2_d_dot = ( x2_d - x2_d_prev ) / delta_time;
            x2_d_prev = x2_d;

            if (x2_d <= 0.0f){ x2_d = 0.0f;     x2_d_dot = 0.0f; }


            sigma1_x = saturation(   0.0f*k1*( ( _local_pos_gera.y - x1_d ) + ( _airspeed_validated->true_ground_minus_wind_m_s - x2_d) )  , L1b_x, M1b_x  ); // tenia _local_pos_gera.x - x1_d, y _local_pos_gera.vx - x2_d
            sigma2_x = saturation(   k2_b*( _airspeed_validated->true_ground_minus_wind_m_s - x2_d) + sigma1_x                          , L2b_x, M2b_x  ); // tenia (_local_pos_gera.vx - x2_d)
            f1 = D * cos( _theta_gera - alpha_gera ) + (m) * x2_d_dot - (m) * sigma2_x ;

            sigma1_z = saturation(   kap1_b * ( (_local_pos_gera.z - z1_d) + ( _local_pos_gera.vz - z2_d ) )  , L1b_z, M1b_z  );
            sigma2_z = saturation(   kap2_b * ( _local_pos_gera.vz - z2_d ) + sigma1_z                        , L2b_z, M2b_z  );
            f2 =  -( m * CONSTANTS_ONE_G  -  L * cos( _theta_gera - alpha_gera ) + (m) * z2_d_dot ) - (m) * sigma2_z ;

            T_gera = sqrt( (f1*f1) + (f2*f2) );
            //_THROTTLE_gera =  _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];
            _THROTTLE_gera = T_gera/115.0f ; //con 80 funciona tenia T_gera/120.0f tenia 115
            _THROTTLE_gera = math::constrain(_THROTTLE_gera, -0.15f, 1.0f);


            _tilt_control = asinf( f1/T_gera ) - _theta_gera; // tenia T_gera
            _tilt_control/=1.5707963268f;
            _tilt_control = math::constrain(_tilt_control, - 0.15f, 1.3f);


//            _tilt_control = 0.5f * 1.0f - 0.5f * 1.0f * tanh( 0.65f * (time_since_trans_start - 0.65f*10.0f) );
//            if (_tilt_control <= 0.0001f){ _tilt_control = _params_tiltrotor.tilt_mc;  }


            /* Gera */

//            _tilt_control = _params_tiltrotor.tilt_fw -
//                    fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_mc) * time_since_trans_start / 1.0f; // ORIGINAL
//        }

        _mc_yaw_weight = 1.0f;

        // control backtransition deceleration using pitch.
//        if (_v_control_mode->flag_control_climb_rate_enabled) {
//            _v_att_sp->pitch_body = update_and_get_backtransition_pitch_sp();
//        }

//        // while we quickly rotate back the motors keep throttle at idle
//        if (time_since_trans_start < 1.0f) {
//            _mc_throttle_weight = 0.0f;
//            _mc_roll_weight = 0.0f;
//            _mc_pitch_weight = 0.0f;

//        } else {
//            _mc_roll_weight = 1.0f;
//            _mc_pitch_weight = 1.0f;
//            // slowly ramp up throttle to avoid step inputs
//            _mc_throttle_weight = (time_since_trans_start - 1.0f) / 1.0f;
//        }

        // las 2 lineas que siguen yo se las puse y le quite las de arriba
        _mc_roll_weight = 1.0f;
        _mc_pitch_weight = 1.0f;
    }

    const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
    q_sp.copyTo(_v_att_sp->q_d);

//    _mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
    _mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);
    _mc_throttle_weight = math::constrain(_mc_throttle_weight, 0.0f, 1.0f);


}

void Tiltrotor::waiting_on_tecs()
{
    // keep multicopter thrust until we get data from TECS
    _v_att_sp->thrust_body[0] = _thrust_transition;
}

/**
* Write data to actuator output topic.
*/
void Tiltrotor::fill_actuator_outputs()
{
    // Multirotor output
    _actuators_out_0->timestamp = hrt_absolute_time();
    _actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;

    _actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
        _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] * _mc_roll_weight;
    _actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
        _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
    _actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
        _actuators_mc_in->control[actuator_controls_s::INDEX_YAW] * _mc_yaw_weight;

    if (_vtol_schedule.flight_mode == vtol_mode::FW_MODE) {
//        _actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
//            _actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];


        /* Gera */
        _actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] = _THROTTLE_gera;
        /* Gera */



        /* allow differential thrust if enabled */
        if (_params->diff_thrust == 1) {
            _actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
                _actuators_fw_in->control[actuator_controls_s::INDEX_YAW] * _params->diff_thrust_scale;
        }

    } else { // !vtol_mode::FW_MODE
//        _actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
//            _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] * _mc_throttle_weight;


        /* Gera */
        _actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] = _THROTTLE_gera * 1.0f;
        /* Gera */


    }

    // Fixed wing output
    _actuators_out_1->timestamp = hrt_absolute_time();
    _actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

    _actuators_out_1->control[4] = _tilt_control;

    if (_params->elevons_mc_lock && _vtol_schedule.flight_mode == vtol_mode::MC_MODE) {
        _actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = 0.0f;
        _actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = 0.0f;
        _actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0.0f;

    } else {
        _actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
            _actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];
        _actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
            _actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];
        _actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
            _actuators_fw_in->control[actuator_controls_s::INDEX_YAW];
    }
}

/*
 * Increase combined thrust of MC propellers if motors are tilted. Assumes that all MC motors are tilted equally.
 */

float Tiltrotor::thrust_compensation_for_tilt()
{
    // only compensate for tilt angle up to 0.5 * max tilt
    float compensated_tilt = math::constrain(_tilt_control, 0.0f, 0.5f);

    // increase vertical thrust by 1/cos(tilt), limmit to [-1,0]
    return math::constrain(_v_att_sp->thrust_body[2] / cosf(compensated_tilt * M_PI_2_F), -1.0f, 0.0f);

}

/*
 *  Funcion de saturación
 */

float Tiltrotor::saturation(float s_, float L_, float M_)
{
        float sigma = 0.0f;
        C = M_PI_F/(2*(M_ - L_));
        if(s_ > L_){
            sigma = (atanf(C*( s_ - L_ ))/C) + L_;
        }else if(fabsf(s_) <= L_ ){
            sigma = s_;
        }else if(s_ < - L_){
            sigma= (atanf(C*(s_ + L_))/C) - L_;
        }

        return sigma;
}
