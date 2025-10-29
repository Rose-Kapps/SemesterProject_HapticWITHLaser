//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2023, CHAI3D
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

 * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

 * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author     Federico Barbagli
    \author     Francois Conti
    \author     Sebastien Grange
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef PhantomIoLib42H
#define PhantomIoLib42H
//---------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
#define __FNCALL __stdcall
#endif
#ifdef LINUX
#define __FNCALL
#endif
//---------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif
//---------------------------------------------------------------------------

struct PHANTOM_INFO_STRUCT 
{
    double data[200];
};

//===========================================================================
// API
//===========================================================================

int __FNCALL apply_encoders_sign();
int __FNCALL calc_encoders_from_joint_angles();
int __FNCALL calc_endpoint_from_joint();
int __FNCALL calc_joint_angles_from_enc();
int __FNCALL command_motor_dac_values();
int __FNCALL command_motor_voltages();
int __FNCALL createServoLoop();
int __FNCALL destroyServoLoop();
int __FNCALL disable_phantom();
int __FNCALL disable_phantom_forces();
int __FNCALL disable_phantom_forces_no_wait();
int __FNCALL enable_phantom_force_check();
int __FNCALL enable_phantom_force_kick_safety();
int __FNCALL enable_phantom_forces();
int __FNCALL enable_phantom_forces_no_wait();
int __FNCALL enable_phantom_ramp_forces();
int __FNCALL enable_phantom_velocity_check();
int __FNCALL getServoLoopTimeStamp();
int __FNCALL get_calibration_status();
int __FNCALL get_ctouch_comport_number();
int __FNCALL get_ctouch_serial_number();
int __FNCALL get_encoder_values();
int __FNCALL get_gimbal_encoders();
int __FNCALL get_hardware_rev();
int __FNCALL get_hardware_serial_number();
int __FNCALL get_inkwell_switch();
int __FNCALL get_last_motor_dac_values();
int __FNCALL get_library_version();
int __FNCALL get_phantom_encoders();
int __FNCALL get_phantom_encoders_6dof();
int __FNCALL get_phantom_force_kick_safety();
int __FNCALL get_phantom_info(PHANTOM_INFO_STRUCT* a_info, unsigned long a_param);
int __FNCALL get_phantom_inst_rate();
int __FNCALL get_phantom_inst_vel();
int __FNCALL get_phantom_jacobian();
int __FNCALL get_phantom_joint_angles();
int __FNCALL get_phantom_joint_vel();
int __FNCALL get_phantom_max_velocity();
int __FNCALL get_phantom_model_type();
int __FNCALL get_phantom_nominal_max_continuous_force();
int __FNCALL get_phantom_nominal_max_damping();
int __FNCALL get_phantom_nominal_max_force();
int __FNCALL get_phantom_nominal_max_stiffness();
int __FNCALL get_phantom_nominal_max_torque_continuous_force();
int __FNCALL get_phantom_nominal_max_torque_damping();
int __FNCALL get_phantom_nominal_max_torque_force();
int __FNCALL get_phantom_nominal_max_torque_stiffness();
int __FNCALL get_phantom_pos();
int __FNCALL get_phantom_ramp_forces_rate();
int __FNCALL get_phantom_rate();
int __FNCALL get_phantom_temp();
int __FNCALL get_phantom_temp_6dof();
int __FNCALL get_phantom_temp_advance();
int __FNCALL get_phantom_temp_normalized();
int __FNCALL get_phantom_temp_normalized_6dof();
int __FNCALL get_phantom_vel();
int __FNCALL get_phantom_vel_6dof();
int __FNCALL get_pinch_value();
int __FNCALL get_status_light();
int __FNCALL get_stylus_matrix();
int __FNCALL get_stylus_presence();
int __FNCALL get_stylus_switch();
int __FNCALL get_vendor_string();
int __FNCALL initOSExtender();
int __FNCALL init_phantom();
int __FNCALL isServoLoopRunning();
int __FNCALL is_calibration_supported();
int __FNCALL is_ctouch_activated();
int __FNCALL is_force_saturation_enabled();
int __FNCALL is_phantom_force_check_enabled();
int __FNCALL is_phantom_force_kick_safety_enabled();
int __FNCALL is_phantom_forces_enabled();
int __FNCALL is_phantom_inkwell_calibration_supported();
int __FNCALL is_phantom_ramp_forces_enabled();
int __FNCALL is_phantom_reset_needed();
int __FNCALL is_phantom_reset_supported();
int __FNCALL is_phantom_velocity_check_enabled();
int __FNCALL is_pinch_calibrated();
int __FNCALL is_status_light_enabled();
int __FNCALL override_status_light();
int __FNCALL phantom_check_for_gimbal();
int __FNCALL phantom_check_for_pinch();
int __FNCALL phantom_reset();
int __FNCALL phantom_status();
int __FNCALL phantom_which_pinch();
int __FNCALL restore_status_light();
int __FNCALL send_phantom_force();
int __FNCALL send_phantom_force_6dof();
int __FNCALL send_phantom_joint_torques();
int __FNCALL setPostServoCallback();
int __FNCALL setPreServoCallback();
int __FNCALL setServoErrorCallback();
int __FNCALL setServoLoopRate();
int __FNCALL setServoLoopTimer();
int __FNCALL set_joint_angle_references();
int __FNCALL set_phantom_force_kick_safety();
int __FNCALL set_phantom_max_velocity();
int __FNCALL set_phantom_ramp_forces_rate();
int __FNCALL set_phantom_temp_advance();
int __FNCALL set_pinch_limits();
int __FNCALL startServoLoop();
int __FNCALL stopServoLoop();
int __FNCALL truncate_force_saturation();
int __FNCALL unfiltered_device_fault();
int __FNCALL update_calibration();
int __FNCALL update_phantom();
int __FNCALL write_phantom_joint_torques();
int __FNCALL zero_force_saturation();

//---------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
