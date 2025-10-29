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
#if defined(WIN32) | defined(WIN64)
#include <windows.h>
#include "assert.h"
#include <conio.h>
#endif
//---------------------------------------------------------------------------
#ifdef LINUX
#include <pthread.h>
struct  Event                                    { pthread_mutex_t m; pthread_cond_t  c; };
typedef Event* HANDLE;
void    SetEvent(HANDLE e)                       { pthread_cond_signal (&(e->c)); }
HANDLE  CreateEvent(void *, bool r, bool, char*) { HANDLE e = new Event; pthread_mutex_init(&(e->m), NULL); pthread_cond_init(&(e->c), NULL); return e; }
int     WaitForSingleObject(HANDLE e, unsigned)  { pthread_mutex_lock(&(e->m)); pthread_cond_wait(&(e->c), &(e->m)); pthread_mutex_unlock(&(e->m)); return 0; }
void    CloseHandle(HANDLE e)                    { pthread_mutex_destroy(&(e->m)); pthread_cond_destroy(&(e->c)); delete e; }
#endif
//---------------------------------------------------------------------------
#include "PhantomIoLib42.h"
#include "HD/hd.h"
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
//---------------------------------------------------------------------------
#include "chai3d.h"
using namespace chai3d;
//---------------------------------------------------------------------------

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// has DLL been initialed
bool initPhantomDLL = false;

// has servo controller been started yet
bool servoStarted = false;

// if true then print debug messgaes
bool verbose = true;

// main servo controller callback
HDCallbackCode servoCallbackHandle;
HDCallbackCode HDCALLBACK servoPhantomDevices(void* pUserData);


//=============================================================================
// library entry point
// =============================================================================

#ifdef LINUX

// these will only get executed once when the library is loaded,
// but should do the job just fine regardless
void __attribute__ ((constructor)) _hdLoad(void);
void __attribute__ ((destructor))  _hdUnload(void);

#endif


void
_hdLoad(void)
{

}

void _hdUnload(void)
{

}

#if defined(WIN32) | defined(WIN64)

BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved)
{
    switch (ul_reason_for_call) 
    {
    case DLL_PROCESS_ATTACH:
        _hdLoad();
        return (true);
    case DLL_PROCESS_DETACH:
        _hdUnload();
        return(true);
    }
}

#endif


//==========================================================================
// FUNCTIONS ACCESSIBLE FROM OUTSIDE
//==========================================================================

int __FNCALL apply_encoders_sign()
{
    if (verbose) { printf("apply_encoders_sign\n"); }
    return (1);
}

int __FNCALL calc_encoders_from_joint_angles()
{
    if (verbose) { printf("calc_encoders_from_joint_angles\n"); }
    return(1);
}

int __FNCALL calc_endpoint_from_joint()
{
    if (verbose) { printf("calc_endpoint_from_joint\n"); }
    return(1);
}

int __FNCALL calc_joint_angles_from_enc()
{
    if (verbose) { printf("calc_joint_angles_from_enc\n"); }
    return(1); 
}

int __FNCALL command_motor_dac_values()
{ 
    if (verbose) { printf("command_motor_dac_values\n"); }
    return(1);
}

int __FNCALL command_motor_voltages()
{ 
    if (verbose) { printf("command_motor_voltages\n"); }
    return(1);
}

int __FNCALL createServoLoop()
{ 
    if (verbose) { printf("createServoLoop\n"); }
    return(1);
}

int __FNCALL destroyServoLoop()
{ 
    if (verbose) { printf("destroyServoLoop\n"); }
    return(1);
}

int __FNCALL disable_phantom()
{ 
    if (verbose) { printf("disable_phantom\n"); }
    return(1); 
}

int __FNCALL disable_phantom_forces()
{ 
    if (verbose) { printf("disable_phantom_forces\n"); }
    return(1);
}

int __FNCALL disable_phantom_forces_no_wait()
{ 
    if (verbose) { printf("disable_phantom_forces_no_wait\n"); }
    return(1);
}

int __FNCALL enable_phantom_force_check()
{ 
    if (verbose) { printf("enable_phantom_force_check\n"); }
    return(1);
}

int __FNCALL enable_phantom_force_kick_safety()
{ 
    if (verbose) { printf("enable_phantom_force_kick_safety\n"); }
    return(1);
}

int __FNCALL enable_phantom_forces()
{ 
    if (verbose) { printf("enable_phantom_forces\n"); }
    return(1);
}

int __FNCALL enable_phantom_forces_no_wait()
{ 
    if (verbose) { printf("enable_phantom_forces_no_wait\n"); }
    return(1);
}

int __FNCALL enable_phantom_ramp_forces()
{ 
    if (verbose) { printf("enable_phantom_ramp_forces\n"); }
    return(1);
}

int __FNCALL enable_phantom_velocity_check()
{ 
    if (verbose) { printf("enable_phantom_velocity_check\n"); }
    return(1);
}

int __FNCALL getServoLoopTimeStamp()
{ 
    if (verbose) { printf("getServoLoopTimeStamp\n"); }
    return(1); 
}

int __FNCALL get_calibration_status()
{ 
    if (verbose) { printf("get_calibration_status\n"); }
    return(1);
}

int __FNCALL get_ctouch_comport_number()
{ 
    if (verbose) { printf("get_ctouch_comport_number\n"); }
    return(1); 
}

int __FNCALL get_ctouch_serial_number()
{ 
    if (verbose) { printf("get_ctouch_serial_number\n"); }
    return(1);
}

int __FNCALL get_encoder_values()
{ 
    if (verbose) { printf("get_encoder_values\n"); }
    return(1);
}

int __FNCALL get_gimbal_encoders()
{ 
    if (verbose) { printf("get_gimbal_encoders\n"); }
    return(1);
}

int __FNCALL get_hardware_rev()
{ 
    if (verbose) { printf("get_hardware_rev\n"); }
    return(1);
}

int __FNCALL get_hardware_serial_number()
{ 
    if (verbose) { printf("get_hardware_serial_number\n"); }
    return(1);
}

int __FNCALL get_inkwell_switch()
{ 
    if (verbose) { printf("get_inkwell_switch\n"); }
    return(1);
}

int __FNCALL get_last_motor_dac_values()
{ 
    if (verbose) { printf("get_last_motor_dac_values\n"); }
    return(1);
}

int __FNCALL get_library_version()
{
    if (verbose) { printf("get_library_version\n"); }
    return(1);
}

int __FNCALL get_phantom_encoders()
{ 
    if (verbose) { printf("get_phantom_encoders\n"); }
    return(1);
}

int __FNCALL get_phantom_encoders_6dof()
{ 
    if (verbose) { printf("get_phantom_encoders_6dof\n"); }
    return(1);
}

int __FNCALL get_phantom_force_kick_safety()
{ 
    if (verbose) { printf("get_phantom_force_kick_safety\n"); }
    return(1);
}

int __FNCALL get_phantom_info(PHANTOM_INFO_STRUCT* a_info, unsigned long a_param)
{ 
    if (verbose) 
    { 
        printf("get_phantom_info\n"); 
        int value = a_param;
        printf("value = %i\n", value); 

    }
    
    a_info = new PHANTOM_INFO_STRUCT;
    a_info->data[1] = 0.1;
    a_info->data[2] = 0.1;
    a_info->data[3] = 0.1;
    a_info->data[4] = 0.1;
    a_info->data[5] = 0.1;

    return(1);
}

int __FNCALL get_phantom_inst_rate()
{ 
    if (verbose) { printf("get_phantom_inst_rate\n"); }
    return(1); 
}

int __FNCALL get_phantom_inst_vel()
{ 
    if (verbose) { printf("get_phantom_inst_vel\n"); }
    return(1);
}

int __FNCALL get_phantom_jacobian()
{ 
    if (verbose) { printf("get_phantom_jacobian\n"); }
    return(1);
}

int __FNCALL get_phantom_joint_angles()
{ 
    if (verbose) { printf("get_phantom_joint_angles\n"); }
    return(1);
}

int __FNCALL get_phantom_joint_vel()
{ 
    if (verbose) { printf("get_phantom_joint_vel\n"); }
    return(1);
}

int __FNCALL get_phantom_max_velocity()
{ 
    if (verbose) { printf("get_phantom_max_velocity\n"); }
    return(1);
}

int __FNCALL get_phantom_model_type()
{ 
    if (verbose) { printf("get_phantom_model_type\n"); }
    //name = "Touch\n";
    return(0);
}

int __FNCALL get_phantom_nominal_max_continuous_force()
{ 
    if (verbose) { printf("get_phantom_nominal_max_continuous_force\n"); }
    return(1); 
}

int __FNCALL get_phantom_nominal_max_damping()
{ 
    if (verbose) { printf("get_phantom_nominal_max_damping\n"); }
    return(1);
}

int __FNCALL get_phantom_nominal_max_force()
{ 
    if (verbose) { printf("get_phantom_nominal_max_force\n"); }
    return(1);
}

int __FNCALL get_phantom_nominal_max_stiffness()
{ 
    if (verbose) { printf("get_phantom_nominal_max_stiffness\n"); }
    return(1);
}

int __FNCALL get_phantom_nominal_max_torque_continuous_force()
{ 
    if (verbose) { printf("get_phantom_nominal_max_torque_continuous_force\n"); }
    return(1);
}

int __FNCALL get_phantom_nominal_max_torque_damping()
{ 
    if (verbose) { printf("get_phantom_nominal_max_torque_damping\n"); }
    return(1);
}

int __FNCALL get_phantom_nominal_max_torque_force()
{ 
    if (verbose) { printf("get_phantom_nominal_max_torque_force\n"); }
    return(1);
}

int __FNCALL get_phantom_nominal_max_torque_stiffness()
{ 
    if (verbose) { printf("get_phantom_nominal_max_torque_stiffness\n"); }
    return(1);
}

int __FNCALL get_phantom_pos()
{ 
    if (verbose) { printf("get_phantom_pos\n"); }
    return(1);
}

int __FNCALL get_phantom_ramp_forces_rate()
{ 
    if (verbose) { printf("get_phantom_ramp_forces_rate\n"); }
    return(1); 
}

int __FNCALL get_phantom_rate()
{ 
    if (verbose) { printf("get_phantom_rate\n"); }
    return(1);
}

int __FNCALL get_phantom_temp()
{ 
    if (verbose) { printf("get_phantom_temp\n"); }
    return(1);
}

int __FNCALL get_phantom_temp_6dof()
{ 
    if (verbose) { printf("get_phantom_temp_6dof\n"); }
    return(1); 
}

int __FNCALL get_phantom_temp_advance()
{ 
    if (verbose) { printf("get_phantom_temp_advance\n"); }
    return(1); 
}

int __FNCALL get_phantom_temp_normalized()
{ 
    if (verbose) { printf("get_phantom_temp_normalized\n"); }
    return(1);
}

int __FNCALL get_phantom_temp_normalized_6dof()
{ 
    if (verbose) { printf("get_phantom_temp_normalized_6dof\n"); }
    return(1);
}

int __FNCALL get_phantom_vel()
{ 
    if (verbose) { printf("get_phantom_vel\n"); }
    return(1); 
}

int __FNCALL get_phantom_vel_6dof()
{ 
    if (verbose) { printf("get_phantom_vel_6dof\n"); }
    return(1);
}

int __FNCALL get_pinch_value()
{ 
    if (verbose) { printf("get_pinch_value\n"); }
    return(1); 
}

int __FNCALL get_status_light()
{ 
    if (verbose) { printf("get_status_light\n"); }
    return(1); 
}

int __FNCALL get_stylus_matrix()
{ 
    if (verbose) { printf("get_stylus_matrix\n"); }
    return(1);
}

int __FNCALL get_stylus_presence()
{ 
    if (verbose) { printf("get_stylus_presence\n"); }
    return(1); 
}

int __FNCALL get_stylus_switch()
{ 
    if (verbose) { printf("get_stylus_switch\n"); }
    return(1); 
}

int __FNCALL get_vendor_string()
{ 
    if (verbose) { printf("get_vendor_string\n"); }
    return(1); 
}

int __FNCALL initOSExtender()
{ 
    if (verbose) { printf("initOSExtender\n"); }
    return(1); 
}

int __FNCALL init_phantom()
{ 
    printf("init_phantom\n");
    return(1);
}

int __FNCALL isServoLoopRunning()
{ 
    if (verbose) { printf("isServoLoopRunning\n"); }
    return(1); 
}

int __FNCALL is_calibration_supported()
{ 
    if (verbose) { printf("is_calibration_supported\n"); }
    return(1); 
}

int __FNCALL is_ctouch_activated()
{ 
    if (verbose) { printf("is_ctouch_activated\n"); }
    return(1); 
}

int __FNCALL is_force_saturation_enabled()
{ 
    if (verbose) { printf("is_force_saturation_enabled\n"); }
    return(1); 
}

int __FNCALL is_phantom_force_check_enabled()
{ 
    if (verbose) { printf("is_phantom_force_check_enabled\n"); }
    return(1); 
}

int __FNCALL is_phantom_force_kick_safety_enabled()
{ 
    if (verbose) { printf("is_phantom_force_kick_safety_enabled\n"); }
    return(1); 
}

int __FNCALL is_phantom_forces_enabled()
{ 
    if (verbose) { printf("is_phantom_forces_enabled\n"); }
    return(1); 
}

int __FNCALL is_phantom_inkwell_calibration_supported()
{ 
    if (verbose) { printf("is_phantom_inkwell_calibration_supported\n"); }
    return(1); 
}

int __FNCALL is_phantom_ramp_forces_enabled()
{ 
    if (verbose) { printf("is_phantom_ramp_forces_enabled\n"); }
    return(1); 
}

int __FNCALL is_phantom_reset_needed()
{ 
    if (verbose) { printf("is_phantom_reset_needed\n"); }
    return(1); 
}

int __FNCALL is_phantom_reset_supported()
{ 
    if (verbose) { printf("is_phantom_reset_supported\n"); }
    return(1); 
}

int __FNCALL is_phantom_velocity_check_enabled()
{ 
    if (verbose) { printf("is_phantom_velocity_check_enabled\n"); }
    return(1); 
}

int __FNCALL is_pinch_calibrated()
{ 
    if (verbose) { printf("is_pinch_calibrated\n"); }
    return(1);
}

int __FNCALL is_status_light_enabled()
{ 
    if (verbose) { printf("is_status_light_enabled\n"); }
    return(1); 
}

int __FNCALL override_status_light()
{ 
    if (verbose) { printf("override_status_light\n"); }
    return(1); 
}

int __FNCALL phantom_check_for_gimbal()
{ 
    if (verbose) { printf("phantom_check_for_gimbal\n"); }
    return(1); 
}

int __FNCALL phantom_check_for_pinch()
{ 
    if (verbose) { printf("phantom_check_for_pinch\n"); }
    return(1); 
}

int __FNCALL phantom_reset()
{ 
    if (verbose) { printf("phantom_reset\n"); }
    return(1); 
}

int __FNCALL phantom_status()
{ 
    if (verbose) { printf("phantom_status\n"); }
    return(1); 
}

int __FNCALL phantom_which_pinch()
{ 
    if (verbose) { printf("phantom_which_pinch\n"); }
    return(1); 
}

int __FNCALL restore_status_light()
{ 
    if (verbose) { printf("restore_status_light\n"); }
    return(1); 
}

int __FNCALL send_phantom_force()
{ 
    if (verbose) { printf("send_phantom_force\n"); }
    return(1); 
}

int __FNCALL send_phantom_force_6dof()
{ 
    if (verbose) { printf("send_phantom_force_6dof\n"); }
    return(1); 
}

int __FNCALL send_phantom_joint_torques()
{ 
    if (verbose) { printf("send_phantom_joint_torques\n"); }
    return(1); 
}

int __FNCALL setPostServoCallback()
{ 
    if (verbose) { printf("setPostServoCallback\n"); }
    return(1); 
}

int __FNCALL setPreServoCallback()
{ 
    if (verbose) { printf("setPreServoCallback\n"); }
    return(1); 
}

int __FNCALL setServoErrorCallback()
{ 
    if (verbose) { printf("setServoErrorCallback\n"); }
    return(1); 
}

int __FNCALL setServoLoopRate()
{ 
    if (verbose) { printf("setServoLoopRate\n"); }
    return(1); 
}

int __FNCALL setServoLoopTimer()
{ 
    if (verbose) { printf("setServoLoopTimer\n"); }
    return(1); 
}

int __FNCALL set_joint_angle_references()
{ 
    if (verbose) { printf("set_joint_angle_references\n"); }
    return(1); 
}

int __FNCALL set_phantom_force_kick_safety()
{ 
    if (verbose) { printf("set_phantom_force_kick_safety\n"); }
    return(1); 
}

int __FNCALL set_phantom_max_velocity()
{ 
    if (verbose) { printf("set_phantom_max_velocity\n"); }
    return(1); 
}

int __FNCALL set_phantom_ramp_forces_rate()
{ 
    if (verbose) { printf("set_phantom_ramp_forces_rate\n"); }
    return(1); 
}

int __FNCALL set_phantom_temp_advance()
{ 
    if (verbose) { printf("set_phantom_temp_advance\n"); }
    return(1); 
}

int __FNCALL set_pinch_limits()
{ 
    if (verbose) { printf("set_pinch_limits\n"); }
    return(1); 
}

int __FNCALL startServoLoop()
{ 
    if (verbose) { printf("startServoLoop\n"); }
    return(1);
}

int __FNCALL stopServoLoop()
{ 
    if (verbose) { printf("stopServoLoop\n"); }
    return(1); 
}

int __FNCALL truncate_force_saturation()
{ 
    if (verbose) { printf("truncate_force_saturation\n"); }
    return(1); 
}

int __FNCALL unfiltered_device_fault()
{ 
    if (verbose) { printf("unfiltered_device_fault\n"); }
    return(1); 
}

int __FNCALL update_calibration()
{ 
    if (verbose) { printf("update_calibration\n"); }
    return(1); 
}

int __FNCALL update_phantom()
{ 
    if (verbose) { printf("update_phantom\n"); }
    return(1);
}

int __FNCALL write_phantom_joint_torques()
{ 
    if (verbose) { printf("write_phantom_joint_torques\n"); }
    return(1); 
}

int __FNCALL zero_force_saturation()
{ 
    if (verbose) { printf("zero_force_saturation\n"); }
    return(1); 
}



//==========================================================================
// FUNCTIONS INTERNAL TO THE DLL
//==========================================================================

//==========================================================================
/*
    Servo controller callback

    \fn     HDLServoOpExitCode servophantomDevices(void* pUserData)

    \param  pUserData pointer to user data information (not used here)
*/
//==========================================================================
/*
HDCallbackCode HDCALLBACK servoPhantomDevices(void* pUserData)
{
    for (int i=0; i<numPhantomDevices; i++)
    {
        // for each activated phantom device
        if (phantomDevices[i].enabled)
        {
            // retrieve handle
            HHD hHD = phantomDevices[i].handle;

            // activate ith device
            hdMakeCurrentDevice(hHD);

            // start sending commands
            hdBeginFrame(hHD);
            
            // retrieve the position and orientation of the end-effector.
            double frame[16];
            hdGetDoublev(HD_CURRENT_TRANSFORM, frame);

            // convert position from [mm] to [m] 
            frame[12] = frame[12] * 0.001;
            frame[13] = frame[13] * 0.001;
            frame[14] = frame[14] * 0.001;

            phantomDevices[i].position[0] = frame[12];
            phantomDevices[i].position[1] = frame[13];
            phantomDevices[i].position[2] = frame[14];

            phantomDevices[i].rotation[0] = frame[0];
            phantomDevices[i].rotation[1] = frame[1];
            phantomDevices[i].rotation[2] = frame[2];
            phantomDevices[i].rotation[3] = frame[4];
            phantomDevices[i].rotation[4] = frame[5];
            phantomDevices[i].rotation[5] = frame[6];
            phantomDevices[i].rotation[6] = frame[8];
            phantomDevices[i].rotation[7] = frame[9];
            phantomDevices[i].rotation[8] = frame[10];

            // read linear velocity
            double vel[3];
            hdGetDoublev(HD_CURRENT_VELOCITY, vel);

            // convert position from [mm] to [m] 
            vel[0] = vel[0] * 0.001;
            vel[1] = vel[1] * 0.001;
            vel[2] = vel[2] * 0.001;
            
            phantomDevices[i].linearVelocity[0] = vel[0];
            phantomDevices[i].linearVelocity[1] = vel[1];
            phantomDevices[i].linearVelocity[2] = vel[2];

            // read user buttons
            int buttons;
            hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
            phantomDevices[i].button = buttons;

            // issue synchronization event
            SetEvent (phantomDevices[i].sync);

            // send force to end-effector
            double force[3];
            force[0] = phantomDevices[i].force[0];
            force[1] = phantomDevices[i].force[1];
            force[2] = phantomDevices[i].force[2];
            hdSetDoublev(HD_CURRENT_FORCE, force);

            // send torque to end-effector
            double torque[3];
            torque[0] = phantomDevices[i].torque[0] * 1000.0;
            torque[1] = phantomDevices[i].torque[1] * 1000.0;
            torque[2] = phantomDevices[i].torque[2] * 1000.0;
            hdSetDoublev(HD_CURRENT_TORQUE, torque);

            // flush commands
            hdEndFrame(hHD);
        }
    }

    return (HD_CALLBACK_CONTINUE);
}
*/