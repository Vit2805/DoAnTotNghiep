//
// File: testros2.h
//
// Code generated for Simulink model 'testros2'.
//
// Model version                  : 1.83
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Thu Jan 22 22:29:37 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef testros2_h_
#define testros2_h_
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "dt_info.h"
#include "ext_work.h"
#include "slros2_initialize.h"
#include "testros2_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include <float.h>
#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

// Block signals (default storage)
struct B_testros2_T {
  SL_Bus_nav_msgs_Odometry In1;        // '<S16>/In1'
  SL_Bus_nav_msgs_Odometry rtb_SourceBlock_o2_m;
  real_T dv[12];
  real_T Q[9];
  real_T A[9];
  real_T dv1[9];
  SL_Bus_geometry_msgs_Twist BusAssignment;// '<Root>/Bus Assignment'
  real_T e_R[3];
  real_T q_ddot[3];
  real_T TmpSignalConversionAtSFunct[3];// '<S4>/MATLAB Function'
  real_T Q_c[3];
  real_T dv2[2];
  real_T x;
  real_T Sum;                          // '<Root>/Sum'
  real_T y;
  real_T Sum1;                         // '<Root>/Sum1'
  real_T Sum2;                         // '<Root>/Sum2'
  real_T theta;                        // '<S9>/MATLAB Function'
  real_T cmd_vel[3];                   // '<S4>/MATLAB Function'
  real_T u_dk[4];                      // '<S2>/MATLAB Function'
  real_T m3;
  real_T m4;
  real_T e_out;                        // '<Root>/MATLAB Function1'
  real_T TSamp;                        // '<S10>/TSamp'
  real_T TSamp_n;                      // '<S12>/TSamp'
  real_T unnamed_idx_0;
  real_T u_dk_k;
  real_T u_dk_c;
  real_T u_dk_b;
  real_T rtb_e_out_tmp;
  real_T rtb_e_out_tmp_p;
  real_T m4_tmp;
  real_T m3_tmp;
  real_T unnamed_idx_0_tmp;
  real_T rtb_TmpSignalConversionAtSFun_c;
  int32_T i;
  int32_T e_R_tmp;
  int32_T i1;
  int32_T i2;
  boolean_T b_varargout_1;
};

// Block states (default storage) for system '<Root>'
struct DW_testros2_T {
  ros_slros2_internal_block_Pub_T obj; // '<S7>/SinkBlock'
  ros_slros2_internal_block_Sub_T obj_n;// '<S8>/SourceBlock'
  real_T UD_DSTATE;                    // '<S10>/UD'
  real_T UD_DSTATE_p;                  // '<S11>/UD'
  real_T UD_DSTATE_b;                  // '<S12>/UD'
  real_T DiscreteTimeIntegrator_DSTATE;// '<S2>/Discrete-Time Integrator'
  real_T DiscreteTimeIntegrator1_DSTATE;// '<S2>/Discrete-Time Integrator1'
  real_T DiscreteTimeIntegrator2_DSTATE;// '<S2>/Discrete-Time Integrator2'
  struct {
    void *LoggedData;
  } Scope2_PWORK;                      // '<Root>/Scope2'

  struct {
    void *LoggedData[3];
  } Scope_PWORK;                       // '<Root>/Scope'

  struct {
    void *LoggedData[3];
  } Scope1_PWORK;                      // '<Root>/Scope1'

  int8_T EnabledSubsystem_SubsysRanBC; // '<S8>/Enabled Subsystem'
};

// Parameters (default storage)
struct P_testros2_T_ {
  real_T DiscreteDerivative_ICPrevScaled;
                              // Mask Parameter: DiscreteDerivative_ICPrevScaled
                                 //  Referenced by: '<S10>/UD'

  real_T DiscreteDerivative1_ICPrevScale;
                              // Mask Parameter: DiscreteDerivative1_ICPrevScale
                                 //  Referenced by: '<S11>/UD'

  real_T DiscreteDerivative2_ICPrevScale;
                              // Mask Parameter: DiscreteDerivative2_ICPrevScale
                                 //  Referenced by: '<S12>/UD'

  SL_Bus_nav_msgs_Odometry Out1_Y0;    // Computed Parameter: Out1_Y0
                                          //  Referenced by: '<S16>/Out1'

  SL_Bus_nav_msgs_Odometry Constant_Value;// Computed Parameter: Constant_Value
                                             //  Referenced by: '<S8>/Constant'

  SL_Bus_geometry_msgs_Twist Constant_Value_d;// Computed Parameter: Constant_Value_d
                                                 //  Referenced by: '<S1>/Constant'

  real_T TSamp_WtEt;                   // Computed Parameter: TSamp_WtEt
                                          //  Referenced by: '<S10>/TSamp'

  real_T TSamp_WtEt_n;                 // Computed Parameter: TSamp_WtEt_n
                                          //  Referenced by: '<S11>/TSamp'

  real_T TSamp_WtEt_a;                 // Computed Parameter: TSamp_WtEt_a
                                          //  Referenced by: '<S12>/TSamp'

  real_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<S2>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator_IC;    // Expression: 0
                                          //  Referenced by: '<S2>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator1_gainval;
                          // Computed Parameter: DiscreteTimeIntegrator1_gainval
                             //  Referenced by: '<S2>/Discrete-Time Integrator1'

  real_T DiscreteTimeIntegrator1_IC;   // Expression: 0
                                          //  Referenced by: '<S2>/Discrete-Time Integrator1'

  real_T DiscreteTimeIntegrator2_gainval;
                          // Computed Parameter: DiscreteTimeIntegrator2_gainval
                             //  Referenced by: '<S2>/Discrete-Time Integrator2'

  real_T DiscreteTimeIntegrator2_IC;   // Expression: 0
                                          //  Referenced by: '<S2>/Discrete-Time Integrator2'

  real_T Constant_Value_j[3];          // Expression: [25; 25; 25]
                                          //  Referenced by: '<Root>/Constant'

  real_T Constant1_Value[3];           // Expression: [5; 5; 5]
                                          //  Referenced by: '<Root>/Constant1'

  real_T Constant2_Value[3];           // Expression: [0.5; 0.5; 0.5]
                                          //  Referenced by: '<Root>/Constant2'

  real_T Constant_Value_b;             // Expression: 0.02
                                          //  Referenced by: '<S4>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_testros2_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    uint32_T checksums[4];
  } Sizes;

  //
  //  SpecialInfo:
  //  The following substructure contains special information
  //  related to other components that are dependent on RTW.

  struct {
    const void *mappingInfo;
  } SpecialInfo;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern P_testros2_T testros2_P;

#ifdef __cplusplus

}

#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_testros2_T testros2_B;

#ifdef __cplusplus

}

#endif

// Block states (default storage)
extern struct DW_testros2_T testros2_DW;

#ifdef __cplusplus

extern "C"
{

#endif

  // Model entry point functions
  extern void testros2_initialize(void);
  extern void testros2_step(void);
  extern void testros2_terminate(void);

#ifdef __cplusplus

}

#endif

// Real-time Model object
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_testros2_T *const testros2_M;

#ifdef __cplusplus

}

#endif

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Constant3' : Unused code path elimination
//  Block '<S10>/Data Type Duplicate' : Unused code path elimination
//  Block '<S11>/Data Type Duplicate' : Unused code path elimination
//  Block '<S12>/Data Type Duplicate' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'testros2'
//  '<S1>'   : 'testros2/Blank Message'
//  '<S2>'   : 'testros2/Kh&#x1ED1;i PID'
//  '<S3>'   : 'testros2/Kh&#x1ED1;i Te '
//  '<S4>'   : 'testros2/Kh&#x1ED1;i robot mecanum'
//  '<S5>'   : 'testros2/MATLAB Function'
//  '<S6>'   : 'testros2/MATLAB Function1'
//  '<S7>'   : 'testros2/Publish'
//  '<S8>'   : 'testros2/Qu&#x1EF9; &#x111;&#x1EA1;o th&#x1EF1;c t&#x1EBF;'
//  '<S9>'   : 'testros2/Subsystem1'
//  '<S10>'  : 'testros2/Kh&#x1ED1;i PID/Discrete Derivative'
//  '<S11>'  : 'testros2/Kh&#x1ED1;i PID/Discrete Derivative1'
//  '<S12>'  : 'testros2/Kh&#x1ED1;i PID/Discrete Derivative2'
//  '<S13>'  : 'testros2/Kh&#x1ED1;i PID/MATLAB Function'
//  '<S14>'  : 'testros2/Kh&#x1ED1;i Te /MATLAB Function'
//  '<S15>'  : 'testros2/Kh&#x1ED1;i robot mecanum/MATLAB Function'
//  '<S16>'  : 'testros2/Qu&#x1EF9; &#x111;&#x1EA1;o th&#x1EF1;c t&#x1EBF;/Enabled Subsystem'
//  '<S17>'  : 'testros2/Subsystem1/MATLAB Function'

#endif                                 // testros2_h_

//
// File trailer for generated code.
//
// [EOF]
//
