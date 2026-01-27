//
// File: testros2.h
//
// Code generated for Simulink model 'testros2'.
//
// Model version                  : 1.159
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Tue Jan 27 15:36:17 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef testros2_h_
#define testros2_h_
#include "rtwtypes.h"
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

#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Block signals (default storage)
struct B_testros2_T {
  real_T outputMFCache[505];
  SL_Bus_nav_msgs_Odometry In1;        // '<S30>/In1'
  SL_Bus_nav_msgs_Odometry rtb_SourceBlock_o2_m;
  real_T aggregatedOutputs[101];       // '<S20>/Evaluate Rule Consequents'
  real_T dv[101];
  real_T dv1[101];
  real_T dv2[101];
  real_T dv3[101];
  real_T dv4[101];
  real_T antecedentOutputs[25];        // '<S20>/Evaluate Rule Antecedents'
  real_T J_inner[12];
  real_T D_T[12];
  real_T inputMFCache[10];
  real_T dv5[9];
  SL_Bus_geometry_msgs_Twist BusAssignment;// '<Root>/Bus Assignment'
  real_T u_w[4];
  real_T S_mat[4];
  real_T B[3];
  real_T D_T_c[3];
  real_T dv6[2];
  real_T dx;
  real_T dy;
  real_T c;
  real_T s;
  real_T c_;
  real_T sumAntecedentOutputs;
  real_T theta;                        // '<S9>/MATLAB Function'
  real_T Sum;                          // '<Root>/Sum'
  real_T Sum1;                         // '<Root>/Sum1'
  real_T TSamp_a;                      // '<S13>/TSamp'
  real_T TSamp_p;                      // '<S14>/TSamp'
  real_T u_w_k;
  real_T rtb_Saturation_idx_0;
  real_T rtb_Saturation_idx_1;
  real_T rtb_Saturation_idx_2;
  real_T rtb_Saturation_idx_3;
  real_T rtb_Saturation_idx_4;
  real_T rtb_Saturation_idx_5;
  real_T rtb_defuzzifiedOutputs_idx_2;
  real_T DiscreteTimeIntegrator_DSTATE_n;
  real_T J_inner_tmp;
  real_T J_inner_tmp_c;
  real_T m3_tmp;
  real_T a;
  real_T b;
  real_T c_b;
  real_T x;
  int32_T sampleID;
  int32_T ruleID;
  int32_T i;
  int32_T i_p;
  int32_T i1;
  boolean_T b_varargout_1;
};

// Block states (default storage) for system '<Root>'
struct DW_testros2_T {
  ros_slros2_internal_block_Pub_T obj; // '<S7>/SinkBlock'
  ros_slros2_internal_block_Sub_T obj_n;// '<S8>/SourceBlock'
  real_T UD_DSTATE[3];                 // '<S17>/UD'
  real_T UD_DSTATE_h;                  // '<S12>/UD'
  real_T UD_DSTATE_l;                  // '<S13>/UD'
  real_T UD_DSTATE_d;                  // '<S14>/UD'
  real_T DiscreteTimeIntegrator_DSTATE;// '<S6>/Discrete-Time Integrator'
  real_T DiscreteTimeIntegrator1_DSTATE;// '<S6>/Discrete-Time Integrator1'
  real_T DiscreteTimeIntegrator2_DSTATE;// '<S6>/Discrete-Time Integrator2'
  real_T DiscreteTimeIntegrator_DSTATE_n[3];// '<S3>/Discrete-Time Integrator'
  real_T DiscreteTimeIntegrator1_DSTAT_m[3];// '<S3>/Discrete-Time Integrator1'
  real_T start_x;                      // '<S9>/MATLAB Function1'
  real_T start_y;                      // '<S9>/MATLAB Function1'
  real_T start_th;                     // '<S9>/MATLAB Function1'
  boolean_T initialized_not_empty;     // '<S9>/MATLAB Function1'
};

// Parameters (default storage)
struct P_testros2_T_ {
  real_T DiscreteDerivative_ICPrevScaled;
                              // Mask Parameter: DiscreteDerivative_ICPrevScaled
                                 //  Referenced by: '<S17>/UD'

  real_T DiscreteDerivative_ICPrevScal_p;
                              // Mask Parameter: DiscreteDerivative_ICPrevScal_p
                                 //  Referenced by: '<S12>/UD'

  real_T DiscreteDerivative1_ICPrevScale;
                              // Mask Parameter: DiscreteDerivative1_ICPrevScale
                                 //  Referenced by: '<S13>/UD'

  real_T DiscreteDerivative2_ICPrevScale;
                              // Mask Parameter: DiscreteDerivative2_ICPrevScale
                                 //  Referenced by: '<S14>/UD'

  SL_Bus_nav_msgs_Odometry Out1_Y0;    // Computed Parameter: Out1_Y0
                                          //  Referenced by: '<S30>/Out1'

  SL_Bus_nav_msgs_Odometry Constant_Value;// Computed Parameter: Constant_Value
                                             //  Referenced by: '<S8>/Constant'

  SL_Bus_geometry_msgs_Twist Constant_Value_d;// Computed Parameter: Constant_Value_d
                                                 //  Referenced by: '<S2>/Constant'

  real_T OutputSamplePoints_Value[101];// Expression: fis.outputSamplePoints
                                          //  Referenced by: '<S18>/Output Sample Points'

  real_T OutputSamplePoints_Value_o[101];// Expression: fis.outputSamplePoints
                                            //  Referenced by: '<S19>/Output Sample Points'

  real_T OutputSamplePoints_Value_j[101];// Expression: fis.outputSamplePoints
                                            //  Referenced by: '<S20>/Output Sample Points'

  real_T TSamp_WtEt;                   // Computed Parameter: TSamp_WtEt
                                          //  Referenced by: '<S17>/TSamp'

  real_T Saturation_UpperSat;          // Expression: 1
                                          //  Referenced by: '<S15>/Saturation'

  real_T Saturation_LowerSat;          // Expression: -1
                                          //  Referenced by: '<S15>/Saturation'

  real_T TSamp_WtEt_o;                 // Computed Parameter: TSamp_WtEt_o
                                          //  Referenced by: '<S12>/TSamp'

  real_T TSamp_WtEt_l;                 // Computed Parameter: TSamp_WtEt_l
                                          //  Referenced by: '<S13>/TSamp'

  real_T TSamp_WtEt_le;                // Computed Parameter: TSamp_WtEt_le
                                          //  Referenced by: '<S14>/TSamp'

  real_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<S6>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator_IC;    // Expression: 0
                                          //  Referenced by: '<S6>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator1_gainval;
                          // Computed Parameter: DiscreteTimeIntegrator1_gainval
                             //  Referenced by: '<S6>/Discrete-Time Integrator1'

  real_T DiscreteTimeIntegrator1_IC;   // Expression: 0
                                          //  Referenced by: '<S6>/Discrete-Time Integrator1'

  real_T DiscreteTimeIntegrator2_gainval;
                          // Computed Parameter: DiscreteTimeIntegrator2_gainval
                             //  Referenced by: '<S6>/Discrete-Time Integrator2'

  real_T DiscreteTimeIntegrator2_IC;   // Expression: 0
                                          //  Referenced by: '<S6>/Discrete-Time Integrator2'

  real_T DiscreteTimeIntegrator_gainva_a;
                          // Computed Parameter: DiscreteTimeIntegrator_gainva_a
                             //  Referenced by: '<S3>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator_IC_l;  // Expression: 0
                                          //  Referenced by: '<S3>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator1_gainv_p;
                          // Computed Parameter: DiscreteTimeIntegrator1_gainv_p
                             //  Referenced by: '<S3>/Discrete-Time Integrator1'

  real_T DiscreteTimeIntegrator1_IC_o; // Expression: 0
                                          //  Referenced by: '<S3>/Discrete-Time Integrator1'

  real_T Saturation_UpperSat_c;        // Expression: 0.2
                                          //  Referenced by: '<S3>/Saturation'

  real_T Saturation_LowerSat_a;        // Expression: -0.2
                                          //  Referenced by: '<S3>/Saturation'

  real_T Saturation1_UpperSat;         // Expression: 0.2
                                          //  Referenced by: '<S3>/Saturation1'

  real_T Saturation1_LowerSat;         // Expression: -0.2
                                          //  Referenced by: '<S3>/Saturation1'

  real_T Saturation2_UpperSat;         // Expression: 0.5
                                          //  Referenced by: '<S3>/Saturation2'

  real_T Saturation2_LowerSat;         // Expression: -0.5
                                          //  Referenced by: '<S3>/Saturation2'

};

// Real-time Model Data Structure
struct tag_RTM_testros2_T {
  const char_T * volatile errorStatus;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
  } Timing;
};

// Class declaration for model testros2
class testros2
{
  // public data and function members
 public:
  // Real-Time Model get method
  RT_MODEL_testros2_T * getRTM();

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  testros2();

  // Destructor
  ~testros2();

  // private data and function members
 private:
  // Block signals
  B_testros2_T testros2_B;

  // Block states
  DW_testros2_T testros2_DW;

  // Tunable parameters
  static P_testros2_T testros2_P;

  // private member function(s) for subsystem '<Root>'
  real_T testros2_rt_atan2d_snf(real_T u0, real_T u1);
  real_T testros2_trimf(real_T x, const real_T params[3]);
  void testros2_trimf_p(const real_T x[101], const real_T params[3], real_T y
                        [101]);
  void testros2_Subscriber_setupImpl(const ros_slros2_internal_block_Sub_T *obj);
  void testros2_Publisher_setupImpl(const ros_slros2_internal_block_Pub_T *obj);

  // Real-Time Model
  RT_MODEL_testros2_T testros2_M;
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Constant' : Unused code path elimination
//  Block '<Root>/Constant1' : Unused code path elimination
//  Block '<Root>/Constant2' : Unused code path elimination
//  Block '<Root>/Constant3' : Unused code path elimination
//  Block '<Root>/Constant4' : Unused code path elimination
//  Block '<Root>/Display' : Unused code path elimination
//  Block '<Root>/Display1' : Unused code path elimination
//  Block '<Root>/Display2' : Unused code path elimination
//  Block '<Root>/Display3' : Unused code path elimination
//  Block '<Root>/Display4' : Unused code path elimination
//  Block '<Root>/Display5' : Unused code path elimination
//  Block '<Root>/Display6' : Unused code path elimination
//  Block '<Root>/Display7' : Unused code path elimination
//  Block '<Root>/Display8' : Unused code path elimination
//  Block '<S3>/Scope' : Unused code path elimination
//  Block '<S3>/Scope1' : Unused code path elimination
//  Block '<S6>/Constant' : Unused code path elimination
//  Block '<S6>/Constant1' : Unused code path elimination
//  Block '<S6>/Constant2' : Unused code path elimination
//  Block '<S12>/Data Type Duplicate' : Unused code path elimination
//  Block '<S13>/Data Type Duplicate' : Unused code path elimination
//  Block '<S14>/Data Type Duplicate' : Unused code path elimination
//  Block '<S6>/Display' : Unused code path elimination
//  Block '<S6>/Display1' : Unused code path elimination
//  Block '<S6>/Display2' : Unused code path elimination
//  Block '<S17>/Data Type Duplicate' : Unused code path elimination
//  Block '<S15>/Scope' : Unused code path elimination
//  Block '<Root>/Scope' : Unused code path elimination
//  Block '<Root>/Scope2' : Unused code path elimination
//  Block '<Root>/odom ' : Unused code path elimination
//  Block '<S18>/InputConversion' : Eliminate redundant data type conversion
//  Block '<S19>/InputConversion' : Eliminate redundant data type conversion
//  Block '<S20>/InputConversion' : Eliminate redundant data type conversion


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
//  '<S1>'   : 'testros2/ chuy&#x1EC3;n &#x111;&#x1ED5;i (Te)'
//  '<S2>'   : 'testros2/Blank Message'
//  '<S3>'   : 'testros2/Kh&#x1ED1;i robot mecanum'
//  '<S4>'   : 'testros2/MATLAB Function'
//  '<S5>'   : 'testros2/MATLAB Function1'
//  '<S6>'   : 'testros2/PID'
//  '<S7>'   : 'testros2/Publish'
//  '<S8>'   : 'testros2/Qu&#x1EF9; &#x111;&#x1EA1;o th&#x1EF1;c t&#x1EBF;'
//  '<S9>'   : 'testros2/Subsystem1'
//  '<S10>'  : 'testros2/ chuy&#x1EC3;n &#x111;&#x1ED5;i (Te)/MATLAB Function'
//  '<S11>'  : 'testros2/Kh&#x1ED1;i robot mecanum/MATLAB Function'
//  '<S12>'  : 'testros2/PID/Discrete Derivative'
//  '<S13>'  : 'testros2/PID/Discrete Derivative1'
//  '<S14>'  : 'testros2/PID/Discrete Derivative2'
//  '<S15>'  : 'testros2/PID/Subsystem'
//  '<S16>'  : 'testros2/PID/matlab function'
//  '<S17>'  : 'testros2/PID/Subsystem/Discrete Derivative'
//  '<S18>'  : 'testros2/PID/Subsystem/Fuzzy_Kd'
//  '<S19>'  : 'testros2/PID/Subsystem/Fuzzy_Ki'
//  '<S20>'  : 'testros2/PID/Subsystem/Fuzzy_Kp'
//  '<S21>'  : 'testros2/PID/Subsystem/Fuzzy_Kd/Defuzzify Outputs'
//  '<S22>'  : 'testros2/PID/Subsystem/Fuzzy_Kd/Evaluate Rule Antecedents'
//  '<S23>'  : 'testros2/PID/Subsystem/Fuzzy_Kd/Evaluate Rule Consequents'
//  '<S24>'  : 'testros2/PID/Subsystem/Fuzzy_Ki/Defuzzify Outputs'
//  '<S25>'  : 'testros2/PID/Subsystem/Fuzzy_Ki/Evaluate Rule Antecedents'
//  '<S26>'  : 'testros2/PID/Subsystem/Fuzzy_Ki/Evaluate Rule Consequents'
//  '<S27>'  : 'testros2/PID/Subsystem/Fuzzy_Kp/Defuzzify Outputs'
//  '<S28>'  : 'testros2/PID/Subsystem/Fuzzy_Kp/Evaluate Rule Antecedents'
//  '<S29>'  : 'testros2/PID/Subsystem/Fuzzy_Kp/Evaluate Rule Consequents'
//  '<S30>'  : 'testros2/Qu&#x1EF9; &#x111;&#x1EA1;o th&#x1EF1;c t&#x1EBF;/Enabled Subsystem'
//  '<S31>'  : 'testros2/Subsystem1/MATLAB Function'
//  '<S32>'  : 'testros2/Subsystem1/MATLAB Function1'

#endif                                 // testros2_h_

//
// File trailer for generated code.
//
// [EOF]
//
