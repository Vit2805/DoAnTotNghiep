//
// File: testros2.cpp
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
#include "testros2.h"
#include "rtwtypes.h"
#include "testros2_types.h"
#include <math.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include <emmintrin.h>
#include "rmw/qos_profiles.h"
#include <stddef.h>
#include "rt_defines.h"

real_T testros2::testros2_rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      testros2_B.i_p = 1;
    } else {
      testros2_B.i_p = -1;
    }

    if (u1 > 0.0) {
      testros2_B.i1 = 1;
    } else {
      testros2_B.i1 = -1;
    }

    y = atan2(static_cast<real_T>(testros2_B.i_p), static_cast<real_T>
              (testros2_B.i1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

// Function for MATLAB Function: '<S18>/Evaluate Rule Antecedents'
real_T testros2::testros2_trimf(real_T x, const real_T params[3])
{
  real_T y;
  y = 0.0;
  if ((params[0] != params[1]) && (params[0] < x) && (x < params[1])) {
    y = 1.0 / (params[1] - params[0]) * (x - params[0]);
  }

  if ((params[1] != params[2]) && (params[1] < x) && (x < params[2])) {
    y = 1.0 / (params[2] - params[1]) * (params[2] - x);
  }

  if (x == params[1]) {
    y = 1.0;
  }

  return y;
}

// Function for MATLAB Function: '<S18>/Evaluate Rule Consequents'
void testros2::testros2_trimf_p(const real_T x[101], const real_T params[3],
  real_T y[101])
{
  testros2_B.a = params[0];
  testros2_B.b = params[1];
  testros2_B.c_b = params[2];
  for (testros2_B.i = 0; testros2_B.i < 101; testros2_B.i++) {
    y[testros2_B.i] = 0.0;
    if (testros2_B.a != testros2_B.b) {
      testros2_B.x = x[testros2_B.i];
      if ((testros2_B.a < testros2_B.x) && (testros2_B.x < testros2_B.b)) {
        y[testros2_B.i] = 1.0 / (testros2_B.b - testros2_B.a) * (testros2_B.x -
          testros2_B.a);
      }
    }

    if (testros2_B.b != testros2_B.c_b) {
      testros2_B.x = x[testros2_B.i];
      if ((testros2_B.b < testros2_B.x) && (testros2_B.x < testros2_B.c_b)) {
        y[testros2_B.i] = 1.0 / (testros2_B.c_b - testros2_B.b) *
          (testros2_B.c_b - testros2_B.x);
      }
    }

    if (x[testros2_B.i] == testros2_B.b) {
      y[testros2_B.i] = 1.0;
    }
  }
}

void testros2::testros2_Subscriber_setupImpl(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[6];
  static const char_T b_zeroDelimTopic_0[6] = "/odom";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S8>/SourceBlock'
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 6; i++) {
    // Start for MATLABSystem: '<S8>/SourceBlock'
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_testros2_63.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
}

void testros2::testros2_Publisher_setupImpl(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[9];
  static const char_T b_zeroDelimTopic_0[9] = "/cmd_vel";
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S7>/SinkBlock'
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 9; i++) {
    // Start for MATLABSystem: '<S7>/SinkBlock'
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_testros2_62.createPublisher(&b_zeroDelimTopic[0], qos_profile);
}

// Model step function
void testros2::step()
{
  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  __m128d tmp_4;
  static const real_T k[3] = { -1.833, -1.0, -0.1667 };

  static const real_T j[3] = { -0.8376, -0.004228, 0.8291 };

  static const real_T i[3] = { 0.166, 1.0, 1.84 };

  static const real_T h[3] = { -1.992, -0.9915, 0.008457 };

  static const real_T g[3] = { -0.9958, 0.004228, 1.004 };

  static const real_T f[3] = { -7.333, -4.0, -0.6667 };

  static const real_T e[3] = { -3.35, -0.01691, 3.316 };

  static const real_T d[3] = { 0.6667, 4.0, 7.333 };

  static const real_T c[3] = { -3.983, 0.01691, 4.017 };

  static const real_T e_0[3] = { -21.04, -0.2114, 20.62 };

  static const int8_T b[50] = { 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1,
    2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4,
    4, 4, 4, 5, 5, 5, 5, 5 };

  static const real_T d_0[3] = { 4.272, 25.11, 45.94 };

  static const real_T c_0[3] = { 29.27, 50.11, 70.94 };

  static const int8_T b_0[25] = { 3, 2, 1, 2, 3, 4, 3, 2, 3, 4, 1, 4, 3, 4, 1, 4,
    3, 2, 3, 4, 3, 2, 1, 2, 3 };

  static const real_T f_0[3] = { 0.1667, 1.0, 1.833 };

  static const real_T e_1[3] = { -3.00422832980973, -1.00422832980973,
    0.995771670190275 };

  static const real_T d_1[3] = { -7.332, -4.0, -0.6668 };

  static const real_T c_1[3] = { 0.6668, 4.0, 7.332 };

  static const real_T d_2[3] = { -2.07242917547569, 0.0105708245243132,
    2.09357082452431 };

  static const real_T c_2[3] = { 0.4167, 2.5, 4.583 };

  static const int8_T b_1[25] = { 1, 1, 2, 2, 3, 1, 2, 3, 3, 4, 2, 3, 5, 3, 2, 4,
    3, 3, 2, 1, 3, 2, 2, 1, 1 };

  static const real_T e_2[3] = { -0.9958, 1.004, 3.004 };

  static const real_T e_3[3] = { -4.18814164904863, -0.0211416490486256,
    4.14585835095137 };

  static const real_T d_3[3] = { 0.8333, 5.0, 9.167 };

  static const real_T c_3[3] = { 5.833, 10.0, 14.17 };

  static const int8_T b_2[25] = { 1, 2, 3, 2, 1, 2, 3, 4, 3, 2, 3, 4, 1, 4, 3, 2,
    3, 4, 3, 2, 1, 2, 3, 2, 1 };

  static const real_T a[12] = { 13.333333333333334, 13.333333333333334,
    13.333333333333334, 13.333333333333334, -13.333333333333334,
    13.333333333333334, -13.333333333333334, 13.333333333333334, -6.4, 6.4, 6.4,
    -6.4 };

  // MATLABSystem: '<S8>/SourceBlock'
  testros2_B.b_varargout_1 = Sub_testros2_63.getLatestMessage
    (&testros2_B.rtb_SourceBlock_o2_m);

  // Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S30>/Enable'

  // Start for MATLABSystem: '<S8>/SourceBlock'
  if (testros2_B.b_varargout_1) {
    // SignalConversion generated from: '<S30>/In1'
    testros2_B.In1 = testros2_B.rtb_SourceBlock_o2_m;
  }

  // End of Start for MATLABSystem: '<S8>/SourceBlock'
  // End of Outputs for SubSystem: '<S8>/Enabled Subsystem'

  // MATLAB Function: '<S9>/MATLAB Function'
  testros2_B.theta = 2.0 * testros2_rt_atan2d_snf
    (testros2_B.In1.pose.pose.orientation.z,
     testros2_B.In1.pose.pose.orientation.w);

  // DigitalClock: '<S9>/Digital Clock' incorporates:
  //   DigitalClock: '<Root>/Digital Clock'

  testros2_B.rtb_Saturation_idx_2 = (((&testros2_M)->Timing.clockTick0) * 0.02);

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   Constant: '<S2>/Constant'

  testros2_B.BusAssignment = testros2_P.Constant_Value_d;

  // MATLAB Function: '<S9>/MATLAB Function1' incorporates:
  //   DigitalClock: '<S9>/Digital Clock'

  if ((!testros2_DW.initialized_not_empty) || (testros2_B.rtb_Saturation_idx_2 <
       0.01)) {
    testros2_DW.start_x = testros2_B.In1.pose.pose.position.x;
    testros2_DW.start_y = testros2_B.In1.pose.pose.position.y;
    testros2_DW.start_th = testros2_B.theta;
    testros2_DW.initialized_not_empty = true;
  }

  testros2_B.dx = testros2_B.In1.pose.pose.position.x - testros2_DW.start_x;
  testros2_B.dy = testros2_B.In1.pose.pose.position.y - testros2_DW.start_y;
  testros2_B.c = cos(-testros2_DW.start_th);
  testros2_B.s = sin(-testros2_DW.start_th);
  testros2_B.theta -= testros2_DW.start_th;
  testros2_B.theta = testros2_rt_atan2d_snf(sin(testros2_B.theta), cos
    (testros2_B.theta));

  // Sum: '<Root>/Sum' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function'
  //   MATLAB Function: '<S9>/MATLAB Function1'

  testros2_B.Sum = 0.5 * testros2_B.rtb_Saturation_idx_2 - (testros2_B.dx *
    testros2_B.c - testros2_B.dy * testros2_B.s);

  // Sum: '<Root>/Sum1' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function'
  //   MATLAB Function: '<S9>/MATLAB Function1'

  testros2_B.Sum1 = 0.0 - (testros2_B.dx * testros2_B.s + testros2_B.dy *
    testros2_B.c);

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function'
  //   Sum: '<Root>/Sum2'

  testros2_B.dx = testros2_rt_atan2d_snf(sin(0.0 - testros2_B.theta), cos(0.0 -
    testros2_B.theta));

  // MATLAB Function: '<S1>/MATLAB Function'
  testros2_B.c = cos(testros2_B.theta);
  testros2_B.s = sin(testros2_B.theta);
  testros2_B.dy = testros2_B.Sum * testros2_B.c + testros2_B.Sum1 * testros2_B.s;
  testros2_B.c = -testros2_B.Sum * testros2_B.s + testros2_B.Sum1 * testros2_B.c;

  // SampleTimeMath: '<S17>/TSamp' incorporates:
  //   MATLAB Function: '<S1>/MATLAB Function'
  //
  //  About '<S17>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  //
  testros2_B.s = testros2_B.dy * testros2_P.TSamp_WtEt;
  testros2_B.theta = testros2_B.c * testros2_P.TSamp_WtEt;
  testros2_B.Sum = testros2_B.dx * testros2_P.TSamp_WtEt;

  // Saturate: '<S15>/Saturation'
  if (testros2_B.dy > testros2_P.Saturation_UpperSat) {
    testros2_B.rtb_Saturation_idx_0 = testros2_P.Saturation_UpperSat;
  } else if (testros2_B.dy < testros2_P.Saturation_LowerSat) {
    testros2_B.rtb_Saturation_idx_0 = testros2_P.Saturation_LowerSat;
  } else {
    testros2_B.rtb_Saturation_idx_0 = testros2_B.dy;
  }

  if (testros2_B.c > testros2_P.Saturation_UpperSat) {
    testros2_B.rtb_Saturation_idx_1 = testros2_P.Saturation_UpperSat;
  } else if (testros2_B.c < testros2_P.Saturation_LowerSat) {
    testros2_B.rtb_Saturation_idx_1 = testros2_P.Saturation_LowerSat;
  } else {
    testros2_B.rtb_Saturation_idx_1 = testros2_B.c;
  }

  if (testros2_B.dx > testros2_P.Saturation_UpperSat) {
    testros2_B.rtb_Saturation_idx_2 = testros2_P.Saturation_UpperSat;
  } else if (testros2_B.dx < testros2_P.Saturation_LowerSat) {
    testros2_B.rtb_Saturation_idx_2 = testros2_P.Saturation_LowerSat;
  } else {
    testros2_B.rtb_Saturation_idx_2 = testros2_B.dx;
  }

  // Sum: '<S17>/Diff' incorporates:
  //   UnitDelay: '<S17>/UD'
  //
  //  Block description for '<S17>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S17>/UD':
  //
  //   Store in Global RAM

  testros2_B.rtb_Saturation_idx_3 = testros2_B.s - testros2_DW.UD_DSTATE[0];

  // Saturate: '<S15>/Saturation'
  if (testros2_B.rtb_Saturation_idx_3 > testros2_P.Saturation_UpperSat) {
    testros2_B.rtb_Saturation_idx_3 = testros2_P.Saturation_UpperSat;
  } else if (testros2_B.rtb_Saturation_idx_3 < testros2_P.Saturation_LowerSat) {
    testros2_B.rtb_Saturation_idx_3 = testros2_P.Saturation_LowerSat;
  }

  // Sum: '<S17>/Diff' incorporates:
  //   UnitDelay: '<S17>/UD'
  //
  //  Block description for '<S17>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S17>/UD':
  //
  //   Store in Global RAM

  testros2_B.rtb_Saturation_idx_4 = testros2_B.theta - testros2_DW.UD_DSTATE[1];

  // Saturate: '<S15>/Saturation'
  if (testros2_B.rtb_Saturation_idx_4 > testros2_P.Saturation_UpperSat) {
    testros2_B.rtb_Saturation_idx_4 = testros2_P.Saturation_UpperSat;
  } else if (testros2_B.rtb_Saturation_idx_4 < testros2_P.Saturation_LowerSat) {
    testros2_B.rtb_Saturation_idx_4 = testros2_P.Saturation_LowerSat;
  }

  // Sum: '<S17>/Diff' incorporates:
  //   UnitDelay: '<S17>/UD'
  //
  //  Block description for '<S17>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S17>/UD':
  //
  //   Store in Global RAM

  testros2_B.rtb_Saturation_idx_5 = testros2_B.Sum - testros2_DW.UD_DSTATE[2];

  // Saturate: '<S15>/Saturation'
  if (testros2_B.rtb_Saturation_idx_5 > testros2_P.Saturation_UpperSat) {
    testros2_B.rtb_Saturation_idx_5 = testros2_P.Saturation_UpperSat;
  } else if (testros2_B.rtb_Saturation_idx_5 < testros2_P.Saturation_LowerSat) {
    testros2_B.rtb_Saturation_idx_5 = testros2_P.Saturation_LowerSat;
  }

  // SampleTimeMath: '<S12>/TSamp'
  //
  //  About '<S12>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  //
  testros2_B.Sum1 = testros2_B.dy * testros2_P.TSamp_WtEt_o;

  // SampleTimeMath: '<S13>/TSamp'
  //
  //  About '<S13>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  //
  testros2_B.TSamp_a = testros2_B.c * testros2_P.TSamp_WtEt_l;

  // SampleTimeMath: '<S14>/TSamp' incorporates:
  //   MATLAB Function: '<S1>/MATLAB Function'
  //
  //  About '<S14>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  //
  testros2_B.TSamp_p = testros2_B.dx * testros2_P.TSamp_WtEt_le;

  // Outputs for Atomic SubSystem: '<S15>/Fuzzy_Kp'
  // MATLAB Function: '<S20>/Evaluate Rule Antecedents'
  testros2_B.sumAntecedentOutputs = 0.0;
  testros2_B.inputMFCache[0] = testros2_trimf(testros2_B.rtb_Saturation_idx_0, k);
  testros2_B.inputMFCache[1] = testros2_trimf(testros2_B.rtb_Saturation_idx_0, j);
  testros2_B.inputMFCache[2] = testros2_trimf(testros2_B.rtb_Saturation_idx_0, i);
  testros2_B.inputMFCache[3] = testros2_trimf(testros2_B.rtb_Saturation_idx_0, h);
  testros2_B.inputMFCache[4] = testros2_trimf(testros2_B.rtb_Saturation_idx_0, g);
  testros2_B.inputMFCache[5] = testros2_trimf(testros2_B.rtb_Saturation_idx_1, f);
  testros2_B.inputMFCache[6] = testros2_trimf(testros2_B.rtb_Saturation_idx_1, e);
  testros2_B.inputMFCache[7] = testros2_trimf(testros2_B.rtb_Saturation_idx_1, d);
  testros2_B.D_T_c[0] = -8.0;
  testros2_B.D_T_c[1] = -4.0;
  testros2_B.D_T_c[2] = 0.0;
  testros2_B.inputMFCache[8] = testros2_trimf(testros2_B.rtb_Saturation_idx_1,
    testros2_B.D_T_c);
  testros2_B.inputMFCache[9] = testros2_trimf(testros2_B.rtb_Saturation_idx_1, c);
  for (testros2_B.ruleID = 0; testros2_B.ruleID < 25; testros2_B.ruleID++) {
    testros2_B.c_ = 1.0;
    testros2_B.rtb_Saturation_idx_1 =
      testros2_B.inputMFCache[b[testros2_B.ruleID] - 1];
    if (testros2_B.rtb_Saturation_idx_1 < 1.0) {
      testros2_B.c_ = testros2_B.rtb_Saturation_idx_1;
    }

    testros2_B.rtb_Saturation_idx_1 =
      testros2_B.inputMFCache[b[testros2_B.ruleID + 25] + 4];
    if (testros2_B.c_ > testros2_B.rtb_Saturation_idx_1) {
      testros2_B.c_ = testros2_B.rtb_Saturation_idx_1;
    }

    testros2_B.antecedentOutputs[testros2_B.ruleID] = testros2_B.c_;
    testros2_B.sumAntecedentOutputs += testros2_B.c_;
  }

  // MATLAB Function: '<S20>/Evaluate Rule Consequents' incorporates:
  //   Constant: '<S20>/Output Sample Points'

  testros2_trimf_p(testros2_P.OutputSamplePoints_Value_j, e_0, testros2_B.dv);
  testros2_trimf_p(testros2_P.OutputSamplePoints_Value_j, d_0, testros2_B.dv1);
  testros2_trimf_p(testros2_P.OutputSamplePoints_Value_j, c_0, testros2_B.dv2);
  testros2_B.D_T_c[0] = -25.0;
  testros2_B.D_T_c[1] = 0.0;
  testros2_B.D_T_c[2] = 25.0;
  testros2_trimf_p(testros2_P.OutputSamplePoints_Value_j, testros2_B.D_T_c,
                   testros2_B.dv3);
  testros2_B.D_T_c[0] = 0.0;
  testros2_B.D_T_c[1] = 25.0;
  testros2_B.D_T_c[2] = 50.0;
  testros2_trimf_p(testros2_P.OutputSamplePoints_Value_j, testros2_B.D_T_c,
                   testros2_B.dv4);
  for (testros2_B.ruleID = 0; testros2_B.ruleID < 101; testros2_B.ruleID++) {
    testros2_B.aggregatedOutputs[testros2_B.ruleID] = 0.0;
    testros2_B.outputMFCache[5 * testros2_B.ruleID] =
      testros2_B.dv[testros2_B.ruleID];
    testros2_B.outputMFCache[5 * testros2_B.ruleID + 1] =
      testros2_B.dv1[testros2_B.ruleID];
    testros2_B.outputMFCache[5 * testros2_B.ruleID + 2] =
      testros2_B.dv2[testros2_B.ruleID];
    testros2_B.outputMFCache[5 * testros2_B.ruleID + 3] =
      testros2_B.dv3[testros2_B.ruleID];
    testros2_B.outputMFCache[5 * testros2_B.ruleID + 4] =
      testros2_B.dv4[testros2_B.ruleID];
  }

  for (testros2_B.ruleID = 0; testros2_B.ruleID < 25; testros2_B.ruleID++) {
    for (testros2_B.sampleID = 0; testros2_B.sampleID < 101; testros2_B.sampleID
         ++) {
      testros2_B.c_ = testros2_B.outputMFCache[(5 * testros2_B.sampleID +
        b_0[testros2_B.ruleID]) - 1];
      testros2_B.rtb_Saturation_idx_1 =
        testros2_B.antecedentOutputs[testros2_B.ruleID];
      if ((!(testros2_B.c_ > testros2_B.rtb_Saturation_idx_1)) && (!rtIsNaN
           (testros2_B.c_))) {
        testros2_B.rtb_Saturation_idx_1 = testros2_B.c_;
      }

      if (testros2_B.aggregatedOutputs[testros2_B.sampleID] <
          testros2_B.rtb_Saturation_idx_1) {
        testros2_B.aggregatedOutputs[testros2_B.sampleID] =
          testros2_B.rtb_Saturation_idx_1;
      }
    }
  }

  // End of MATLAB Function: '<S20>/Evaluate Rule Consequents'

  // MATLAB Function: '<S20>/Defuzzify Outputs' incorporates:
  //   Constant: '<S20>/Output Sample Points'
  //   MATLAB Function: '<S20>/Evaluate Rule Antecedents'

  if (testros2_B.sumAntecedentOutputs == 0.0) {
    testros2_B.rtb_Saturation_idx_0 = 25.0;
  } else {
    testros2_B.sumAntecedentOutputs = 0.0;
    testros2_B.rtb_Saturation_idx_1 = 0.0;
    for (testros2_B.ruleID = 0; testros2_B.ruleID < 101; testros2_B.ruleID++) {
      testros2_B.rtb_Saturation_idx_1 +=
        testros2_B.aggregatedOutputs[testros2_B.ruleID];
    }

    if (testros2_B.rtb_Saturation_idx_1 == 0.0) {
      testros2_B.rtb_Saturation_idx_0 = (testros2_P.OutputSamplePoints_Value_j[0]
        + testros2_P.OutputSamplePoints_Value_j[100]) / 2.0;
    } else {
      for (testros2_B.ruleID = 0; testros2_B.ruleID < 101; testros2_B.ruleID++)
      {
        testros2_B.sumAntecedentOutputs +=
          testros2_P.OutputSamplePoints_Value_j[testros2_B.ruleID] *
          testros2_B.aggregatedOutputs[testros2_B.ruleID];
      }

      testros2_B.rtb_Saturation_idx_0 = 1.0 / testros2_B.rtb_Saturation_idx_1 *
        testros2_B.sumAntecedentOutputs;
    }
  }

  // End of MATLAB Function: '<S20>/Defuzzify Outputs'
  // End of Outputs for SubSystem: '<S15>/Fuzzy_Kp'

  // Outputs for Atomic SubSystem: '<S15>/Fuzzy_Ki'
  // MATLAB Function: '<S19>/Evaluate Rule Antecedents'
  testros2_B.sumAntecedentOutputs = 0.0;
  testros2_B.inputMFCache[0] = testros2_trimf(testros2_B.rtb_Saturation_idx_2, k);
  testros2_B.D_T_c[0] = -0.8333;
  testros2_B.D_T_c[1] = 0.0;
  testros2_B.D_T_c[2] = 0.8333;
  testros2_B.inputMFCache[1] = testros2_trimf(testros2_B.rtb_Saturation_idx_2,
    testros2_B.D_T_c);
  testros2_B.inputMFCache[2] = testros2_trimf(testros2_B.rtb_Saturation_idx_2,
    f_0);
  testros2_B.inputMFCache[3] = testros2_trimf(testros2_B.rtb_Saturation_idx_2,
    e_1);
  testros2_B.D_T_c[0] = -1.0;
  testros2_B.D_T_c[1] = 1.0;
  testros2_B.D_T_c[2] = 3.0;
  testros2_B.inputMFCache[4] = testros2_trimf(testros2_B.rtb_Saturation_idx_2,
    testros2_B.D_T_c);
  testros2_B.inputMFCache[5] = testros2_trimf(testros2_B.rtb_Saturation_idx_3,
    d_1);
  testros2_B.D_T_c[0] = -3.333;
  testros2_B.D_T_c[1] = 0.0;
  testros2_B.D_T_c[2] = 3.333;
  testros2_B.inputMFCache[6] = testros2_trimf(testros2_B.rtb_Saturation_idx_3,
    testros2_B.D_T_c);
  testros2_B.inputMFCache[7] = testros2_trimf(testros2_B.rtb_Saturation_idx_3,
    c_1);
  testros2_B.D_T_c[0] = -12.0;
  testros2_B.D_T_c[1] = -4.0;
  testros2_B.D_T_c[2] = 4.0;
  testros2_B.inputMFCache[8] = testros2_trimf(testros2_B.rtb_Saturation_idx_3,
    testros2_B.D_T_c);
  testros2_B.D_T_c[0] = -4.0;
  testros2_B.D_T_c[1] = 4.0;
  testros2_B.D_T_c[2] = 12.0;
  testros2_B.inputMFCache[9] = testros2_trimf(testros2_B.rtb_Saturation_idx_3,
    testros2_B.D_T_c);
  for (testros2_B.ruleID = 0; testros2_B.ruleID < 25; testros2_B.ruleID++) {
    testros2_B.c_ = 1.0;
    testros2_B.rtb_Saturation_idx_1 =
      testros2_B.inputMFCache[b[testros2_B.ruleID] - 1];
    if (testros2_B.rtb_Saturation_idx_1 < 1.0) {
      testros2_B.c_ = testros2_B.rtb_Saturation_idx_1;
    }

    testros2_B.rtb_Saturation_idx_1 =
      testros2_B.inputMFCache[b[testros2_B.ruleID + 25] + 4];
    if (testros2_B.c_ > testros2_B.rtb_Saturation_idx_1) {
      testros2_B.c_ = testros2_B.rtb_Saturation_idx_1;
    }

    testros2_B.antecedentOutputs[testros2_B.ruleID] = testros2_B.c_;
    testros2_B.sumAntecedentOutputs += testros2_B.c_;
  }

  // MATLAB Function: '<S19>/Evaluate Rule Consequents' incorporates:
  //   Constant: '<S19>/Output Sample Points'

  testros2_trimf_p(testros2_P.OutputSamplePoints_Value_o, d_2, testros2_B.dv);
  testros2_trimf_p(testros2_P.OutputSamplePoints_Value_o, c_2, testros2_B.dv1);
  testros2_B.D_T_c[0] = 2.917;
  testros2_B.D_T_c[1] = 5.0;
  testros2_B.D_T_c[2] = 7.083;
  testros2_trimf_p(testros2_P.OutputSamplePoints_Value_o, testros2_B.D_T_c,
                   testros2_B.dv2);
  testros2_B.D_T_c[0] = -5.0;
  testros2_B.D_T_c[1] = 0.0;
  testros2_B.D_T_c[2] = 5.0;
  testros2_trimf_p(testros2_P.OutputSamplePoints_Value_o, testros2_B.D_T_c,
                   testros2_B.dv3);
  testros2_B.D_T_c[0] = 0.0;
  testros2_B.D_T_c[1] = 5.0;
  testros2_B.D_T_c[2] = 10.0;
  testros2_trimf_p(testros2_P.OutputSamplePoints_Value_o, testros2_B.D_T_c,
                   testros2_B.dv4);
  for (testros2_B.ruleID = 0; testros2_B.ruleID < 101; testros2_B.ruleID++) {
    testros2_B.aggregatedOutputs[testros2_B.ruleID] = 0.0;
    testros2_B.outputMFCache[5 * testros2_B.ruleID] =
      testros2_B.dv[testros2_B.ruleID];
    testros2_B.outputMFCache[5 * testros2_B.ruleID + 1] =
      testros2_B.dv1[testros2_B.ruleID];
    testros2_B.outputMFCache[5 * testros2_B.ruleID + 2] =
      testros2_B.dv2[testros2_B.ruleID];
    testros2_B.outputMFCache[5 * testros2_B.ruleID + 3] =
      testros2_B.dv3[testros2_B.ruleID];
    testros2_B.outputMFCache[5 * testros2_B.ruleID + 4] =
      testros2_B.dv4[testros2_B.ruleID];
  }

  for (testros2_B.ruleID = 0; testros2_B.ruleID < 25; testros2_B.ruleID++) {
    for (testros2_B.sampleID = 0; testros2_B.sampleID < 101; testros2_B.sampleID
         ++) {
      testros2_B.c_ = testros2_B.outputMFCache[(5 * testros2_B.sampleID +
        b_1[testros2_B.ruleID]) - 1];
      testros2_B.rtb_Saturation_idx_1 =
        testros2_B.antecedentOutputs[testros2_B.ruleID];
      if ((!(testros2_B.c_ > testros2_B.rtb_Saturation_idx_1)) && (!rtIsNaN
           (testros2_B.c_))) {
        testros2_B.rtb_Saturation_idx_1 = testros2_B.c_;
      }

      if (testros2_B.aggregatedOutputs[testros2_B.sampleID] <
          testros2_B.rtb_Saturation_idx_1) {
        testros2_B.aggregatedOutputs[testros2_B.sampleID] =
          testros2_B.rtb_Saturation_idx_1;
      }
    }
  }

  // End of MATLAB Function: '<S19>/Evaluate Rule Consequents'

  // MATLAB Function: '<S19>/Defuzzify Outputs' incorporates:
  //   Constant: '<S19>/Output Sample Points'
  //   MATLAB Function: '<S19>/Evaluate Rule Antecedents'

  if (testros2_B.sumAntecedentOutputs == 0.0) {
    testros2_B.rtb_Saturation_idx_2 = 2.5;
  } else {
    testros2_B.sumAntecedentOutputs = 0.0;
    testros2_B.rtb_Saturation_idx_1 = 0.0;
    for (testros2_B.ruleID = 0; testros2_B.ruleID < 101; testros2_B.ruleID++) {
      testros2_B.rtb_Saturation_idx_1 +=
        testros2_B.aggregatedOutputs[testros2_B.ruleID];
    }

    if (testros2_B.rtb_Saturation_idx_1 == 0.0) {
      testros2_B.rtb_Saturation_idx_2 = (testros2_P.OutputSamplePoints_Value_o[0]
        + testros2_P.OutputSamplePoints_Value_o[100]) / 2.0;
    } else {
      for (testros2_B.ruleID = 0; testros2_B.ruleID < 101; testros2_B.ruleID++)
      {
        testros2_B.sumAntecedentOutputs +=
          testros2_P.OutputSamplePoints_Value_o[testros2_B.ruleID] *
          testros2_B.aggregatedOutputs[testros2_B.ruleID];
      }

      testros2_B.rtb_Saturation_idx_2 = 1.0 / testros2_B.rtb_Saturation_idx_1 *
        testros2_B.sumAntecedentOutputs;
    }
  }

  // End of MATLAB Function: '<S19>/Defuzzify Outputs'
  // End of Outputs for SubSystem: '<S15>/Fuzzy_Ki'

  // Outputs for Atomic SubSystem: '<S15>/Fuzzy_Kd'
  // MATLAB Function: '<S18>/Evaluate Rule Antecedents'
  testros2_B.sumAntecedentOutputs = 0.0;
  testros2_B.inputMFCache[0] = testros2_trimf(testros2_B.rtb_Saturation_idx_4, k);
  testros2_B.D_T_c[0] = -0.8333;
  testros2_B.D_T_c[1] = 0.0;
  testros2_B.D_T_c[2] = 0.8333;
  testros2_B.inputMFCache[1] = testros2_trimf(testros2_B.rtb_Saturation_idx_4,
    testros2_B.D_T_c);
  testros2_B.inputMFCache[2] = testros2_trimf(testros2_B.rtb_Saturation_idx_4,
    f_0);
  testros2_B.D_T_c[0] = -3.0;
  testros2_B.D_T_c[1] = -1.0;
  testros2_B.D_T_c[2] = 1.0;
  testros2_B.inputMFCache[3] = testros2_trimf(testros2_B.rtb_Saturation_idx_4,
    testros2_B.D_T_c);
  testros2_B.inputMFCache[4] = testros2_trimf(testros2_B.rtb_Saturation_idx_4,
    e_2);
  testros2_B.inputMFCache[5] = testros2_trimf(testros2_B.rtb_Saturation_idx_5, f);
  testros2_B.D_T_c[0] = -3.333;
  testros2_B.D_T_c[1] = 0.0;
  testros2_B.D_T_c[2] = 3.333;
  testros2_B.inputMFCache[6] = testros2_trimf(testros2_B.rtb_Saturation_idx_5,
    testros2_B.D_T_c);
  testros2_B.inputMFCache[7] = testros2_trimf(testros2_B.rtb_Saturation_idx_5, d);
  testros2_B.D_T_c[0] = -12.0;
  testros2_B.D_T_c[1] = -4.0;
  testros2_B.D_T_c[2] = 4.0;
  testros2_B.inputMFCache[8] = testros2_trimf(testros2_B.rtb_Saturation_idx_5,
    testros2_B.D_T_c);
  testros2_B.D_T_c[0] = -4.0;
  testros2_B.D_T_c[1] = 4.0;
  testros2_B.D_T_c[2] = 12.0;
  testros2_B.inputMFCache[9] = testros2_trimf(testros2_B.rtb_Saturation_idx_5,
    testros2_B.D_T_c);
  for (testros2_B.ruleID = 0; testros2_B.ruleID < 25; testros2_B.ruleID++) {
    testros2_B.c_ = 1.0;
    testros2_B.rtb_Saturation_idx_1 =
      testros2_B.inputMFCache[b[testros2_B.ruleID] - 1];
    if (testros2_B.rtb_Saturation_idx_1 < 1.0) {
      testros2_B.c_ = testros2_B.rtb_Saturation_idx_1;
    }

    testros2_B.rtb_Saturation_idx_1 =
      testros2_B.inputMFCache[b[testros2_B.ruleID + 25] + 4];
    if (testros2_B.c_ > testros2_B.rtb_Saturation_idx_1) {
      testros2_B.c_ = testros2_B.rtb_Saturation_idx_1;
    }

    testros2_B.antecedentOutputs[testros2_B.ruleID] = testros2_B.c_;
    testros2_B.sumAntecedentOutputs += testros2_B.c_;
  }

  // MATLAB Function: '<S18>/Evaluate Rule Consequents' incorporates:
  //   Constant: '<S18>/Output Sample Points'

  testros2_trimf_p(testros2_P.OutputSamplePoints_Value, e_3, testros2_B.dv);
  testros2_trimf_p(testros2_P.OutputSamplePoints_Value, d_3, testros2_B.dv1);
  testros2_trimf_p(testros2_P.OutputSamplePoints_Value, c_3, testros2_B.dv2);
  testros2_B.D_T_c[0] = -10.0;
  testros2_B.D_T_c[1] = 0.0;
  testros2_B.D_T_c[2] = 10.0;
  testros2_trimf_p(testros2_P.OutputSamplePoints_Value, testros2_B.D_T_c,
                   testros2_B.dv3);
  testros2_B.D_T_c[0] = 0.0;
  testros2_B.D_T_c[1] = 10.0;
  testros2_B.D_T_c[2] = 20.0;
  testros2_trimf_p(testros2_P.OutputSamplePoints_Value, testros2_B.D_T_c,
                   testros2_B.dv4);
  for (testros2_B.ruleID = 0; testros2_B.ruleID < 101; testros2_B.ruleID++) {
    testros2_B.aggregatedOutputs[testros2_B.ruleID] = 0.0;
    testros2_B.outputMFCache[5 * testros2_B.ruleID] =
      testros2_B.dv[testros2_B.ruleID];
    testros2_B.outputMFCache[5 * testros2_B.ruleID + 1] =
      testros2_B.dv1[testros2_B.ruleID];
    testros2_B.outputMFCache[5 * testros2_B.ruleID + 2] =
      testros2_B.dv2[testros2_B.ruleID];
    testros2_B.outputMFCache[5 * testros2_B.ruleID + 3] =
      testros2_B.dv3[testros2_B.ruleID];
    testros2_B.outputMFCache[5 * testros2_B.ruleID + 4] =
      testros2_B.dv4[testros2_B.ruleID];
  }

  for (testros2_B.ruleID = 0; testros2_B.ruleID < 25; testros2_B.ruleID++) {
    for (testros2_B.sampleID = 0; testros2_B.sampleID < 101; testros2_B.sampleID
         ++) {
      testros2_B.c_ = testros2_B.outputMFCache[(5 * testros2_B.sampleID +
        b_2[testros2_B.ruleID]) - 1];
      testros2_B.rtb_Saturation_idx_1 =
        testros2_B.antecedentOutputs[testros2_B.ruleID];
      if ((!(testros2_B.c_ > testros2_B.rtb_Saturation_idx_1)) && (!rtIsNaN
           (testros2_B.c_))) {
        testros2_B.rtb_Saturation_idx_1 = testros2_B.c_;
      }

      if (testros2_B.aggregatedOutputs[testros2_B.sampleID] <
          testros2_B.rtb_Saturation_idx_1) {
        testros2_B.aggregatedOutputs[testros2_B.sampleID] =
          testros2_B.rtb_Saturation_idx_1;
      }
    }
  }

  // End of MATLAB Function: '<S18>/Evaluate Rule Consequents'

  // MATLAB Function: '<S18>/Defuzzify Outputs' incorporates:
  //   Constant: '<S18>/Output Sample Points'
  //   MATLAB Function: '<S18>/Evaluate Rule Antecedents'

  if (testros2_B.sumAntecedentOutputs == 0.0) {
    testros2_B.rtb_Saturation_idx_4 = 5.0;
  } else {
    testros2_B.sumAntecedentOutputs = 0.0;
    testros2_B.rtb_Saturation_idx_1 = 0.0;
    for (testros2_B.ruleID = 0; testros2_B.ruleID < 101; testros2_B.ruleID++) {
      testros2_B.rtb_Saturation_idx_1 +=
        testros2_B.aggregatedOutputs[testros2_B.ruleID];
    }

    if (testros2_B.rtb_Saturation_idx_1 == 0.0) {
      testros2_B.rtb_Saturation_idx_4 = (testros2_P.OutputSamplePoints_Value[0]
        + testros2_P.OutputSamplePoints_Value[100]) / 2.0;
    } else {
      for (testros2_B.ruleID = 0; testros2_B.ruleID < 101; testros2_B.ruleID++)
      {
        testros2_B.sumAntecedentOutputs +=
          testros2_P.OutputSamplePoints_Value[testros2_B.ruleID] *
          testros2_B.aggregatedOutputs[testros2_B.ruleID];
      }

      testros2_B.rtb_Saturation_idx_4 = 1.0 / testros2_B.rtb_Saturation_idx_1 *
        testros2_B.sumAntecedentOutputs;
    }
  }

  // End of MATLAB Function: '<S18>/Defuzzify Outputs'
  // End of Outputs for SubSystem: '<S15>/Fuzzy_Kd'

  // MATLAB Function: '<S6>/matlab function' incorporates:
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator2'
  //   MATLAB Function: '<S1>/MATLAB Function'
  //   Sum: '<S12>/Diff'
  //   Sum: '<S13>/Diff'
  //   Sum: '<S14>/Diff'
  //   UnitDelay: '<S12>/UD'
  //   UnitDelay: '<S13>/UD'
  //   UnitDelay: '<S14>/UD'
  //
  //  Block description for '<S12>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S13>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S14>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S12>/UD':
  //
  //   Store in Global RAM
  //
  //  Block description for '<S13>/UD':
  //
  //   Store in Global RAM
  //
  //  Block description for '<S14>/UD':
  //
  //   Store in Global RAM

  testros2_B.rtb_Saturation_idx_3 = (testros2_B.rtb_Saturation_idx_0 *
    testros2_B.dy + testros2_B.rtb_Saturation_idx_2 *
    testros2_DW.DiscreteTimeIntegrator_DSTATE) + (testros2_B.Sum1 -
    testros2_DW.UD_DSTATE_h) * testros2_B.rtb_Saturation_idx_4;
  testros2_B.sumAntecedentOutputs = (testros2_B.rtb_Saturation_idx_0 *
    testros2_B.c + testros2_B.rtb_Saturation_idx_2 *
    testros2_DW.DiscreteTimeIntegrator1_DSTATE) + (testros2_B.TSamp_a -
    testros2_DW.UD_DSTATE_l) * testros2_B.rtb_Saturation_idx_4;
  testros2_B.rtb_defuzzifiedOutputs_idx_2 = (testros2_B.rtb_Saturation_idx_0 *
    testros2_B.dx + testros2_B.rtb_Saturation_idx_2 *
    testros2_DW.DiscreteTimeIntegrator2_DSTATE) + (testros2_B.TSamp_p -
    testros2_DW.UD_DSTATE_d) * testros2_B.rtb_Saturation_idx_4;

  // DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
  testros2_B.rtb_Saturation_idx_0 = testros2_DW.DiscreteTimeIntegrator_DSTATE_n
    [0];

  // MATLAB Function: '<S3>/MATLAB Function' incorporates:
  //   DiscreteIntegrator: '<S3>/Discrete-Time Integrator1'

  testros2_B.rtb_Saturation_idx_1 = sin
    (testros2_DW.DiscreteTimeIntegrator1_DSTAT_m[2]);
  testros2_B.c_ = cos(testros2_DW.DiscreteTimeIntegrator1_DSTAT_m[2]);
  testros2_B.m3_tmp = 0.0 * testros2_B.rtb_Saturation_idx_1 + 0.0 *
    testros2_B.c_;
  testros2_B.rtb_Saturation_idx_4 = testros2_B.m3_tmp * 50.0;
  testros2_B.rtb_Saturation_idx_5 = (-0.0 * testros2_B.c_ + 0.0 *
    testros2_B.rtb_Saturation_idx_1) * 50.0;
  testros2_B.rtb_Saturation_idx_2 = testros2_B.c_ +
    testros2_B.rtb_Saturation_idx_1;
  testros2_B.J_inner[0] = testros2_B.rtb_Saturation_idx_2;
  testros2_B.J_inner_tmp = testros2_B.rtb_Saturation_idx_1 - testros2_B.c_;
  testros2_B.J_inner[4] = testros2_B.J_inner_tmp;
  testros2_B.J_inner[8] = -0.48;
  testros2_B.J_inner_tmp_c = testros2_B.c_ - testros2_B.rtb_Saturation_idx_1;
  testros2_B.J_inner[1] = testros2_B.J_inner_tmp_c;
  testros2_B.J_inner[5] = testros2_B.rtb_Saturation_idx_2;
  testros2_B.J_inner[9] = 0.48;
  testros2_B.J_inner[2] = testros2_B.rtb_Saturation_idx_2;
  testros2_B.J_inner[6] = testros2_B.J_inner_tmp;
  testros2_B.J_inner[10] = 0.48;
  testros2_B.J_inner[3] = testros2_B.J_inner_tmp_c;
  testros2_B.J_inner[7] = testros2_B.rtb_Saturation_idx_2;
  testros2_B.J_inner[11] = -0.48;

  // DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
  //   MATLAB Function: '<S3>/MATLAB Function'

  testros2_B.J_inner_tmp = testros2_DW.DiscreteTimeIntegrator_DSTATE_n[0];
  testros2_B.J_inner_tmp_c = testros2_DW.DiscreteTimeIntegrator_DSTATE_n[1];
  testros2_B.DiscreteTimeIntegrator_DSTATE_n =
    testros2_DW.DiscreteTimeIntegrator_DSTATE_n[2];
  for (testros2_B.ruleID = 0; testros2_B.ruleID < 4; testros2_B.ruleID++) {
    // MATLAB Function: '<S6>/matlab function'
    testros2_B.u_w_k = (a[testros2_B.ruleID + 4] *
                        testros2_B.sumAntecedentOutputs + a[testros2_B.ruleID] *
                        testros2_B.rtb_Saturation_idx_3) + a[testros2_B.ruleID +
      8] * testros2_B.rtb_defuzzifiedOutputs_idx_2;
    if (rtIsNaN(testros2_B.u_w_k)) {
      testros2_B.rtb_Saturation_idx_2 = (rtNaN);
    } else if (testros2_B.u_w_k < 0.0) {
      testros2_B.rtb_Saturation_idx_2 = -1.0;
    } else {
      testros2_B.rtb_Saturation_idx_2 = (testros2_B.u_w_k > 0.0);
    }

    testros2_B.u_w[testros2_B.ruleID] = testros2_B.u_w_k * 10.0 + 1.5 *
      testros2_B.rtb_Saturation_idx_2;

    // MATLAB Function: '<S3>/MATLAB Function' incorporates:
    //   DiscreteIntegrator: '<S3>/Discrete-Time Integrator'

    testros2_B.u_w_k = testros2_B.J_inner[testros2_B.ruleID];
    testros2_B.D_T[3 * testros2_B.ruleID] = 13.333333333333334 *
      testros2_B.u_w_k;
    testros2_B.rtb_Saturation_idx_2 = 13.333333333333334 * testros2_B.u_w_k *
      testros2_B.J_inner_tmp;
    testros2_B.u_w_k = testros2_B.J_inner[testros2_B.ruleID + 4];
    testros2_B.D_T[3 * testros2_B.ruleID + 1] = 13.333333333333334 *
      testros2_B.u_w_k;
    testros2_B.rtb_Saturation_idx_2 += 13.333333333333334 * testros2_B.u_w_k *
      testros2_B.J_inner_tmp_c;
    testros2_B.u_w_k = testros2_B.J_inner[testros2_B.ruleID + 8];
    testros2_B.D_T[3 * testros2_B.ruleID + 2] = 13.333333333333334 *
      testros2_B.u_w_k;
    testros2_B.S_mat[testros2_B.ruleID] = tanh((13.333333333333334 *
      testros2_B.u_w_k * testros2_B.DiscreteTimeIntegrator_DSTATE_n +
      testros2_B.rtb_Saturation_idx_2) * 100.0);
  }

  // MATLAB Function: '<S6>/matlab function'
  if (testros2_B.u_w[0] <= 100.0) {
    testros2_B.rtb_Saturation_idx_2 = testros2_B.u_w[0];
  } else {
    testros2_B.rtb_Saturation_idx_2 = 100.0;
  }

  if (!(testros2_B.rtb_Saturation_idx_2 >= -100.0)) {
    testros2_B.rtb_Saturation_idx_2 = -100.0;
  }

  if (testros2_B.u_w[1] <= 100.0) {
    testros2_B.rtb_Saturation_idx_3 = testros2_B.u_w[1];
  } else {
    testros2_B.rtb_Saturation_idx_3 = 100.0;
  }

  if (!(testros2_B.rtb_Saturation_idx_3 >= -100.0)) {
    testros2_B.rtb_Saturation_idx_3 = -100.0;
  }

  if (testros2_B.u_w[2] <= 100.0) {
    testros2_B.sumAntecedentOutputs = testros2_B.u_w[2];
  } else {
    testros2_B.sumAntecedentOutputs = 100.0;
  }

  if (!(testros2_B.sumAntecedentOutputs >= -100.0)) {
    testros2_B.sumAntecedentOutputs = -100.0;
  }

  if (testros2_B.u_w[3] <= 100.0) {
    testros2_B.J_inner_tmp = testros2_B.u_w[3];
  } else {
    testros2_B.J_inner_tmp = 100.0;
  }

  if (!(testros2_B.J_inner_tmp >= -100.0)) {
    testros2_B.J_inner_tmp = -100.0;
  }

  // MATLAB Function: '<S3>/MATLAB Function' incorporates:
  //   DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
  //   MATLAB Function: '<S6>/matlab function'

  testros2_B.dv5[0] = 0.0;
  testros2_B.dv5[3] = 0.0;
  testros2_B.dv5[6] = (0.0 * testros2_B.c_ - 0.0 *
                       testros2_B.rtb_Saturation_idx_1) * (50.0 *
    testros2_DW.DiscreteTimeIntegrator_DSTATE_n[2]);
  testros2_B.dv5[1] = 0.0;
  testros2_B.dv5[4] = 0.0;
  testros2_B.dv5[7] = 50.0 * testros2_DW.DiscreteTimeIntegrator_DSTATE_n[2] *
    testros2_B.m3_tmp;
  for (testros2_B.ruleID = 0; testros2_B.ruleID < 3; testros2_B.ruleID++) {
    testros2_B.dv5[3 * testros2_B.ruleID + 2] = 0.0;
    testros2_B.D_T_c[testros2_B.ruleID] = ((testros2_B.D_T[testros2_B.ruleID + 3]
      * testros2_B.rtb_Saturation_idx_3 + testros2_B.D_T[testros2_B.ruleID] *
      testros2_B.rtb_Saturation_idx_2) + testros2_B.D_T[testros2_B.ruleID + 6] *
      testros2_B.sumAntecedentOutputs) + testros2_B.D_T[testros2_B.ruleID + 9] *
      testros2_B.J_inner_tmp;
  }

  // DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
  //   MATLAB Function: '<S3>/MATLAB Function'

  testros2_B.J_inner_tmp = testros2_DW.DiscreteTimeIntegrator_DSTATE_n[0];
  testros2_B.J_inner_tmp_c = testros2_DW.DiscreteTimeIntegrator_DSTATE_n[1];
  testros2_B.DiscreteTimeIntegrator_DSTATE_n =
    testros2_DW.DiscreteTimeIntegrator_DSTATE_n[2];

  // MATLAB Function: '<S3>/MATLAB Function' incorporates:
  //   DiscreteIntegrator: '<S3>/Discrete-Time Integrator'

  testros2_B.rtb_Saturation_idx_2 = testros2_B.S_mat[0];
  testros2_B.rtb_Saturation_idx_3 = testros2_B.S_mat[1];
  testros2_B.sumAntecedentOutputs = testros2_B.S_mat[2];
  testros2_B.rtb_Saturation_idx_1 = testros2_B.S_mat[3];
  for (testros2_B.ruleID = 0; testros2_B.ruleID <= 0; testros2_B.ruleID += 2) {
    tmp = _mm_loadu_pd(&testros2_B.dv5[testros2_B.ruleID + 6]);
    tmp_0 = _mm_loadu_pd(&testros2_B.D_T_c[testros2_B.ruleID]);
    tmp_1 = _mm_loadu_pd(&testros2_B.D_T[testros2_B.ruleID]);
    tmp_2 = _mm_loadu_pd(&testros2_B.D_T[testros2_B.ruleID + 3]);
    tmp_3 = _mm_loadu_pd(&testros2_B.D_T[testros2_B.ruleID + 6]);
    tmp_4 = _mm_loadu_pd(&testros2_B.D_T[testros2_B.ruleID + 9]);
    _mm_storeu_pd(&testros2_B.B[testros2_B.ruleID], _mm_sub_pd(_mm_sub_pd(tmp_0,
      _mm_add_pd(_mm_set1_pd(0.0 * testros2_B.J_inner_tmp + 0.0 *
      testros2_B.J_inner_tmp_c), _mm_mul_pd(tmp, _mm_set1_pd
      (testros2_B.DiscreteTimeIntegrator_DSTATE_n)))), _mm_add_pd(_mm_add_pd
      (_mm_add_pd(_mm_mul_pd(_mm_set1_pd(testros2_B.rtb_Saturation_idx_2 * 5.0 *
      0.075), tmp_1), _mm_mul_pd(_mm_set1_pd(testros2_B.rtb_Saturation_idx_3 *
      5.0 * 0.075), tmp_2)), _mm_mul_pd(_mm_set1_pd
      (testros2_B.sumAntecedentOutputs * 5.0 * 0.075), tmp_3)), _mm_mul_pd
      (_mm_set1_pd(testros2_B.rtb_Saturation_idx_1 * 5.0 * 0.075), tmp_4))));
  }

  for (testros2_B.ruleID = 2; testros2_B.ruleID < 3; testros2_B.ruleID++) {
    testros2_B.B[testros2_B.ruleID] = (testros2_B.D_T_c[testros2_B.ruleID] -
      ((0.0 * testros2_B.J_inner_tmp + 0.0 * testros2_B.J_inner_tmp_c) +
       testros2_B.dv5[testros2_B.ruleID + 6] *
       testros2_B.DiscreteTimeIntegrator_DSTATE_n)) -
      (((testros2_B.rtb_Saturation_idx_2 * 5.0 * 0.075 *
         testros2_B.D_T[testros2_B.ruleID] + testros2_B.rtb_Saturation_idx_3 *
         5.0 * 0.075 * testros2_B.D_T[testros2_B.ruleID + 3]) +
        testros2_B.sumAntecedentOutputs * 5.0 * 0.075 *
        testros2_B.D_T[testros2_B.ruleID + 6]) + testros2_B.rtb_Saturation_idx_1
       * 5.0 * 0.075 * testros2_B.D_T[testros2_B.ruleID + 9]);
  }

  testros2_B.rtb_Saturation_idx_2 = testros2_B.rtb_Saturation_idx_4 /
    72.222222222222229;
  testros2_B.rtb_Saturation_idx_1 = testros2_B.rtb_Saturation_idx_5 - 0.0 *
    testros2_B.rtb_Saturation_idx_4;
  testros2_B.rtb_Saturation_idx_3 = 10.120000000000001 -
    testros2_B.rtb_Saturation_idx_2 * testros2_B.rtb_Saturation_idx_4;
  testros2_B.rtb_Saturation_idx_5 = (testros2_B.rtb_Saturation_idx_5 -
    testros2_B.rtb_Saturation_idx_2 * 0.0) / 72.222222222222229;
  _mm_storeu_pd(&testros2_B.dv6[0], _mm_sub_pd(_mm_set_pd(testros2_B.B[1],
    testros2_B.rtb_Saturation_idx_3), _mm_mul_pd(_mm_set_pd(testros2_B.B[0],
    testros2_B.rtb_Saturation_idx_5), _mm_set_pd(0.0,
    testros2_B.rtb_Saturation_idx_1))));
  testros2_B.rtb_Saturation_idx_5 = ((testros2_B.B[2] - testros2_B.B[0] *
    testros2_B.rtb_Saturation_idx_2) - testros2_B.dv6[1] *
    testros2_B.rtb_Saturation_idx_5) / testros2_B.dv6[0];
  testros2_B.rtb_Saturation_idx_2 = (testros2_B.dv6[1] -
    testros2_B.rtb_Saturation_idx_5 * testros2_B.rtb_Saturation_idx_1) /
    72.222222222222229;

  // Saturate: '<S3>/Saturation' incorporates:
  //   DiscreteIntegrator: '<S3>/Discrete-Time Integrator'

  if (testros2_DW.DiscreteTimeIntegrator_DSTATE_n[0] >
      testros2_P.Saturation_UpperSat_c) {
    // BusAssignment: '<Root>/Bus Assignment'
    testros2_B.BusAssignment.linear.x = testros2_P.Saturation_UpperSat_c;
  } else if (testros2_DW.DiscreteTimeIntegrator_DSTATE_n[0] <
             testros2_P.Saturation_LowerSat_a) {
    // BusAssignment: '<Root>/Bus Assignment'
    testros2_B.BusAssignment.linear.x = testros2_P.Saturation_LowerSat_a;
  } else {
    // BusAssignment: '<Root>/Bus Assignment'
    testros2_B.BusAssignment.linear.x =
      testros2_DW.DiscreteTimeIntegrator_DSTATE_n[0];
  }

  // End of Saturate: '<S3>/Saturation'

  // Saturate: '<S3>/Saturation1' incorporates:
  //   DiscreteIntegrator: '<S3>/Discrete-Time Integrator'

  if (testros2_DW.DiscreteTimeIntegrator_DSTATE_n[1] >
      testros2_P.Saturation1_UpperSat) {
    // BusAssignment: '<Root>/Bus Assignment'
    testros2_B.BusAssignment.linear.y = testros2_P.Saturation1_UpperSat;
  } else if (testros2_DW.DiscreteTimeIntegrator_DSTATE_n[1] <
             testros2_P.Saturation1_LowerSat) {
    // BusAssignment: '<Root>/Bus Assignment'
    testros2_B.BusAssignment.linear.y = testros2_P.Saturation1_LowerSat;
  } else {
    // BusAssignment: '<Root>/Bus Assignment'
    testros2_B.BusAssignment.linear.y =
      testros2_DW.DiscreteTimeIntegrator_DSTATE_n[1];
  }

  // End of Saturate: '<S3>/Saturation1'

  // Saturate: '<S3>/Saturation2' incorporates:
  //   DiscreteIntegrator: '<S3>/Discrete-Time Integrator'

  if (testros2_DW.DiscreteTimeIntegrator_DSTATE_n[2] >
      testros2_P.Saturation2_UpperSat) {
    // BusAssignment: '<Root>/Bus Assignment'
    testros2_B.BusAssignment.angular.z = testros2_P.Saturation2_UpperSat;
  } else if (testros2_DW.DiscreteTimeIntegrator_DSTATE_n[2] <
             testros2_P.Saturation2_LowerSat) {
    // BusAssignment: '<Root>/Bus Assignment'
    testros2_B.BusAssignment.angular.z = testros2_P.Saturation2_LowerSat;
  } else {
    // BusAssignment: '<Root>/Bus Assignment'
    testros2_B.BusAssignment.angular.z =
      testros2_DW.DiscreteTimeIntegrator_DSTATE_n[2];
  }

  // End of Saturate: '<S3>/Saturation2'

  // MATLABSystem: '<S7>/SinkBlock'
  Pub_testros2_62.publish(&testros2_B.BusAssignment);

  // Update for UnitDelay: '<S12>/UD'
  //
  //  Block description for '<S12>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE_h = testros2_B.Sum1;

  // Update for UnitDelay: '<S13>/UD'
  //
  //  Block description for '<S13>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE_l = testros2_B.TSamp_a;

  // Update for UnitDelay: '<S14>/UD'
  //
  //  Block description for '<S14>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE_d = testros2_B.TSamp_p;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
  testros2_DW.DiscreteTimeIntegrator_DSTATE +=
    testros2_P.DiscreteTimeIntegrator_gainval * testros2_B.dy;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'
  testros2_DW.DiscreteTimeIntegrator1_DSTATE +=
    testros2_P.DiscreteTimeIntegrator1_gainval * testros2_B.c;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator2' incorporates:
  //   MATLAB Function: '<S1>/MATLAB Function'

  testros2_DW.DiscreteTimeIntegrator2_DSTATE +=
    testros2_P.DiscreteTimeIntegrator2_gainval * testros2_B.dx;

  // Update for UnitDelay: '<S17>/UD'
  //
  //  Block description for '<S17>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE[0] = testros2_B.s;

  // Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
  //   MATLAB Function: '<S3>/MATLAB Function'

  testros2_DW.DiscreteTimeIntegrator_DSTATE_n[0] += ((testros2_B.B[0] -
    testros2_B.rtb_Saturation_idx_5 * testros2_B.rtb_Saturation_idx_4) -
    testros2_B.rtb_Saturation_idx_2 * 0.0) / 72.222222222222229 *
    testros2_P.DiscreteTimeIntegrator_gainva_a;

  // Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S3>/Discrete-Time Integrator'

  testros2_DW.DiscreteTimeIntegrator1_DSTAT_m[0] +=
    testros2_P.DiscreteTimeIntegrator1_gainv_p * testros2_B.rtb_Saturation_idx_0;

  // Update for UnitDelay: '<S17>/UD'
  //
  //  Block description for '<S17>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE[1] = testros2_B.theta;

  // Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S3>/Discrete-Time Integrator'

  tmp = _mm_set_pd(testros2_P.DiscreteTimeIntegrator1_gainv_p,
                   testros2_P.DiscreteTimeIntegrator_gainva_a);

  // DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
  //   DiscreteIntegrator: '<S3>/Discrete-Time Integrator1'
  //   MATLAB Function: '<S3>/MATLAB Function'

  _mm_storeu_pd(&testros2_B.dv6[0], _mm_add_pd(_mm_mul_pd(tmp, _mm_set_pd
    (testros2_DW.DiscreteTimeIntegrator_DSTATE_n[1],
     testros2_B.rtb_Saturation_idx_2)), _mm_set_pd
    (testros2_DW.DiscreteTimeIntegrator1_DSTAT_m[1],
     testros2_DW.DiscreteTimeIntegrator_DSTATE_n[1])));

  // Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
  testros2_DW.DiscreteTimeIntegrator_DSTATE_n[1] = testros2_B.dv6[0];

  // Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator1'
  testros2_DW.DiscreteTimeIntegrator1_DSTAT_m[1] = testros2_B.dv6[1];

  // Update for UnitDelay: '<S17>/UD'
  //
  //  Block description for '<S17>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE[2] = testros2_B.Sum;

  // DiscreteIntegrator: '<S3>/Discrete-Time Integrator' incorporates:
  //   DiscreteIntegrator: '<S3>/Discrete-Time Integrator1'
  //   MATLAB Function: '<S3>/MATLAB Function'

  _mm_storeu_pd(&testros2_B.dv6[0], _mm_add_pd(_mm_mul_pd(tmp, _mm_set_pd
    (testros2_DW.DiscreteTimeIntegrator_DSTATE_n[2],
     testros2_B.rtb_Saturation_idx_5)), _mm_set_pd
    (testros2_DW.DiscreteTimeIntegrator1_DSTAT_m[2],
     testros2_DW.DiscreteTimeIntegrator_DSTATE_n[2])));

  // Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator'
  testros2_DW.DiscreteTimeIntegrator_DSTATE_n[2] = testros2_B.dv6[0];

  // Update for DiscreteIntegrator: '<S3>/Discrete-Time Integrator1'
  testros2_DW.DiscreteTimeIntegrator1_DSTAT_m[2] = testros2_B.dv6[1];

  // Update absolute time for base rate
  // The "clockTick0" counts the number of times the code of this task has
  //  been executed. The resolution of this integer timer is 0.02, which is the step size
  //  of the task. Size of "clockTick0" ensures timer will not overflow during the
  //  application lifespan selected.

  (&testros2_M)->Timing.clockTick0++;
}

// Model initialize function
void testros2::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // InitializeConditions for UnitDelay: '<S12>/UD'
  //
  //  Block description for '<S12>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE_h = testros2_P.DiscreteDerivative_ICPrevScal_p;

  // InitializeConditions for UnitDelay: '<S13>/UD'
  //
  //  Block description for '<S13>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE_l = testros2_P.DiscreteDerivative1_ICPrevScale;

  // InitializeConditions for UnitDelay: '<S14>/UD'
  //
  //  Block description for '<S14>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE_d = testros2_P.DiscreteDerivative2_ICPrevScale;

  // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' 
  testros2_DW.DiscreteTimeIntegrator_DSTATE =
    testros2_P.DiscreteTimeIntegrator_IC;

  // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1' 
  testros2_DW.DiscreteTimeIntegrator1_DSTATE =
    testros2_P.DiscreteTimeIntegrator1_IC;

  // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator2' 
  testros2_DW.DiscreteTimeIntegrator2_DSTATE =
    testros2_P.DiscreteTimeIntegrator2_IC;

  // InitializeConditions for UnitDelay: '<S17>/UD'
  //
  //  Block description for '<S17>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE[0] = testros2_P.DiscreteDerivative_ICPrevScaled;

  // InitializeConditions for DiscreteIntegrator: '<S3>/Discrete-Time Integrator' 
  testros2_DW.DiscreteTimeIntegrator_DSTATE_n[0] =
    testros2_P.DiscreteTimeIntegrator_IC_l;

  // InitializeConditions for DiscreteIntegrator: '<S3>/Discrete-Time Integrator1' 
  testros2_DW.DiscreteTimeIntegrator1_DSTAT_m[0] =
    testros2_P.DiscreteTimeIntegrator1_IC_o;

  // InitializeConditions for UnitDelay: '<S17>/UD'
  //
  //  Block description for '<S17>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE[1] = testros2_P.DiscreteDerivative_ICPrevScaled;

  // InitializeConditions for DiscreteIntegrator: '<S3>/Discrete-Time Integrator' 
  testros2_DW.DiscreteTimeIntegrator_DSTATE_n[1] =
    testros2_P.DiscreteTimeIntegrator_IC_l;

  // InitializeConditions for DiscreteIntegrator: '<S3>/Discrete-Time Integrator1' 
  testros2_DW.DiscreteTimeIntegrator1_DSTAT_m[1] =
    testros2_P.DiscreteTimeIntegrator1_IC_o;

  // InitializeConditions for UnitDelay: '<S17>/UD'
  //
  //  Block description for '<S17>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE[2] = testros2_P.DiscreteDerivative_ICPrevScaled;

  // InitializeConditions for DiscreteIntegrator: '<S3>/Discrete-Time Integrator' 
  testros2_DW.DiscreteTimeIntegrator_DSTATE_n[2] =
    testros2_P.DiscreteTimeIntegrator_IC_l;

  // InitializeConditions for DiscreteIntegrator: '<S3>/Discrete-Time Integrator1' 
  testros2_DW.DiscreteTimeIntegrator1_DSTAT_m[2] =
    testros2_P.DiscreteTimeIntegrator1_IC_o;

  // SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem'
  // SystemInitialize for SignalConversion generated from: '<S30>/In1' incorporates:
  //   Outport: '<S30>/Out1'

  testros2_B.In1 = testros2_P.Out1_Y0;

  // End of SystemInitialize for SubSystem: '<S8>/Enabled Subsystem'

  // Start for MATLABSystem: '<S8>/SourceBlock'
  testros2_DW.obj_n.QOSAvoidROSNamespaceConventions = false;
  testros2_DW.obj_n.matlabCodegenIsDeleted = false;
  testros2_DW.obj_n.isSetupComplete = false;
  testros2_DW.obj_n.isInitialized = 1;
  testros2_Subscriber_setupImpl(&testros2_DW.obj_n);
  testros2_DW.obj_n.isSetupComplete = true;

  // Start for MATLABSystem: '<S7>/SinkBlock'
  testros2_DW.obj.QOSAvoidROSNamespaceConventions = false;
  testros2_DW.obj.matlabCodegenIsDeleted = false;
  testros2_DW.obj.isSetupComplete = false;
  testros2_DW.obj.isInitialized = 1;
  testros2_Publisher_setupImpl(&testros2_DW.obj);
  testros2_DW.obj.isSetupComplete = true;
}

// Model terminate function
void testros2::terminate()
{
  // Terminate for MATLABSystem: '<S8>/SourceBlock'
  if (!testros2_DW.obj_n.matlabCodegenIsDeleted) {
    testros2_DW.obj_n.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S8>/SourceBlock'

  // Terminate for MATLABSystem: '<S7>/SinkBlock'
  if (!testros2_DW.obj.matlabCodegenIsDeleted) {
    testros2_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S7>/SinkBlock'
}

// Constructor
testros2::testros2() :
  testros2_B(),
  testros2_DW(),
  testros2_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
testros2::~testros2()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_testros2_T * testros2::getRTM()
{
  return (&testros2_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
