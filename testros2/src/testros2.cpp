//
// File: testros2.cpp
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
#include "testros2.h"
#include "testros2_types.h"
#include "rtwtypes.h"
#include <math.h>
#include <emmintrin.h>
#include "rmw/qos_profiles.h"
#include <stddef.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rt_defines.h"
#include "testros2_private.h"
#include "testros2_dt.h"

// Block signals (default storage)
B_testros2_T testros2_B;

// Block states (default storage)
DW_testros2_T testros2_DW;

// Real-time model
RT_MODEL_testros2_T testros2_M_ = RT_MODEL_testros2_T();
RT_MODEL_testros2_T *const testros2_M = &testros2_M_;

// Forward declaration for local functions
static real_T testros2_rt_atan2d_snf(real_T u0, real_T u1);
static void testros2_Subscriber_setupImpl(const ros_slros2_internal_block_Sub_T *
  obj);
static void testros2_Publisher_setupImpl(const ros_slros2_internal_block_Pub_T
  *obj);
static real_T testros2_rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      testros2_B.i1 = 1;
    } else {
      testros2_B.i1 = -1;
    }

    if (u1 > 0.0) {
      testros2_B.i2 = 1;
    } else {
      testros2_B.i2 = -1;
    }

    y = atan2(static_cast<real_T>(testros2_B.i1), static_cast<real_T>
              (testros2_B.i2));
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

static void testros2_Subscriber_setupImpl(const ros_slros2_internal_block_Sub_T *
  obj)
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

static void testros2_Publisher_setupImpl(const ros_slros2_internal_block_Pub_T
  *obj)
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
void testros2_step(void)
{
  static const real_T a[12] = { 13.333333333333334, 13.333333333333334,
    13.333333333333334, 13.333333333333334, -13.333333333333334,
    13.333333333333334, 13.333333333333334, -13.333333333333334, -6.4, 6.4, -6.4,
    6.4 };

  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  static const int8_T b[3] = { 0, 0, 1 };

  // Reset subsysRan breadcrumbs
  srClearBC(testros2_DW.EnabledSubsystem_SubsysRanBC);

  // MATLABSystem: '<S8>/SourceBlock'
  testros2_B.b_varargout_1 = Sub_testros2_63.getLatestMessage
    (&testros2_B.rtb_SourceBlock_o2_m);

  // Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S16>/Enable'

  // Start for MATLABSystem: '<S8>/SourceBlock'
  if (testros2_B.b_varargout_1) {
    // SignalConversion generated from: '<S16>/In1'
    testros2_B.In1 = testros2_B.rtb_SourceBlock_o2_m;
    srUpdateBC(testros2_DW.EnabledSubsystem_SubsysRanBC);
  }

  // End of Start for MATLABSystem: '<S8>/SourceBlock'
  // End of Outputs for SubSystem: '<S8>/Enabled Subsystem'

  // SignalConversion generated from: '<S9>/Bus Selector'
  testros2_B.x = testros2_B.In1.pose.pose.position.x;

  // Sum: '<Root>/Sum' incorporates:
  //   DigitalClock: '<Root>/Digital Clock'
  //   MATLAB Function: '<Root>/MATLAB Function'

  testros2_B.Sum = 0.1 * testros2_M->Timing.taskTime0 - testros2_B.x;

  // SignalConversion generated from: '<S9>/Bus Selector'
  testros2_B.y = testros2_B.In1.pose.pose.position.y;

  // Sum: '<Root>/Sum1' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function'

  testros2_B.Sum1 = 0.0 - testros2_B.y;

  // MATLAB Function: '<S9>/MATLAB Function'
  testros2_B.theta = 2.0 * testros2_rt_atan2d_snf
    (testros2_B.In1.pose.pose.orientation.z,
     testros2_B.In1.pose.pose.orientation.w);

  // Sum: '<Root>/Sum2' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function'

  testros2_B.Sum2 = 0.0 - testros2_B.theta;

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   MATLAB Function: '<S4>/MATLAB Function'

  testros2_B.rtb_e_out_tmp = cos(testros2_B.theta);
  testros2_B.rtb_e_out_tmp_p = sin(testros2_B.theta);
  testros2_B.e_out = testros2_rt_atan2d_snf(testros2_B.rtb_e_out_tmp_p,
    testros2_B.rtb_e_out_tmp);

  // MATLAB Function: '<S3>/MATLAB Function' incorporates:
  //   SampleTimeMath: '<S10>/TSamp'
  //   SampleTimeMath: '<S11>/TSamp'
  //
  //  About '<S10>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  //    *
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  //
  testros2_B.TSamp = sin(testros2_B.e_out);
  testros2_B.e_out = cos(testros2_B.e_out);
  testros2_B.Q[0] = testros2_B.e_out;
  testros2_B.Q[1] = -testros2_B.TSamp;
  testros2_B.Q[2] = 0.0;
  testros2_B.Q[3] = testros2_B.TSamp;
  testros2_B.Q[4] = testros2_B.e_out;
  testros2_B.Q[5] = 0.0;
  for (testros2_B.i = 0; testros2_B.i < 3; testros2_B.i++) {
    testros2_B.e_R_tmp = b[testros2_B.i];
    testros2_B.Q[testros2_B.i + 6] = testros2_B.e_R_tmp;
    testros2_B.e_R[testros2_B.i] = (testros2_B.Q[testros2_B.i + 3] *
      testros2_B.Sum1 + testros2_B.Q[testros2_B.i] * testros2_B.Sum) +
      static_cast<real_T>(testros2_B.e_R_tmp) * testros2_B.Sum2;
  }

  tmp_2 = _mm_mul_pd(_mm_loadu_pd(&testros2_B.e_R[0]), _mm_set_pd
                     (testros2_P.TSamp_WtEt_n, testros2_P.TSamp_WtEt));
  _mm_storeu_pd(&testros2_B.dv2[0], tmp_2);

  // SampleTimeMath: '<S10>/TSamp'
  //
  //  About '<S10>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  //
  testros2_B.TSamp = testros2_B.dv2[0];

  // SampleTimeMath: '<S11>/TSamp'
  //
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  //
  testros2_B.e_out = testros2_B.dv2[1];

  // SampleTimeMath: '<S12>/TSamp' incorporates:
  //   MATLAB Function: '<S3>/MATLAB Function'
  //
  //  About '<S12>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  //
  testros2_B.TSamp_n = testros2_B.e_R[2] * testros2_P.TSamp_WtEt_a;

  // MATLAB Function: '<S2>/MATLAB Function' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator1'

  tmp_2 = _mm_set_pd(testros2_DW.DiscreteTimeIntegrator1_DSTATE,
                     testros2_DW.DiscreteTimeIntegrator_DSTATE);

  // Constant: '<Root>/Constant' incorporates:
  //   Constant: '<Root>/Constant1'
  //   Constant: '<Root>/Constant2'
  //   MATLAB Function: '<S2>/MATLAB Function'
  //   MATLAB Function: '<S3>/MATLAB Function'
  //   Sum: '<S10>/Diff'
  //   Sum: '<S11>/Diff'
  //   UnitDelay: '<S10>/UD'
  //   UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S10>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S11>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S10>/UD':
  //
  //   Store in Global RAM
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  tmp_3 = _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
    (&testros2_P.Constant_Value_j[0]), _mm_loadu_pd(&testros2_B.e_R[0])),
    _mm_mul_pd(_mm_loadu_pd(&testros2_P.Constant1_Value[0]), tmp_2)), _mm_mul_pd
                     (_mm_sub_pd(_mm_set_pd(testros2_B.dv2[1], testros2_B.dv2[0]),
    _mm_set_pd(testros2_DW.UD_DSTATE_p, testros2_DW.UD_DSTATE)), _mm_loadu_pd
                      (&testros2_P.Constant2_Value[0])));
  _mm_storeu_pd(&testros2_B.dv2[0], tmp_3);

  // MATLAB Function: '<S2>/MATLAB Function' incorporates:
  //   Constant: '<Root>/Constant'
  //   Constant: '<Root>/Constant1'
  //   Constant: '<Root>/Constant2'
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator2'
  //   MATLAB Function: '<S3>/MATLAB Function'
  //   Sum: '<S12>/Diff'
  //   UnitDelay: '<S12>/UD'
  //
  //  Block description for '<S12>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S12>/UD':
  //
  //   Store in Global RAM

  testros2_B.unnamed_idx_0 = testros2_B.dv2[0];
  testros2_B.m3 = testros2_B.dv2[1];
  testros2_B.m4 = (testros2_P.Constant_Value_j[2] * testros2_B.e_R[2] +
                   testros2_P.Constant1_Value[2] *
                   testros2_DW.DiscreteTimeIntegrator2_DSTATE) +
    (testros2_B.TSamp_n - testros2_DW.UD_DSTATE_b) * testros2_P.Constant2_Value
    [2];
  for (testros2_B.i = 0; testros2_B.i <= 2; testros2_B.i += 2) {
    _mm_storeu_pd(&testros2_B.u_dk[testros2_B.i], _mm_add_pd(_mm_add_pd
      (_mm_mul_pd(_mm_loadu_pd(&a[testros2_B.i + 4]), _mm_set1_pd(testros2_B.m3)),
       _mm_mul_pd(_mm_loadu_pd(&a[testros2_B.i]), _mm_set1_pd
                  (testros2_B.unnamed_idx_0))), _mm_mul_pd(_mm_loadu_pd
      (&a[testros2_B.i + 8]), _mm_set1_pd(testros2_B.m4))));
  }

  // SignalConversion generated from: '<S15>/ SFunction ' incorporates:
  //   MATLAB Function: '<S4>/MATLAB Function'

  testros2_B.unnamed_idx_0 = testros2_B.In1.twist.twist.linear.x;
  testros2_B.TmpSignalConversionAtSFunct[0] =
    testros2_B.In1.twist.twist.linear.x;
  testros2_B.rtb_TmpSignalConversionAtSFun_c =
    testros2_B.In1.twist.twist.linear.y;
  testros2_B.TmpSignalConversionAtSFunct[1] =
    testros2_B.In1.twist.twist.linear.y;
  testros2_B.unnamed_idx_0_tmp = testros2_B.In1.twist.twist.angular.z;
  testros2_B.TmpSignalConversionAtSFunct[2] =
    testros2_B.In1.twist.twist.angular.z;

  // MATLAB Function: '<S4>/MATLAB Function' incorporates:
  //   SignalConversion generated from: '<S15>/ SFunction '

  testros2_B.m3_tmp = 0.0 * testros2_B.rtb_e_out_tmp_p + 0.0 *
    testros2_B.rtb_e_out_tmp;
  testros2_B.m3 = testros2_B.m3_tmp * 50.0;
  testros2_B.m4_tmp = 0.0 * testros2_B.rtb_e_out_tmp_p;
  testros2_B.m4 = (-0.0 * testros2_B.rtb_e_out_tmp + testros2_B.m4_tmp) * 50.0;
  testros2_B.Q[0] = testros2_B.rtb_e_out_tmp;
  testros2_B.Q[3] = -testros2_B.rtb_e_out_tmp_p;
  testros2_B.Q[6] = 0.0;
  testros2_B.Q[1] = testros2_B.rtb_e_out_tmp_p;
  testros2_B.Q[4] = testros2_B.rtb_e_out_tmp;
  testros2_B.Q[7] = 0.0;
  testros2_B.u_dk_k = (testros2_B.rtb_e_out_tmp + testros2_B.rtb_e_out_tmp_p) *
    13.333333333333334;
  testros2_B.dv[0] = testros2_B.u_dk_k;
  testros2_B.u_dk_c = (testros2_B.rtb_e_out_tmp_p - testros2_B.rtb_e_out_tmp) *
    13.333333333333334;
  testros2_B.dv[1] = testros2_B.u_dk_c;
  testros2_B.dv[2] = -6.4;
  testros2_B.u_dk_b = (testros2_B.rtb_e_out_tmp - testros2_B.rtb_e_out_tmp_p) *
    13.333333333333334;
  testros2_B.dv[3] = testros2_B.u_dk_b;
  testros2_B.dv[4] = testros2_B.u_dk_k;
  testros2_B.dv[5] = 6.4;
  testros2_B.dv[6] = testros2_B.u_dk_k;
  testros2_B.dv[7] = testros2_B.u_dk_c;
  testros2_B.dv[8] = 6.4;
  testros2_B.dv[9] = testros2_B.u_dk_b;
  testros2_B.dv[10] = testros2_B.u_dk_k;
  testros2_B.dv[11] = -6.4;
  testros2_B.dv1[0] = 0.0;
  testros2_B.dv1[3] = 0.0;
  testros2_B.u_dk_k = 50.0 * testros2_B.In1.twist.twist.angular.z;
  testros2_B.dv1[6] = (0.0 * testros2_B.rtb_e_out_tmp - testros2_B.m4_tmp) *
    testros2_B.u_dk_k;
  testros2_B.dv1[1] = 0.0;
  testros2_B.dv1[4] = 0.0;
  testros2_B.dv1[7] = testros2_B.m3_tmp * testros2_B.u_dk_k;
  testros2_B.Q[2] = 0.0;
  testros2_B.dv1[2] = 0.0;
  testros2_B.Q[5] = 0.0;
  testros2_B.dv1[5] = 0.0;
  testros2_B.Q[8] = 1.0;
  testros2_B.dv1[8] = 0.0;

  // BusAssignment: '<Root>/Bus Assignment' incorporates:
  //   Constant: '<S1>/Constant'

  testros2_B.BusAssignment = testros2_P.Constant_Value_d;

  // MATLAB Function: '<S4>/MATLAB Function' incorporates:
  //   Constant: '<S4>/Constant'
  //   SignalConversion generated from: '<S15>/ SFunction '

  testros2_B.u_dk_k = testros2_B.u_dk[1];
  testros2_B.u_dk_c = testros2_B.u_dk[0];
  testros2_B.u_dk_b = testros2_B.u_dk[2];
  testros2_B.m3_tmp = testros2_B.u_dk[3];
  for (testros2_B.i = 0; testros2_B.i <= 0; testros2_B.i += 2) {
    tmp_3 = _mm_loadu_pd(&testros2_B.Q[testros2_B.i + 3]);
    tmp = _mm_loadu_pd(&testros2_B.Q[testros2_B.i]);
    tmp_0 = _mm_loadu_pd(&testros2_B.Q[testros2_B.i + 6]);
    _mm_storeu_pd(&testros2_B.Q_c[testros2_B.i], _mm_add_pd(_mm_add_pd
      (_mm_mul_pd(tmp_3, _mm_set1_pd(testros2_B.rtb_TmpSignalConversionAtSFun_c)),
       _mm_mul_pd(tmp, _mm_set1_pd(testros2_B.unnamed_idx_0))), _mm_mul_pd(tmp_0,
      _mm_set1_pd(testros2_B.unnamed_idx_0_tmp))));
    tmp_3 = _mm_loadu_pd(&testros2_B.dv[testros2_B.i + 3]);
    tmp = _mm_loadu_pd(&testros2_B.dv[testros2_B.i]);
    tmp_0 = _mm_loadu_pd(&testros2_B.dv[testros2_B.i + 6]);
    tmp_1 = _mm_loadu_pd(&testros2_B.dv[testros2_B.i + 9]);
    _mm_storeu_pd(&testros2_B.q_ddot[testros2_B.i], _mm_add_pd(_mm_add_pd
      (_mm_add_pd(_mm_mul_pd(tmp_3, _mm_set1_pd(testros2_B.u_dk_k)), _mm_mul_pd
                  (tmp, _mm_set1_pd(testros2_B.u_dk_c))), _mm_mul_pd(tmp_0,
      _mm_set1_pd(testros2_B.u_dk_b))), _mm_mul_pd(tmp_1, _mm_set1_pd
      (testros2_B.m3_tmp))));
  }

  for (testros2_B.i = 2; testros2_B.i < 3; testros2_B.i++) {
    testros2_B.Q_c[testros2_B.i] = (testros2_B.Q[testros2_B.i + 3] *
      testros2_B.rtb_TmpSignalConversionAtSFun_c + testros2_B.Q[testros2_B.i] *
      testros2_B.unnamed_idx_0) + testros2_B.Q[testros2_B.i + 6] *
      testros2_B.unnamed_idx_0_tmp;
    testros2_B.q_ddot[testros2_B.i] = ((testros2_B.dv[testros2_B.i + 3] *
      testros2_B.u_dk_k + testros2_B.dv[testros2_B.i] * testros2_B.u_dk_c) +
      testros2_B.dv[testros2_B.i + 6] * testros2_B.u_dk_b) +
      testros2_B.dv[testros2_B.i + 9] * testros2_B.m3_tmp;
  }

  testros2_B.m3_tmp = testros2_B.Q_c[0];
  testros2_B.m4_tmp = testros2_B.Q_c[1];
  testros2_B.u_dk_k = testros2_B.Q_c[2];
  for (testros2_B.i = 0; testros2_B.i <= 0; testros2_B.i += 2) {
    tmp_3 = _mm_loadu_pd(&testros2_B.dv1[testros2_B.i + 6]);
    tmp = _mm_loadu_pd(&testros2_B.q_ddot[testros2_B.i]);
    _mm_storeu_pd(&testros2_B.Q_c[testros2_B.i], _mm_sub_pd(tmp, _mm_add_pd
      (_mm_set1_pd(0.0 * testros2_B.m3_tmp + 0.0 * testros2_B.m4_tmp),
       _mm_mul_pd(tmp_3, _mm_set1_pd(testros2_B.u_dk_k)))));
  }

  for (testros2_B.i = 2; testros2_B.i < 3; testros2_B.i++) {
    testros2_B.Q_c[testros2_B.i] = testros2_B.q_ddot[testros2_B.i] - ((0.0 *
      testros2_B.m3_tmp + 0.0 * testros2_B.m4_tmp) + testros2_B.dv1[testros2_B.i
      + 6] * testros2_B.u_dk_k);
  }

  testros2_B.A[2] = testros2_B.m3 / 72.222222222222229;
  testros2_B.A[5] = testros2_B.m4 - testros2_B.A[2] * 0.0;
  testros2_B.A[7] = testros2_B.m4 - 0.0 * testros2_B.m3;
  testros2_B.A[8] = 10.120000000000001 - testros2_B.A[2] * testros2_B.m3;
  testros2_B.A[5] /= 72.222222222222229;
  testros2_B.A[8] -= testros2_B.A[5] * testros2_B.A[7];
  testros2_B.q_ddot[1] = testros2_B.Q_c[1] - testros2_B.Q_c[0] * 0.0;
  testros2_B.q_ddot[2] = (testros2_B.Q_c[2] - testros2_B.Q_c[0] * testros2_B.A[2])
    - testros2_B.q_ddot[1] * testros2_B.A[5];
  testros2_B.q_ddot[2] /= testros2_B.A[8];
  _mm_storeu_pd(&testros2_B.q_ddot[0], _mm_sub_pd(_mm_set_pd(testros2_B.q_ddot[1],
    testros2_B.Q_c[0]), _mm_mul_pd(_mm_set1_pd(testros2_B.q_ddot[2]), _mm_set_pd
    (testros2_B.A[7], testros2_B.m3))));
  testros2_B.q_ddot[1] /= 72.222222222222229;
  testros2_B.q_ddot[0] -= testros2_B.q_ddot[1] * 0.0;
  testros2_B.q_ddot[0] /= 72.222222222222229;
  testros2_B.rtb_e_out_tmp_p = testros2_B.In1.twist.twist.angular.z *
    -testros2_B.rtb_e_out_tmp_p;
  testros2_B.A[0] = testros2_B.rtb_e_out_tmp_p;
  testros2_B.A[3] = testros2_B.In1.twist.twist.angular.z *
    -testros2_B.rtb_e_out_tmp;
  testros2_B.m3 = testros2_B.In1.twist.twist.angular.z * 0.0;
  testros2_B.A[6] = testros2_B.m3;
  testros2_B.A[1] = testros2_B.In1.twist.twist.angular.z *
    testros2_B.rtb_e_out_tmp;
  testros2_B.A[4] = testros2_B.rtb_e_out_tmp_p;
  testros2_B.A[7] = testros2_B.m3;
  testros2_B.A[2] = testros2_B.m3;
  testros2_B.A[5] = testros2_B.m3;
  testros2_B.A[8] = testros2_B.m3;
  for (testros2_B.i = 0; testros2_B.i <= 0; testros2_B.i += 2) {
    tmp_3 = _mm_loadu_pd(&testros2_B.A[testros2_B.i + 3]);
    tmp = _mm_loadu_pd(&testros2_B.A[testros2_B.i]);
    tmp_0 = _mm_loadu_pd(&testros2_B.A[testros2_B.i + 6]);
    tmp_1 = _mm_loadu_pd(&testros2_B.q_ddot[testros2_B.i]);
    _mm_storeu_pd(&testros2_B.Q_c[testros2_B.i], _mm_sub_pd(tmp_1, _mm_add_pd
      (_mm_add_pd(_mm_mul_pd(tmp_3, _mm_set1_pd
      (testros2_B.rtb_TmpSignalConversionAtSFun_c)), _mm_mul_pd(tmp, _mm_set1_pd
      (testros2_B.unnamed_idx_0))), _mm_mul_pd(tmp_0, _mm_set1_pd
      (testros2_B.In1.twist.twist.angular.z)))));
  }

  for (testros2_B.i = 2; testros2_B.i < 3; testros2_B.i++) {
    testros2_B.Q_c[testros2_B.i] = testros2_B.q_ddot[testros2_B.i] -
      ((testros2_B.A[testros2_B.i + 3] *
        testros2_B.rtb_TmpSignalConversionAtSFun_c + testros2_B.A[testros2_B.i] *
        testros2_B.unnamed_idx_0) + testros2_B.A[testros2_B.i + 6] *
       testros2_B.unnamed_idx_0_tmp);
  }

  testros2_B.rtb_e_out_tmp = testros2_B.Q_c[1];
  testros2_B.rtb_e_out_tmp_p = testros2_B.Q_c[0];
  testros2_B.unnamed_idx_0 = testros2_B.Q_c[2];
  for (testros2_B.i = 0; testros2_B.i < 3; testros2_B.i++) {
    testros2_B.cmd_vel[testros2_B.i] = ((testros2_B.Q[3 * testros2_B.i + 1] *
      testros2_B.rtb_e_out_tmp + testros2_B.Q[3 * testros2_B.i] *
      testros2_B.rtb_e_out_tmp_p) + testros2_B.Q[3 * testros2_B.i + 2] *
      testros2_B.unnamed_idx_0) * testros2_P.Constant_Value_b +
      testros2_B.TmpSignalConversionAtSFunct[testros2_B.i];
  }

  if (testros2_B.cmd_vel[0] <= 1.0) {
    testros2_B.rtb_e_out_tmp = testros2_B.cmd_vel[0];
  } else {
    testros2_B.rtb_e_out_tmp = 1.0;
  }

  if (testros2_B.rtb_e_out_tmp >= -1.0) {
    testros2_B.cmd_vel[0] = testros2_B.rtb_e_out_tmp;
  } else {
    testros2_B.cmd_vel[0] = -1.0;
  }

  if (testros2_B.cmd_vel[1] <= 1.0) {
    testros2_B.rtb_e_out_tmp = testros2_B.cmd_vel[1];
  } else {
    testros2_B.rtb_e_out_tmp = 1.0;
  }

  if (testros2_B.rtb_e_out_tmp >= -1.0) {
    testros2_B.cmd_vel[1] = testros2_B.rtb_e_out_tmp;
  } else {
    testros2_B.cmd_vel[1] = -1.0;
  }

  if (testros2_B.cmd_vel[2] <= 2.0) {
    testros2_B.rtb_e_out_tmp = testros2_B.cmd_vel[2];
  } else {
    testros2_B.rtb_e_out_tmp = 2.0;
  }

  if (testros2_B.rtb_e_out_tmp >= -2.0) {
    testros2_B.cmd_vel[2] = testros2_B.rtb_e_out_tmp;
  } else {
    testros2_B.cmd_vel[2] = -2.0;
  }

  // BusAssignment: '<Root>/Bus Assignment'
  testros2_B.BusAssignment.linear.x = testros2_B.cmd_vel[0];
  testros2_B.BusAssignment.linear.y = testros2_B.cmd_vel[1];
  testros2_B.BusAssignment.angular.z = testros2_B.cmd_vel[2];

  // MATLABSystem: '<S7>/SinkBlock'
  Pub_testros2_62.publish(&testros2_B.BusAssignment);

  // Update for UnitDelay: '<S10>/UD'
  //
  //  Block description for '<S10>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE = testros2_B.TSamp;

  // Update for UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE_p = testros2_B.e_out;

  // Update for UnitDelay: '<S12>/UD'
  //
  //  Block description for '<S12>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE_b = testros2_B.TSamp_n;

  // Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  //   MATLAB Function: '<S3>/MATLAB Function'

  tmp_2 = _mm_add_pd(_mm_mul_pd(_mm_set_pd
    (testros2_P.DiscreteTimeIntegrator1_gainval,
     testros2_P.DiscreteTimeIntegrator_gainval), _mm_loadu_pd(&testros2_B.e_R[0])),
                     tmp_2);
  _mm_storeu_pd(&testros2_B.dv2[0], tmp_2);

  // Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  testros2_DW.DiscreteTimeIntegrator_DSTATE = testros2_B.dv2[0];

  // Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator1'
  testros2_DW.DiscreteTimeIntegrator1_DSTATE = testros2_B.dv2[1];

  // Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator2' incorporates:
  //   MATLAB Function: '<S3>/MATLAB Function'

  testros2_DW.DiscreteTimeIntegrator2_DSTATE +=
    testros2_P.DiscreteTimeIntegrator2_gainval * testros2_B.e_R[2];

  // External mode
  rtExtModeUploadCheckTrigger(1);

  {                                    // Sample time: [0.02s, 0.0s]
    rtExtModeUpload(0, (real_T)testros2_M->Timing.taskTime0);
  }

  // signal main to stop simulation
  {                                    // Sample time: [0.02s, 0.0s]
    if ((rtmGetTFinal(testros2_M)!=-1) &&
        !((rtmGetTFinal(testros2_M)-testros2_M->Timing.taskTime0) >
          testros2_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(testros2_M, "Simulation finished");
    }

    if (rtmGetStopRequested(testros2_M)) {
      rtmSetErrorStatus(testros2_M, "Simulation finished");
    }
  }

  // Update absolute time for base rate
  // The "clockTick0" counts the number of times the code of this task has
  //  been executed. The absolute time is the multiplication of "clockTick0"
  //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
  //  overflow during the application lifespan selected.

  testros2_M->Timing.taskTime0 =
    ((time_T)(++testros2_M->Timing.clockTick0)) * testros2_M->Timing.stepSize0;
}

// Model initialize function
void testros2_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));
  rtmSetTFinal(testros2_M, -1);
  testros2_M->Timing.stepSize0 = 0.02;

  // External mode info
  testros2_M->Sizes.checksums[0] = (1955842633U);
  testros2_M->Sizes.checksums[1] = (382920088U);
  testros2_M->Sizes.checksums[2] = (4113345176U);
  testros2_M->Sizes.checksums[3] = (3383921713U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[10];
    testros2_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = &rtAlwaysEnabled;
    systemRan[7] = (sysRanDType *)&testros2_DW.EnabledSubsystem_SubsysRanBC;
    systemRan[8] = &rtAlwaysEnabled;
    systemRan[9] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(testros2_M->extModeInfo,
      &testros2_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(testros2_M->extModeInfo, testros2_M->Sizes.checksums);
    rteiSetTPtr(testros2_M->extModeInfo, rtmGetTPtr(testros2_M));
  }

  // data type transition information
  {
    static DataTypeTransInfo dtInfo;
    testros2_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 32;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    // Block I/O transition table
    dtInfo.BTransTable = &rtBTransTable;

    // Parameters transition table
    dtInfo.PTransTable = &rtPTransTable;
  }

  // InitializeConditions for UnitDelay: '<S10>/UD'
  //
  //  Block description for '<S10>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE = testros2_P.DiscreteDerivative_ICPrevScaled;

  // InitializeConditions for UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE_p = testros2_P.DiscreteDerivative1_ICPrevScale;

  // InitializeConditions for UnitDelay: '<S12>/UD'
  //
  //  Block description for '<S12>/UD':
  //
  //   Store in Global RAM

  testros2_DW.UD_DSTATE_b = testros2_P.DiscreteDerivative2_ICPrevScale;

  // InitializeConditions for DiscreteIntegrator: '<S2>/Discrete-Time Integrator' 
  testros2_DW.DiscreteTimeIntegrator_DSTATE =
    testros2_P.DiscreteTimeIntegrator_IC;

  // InitializeConditions for DiscreteIntegrator: '<S2>/Discrete-Time Integrator1' 
  testros2_DW.DiscreteTimeIntegrator1_DSTATE =
    testros2_P.DiscreteTimeIntegrator1_IC;

  // InitializeConditions for DiscreteIntegrator: '<S2>/Discrete-Time Integrator2' 
  testros2_DW.DiscreteTimeIntegrator2_DSTATE =
    testros2_P.DiscreteTimeIntegrator2_IC;

  // SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem'
  // SystemInitialize for SignalConversion generated from: '<S16>/In1' incorporates:
  //   Outport: '<S16>/Out1'

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
void testros2_terminate(void)
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

//
// File trailer for generated code.
//
// [EOF]
//
