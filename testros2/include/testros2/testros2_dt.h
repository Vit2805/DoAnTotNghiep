//
//  testros2_dt.h
//
//  Code generation for model "testros2".
//
//  Model version              : 1.83
//  Simulink Coder version : 24.1 (R2024a) 19-Nov-2023
//  C++ source code generated on : Thu Jan 22 22:29:37 2026
//
//  Target selection: ert.tlc
//  Embedded hardware selection: Intel->x86-64 (Windows64)
//  Code generation objectives: Unspecified
//  Validation result: Not run


#include "ext_types.h"

// data type size table
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T),
  sizeof(int32_T),
  sizeof(SL_Bus_geometry_msgs_Vector3),
  sizeof(SL_Bus_geometry_msgs_Twist),
  sizeof(SL_Bus_builtin_interfaces_Time),
  sizeof(SL_Bus_ROSVariableLengthArrayInfo),
  sizeof(SL_Bus_std_msgs_Header),
  sizeof(SL_Bus_geometry_msgs_Point),
  sizeof(SL_Bus_geometry_msgs_Quaternion),
  sizeof(SL_Bus_geometry_msgs_Pose),
  sizeof(SL_Bus_geometry_msgs_PoseWithCovariance),
  sizeof(SL_Bus_geometry_msgs_TwistWithCovariance),
  sizeof(SL_Bus_nav_msgs_Odometry),
  sizeof(ros_slros2_internal_block_Pub_T),
  sizeof(ros_slros2_internal_block_Sub_T),
  sizeof(uint_T),
  sizeof(char_T),
  sizeof(uchar_T),
  sizeof(time_T)
};

// data type name table
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T",
  "physical_connection",
  "SL_Bus_geometry_msgs_Vector3",
  "SL_Bus_geometry_msgs_Twist",
  "SL_Bus_builtin_interfaces_Time",
  "SL_Bus_ROSVariableLengthArrayInfo",
  "SL_Bus_std_msgs_Header",
  "SL_Bus_geometry_msgs_Point",
  "SL_Bus_geometry_msgs_Quaternion",
  "SL_Bus_geometry_msgs_Pose",
  "SL_Bus_geometry_msgs_PoseWithCovariance",
  "SL_Bus_geometry_msgs_TwistWithCovariance",
  "SL_Bus_nav_msgs_Odometry",
  "ros_slros2_internal_block_Pub_T",
  "ros_slros2_internal_block_Sub_T",
  "uint_T",
  "char_T",
  "uchar_T",
  "time_T"
};

// data type transitions for block I/O structure
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&testros2_B.In1), 25, 0, 1 },

  { (char_T *)(&testros2_B.x), 0, 0, 13 }
  ,

  { (char_T *)(&testros2_DW.obj), 26, 0, 1 },

  { (char_T *)(&testros2_DW.obj_n), 27, 0, 1 },

  { (char_T *)(&testros2_DW.UD_DSTATE), 0, 0, 6 },

  { (char_T *)(&testros2_DW.Scope2_PWORK.LoggedData), 11, 0, 7 },

  { (char_T *)(&testros2_DW.EnabledSubsystem_SubsysRanBC), 2, 0, 1 }
};

// data type transition table for block I/O structure
static DataTypeTransitionTable rtBTransTable = {
  7U,
  rtBTransitions
};

// data type transitions for Parameters structure
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&testros2_P.DiscreteDerivative_ICPrevScaled), 0, 0, 3 },

  { (char_T *)(&testros2_P.Out1_Y0), 25, 0, 1 },

  { (char_T *)(&testros2_P.Constant_Value), 25, 0, 1 },

  { (char_T *)(&testros2_P.Constant_Value_d), 16, 0, 1 },

  { (char_T *)(&testros2_P.TSamp_WtEt), 0, 0, 19 }
};

// data type transition table for Parameters structure
static DataTypeTransitionTable rtPTransTable = {
  5U,
  rtPTransitions
};

// [EOF] testros2_dt.h
