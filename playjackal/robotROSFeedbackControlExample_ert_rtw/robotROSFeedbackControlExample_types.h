/*
 * robotROSFeedbackControlExample_types.h
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "robotROSFeedbackControlExample".
 *
 * Model version              : 1.84
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C++ source code generated on : Thu Nov  1 12:31:15 2018
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_robotROSFeedbackControlExample_types_h_
#define RTW_HEADER_robotROSFeedbackControlExample_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Vector3_

typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Vector3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Twist_
#define DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Twist_

typedef struct {
  SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Vector3 Linear;
  SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Vector3 Angular;
} SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Twist;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_ros_time_Time_

typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_robotROSFeedbackControlExample_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_std_msgs_Header_

typedef struct {
  uint32_T Seq;
  uint8_T FrameId[128];
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;
  SL_Bus_robotROSFeedbackControlExample_ros_time_Time Stamp;
} SL_Bus_robotROSFeedbackControlExample_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Point_

typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Point;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlEx_Quaternion_pzdb3q_
#define DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlEx_Quaternion_pzdb3q_

typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
} SL_Bus_robotROSFeedbackControlEx_Quaternion_pzdb3q;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Pose_
#define DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Pose_

typedef struct {
  SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Point Position;
  SL_Bus_robotROSFeedbackControlEx_Quaternion_pzdb3q Orientation;
} SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Pose;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlEx_PoseWithCovariance_4of9uo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlEx_PoseWithCovariance_4of9uo_

typedef struct {
  real_T Covariance[36];
  SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Pose Pose;
} SL_Bus_robotROSFeedbackControlEx_PoseWithCovariance_4of9uo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlEx_TwistWithCovariance_wj8h42_
#define DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlEx_TwistWithCovariance_wj8h42_

typedef struct {
  real_T Covariance[36];
  SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Twist Twist;
} SL_Bus_robotROSFeedbackControlEx_TwistWithCovariance_wj8h42;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_nav_msgs_Odometry_
#define DEFINED_TYPEDEF_FOR_SL_Bus_robotROSFeedbackControlExample_nav_msgs_Odometry_

typedef struct {
  uint8_T ChildFrameId[128];
  SL_Bus_ROSVariableLengthArrayInfo ChildFrameId_SL_Info;
  SL_Bus_robotROSFeedbackControlExample_std_msgs_Header Header;
  SL_Bus_robotROSFeedbackControlEx_PoseWithCovariance_4of9uo Pose;
  SL_Bus_robotROSFeedbackControlEx_TwistWithCovariance_wj8h42 Twist;
} SL_Bus_robotROSFeedbackControlExample_nav_msgs_Odometry;

#endif

#ifndef typedef_ExampleHelperSimulationRateCo_T
#define typedef_ExampleHelperSimulationRateCo_T

typedef struct {
  int32_T isInitialized;
} ExampleHelperSimulationRateCo_T;

#endif                                 /*typedef_ExampleHelperSimulationRateCo_T*/

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_block_T;

#endif                                 /*typedef_robotics_slros_internal_block_T*/

#ifndef typedef_robotics_slros_internal_blo_j_T
#define typedef_robotics_slros_internal_blo_j_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_blo_j_T;

#endif                                 /*typedef_robotics_slros_internal_blo_j_T*/

/* Parameters (default storage) */
typedef struct P_robotROSFeedbackControlExample_T_
  P_robotROSFeedbackControlExample_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_robotROSFeedbackControlExample_T
  RT_MODEL_robotROSFeedbackControlExample_T;

#endif                                 /* RTW_HEADER_robotROSFeedbackControlExample_types_h_ */
