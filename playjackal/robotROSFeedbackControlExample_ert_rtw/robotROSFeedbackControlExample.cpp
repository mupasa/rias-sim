/*
 * robotROSFeedbackControlExample.cpp
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "robotROSFeedbackControlExample".
 *
 * Model version              : 1.84
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C++ source code generated on : Thu Nov  1 11:28:40 2018
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "robotROSFeedbackControlExample.h"
#include "robotROSFeedbackControlExample_private.h"

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2((real_T)u0_0, (real_T)u1_0);
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

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  real_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

void robotROSFeedbackControlExampleModelClass::matlabCodegenHandle_matlabCod_j
  (robotics_slros_internal_blo_j_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

void robotROSFeedbackControlExampleModelClass::matlabCodegenHandle_matlabCodeg
  (robotics_slros_internal_block_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

/* Model step function */
void robotROSFeedbackControlExampleModelClass::step()
{
  /* local block i/o variables */
  boolean_T rtb_SourceBlock_o1;
  real_T rtb_Distance;
  real_T rtb_Sum3;
  real_T Switch;
  real_T q_idx_1;
  real_T q_idx_2;

  /* Outputs for Atomic SubSystem: '<Root>/Subscribe' */
  /* MATLABSystem: '<S4>/SourceBlock' */
  rtb_SourceBlock_o1 = Sub_robotROSFeedbackControlExample_126.getLatestMessage
    (&robotROSFeedbackControlExample_B.b_varargout_2);

  /* Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S11>/Enable'
   */
  if (rtb_SourceBlock_o1) {
    /* Inport: '<S11>/In1' incorporates:
     *  MATLABSystem: '<S4>/SourceBlock'
     */
    robotROSFeedbackControlExample_B.In1 =
      robotROSFeedbackControlExample_B.b_varargout_2;
  }

  /* End of Outputs for SubSystem: '<S4>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<Root>/Subscribe' */

  /* MATLAB Function: '<Root>/Conversion' */
  /* MATLAB Function 'Conversion': '<S2>:1' */
  /* '<S2>:1:5' */
  rtb_Sum3 = 1.0 / sqrt
    (((robotROSFeedbackControlExample_B.In1.Pose.Pose.Orientation.W *
       robotROSFeedbackControlExample_B.In1.Pose.Pose.Orientation.W +
       robotROSFeedbackControlExample_B.In1.Pose.Pose.Orientation.X *
       robotROSFeedbackControlExample_B.In1.Pose.Pose.Orientation.X) +
      robotROSFeedbackControlExample_B.In1.Pose.Pose.Orientation.Y *
      robotROSFeedbackControlExample_B.In1.Pose.Pose.Orientation.Y) +
     robotROSFeedbackControlExample_B.In1.Pose.Pose.Orientation.Z *
     robotROSFeedbackControlExample_B.In1.Pose.Pose.Orientation.Z);
  Switch = robotROSFeedbackControlExample_B.In1.Pose.Pose.Orientation.W *
    rtb_Sum3;
  q_idx_1 = robotROSFeedbackControlExample_B.In1.Pose.Pose.Orientation.X *
    rtb_Sum3;
  q_idx_2 = robotROSFeedbackControlExample_B.In1.Pose.Pose.Orientation.Y *
    rtb_Sum3;
  rtb_Sum3 *= robotROSFeedbackControlExample_B.In1.Pose.Pose.Orientation.Z;

  /* Outputs for Enabled SubSystem: '<Root>/Command Velocity Publisher' incorporates:
   *  EnablePort: '<S1>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<Root>/Proportional Controller' incorporates:
   *  EnablePort: '<S3>/Enable'
   */
  /* '<S2>:1:6' */
  if (rtb_SourceBlock_o1) {
    /* Sum: '<S3>/Sum1' incorporates:
     *  Constant: '<Root>/Desired Position'
     *  SignalConversion: '<Root>/SigConversion_InsertedFor_Bus Selector3_at_outport_1'
     */
    robotROSFeedbackControlExample_B.Sum1 =
      robotROSFeedbackControlExample_P.DesiredPosition_Value[1] -
      robotROSFeedbackControlExample_B.In1.Pose.Pose.Position.Y;

    /* Sum: '<S3>/Sum' incorporates:
     *  Constant: '<Root>/Desired Position'
     *  SignalConversion: '<Root>/SigConversion_InsertedFor_Bus Selector3_at_outport_0'
     */
    rtb_Distance = robotROSFeedbackControlExample_P.DesiredPosition_Value[0] -
      robotROSFeedbackControlExample_B.In1.Pose.Pose.Position.X;

    /* Sum: '<S3>/Sum3' incorporates:
     *  MATLAB Function: '<Root>/Conversion'
     *  Trigonometry: '<S3>/Desired Yaw'
     */
    rtb_Sum3 = rt_atan2d_snf(robotROSFeedbackControlExample_B.Sum1, rtb_Distance)
      - rt_atan2d_snf((q_idx_1 * q_idx_2 + Switch * rtb_Sum3) * 2.0, ((Switch *
      Switch + q_idx_1 * q_idx_1) - q_idx_2 * q_idx_2) - rtb_Sum3 * rtb_Sum3);

    /* Fcn: '<S3>/Distance' */
    robotROSFeedbackControlExample_B.Sum1 = rt_powd_snf(rtb_Distance, 2.0) +
      rt_powd_snf(robotROSFeedbackControlExample_B.Sum1, 2.0);
    if (robotROSFeedbackControlExample_B.Sum1 < 0.0) {
      robotROSFeedbackControlExample_B.Sum1 = -sqrt
        (-robotROSFeedbackControlExample_B.Sum1);
    } else {
      robotROSFeedbackControlExample_B.Sum1 = sqrt
        (robotROSFeedbackControlExample_B.Sum1);
    }

    /* End of Fcn: '<S3>/Distance' */

    /* Switch: '<S3>/Switch' incorporates:
     *  Constant: '<S3>/Constant'
     *  Constant: '<S3>/Stop'
     *  Constant: '<S8>/Constant'
     *  Gain: '<S10>/Slider Gain'
     *  RelationalOperator: '<S8>/Compare'
     *  Switch: '<S3>/Switch1'
     */
    if (robotROSFeedbackControlExample_B.Sum1 <=
        robotROSFeedbackControlExample_P.DistanceThreshold_const) {
      Switch = robotROSFeedbackControlExample_P.Stop_Value;
      rtb_Sum3 = robotROSFeedbackControlExample_P.Stop_Value;
    } else {
      Switch = robotROSFeedbackControlExample_P.LinearVelocitySlider_gain *
        robotROSFeedbackControlExample_P.Constant_Value_m;

      /* Gain: '<S9>/Slider Gain' incorporates:
       *  Constant: '<S3>/Constant'
       *  Fcn: '<S3>/Bound [-pi,pi]'
       *  Gain: '<S10>/Slider Gain'
       */
      rtb_Sum3 = robotROSFeedbackControlExample_P.GainSlider_gain *
        rt_atan2d_snf(sin(rtb_Sum3), cos(rtb_Sum3));

      /* Saturate: '<S3>/Saturation' */
      if (rtb_Sum3 > robotROSFeedbackControlExample_P.Saturation_UpperSat) {
        rtb_Sum3 = robotROSFeedbackControlExample_P.Saturation_UpperSat;
      } else {
        if (rtb_Sum3 < robotROSFeedbackControlExample_P.Saturation_LowerSat) {
          rtb_Sum3 = robotROSFeedbackControlExample_P.Saturation_LowerSat;
        }
      }

      /* End of Saturate: '<S3>/Saturation' */
    }

    /* End of Switch: '<S3>/Switch' */

    /* BusAssignment: '<S1>/Bus Assignment1' incorporates:
     *  Constant: '<S6>/Constant'
     */
    robotROSFeedbackControlExample_B.BusAssignment1 =
      robotROSFeedbackControlExample_P.Constant_Value_o;
    robotROSFeedbackControlExample_B.BusAssignment1.Linear.X = Switch;
    robotROSFeedbackControlExample_B.BusAssignment1.Angular.Z = rtb_Sum3;

    /* Outputs for Atomic SubSystem: '<S1>/Publish2' */
    /* MATLABSystem: '<S7>/SinkBlock' */
    Pub_robotROSFeedbackControlExample_128.publish
      (&robotROSFeedbackControlExample_B.BusAssignment1);

    /* End of Outputs for SubSystem: '<S1>/Publish2' */
  }

  /* End of Outputs for SubSystem: '<Root>/Proportional Controller' */
  /* End of Outputs for SubSystem: '<Root>/Command Velocity Publisher' */
}

/* Model initialize function */
void robotROSFeedbackControlExampleModelClass::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(getRTM(), (NULL));

  /* block I/O */
  (void) memset(((void *) &robotROSFeedbackControlExample_B), 0,
                sizeof(B_robotROSFeedbackControlExample_T));

  /* states (dwork) */
  (void) memset((void *)&robotROSFeedbackControlExample_DW, 0,
                sizeof(DW_robotROSFeedbackControlExample_T));

  {
    static const char_T tmp[35] = { '/', 'j', 'a', 'c', 'k', 'a', 'l', '_', 'v',
      'e', 'l', 'o', 'c', 'i', 't', 'y', '_', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      'l', 'e', 'r', '/', 'c', 'm', 'd', '_', 'v', 'e', 'l' };

    static const char_T tmp_0[32] = { '/', 'j', 'a', 'c', 'k', 'a', 'l', '_',
      'v', 'e', 'l', 'o', 'c', 'i', 't', 'y', '_', 'c', 'o', 'n', 't', 'r', 'o',
      'l', 'l', 'e', 'r', '/', 'o', 'd', 'o', 'm' };

    char_T tmp_1[33];
    int32_T i;

    /* Start for Atomic SubSystem: '<Root>/Subscribe' */
    /* Start for MATLABSystem: '<S4>/SourceBlock' */
    robotROSFeedbackControlExample_DW.obj_n.matlabCodegenIsDeleted = true;
    robotROSFeedbackControlExample_DW.obj_n.isInitialized = 0;
    robotROSFeedbackControlExample_DW.obj_n.matlabCodegenIsDeleted = false;
    robotROSFeedbackControlExample_DW.objisempty = true;
    robotROSFeedbackControlExample_DW.obj_n.isSetupComplete = false;
    robotROSFeedbackControlExample_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 32; i++) {
      tmp_1[i] = tmp_0[i];
    }

    tmp_1[32] = '\x00';
    Sub_robotROSFeedbackControlExample_126.createSubscriber(tmp_1, 51);
    robotROSFeedbackControlExample_DW.obj_n.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S4>/SourceBlock' */
    /* End of Start for SubSystem: '<Root>/Subscribe' */

    /* Start for Enabled SubSystem: '<Root>/Command Velocity Publisher' */
    /* Start for Atomic SubSystem: '<S1>/Publish2' */
    /* Start for MATLABSystem: '<S7>/SinkBlock' */
    robotROSFeedbackControlExample_DW.obj.matlabCodegenIsDeleted = true;
    robotROSFeedbackControlExample_DW.obj.isInitialized = 0;
    robotROSFeedbackControlExample_DW.obj.matlabCodegenIsDeleted = false;
    robotROSFeedbackControlExample_DW.objisempty_g = true;
    robotROSFeedbackControlExample_DW.obj.isSetupComplete = false;
    robotROSFeedbackControlExample_DW.obj.isInitialized = 1;
    for (i = 0; i < 35; i++) {
      robotROSFeedbackControlExample_B.cv0[i] = tmp[i];
    }

    robotROSFeedbackControlExample_B.cv0[35] = '\x00';
    Pub_robotROSFeedbackControlExample_128.createPublisher
      (robotROSFeedbackControlExample_B.cv0, 105);
    robotROSFeedbackControlExample_DW.obj.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S7>/SinkBlock' */
    /* End of Start for SubSystem: '<S1>/Publish2' */
    /* End of Start for SubSystem: '<Root>/Command Velocity Publisher' */
    /* Start for MATLABSystem: '<Root>/Simulation Rate Control' */
    robotROSFeedbackControlExample_DW.obj_h.isInitialized = 0;
    robotROSFeedbackControlExample_DW.objisempty_f = true;
    robotROSFeedbackControlExample_DW.obj_h.isInitialized = 1;
  }

  /* SystemInitialize for Atomic SubSystem: '<Root>/Subscribe' */
  /* SystemInitialize for Enabled SubSystem: '<S4>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S11>/Out1' */
  robotROSFeedbackControlExample_B.In1 =
    robotROSFeedbackControlExample_P.Out1_Y0;

  /* End of SystemInitialize for SubSystem: '<S4>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<Root>/Subscribe' */
}

/* Model terminate function */
void robotROSFeedbackControlExampleModelClass::terminate()
{
  /* Terminate for Atomic SubSystem: '<Root>/Subscribe' */
  /* Terminate for MATLABSystem: '<S4>/SourceBlock' */
  matlabCodegenHandle_matlabCod_j(&robotROSFeedbackControlExample_DW.obj_n);

  /* End of Terminate for SubSystem: '<Root>/Subscribe' */

  /* Terminate for Enabled SubSystem: '<Root>/Command Velocity Publisher' */
  /* Terminate for Atomic SubSystem: '<S1>/Publish2' */
  /* Terminate for MATLABSystem: '<S7>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&robotROSFeedbackControlExample_DW.obj);

  /* End of Terminate for SubSystem: '<S1>/Publish2' */
  /* End of Terminate for SubSystem: '<Root>/Command Velocity Publisher' */
}

/* Constructor */
robotROSFeedbackControlExampleModelClass::
  robotROSFeedbackControlExampleModelClass()
{
  static const P_robotROSFeedbackControlExample_T
    robotROSFeedbackControlExample_P_temp = {
    /* Mask Parameter: DistanceThreshold_const
     * Referenced by: '<S8>/Constant'
     */
    0.1,

    /* Mask Parameter: GainSlider_gain
     * Referenced by: '<S9>/Slider Gain'
     */
    3.0,

    /* Mask Parameter: LinearVelocitySlider_gain
     * Referenced by: '<S10>/Slider Gain'
     */
    0.1,

    /* Computed Parameter: Out1_Y0
     * Referenced by: '<S11>/Out1'
     */
    {
      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                /* ChildFrameId */

      {
        0U,                            /* CurrentLength */
        0U                             /* ReceivedLength */
      },                               /* ChildFrameId_SL_Info */

      {
        0U,                            /* Seq */

        {
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U }
        ,                              /* FrameId */

        {
          0U,                          /* CurrentLength */
          0U                           /* ReceivedLength */
        },                             /* FrameId_SL_Info */

        {
          0.0,                         /* Sec */
          0.0                          /* Nsec */
        }                              /* Stamp */
      },                               /* Header */

      {
        {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
        ,                              /* Covariance */

        {
          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0                        /* Z */
          },                           /* Position */

          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0,                       /* Z */
            0.0                        /* W */
          }                            /* Orientation */
        }                              /* Pose */
      },                               /* Pose */

      {
        {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
        ,                              /* Covariance */

        {
          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0                        /* Z */
          },                           /* Linear */

          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0                        /* Z */
          }                            /* Angular */
        }                              /* Twist */
      }                                /* Twist */
    },

    /* Computed Parameter: Constant_Value
     * Referenced by: '<S4>/Constant'
     */
    {
      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                /* ChildFrameId */

      {
        0U,                            /* CurrentLength */
        0U                             /* ReceivedLength */
      },                               /* ChildFrameId_SL_Info */

      {
        0U,                            /* Seq */

        {
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U }
        ,                              /* FrameId */

        {
          0U,                          /* CurrentLength */
          0U                           /* ReceivedLength */
        },                             /* FrameId_SL_Info */

        {
          0.0,                         /* Sec */
          0.0                          /* Nsec */
        }                              /* Stamp */
      },                               /* Header */

      {
        {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
        ,                              /* Covariance */

        {
          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0                        /* Z */
          },                           /* Position */

          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0,                       /* Z */
            0.0                        /* W */
          }                            /* Orientation */
        }                              /* Pose */
      },                               /* Pose */

      {
        {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
        ,                              /* Covariance */

        {
          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0                        /* Z */
          },                           /* Linear */

          {
            0.0,                       /* X */
            0.0,                       /* Y */
            0.0                        /* Z */
          }                            /* Angular */
        }                              /* Twist */
      }                                /* Twist */
    },

    /* Computed Parameter: Constant_Value_o
     * Referenced by: '<S6>/Constant'
     */
    {
      {
        0.0,                           /* X */
        0.0,                           /* Y */
        0.0                            /* Z */
      },                               /* Linear */

      {
        0.0,                           /* X */
        0.0,                           /* Y */
        0.0                            /* Z */
      }                                /* Angular */
    },

    /* Expression: 0.5
     * Referenced by: '<S3>/Saturation'
     */
    0.5,

    /* Expression: -0.5
     * Referenced by: '<S3>/Saturation'
     */
    -0.5,

    /* Computed Parameter: LinearVelocityv_Y0
     * Referenced by: '<S3>/Linear Velocity (v)'
     */
    0.0,

    /* Computed Parameter: AngularVelocityw_Y0
     * Referenced by: '<S3>/Angular Velocity (w)'
     */
    0.0,

    /* Expression: 1
     * Referenced by: '<S3>/Constant'
     */
    1.0,

    /* Expression: 0
     * Referenced by: '<S3>/Stop'
     */
    0.0,

    /* Expression: [10 10]
     * Referenced by: '<Root>/Desired Position'
     */
    { 10.0, 10.0 }
  };                                   /* Modifiable parameters */

  /* Initialize tunable parameters */
  robotROSFeedbackControlExample_P = robotROSFeedbackControlExample_P_temp;
}

/* Destructor */
robotROSFeedbackControlExampleModelClass::
  ~robotROSFeedbackControlExampleModelClass()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_robotROSFeedbackControlExample_T
  * robotROSFeedbackControlExampleModelClass::getRTM()
{
  return (&robotROSFeedbackControlExample_M);
}
