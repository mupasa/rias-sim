/*
 *  rtmodel.h:
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
/*
 * rtmodel.h
 *
 * Code generation for Simulink model "robotROSFeedbackControlExample".
 *
 * Simulink Coder version                : 9.0 (R2018b) 24-May-2018
 * C++ source code generated on : Thu Nov  1 11:28:40 2018
 *
 * Note that the generated code is not dependent on this header file.
 * The file is used in cojuction with the automatic build procedure.
 * It is included by the sample main executable harness
 * MATLAB/rtw/c/src/common/rt_main.c.
 *
 */
#ifndef RTW_HEADER_rtmodel_h_
#define RTW_HEADER_rtmodel_h_

/*
 *  Includes the appropriate headers when we are using rtModel
 */
#include "robotROSFeedbackControlExample.h"
#define GRTINTERFACE                   0

/*
 * ROOT_IO_FORMAT: 0 (Individual arguments)
 * ROOT_IO_FORMAT: 1 (Structure reference)
 * ROOT_IO_FORMAT: 2 (Part of model data structure)
 */
# define ROOT_IO_FORMAT                2

/* Macros generated for backwards compatibility  */
#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((void*) 0)
#endif

#include "robotROSFeedbackControlExample.h"

/*
 * ROOT_IO_FORMAT: 0 (Individual arguments)
 * ROOT_IO_FORMAT: 1 (Structure reference)
 * ROOT_IO_FORMAT: 2 (Part of model data structure)
 */
# define ROOT_IO_FORMAT                2

/* Macros generated for backwards compatibility  */
#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((void*) 0)
#endif

#if 0

/* Example parameter data definition with initial values */
static P_robotROSFeedbackControlExample_T robotROSFeedbackControlExample_P = {
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
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  /* ChildFrameId */

    {
      0U,                              /* CurrentLength */
      0U                               /* ReceivedLength */
    },                                 /* ChildFrameId_SL_Info */

    {
      0U,                              /* Seq */

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                /* FrameId */

      {
        0U,                            /* CurrentLength */
        0U                             /* ReceivedLength */
      },                               /* FrameId_SL_Info */

      {
        0.0,                           /* Sec */
        0.0                            /* Nsec */
      }                                /* Stamp */
    },                                 /* Header */

    {
      {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
      ,                                /* Covariance */

      {
        {
          0.0,                         /* X */
          0.0,                         /* Y */
          0.0                          /* Z */
        },                             /* Position */

        {
          0.0,                         /* X */
          0.0,                         /* Y */
          0.0,                         /* Z */
          0.0                          /* W */
        }                              /* Orientation */
      }                                /* Pose */
    },                                 /* Pose */

    {
      {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
      ,                                /* Covariance */

      {
        {
          0.0,                         /* X */
          0.0,                         /* Y */
          0.0                          /* Z */
        },                             /* Linear */

        {
          0.0,                         /* X */
          0.0,                         /* Y */
          0.0                          /* Z */
        }                              /* Angular */
      }                                /* Twist */
    }                                  /* Twist */
  },

  /* Computed Parameter: Constant_Value
   * Referenced by: '<S4>/Constant'
   */
  {
    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  /* ChildFrameId */

    {
      0U,                              /* CurrentLength */
      0U                               /* ReceivedLength */
    },                                 /* ChildFrameId_SL_Info */

    {
      0U,                              /* Seq */

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                /* FrameId */

      {
        0U,                            /* CurrentLength */
        0U                             /* ReceivedLength */
      },                               /* FrameId_SL_Info */

      {
        0.0,                           /* Sec */
        0.0                            /* Nsec */
      }                                /* Stamp */
    },                                 /* Header */

    {
      {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
      ,                                /* Covariance */

      {
        {
          0.0,                         /* X */
          0.0,                         /* Y */
          0.0                          /* Z */
        },                             /* Position */

        {
          0.0,                         /* X */
          0.0,                         /* Y */
          0.0,                         /* Z */
          0.0                          /* W */
        }                              /* Orientation */
      }                                /* Pose */
    },                                 /* Pose */

    {
      {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
      ,                                /* Covariance */

      {
        {
          0.0,                         /* X */
          0.0,                         /* Y */
          0.0                          /* Z */
        },                             /* Linear */

        {
          0.0,                         /* X */
          0.0,                         /* Y */
          0.0                          /* Z */
        }                              /* Angular */
      }                                /* Twist */
    }                                  /* Twist */
  },

  /* Computed Parameter: Constant_Value_o
   * Referenced by: '<S6>/Constant'
   */
  {
    {
      0.0,                             /* X */
      0.0,                             /* Y */
      0.0                              /* Z */
    },                                 /* Linear */

    {
      0.0,                             /* X */
      0.0,                             /* Y */
      0.0                              /* Z */
    }                                  /* Angular */
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
};                                     /* Modifiable parameters */

#endif

#define MODEL_CLASSNAME                robotROSFeedbackControlExampleModelClass
#define MODEL_STEPNAME                 step
#endif                                 /* RTW_HEADER_rtmodel_h_ */
