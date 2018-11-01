#ifndef __c9_sharedTrackingLibrary_h__
#define __c9_sharedTrackingLibrary_h__

/* Type Definitions */
#ifndef struct_tag_sGeqiQRKfMYBjhkuVHSNkYD
#define struct_tag_sGeqiQRKfMYBjhkuVHSNkYD

struct tag_sGeqiQRKfMYBjhkuVHSNkYD
{
  char_T FcnName[18];
  boolean_T IsSimulinkFcn;
  real_T NumberOfExtraArgumentInports;
  char_T JacobianFcnName[26];
  real_T HasJacobian;
  boolean_T HasAdditiveNoise;
};

#endif                                 /*struct_tag_sGeqiQRKfMYBjhkuVHSNkYD*/

#ifndef typedef_c9_sGeqiQRKfMYBjhkuVHSNkYD
#define typedef_c9_sGeqiQRKfMYBjhkuVHSNkYD

typedef struct tag_sGeqiQRKfMYBjhkuVHSNkYD c9_sGeqiQRKfMYBjhkuVHSNkYD;

#endif                                 /*typedef_c9_sGeqiQRKfMYBjhkuVHSNkYD*/

#ifndef typedef_SFc9_sharedTrackingLibraryInstanceStruct
#define typedef_SFc9_sharedTrackingLibraryInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c9_sfEvent;
  boolean_T c9_doneDoubleBufferReInit;
  uint8_T c9_is_active_c9_sharedTrackingLibrary;
  c9_sGeqiQRKfMYBjhkuVHSNkYD c9_pS;
  void *c9_fEmlrtCtx;
  real_T (*c9_x)[4];
  real_T (*c9_xNew)[4];
  real_T (*c9_P)[16];
  real_T (*c9_Q)[4];
  real_T *c9_uState;
  boolean_T *c9_unused;
  real_T (*c9_b_P)[16];
} SFc9_sharedTrackingLibraryInstanceStruct;

#endif                                 /*typedef_SFc9_sharedTrackingLibraryInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c9_sharedTrackingLibrary_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c9_sharedTrackingLibrary_get_check_sum(mxArray *plhs[]);
extern void c9_sharedTrackingLibrary_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
