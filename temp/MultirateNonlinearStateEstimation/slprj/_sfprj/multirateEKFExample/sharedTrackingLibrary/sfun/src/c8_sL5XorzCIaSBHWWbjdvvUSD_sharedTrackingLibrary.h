#ifndef __c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_h__
#define __c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_h__

/* Type Definitions */
#ifndef typedef_c8_sL5XorzCIaSBHWWbjdvvUSD_cell_2
#define typedef_c8_sL5XorzCIaSBHWWbjdvvUSD_cell_2

typedef struct {
  real_T f1[4];
  real_T f2[2];
} c8_sL5XorzCIaSBHWWbjdvvUSD_cell_2;

#endif                                 /*typedef_c8_sL5XorzCIaSBHWWbjdvvUSD_cell_2*/

#ifndef struct_tag_sUAIaS8TPWuzgg7KvQbnkvB
#define struct_tag_sUAIaS8TPWuzgg7KvQbnkvB

struct tag_sUAIaS8TPWuzgg7KvQbnkvB
{
  char_T FcnName[19];
  boolean_T IsSimulinkFcn;
  real_T NumberOfExtraArgumentInports;
  boolean_T HasJacobian;
  boolean_T HasAdditiveNoise;
};

#endif                                 /*struct_tag_sUAIaS8TPWuzgg7KvQbnkvB*/

#ifndef typedef_c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB
#define typedef_c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB

typedef struct tag_sUAIaS8TPWuzgg7KvQbnkvB
  c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB;

#endif                                 /*typedef_c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB*/

#ifndef typedef_SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
#define typedef_SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent;
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_doneDoubleBufferReInit;
  uint8_T c8_sL5XorzCIaSBHWWbjdvvUSD_is_active_c8_sL5XorzCIaSBHWWbjdvvUSD_;
  c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB
    c8_sL5XorzCIaSBHWWbjdvvUSD_pM;
  void *c8_sL5XorzCIaSBHWWbjdvvUSD_fEmlrtCtx;
  real_T (*c8_sL5XorzCIaSBHWWbjdvvUSD_x)[4];
  real_T (*c8_sL5XorzCIaSBHWWbjdvvUSD_xNew)[4];
  real_T (*c8_sL5XorzCIaSBHWWbjdvvUSD_P)[16];
  real_T (*c8_sL5XorzCIaSBHWWbjdvvUSD_b_P)[16];
  real_T (*c8_sL5XorzCIaSBHWWbjdvvUSD_yMeas)[2];
  real_T (*c8_sL5XorzCIaSBHWWbjdvvUSD_R)[4];
  real_T *c8_sL5XorzCIaSBHWWbjdvvUSD_uMeas;
  boolean_T *c8_sL5XorzCIaSBHWWbjdvvUSD_blockOrdering;
  boolean_T *c8_sL5XorzCIaSBHWWbjdvvUSD_b_blockOrdering;
} SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct;

#endif                                 /*typedef_SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_get_check_sum
  (mxArray *plhs[]);
extern void c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_method_dispatcher
  (SimStruct *S, int_T method, void *data);

#endif
