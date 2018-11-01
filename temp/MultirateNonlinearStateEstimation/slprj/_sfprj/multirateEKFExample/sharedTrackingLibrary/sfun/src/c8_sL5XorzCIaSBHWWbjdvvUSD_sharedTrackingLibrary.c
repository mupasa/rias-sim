/* Include files */

#include "sharedTrackingLibrary_sfun.h"
#include "c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "sharedTrackingLibrary_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(S);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c8_sL5XorzCIaSBHWWbjdvvUSD_debug_family_names[12] = {
  "MeasurementJacobianFcnH", "extraArgs", "pM", "nargin", "nargout", "x", "P",
  "yMeas", "R", "uMeas", "blockOrdering", "xNew" };

/* Function Declarations */
static void initialize_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static void initialize_params_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static void enable_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static void disable_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_update_debugger_state_c8_sL5XorzCIaSB
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static const mxArray
  *get_sim_state_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static void set_sim_state_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_st);
static void finalize_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static void sf_gateway_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static void mdl_start_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_chartstep_c8_sL5XorzCIaSBHWWbjdvvUSD_
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static void initSimStructsc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_radarMeasurementFcn_invoke
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_x[4], real_T
   c8_sL5XorzCIaSBHWWbjdvvUSD_v[2], real_T c8_sL5XorzCIaSBHWWbjdvvUSD_y[2]);
static void init_script_number_translation(uint32_T
  c8_sL5XorzCIaSBHWWbjdvvUSD_machineNumber, uint32_T
  c8_sL5XorzCIaSBHWWbjdvvUSD_chartNumber, uint32_T
  c8_sL5XorzCIaSBHWWbjdvvUSD_instanceNumber);
static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData);
static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_b_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData);
static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_c_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData);
static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_d_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData);
static real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_b_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData);
static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_e_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData);
static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_f_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData);
static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_g_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData);
static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_h_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData);
static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_i_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_c_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew, const
   char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier, real_T
   c8_sL5XorzCIaSBHWWbjdvvUSD_y[4]);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_d_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, real_T
   c8_sL5XorzCIaSBHWWbjdvvUSD_y[4]);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_c_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_e_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_c_P, const char_T
   *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier, real_T c8_sL5XorzCIaSBHWWbjdvvUSD_y
   [16]);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_f_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, real_T
   c8_sL5XorzCIaSBHWWbjdvvUSD_y[16]);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_d_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData);
static boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_g_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_c_blockOrdering,
   const char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier);
static boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_h_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_e_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_warning
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_j_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData);
static int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_f_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData);
static c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB
  c8_sL5XorzCIaSBHWWbjdvvUSD_j_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_k_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, char_T
   c8_sL5XorzCIaSBHWWbjdvvUSD_y[19]);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_l_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId);
static void c8_sL5XorzCIaSBHWWbjdvvUSD_g_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData);
static uint8_T c8_sL5XorzCIaSBHWWbjdvvUSD_m_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray
   *c8_sL5XorzCIaSBHWWbjdvvUSD_b_is_active_c8_sL5XorzCIaSBHWWbjdvvUS, const
   char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier);
static uint8_T c8_sL5XorzCIaSBHWWbjdvvUSD_n_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId);
static void init_dsm_address_info
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);
static void init_simulink_io_address
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance);

/* Function Definitions */
static void initialize_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_is_active_c8_sL5XorzCIaSBHWWbjdvvUSD_
    = 0U;
}

static void initialize_params_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_m0 = NULL;
  static const char * c8_sL5XorzCIaSBHWWbjdvvUSD_fieldNames[6] = { "FcnName",
    "IsSimulinkFcn", "NumberOfExtraArgumentInports", "HasJacobian",
    "JacobianFcnName", "HasAdditiveNoise" };

  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxField;
  c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB
    c8_sL5XorzCIaSBHWWbjdvvUSD_r0;
  c8_sL5XorzCIaSBHWWbjdvvUSD_m0 = sf_mex_get_sfun_param(chartInstance->S, 0U, 1U);
  sf_mex_check_bus_parameter(c8_sL5XorzCIaSBHWWbjdvvUSD_m0, 0, NULL, 6,
    c8_sL5XorzCIaSBHWWbjdvvUSD_fieldNames, "pM", "sUAIaS8TPWuzgg7KvQbnkvB");
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxField = sf_mex_getfield
    (c8_sL5XorzCIaSBHWWbjdvvUSD_m0, "FcnName", "pM", 0);
  sf_mex_import_named("pM", sf_mex_dup(c8_sL5XorzCIaSBHWWbjdvvUSD_mxField),
                      c8_sL5XorzCIaSBHWWbjdvvUSD_r0.FcnName, 1, 10, 0U, 1, 0U, 2,
                      1, 19);
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxField = sf_mex_getfield
    (c8_sL5XorzCIaSBHWWbjdvvUSD_m0, "IsSimulinkFcn", "pM", 0);
  sf_mex_import_named("pM", sf_mex_dup(c8_sL5XorzCIaSBHWWbjdvvUSD_mxField),
                      &c8_sL5XorzCIaSBHWWbjdvvUSD_r0.IsSimulinkFcn, 1, 11, 0U, 0,
                      0U, 0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxField = sf_mex_getfield
    (c8_sL5XorzCIaSBHWWbjdvvUSD_m0, "NumberOfExtraArgumentInports", "pM", 0);
  sf_mex_import_named("pM", sf_mex_dup(c8_sL5XorzCIaSBHWWbjdvvUSD_mxField),
                      &c8_sL5XorzCIaSBHWWbjdvvUSD_r0.NumberOfExtraArgumentInports,
                      1, 0, 0U, 0, 0U, 0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxField = sf_mex_getfield
    (c8_sL5XorzCIaSBHWWbjdvvUSD_m0, "HasJacobian", "pM", 0);
  sf_mex_import_named("pM", sf_mex_dup(c8_sL5XorzCIaSBHWWbjdvvUSD_mxField),
                      &c8_sL5XorzCIaSBHWWbjdvvUSD_r0.HasJacobian, 1, 11, 0U, 0,
                      0U, 0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxField = sf_mex_getfield
    (c8_sL5XorzCIaSBHWWbjdvvUSD_m0, "JacobianFcnName", "pM", 0);
  sf_mex_import_named("pM", sf_mex_dup(c8_sL5XorzCIaSBHWWbjdvvUSD_mxField), NULL,
                      1, 10, 0U, 1, 0U, 2, 0, 0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxField = sf_mex_getfield
    (c8_sL5XorzCIaSBHWWbjdvvUSD_m0, "HasAdditiveNoise", "pM", 0);
  sf_mex_import_named("pM", sf_mex_dup(c8_sL5XorzCIaSBHWWbjdvvUSD_mxField),
                      &c8_sL5XorzCIaSBHWWbjdvvUSD_r0.HasAdditiveNoise, 1, 11, 0U,
                      0, 0U, 0);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_m0);
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_pM = c8_sL5XorzCIaSBHWWbjdvvUSD_r0;
}

static void enable_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_update_debugger_state_c8_sL5XorzCIaSB
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static const mxArray
  *get_sim_state_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_st;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_b_y = NULL;
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_hoistedGlobal;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_c_y = NULL;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_d_y = NULL;
  uint8_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_hoistedGlobal;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_e_y = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_st = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_st = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_y, sf_mex_createcellmatrix(4, 1),
                false);
  c8_sL5XorzCIaSBHWWbjdvvUSD_b_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_b_y, sf_mex_create("y",
    *chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_b_P, 0, 0U, 1U, 0U, 2, 4, 4),
                false);
  sf_mex_setcell(c8_sL5XorzCIaSBHWWbjdvvUSD_y, 0, c8_sL5XorzCIaSBHWWbjdvvUSD_b_y);
  c8_sL5XorzCIaSBHWWbjdvvUSD_hoistedGlobal =
    *chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_b_blockOrdering;
  c8_sL5XorzCIaSBHWWbjdvvUSD_c_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_c_y, sf_mex_create("y",
    &c8_sL5XorzCIaSBHWWbjdvvUSD_hoistedGlobal, 11, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c8_sL5XorzCIaSBHWWbjdvvUSD_y, 1, c8_sL5XorzCIaSBHWWbjdvvUSD_c_y);
  c8_sL5XorzCIaSBHWWbjdvvUSD_d_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_d_y, sf_mex_create("y",
    *chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_xNew, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_setcell(c8_sL5XorzCIaSBHWWbjdvvUSD_y, 2, c8_sL5XorzCIaSBHWWbjdvvUSD_d_y);
  c8_sL5XorzCIaSBHWWbjdvvUSD_b_hoistedGlobal =
    chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_is_active_c8_sL5XorzCIaSBHWWbjdvvUSD_;
  c8_sL5XorzCIaSBHWWbjdvvUSD_e_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_e_y, sf_mex_create("y",
    &c8_sL5XorzCIaSBHWWbjdvvUSD_b_hoistedGlobal, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c8_sL5XorzCIaSBHWWbjdvvUSD_y, 3, c8_sL5XorzCIaSBHWWbjdvvUSD_e_y);
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_st, c8_sL5XorzCIaSBHWWbjdvvUSD_y,
                false);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_st;
}

static void set_sim_state_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_st)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_dv0[16];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i0;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_dv1[4];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i1;
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_doneDoubleBufferReInit = true;
  c8_sL5XorzCIaSBHWWbjdvvUSD_u = sf_mex_dup(c8_sL5XorzCIaSBHWWbjdvvUSD_st);
  c8_sL5XorzCIaSBHWWbjdvvUSD_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c8_sL5XorzCIaSBHWWbjdvvUSD_u, 0)), "P",
    c8_sL5XorzCIaSBHWWbjdvvUSD_dv0);
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i0 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i0 < 16;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i0++) {
    (*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_b_P)
      [c8_sL5XorzCIaSBHWWbjdvvUSD_i0] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_dv0[c8_sL5XorzCIaSBHWWbjdvvUSD_i0];
  }

  *chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_b_blockOrdering =
    c8_sL5XorzCIaSBHWWbjdvvUSD_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c8_sL5XorzCIaSBHWWbjdvvUSD_u, 1)), "blockOrdering");
  c8_sL5XorzCIaSBHWWbjdvvUSD_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c8_sL5XorzCIaSBHWWbjdvvUSD_u, 2)), "xNew",
    c8_sL5XorzCIaSBHWWbjdvvUSD_dv1);
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i1 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i1 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i1++) {
    (*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_xNew)
      [c8_sL5XorzCIaSBHWWbjdvvUSD_i1] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_dv1[c8_sL5XorzCIaSBHWWbjdvvUSD_i1];
  }

  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_is_active_c8_sL5XorzCIaSBHWWbjdvvUSD_
    = c8_sL5XorzCIaSBHWWbjdvvUSD_m_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c8_sL5XorzCIaSBHWWbjdvvUSD_u, 3)),
    "is_active_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary");
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_u);
  c8_sL5XorzCIaSBHWWbjdvvUSD_update_debugger_state_c8_sL5XorzCIaSB(chartInstance);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_st);
}

static void finalize_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i2;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i3;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i4;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i5;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i6;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i7;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U,
               chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent);
  _SFD_DATA_RANGE_CHECK((real_T)
                        *chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_blockOrdering,
                        5U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_uMeas, 4U);
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i2 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i2 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i2++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_R)
                          [c8_sL5XorzCIaSBHWWbjdvvUSD_i2], 3U);
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i3 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i3 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i3++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_yMeas)
                          [c8_sL5XorzCIaSBHWWbjdvvUSD_i3], 2U);
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i4 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i4 < 16;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i4++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_P)
                          [c8_sL5XorzCIaSBHWWbjdvvUSD_i4], 1U);
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i5 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i5 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i5++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_x)
                          [c8_sL5XorzCIaSBHWWbjdvvUSD_i5], 0U);
  }

  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent = CALL_EVENT;
  c8_sL5XorzCIaSBHWWbjdvvUSD_chartstep_c8_sL5XorzCIaSBHWWbjdvvUSD_(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i6 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i6 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i6++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_xNew)
                          [c8_sL5XorzCIaSBHWWbjdvvUSD_i6], 6U);
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i7 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i7 < 16;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i7++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_b_P)
                          [c8_sL5XorzCIaSBHWWbjdvvUSD_i7], 7U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)
                        *chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_b_blockOrdering,
                        8U);
}

static void mdl_start_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  sim_mode_is_external(chartInstance->S);
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_chartstep_c8_sL5XorzCIaSBHWWbjdvvUSD_
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_hoistedGlobal;
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_hoistedGlobal;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i8;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i9;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_x[4];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i10;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_c_P[16];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i11;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_yMeas[2];
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_uMeas;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_R[4];
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_c_blockOrdering;
  uint32_T c8_sL5XorzCIaSBHWWbjdvvUSD_debug_family_var_map[12];
  c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_pM;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_nargin = 6.0;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_nargout = 3.0;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew[4];
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_d_P[16];
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_d_blockOrdering;
  static c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB
    c8_sL5XorzCIaSBHWWbjdvvUSD_r1 = { { 'r', 'a', 'd', 'a', 'r', 'M', 'e', 'a',
      's', 'u', 'r', 'e', 'm', 'e', 'n', 't', 'F', 'c', 'n' },/* FcnName */
    true,                              /* IsSimulinkFcn */
    0.0,                               /* NumberOfExtraArgumentInports */
    false,                             /* HasJacobian */
    false                              /* HasAdditiveNoise */
  };

  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i12;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i13;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i14;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i15;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_z[2];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i16;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_zcov[4];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i17;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_c_x[4];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i18;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_e_P[16];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i19;
  c8_sL5XorzCIaSBHWWbjdvvUSD_cell_2 c8_sL5XorzCIaSBHWWbjdvvUSD_vec;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i20;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i21;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[4];
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec[2];
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_z[2];
  c8_sL5XorzCIaSBHWWbjdvvUSD_cell_2 c8_sL5XorzCIaSBHWWbjdvvUSD_specvec;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_j;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i22;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_j;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i23;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i24;
  c8_sL5XorzCIaSBHWWbjdvvUSD_cell_2 c8_sL5XorzCIaSBHWWbjdvvUSD_b_vec;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_d_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_e_x;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i25;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_f_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_varargin_2;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i26;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_varargin_1;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_c_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_d_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_e_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_f_y;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_c_j;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_maxval;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_epsilon;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i27;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_d_j;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i28;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i29;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i30;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i31;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i32;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i33;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_g_x;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i34;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_h_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_i_x;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i35;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i36;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i37;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_g_y;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i38;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_gain[8];
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_dHdx[8];
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_varargin_2;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_varargin_1;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i39;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i40;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_h_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_imz[2];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i41;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i42;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_Pxy[8];
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_i_y;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i43;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i44;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i45;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_j_y;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i46;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_k_y;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i47;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i48;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_l_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_B;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i49;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_m_y[8];
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_maxval;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_n_y;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i50;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_epsilon;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_o_y;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i51;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i52;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i53;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_e_j;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i54;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i55;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i56;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i57;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i58;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i59;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i60;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i61;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i62;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[4];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i63;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i64;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i65;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_p_y[4];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i66;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i67;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i68;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i69;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i70;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i71;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i72;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i73;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_dHdv[4];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i74;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i75;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_q_y[4];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i76;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_B;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i77;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i78;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_r_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_s_y;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_f_j;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i79;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i80;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_dv2[2];
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_j_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_k_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_l_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_m_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_t_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_n_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_o_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_p_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_u_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_d;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_q_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_r_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_s_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_t_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_v_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_u_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_v_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_w_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_w_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_d;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_r1;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_r2;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_x_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_x_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_y_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_y_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_a21;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_a22;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_k;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i81;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_ab_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_ab_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_bb_x;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i82;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_bb_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_c_z;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i83;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_gain[4];
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_cb_x;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i84;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i85;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_cb_y;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i86;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_db_x;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_db_y;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i87;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i88;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_d_z;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i89;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i90;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i91;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i92;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i93;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i94;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i95;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i96;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_eb_y[16];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i97;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_fb_y[16];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i98;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i99;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i100;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i101;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i102;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i103;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i104;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U,
               chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent);
  c8_sL5XorzCIaSBHWWbjdvvUSD_hoistedGlobal =
    *chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_uMeas;
  c8_sL5XorzCIaSBHWWbjdvvUSD_b_hoistedGlobal =
    *chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_blockOrdering;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i8 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i8 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i8++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_x[c8_sL5XorzCIaSBHWWbjdvvUSD_i8] =
      (*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_x)
      [c8_sL5XorzCIaSBHWWbjdvvUSD_i8];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i9 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i9 < 16;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i9++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_c_P[c8_sL5XorzCIaSBHWWbjdvvUSD_i9] =
      (*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_P)
      [c8_sL5XorzCIaSBHWWbjdvvUSD_i9];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i10 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i10 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i10++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_yMeas[c8_sL5XorzCIaSBHWWbjdvvUSD_i10] =
      (*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_yMeas)
      [c8_sL5XorzCIaSBHWWbjdvvUSD_i10];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i11 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i11 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i11++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_R[c8_sL5XorzCIaSBHWWbjdvvUSD_i11] =
      (*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_R)
      [c8_sL5XorzCIaSBHWWbjdvvUSD_i11];
  }

  c8_sL5XorzCIaSBHWWbjdvvUSD_b_uMeas = c8_sL5XorzCIaSBHWWbjdvvUSD_hoistedGlobal;
  c8_sL5XorzCIaSBHWWbjdvvUSD_c_blockOrdering =
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 12U, 14U,
    c8_sL5XorzCIaSBHWWbjdvvUSD_debug_family_names,
    c8_sL5XorzCIaSBHWWbjdvvUSD_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(NULL, 0U, c8_sL5XorzCIaSBHWWbjdvvUSD_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(NULL, 1U,
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_sf_marshallOut,
    c8_sL5XorzCIaSBHWWbjdvvUSD_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c8_sL5XorzCIaSBHWWbjdvvUSD_b_pM, 2U,
    c8_sL5XorzCIaSBHWWbjdvvUSD_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_sL5XorzCIaSBHWWbjdvvUSD_nargin, 3U,
    c8_sL5XorzCIaSBHWWbjdvvUSD_d_sf_marshallOut,
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_sL5XorzCIaSBHWWbjdvvUSD_nargout, 4U,
    c8_sL5XorzCIaSBHWWbjdvvUSD_d_sf_marshallOut,
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c8_sL5XorzCIaSBHWWbjdvvUSD_b_x, 5U,
    c8_sL5XorzCIaSBHWWbjdvvUSD_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c8_sL5XorzCIaSBHWWbjdvvUSD_c_P, 6U,
    c8_sL5XorzCIaSBHWWbjdvvUSD_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c8_sL5XorzCIaSBHWWbjdvvUSD_b_yMeas, 7U,
    c8_sL5XorzCIaSBHWWbjdvvUSD_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c8_sL5XorzCIaSBHWWbjdvvUSD_b_R, 8U,
    c8_sL5XorzCIaSBHWWbjdvvUSD_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c8_sL5XorzCIaSBHWWbjdvvUSD_b_uMeas, 9U,
    c8_sL5XorzCIaSBHWWbjdvvUSD_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c8_sL5XorzCIaSBHWWbjdvvUSD_c_blockOrdering, 10U,
    c8_sL5XorzCIaSBHWWbjdvvUSD_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew, 11U,
    c8_sL5XorzCIaSBHWWbjdvvUSD_e_sf_marshallOut,
    c8_sL5XorzCIaSBHWWbjdvvUSD_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_sL5XorzCIaSBHWWbjdvvUSD_d_P,
    MAX_uint32_T, c8_sL5XorzCIaSBHWWbjdvvUSD_f_sf_marshallOut,
    c8_sL5XorzCIaSBHWWbjdvvUSD_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE
    (&c8_sL5XorzCIaSBHWWbjdvvUSD_d_blockOrdering, MAX_uint32_T,
     c8_sL5XorzCIaSBHWWbjdvvUSD_i_sf_marshallOut,
     c8_sL5XorzCIaSBHWWbjdvvUSD_e_sf_marshallIn);
  c8_sL5XorzCIaSBHWWbjdvvUSD_b_pM = c8_sL5XorzCIaSBHWWbjdvvUSD_r1;
  c8_sL5XorzCIaSBHWWbjdvvUSD_d_blockOrdering =
    c8_sL5XorzCIaSBHWWbjdvvUSD_c_blockOrdering;
  _SFD_SYMBOL_SWITCH(10U, 13U);
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i12 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i12 < 16;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i12++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_d_P[c8_sL5XorzCIaSBHWWbjdvvUSD_i12] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_c_P[c8_sL5XorzCIaSBHWWbjdvvUSD_i12];
  }

  _SFD_SYMBOL_SWITCH(6U, 12U);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent, 8);
  CV_EML_IF(0, 1, 0, c8_sL5XorzCIaSBHWWbjdvvUSD_b_pM.IsSimulinkFcn);
  _SFD_EML_CALL(0U, chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent, 9);
  _SFD_EML_CALL(0U, chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent, 10);
  CV_EML_IF(0, 1, 1, c8_sL5XorzCIaSBHWWbjdvvUSD_b_pM.HasAdditiveNoise);
  _SFD_EML_CALL(0U, chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent, 13);
  _SFD_EML_CALL(0U, chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent, 18);
  CV_EML_IF(0, 1, 2, c8_sL5XorzCIaSBHWWbjdvvUSD_b_pM.HasJacobian);
  _SFD_EML_CALL(0U, chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent, 30);
  _SFD_EML_CALL(0U, chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent, 33);
  CV_EML_SWITCH(0, 1, 0, 1);
  _SFD_EML_CALL(0U, chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent, 35);
  _SFD_EML_CALL(0U, chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent, 42);
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i13 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i13 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i13++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew[c8_sL5XorzCIaSBHWWbjdvvUSD_i13] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent, 43);
  CV_EML_IF(0, 1, 5, c8_sL5XorzCIaSBHWWbjdvvUSD_b_pM.HasAdditiveNoise);
  _SFD_EML_CALL(0U, chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent, 48);
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i14 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i14 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i14++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_z[c8_sL5XorzCIaSBHWWbjdvvUSD_i14] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_yMeas[c8_sL5XorzCIaSBHWWbjdvvUSD_i14];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i15 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i15 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i15++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_zcov[c8_sL5XorzCIaSBHWWbjdvvUSD_i15] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_R[c8_sL5XorzCIaSBHWWbjdvvUSD_i15];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i16 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i16 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i16++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_c_x[c8_sL5XorzCIaSBHWWbjdvvUSD_i16] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_x[c8_sL5XorzCIaSBHWWbjdvvUSD_i16];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i17 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i17 < 16;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i17++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_e_P[c8_sL5XorzCIaSBHWWbjdvvUSD_i17] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_d_P[c8_sL5XorzCIaSBHWWbjdvvUSD_i17];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i18 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i18 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i18++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_vec.f1[c8_sL5XorzCIaSBHWWbjdvvUSD_i18] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_c_x[c8_sL5XorzCIaSBHWWbjdvvUSD_i18];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i19 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i19 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i19++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_vec.f2[c8_sL5XorzCIaSBHWWbjdvvUSD_i19] = 0.0;
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i20 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i20 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i20++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i20] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_vec.f1[c8_sL5XorzCIaSBHWWbjdvvUSD_i20];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i21 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i21 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i21++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i21] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_vec.f2[c8_sL5XorzCIaSBHWWbjdvvUSD_i21];
  }

  CV_EML_FCN(0, 2);
  c8_sL5XorzCIaSBHWWbjdvvUSD_radarMeasurementFcn_invoke(chartInstance,
    c8_sL5XorzCIaSBHWWbjdvvUSD_imvec, c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec,
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_z);
  c8_sL5XorzCIaSBHWWbjdvvUSD_specvec = c8_sL5XorzCIaSBHWWbjdvvUSD_vec;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_j = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_j < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_j++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_j = 1.0 + (real_T)c8_sL5XorzCIaSBHWWbjdvvUSD_j;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i23 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i23 < 4;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i23++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i23] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_vec.f1[c8_sL5XorzCIaSBHWWbjdvvUSD_i23];
    }

    c8_sL5XorzCIaSBHWWbjdvvUSD_d_x =
      c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 1351, 151,
       MAX_uint32_T, (int32_T)sf_integer_check(chartInstance->S, 1U, 1351U, 151U,
        c8_sL5XorzCIaSBHWWbjdvvUSD_b_j), 1, 4) - 1];
    c8_sL5XorzCIaSBHWWbjdvvUSD_e_x = c8_sL5XorzCIaSBHWWbjdvvUSD_d_x;
    c8_sL5XorzCIaSBHWWbjdvvUSD_f_x = c8_sL5XorzCIaSBHWWbjdvvUSD_e_x;
    c8_sL5XorzCIaSBHWWbjdvvUSD_y = muDoubleScalarAbs
      (c8_sL5XorzCIaSBHWWbjdvvUSD_f_x);
    c8_sL5XorzCIaSBHWWbjdvvUSD_varargin_2 = 1.4901161193847656E-8 *
      c8_sL5XorzCIaSBHWWbjdvvUSD_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_varargin_1 =
      c8_sL5XorzCIaSBHWWbjdvvUSD_varargin_2;
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_y = c8_sL5XorzCIaSBHWWbjdvvUSD_varargin_1;
    c8_sL5XorzCIaSBHWWbjdvvUSD_c_y = c8_sL5XorzCIaSBHWWbjdvvUSD_b_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_d_y = c8_sL5XorzCIaSBHWWbjdvvUSD_c_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_e_y = c8_sL5XorzCIaSBHWWbjdvvUSD_d_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_f_y = c8_sL5XorzCIaSBHWWbjdvvUSD_e_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_maxval = muDoubleScalarMax(1.4901161193847656E-8,
      c8_sL5XorzCIaSBHWWbjdvvUSD_f_y);
    c8_sL5XorzCIaSBHWWbjdvvUSD_epsilon = c8_sL5XorzCIaSBHWWbjdvvUSD_maxval;
    c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 1351, 151,
       MAX_uint32_T, (int32_T)sf_integer_check(chartInstance->S, 1U, 1351U, 151U,
        c8_sL5XorzCIaSBHWWbjdvvUSD_b_j), 1, 4) - 1] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 1351, 151,
       MAX_uint32_T, (int32_T)sf_integer_check(chartInstance->S, 1U, 1351U, 151U,
        c8_sL5XorzCIaSBHWWbjdvvUSD_b_j), 1, 4) - 1] +
      c8_sL5XorzCIaSBHWWbjdvvUSD_epsilon;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i28 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i28 < 4;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i28++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_specvec.f1[c8_sL5XorzCIaSBHWWbjdvvUSD_i28] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i28];
    }

    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i31 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i31 < 4;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i31++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i31] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_specvec.f1[c8_sL5XorzCIaSBHWWbjdvvUSD_i31];
    }

    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i35 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i35 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i35++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i35] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_specvec.f2[c8_sL5XorzCIaSBHWWbjdvvUSD_i35];
    }

    CV_EML_FCN(0, 2);
    c8_sL5XorzCIaSBHWWbjdvvUSD_radarMeasurementFcn_invoke(chartInstance,
      c8_sL5XorzCIaSBHWWbjdvvUSD_imvec, c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec,
      c8_sL5XorzCIaSBHWWbjdvvUSD_imz);
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i43 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i43 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i43++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_imz[c8_sL5XorzCIaSBHWWbjdvvUSD_i43] -=
        c8_sL5XorzCIaSBHWWbjdvvUSD_b_z[c8_sL5XorzCIaSBHWWbjdvvUSD_i43];
    }

    c8_sL5XorzCIaSBHWWbjdvvUSD_B = c8_sL5XorzCIaSBHWWbjdvvUSD_epsilon;
    c8_sL5XorzCIaSBHWWbjdvvUSD_n_y = c8_sL5XorzCIaSBHWWbjdvvUSD_B;
    c8_sL5XorzCIaSBHWWbjdvvUSD_o_y = c8_sL5XorzCIaSBHWWbjdvvUSD_n_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_e_j = sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 1351, 151,
       MAX_uint32_T, (int32_T)sf_integer_check(chartInstance->S, 1U, 1351U, 151U,
        c8_sL5XorzCIaSBHWWbjdvvUSD_b_j), 1, 4) - 1;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i56 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i56 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i56++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_dHdx[c8_sL5XorzCIaSBHWWbjdvvUSD_i56 +
        (c8_sL5XorzCIaSBHWWbjdvvUSD_e_j << 1)] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_imz[c8_sL5XorzCIaSBHWWbjdvvUSD_i56] /
        c8_sL5XorzCIaSBHWWbjdvvUSD_o_y;
    }
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i22 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i22 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i22++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_vec.f1[c8_sL5XorzCIaSBHWWbjdvvUSD_i22] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_c_x[c8_sL5XorzCIaSBHWWbjdvvUSD_i22];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i24 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i24 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i24++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_vec.f2[c8_sL5XorzCIaSBHWWbjdvvUSD_i24] = 0.0;
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i25 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i25 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i25++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i25] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_vec.f1[c8_sL5XorzCIaSBHWWbjdvvUSD_i25];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i26 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i26 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i26++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i26] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_vec.f2[c8_sL5XorzCIaSBHWWbjdvvUSD_i26];
  }

  CV_EML_FCN(0, 2);
  c8_sL5XorzCIaSBHWWbjdvvUSD_radarMeasurementFcn_invoke(chartInstance,
    c8_sL5XorzCIaSBHWWbjdvvUSD_imvec, c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec,
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_z);
  c8_sL5XorzCIaSBHWWbjdvvUSD_specvec = c8_sL5XorzCIaSBHWWbjdvvUSD_b_vec;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_c_j = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_c_j < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_c_j++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_d_j = 1.0 + (real_T)
      c8_sL5XorzCIaSBHWWbjdvvUSD_c_j;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i30 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i30 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i30++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i30] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_b_vec.f2[c8_sL5XorzCIaSBHWWbjdvvUSD_i30];
    }

    c8_sL5XorzCIaSBHWWbjdvvUSD_g_x =
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec[sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 1351, 151,
       MAX_uint32_T, (int32_T)sf_integer_check(chartInstance->S, 1U, 1351U, 151U,
        c8_sL5XorzCIaSBHWWbjdvvUSD_d_j), 1, 2) - 1];
    c8_sL5XorzCIaSBHWWbjdvvUSD_h_x = c8_sL5XorzCIaSBHWWbjdvvUSD_g_x;
    c8_sL5XorzCIaSBHWWbjdvvUSD_i_x = c8_sL5XorzCIaSBHWWbjdvvUSD_h_x;
    c8_sL5XorzCIaSBHWWbjdvvUSD_g_y = muDoubleScalarAbs
      (c8_sL5XorzCIaSBHWWbjdvvUSD_i_x);
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_varargin_2 = 1.4901161193847656E-8 *
      c8_sL5XorzCIaSBHWWbjdvvUSD_g_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_varargin_1 =
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_varargin_2;
    c8_sL5XorzCIaSBHWWbjdvvUSD_h_y = c8_sL5XorzCIaSBHWWbjdvvUSD_b_varargin_1;
    c8_sL5XorzCIaSBHWWbjdvvUSD_i_y = c8_sL5XorzCIaSBHWWbjdvvUSD_h_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_j_y = c8_sL5XorzCIaSBHWWbjdvvUSD_i_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_k_y = c8_sL5XorzCIaSBHWWbjdvvUSD_j_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_l_y = c8_sL5XorzCIaSBHWWbjdvvUSD_k_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_maxval = muDoubleScalarMax
      (1.4901161193847656E-8, c8_sL5XorzCIaSBHWWbjdvvUSD_l_y);
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_epsilon = c8_sL5XorzCIaSBHWWbjdvvUSD_b_maxval;
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec[sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 1351, 151,
       MAX_uint32_T, (int32_T)sf_integer_check(chartInstance->S, 1U, 1351U, 151U,
        c8_sL5XorzCIaSBHWWbjdvvUSD_d_j), 1, 2) - 1] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec[sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 1351, 151,
       MAX_uint32_T, (int32_T)sf_integer_check(chartInstance->S, 1U, 1351U, 151U,
        c8_sL5XorzCIaSBHWWbjdvvUSD_d_j), 1, 2) - 1] +
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_epsilon;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i55 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i55 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i55++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_specvec.f2[c8_sL5XorzCIaSBHWWbjdvvUSD_i55] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i55];
    }

    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i62 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i62 < 4;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i62++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i62] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_specvec.f1[c8_sL5XorzCIaSBHWWbjdvvUSD_i62];
    }

    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i67 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i67 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i67++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i67] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_specvec.f2[c8_sL5XorzCIaSBHWWbjdvvUSD_i67];
    }

    CV_EML_FCN(0, 2);
    c8_sL5XorzCIaSBHWWbjdvvUSD_radarMeasurementFcn_invoke(chartInstance,
      c8_sL5XorzCIaSBHWWbjdvvUSD_imvec, c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec,
      c8_sL5XorzCIaSBHWWbjdvvUSD_imz);
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i74 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i74 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i74++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_imz[c8_sL5XorzCIaSBHWWbjdvvUSD_i74] -=
        c8_sL5XorzCIaSBHWWbjdvvUSD_b_z[c8_sL5XorzCIaSBHWWbjdvvUSD_i74];
    }

    c8_sL5XorzCIaSBHWWbjdvvUSD_b_B = c8_sL5XorzCIaSBHWWbjdvvUSD_b_epsilon;
    c8_sL5XorzCIaSBHWWbjdvvUSD_r_y = c8_sL5XorzCIaSBHWWbjdvvUSD_b_B;
    c8_sL5XorzCIaSBHWWbjdvvUSD_s_y = c8_sL5XorzCIaSBHWWbjdvvUSD_r_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_f_j = sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 1351, 151,
       MAX_uint32_T, (int32_T)sf_integer_check(chartInstance->S, 1U, 1351U, 151U,
        c8_sL5XorzCIaSBHWWbjdvvUSD_d_j), 1, 2) - 1;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i79 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i79 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i79++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_dHdv[c8_sL5XorzCIaSBHWWbjdvvUSD_i79 +
        (c8_sL5XorzCIaSBHWWbjdvvUSD_f_j << 1)] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_imz[c8_sL5XorzCIaSBHWWbjdvvUSD_i79] /
        c8_sL5XorzCIaSBHWWbjdvvUSD_s_y;
    }
  }

  c8_sL5XorzCIaSBHWWbjdvvUSD_i27 = 0;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i29 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i29 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i29++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_i33 = 0;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i34 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i34 < 4;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i34++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_i34 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i27] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_dHdx[c8_sL5XorzCIaSBHWWbjdvvUSD_i33 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i29];
      c8_sL5XorzCIaSBHWWbjdvvUSD_i33 += 2;
    }

    c8_sL5XorzCIaSBHWWbjdvvUSD_i27 += 4;
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i32 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i32 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i32++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_i37 = 0;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i38 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i38 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i38++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_Pxy[c8_sL5XorzCIaSBHWWbjdvvUSD_i37 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i32] = 0.0;
      c8_sL5XorzCIaSBHWWbjdvvUSD_i45 = 0;
      for (c8_sL5XorzCIaSBHWWbjdvvUSD_i46 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i46 <
           4; c8_sL5XorzCIaSBHWWbjdvvUSD_i46++) {
        c8_sL5XorzCIaSBHWWbjdvvUSD_Pxy[c8_sL5XorzCIaSBHWWbjdvvUSD_i37 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i32] +=
          c8_sL5XorzCIaSBHWWbjdvvUSD_e_P[c8_sL5XorzCIaSBHWWbjdvvUSD_i45 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i32] *
          c8_sL5XorzCIaSBHWWbjdvvUSD_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_i46 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i37];
        c8_sL5XorzCIaSBHWWbjdvvUSD_i45 += 4;
      }

      c8_sL5XorzCIaSBHWWbjdvvUSD_i37 += 4;
    }
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i36 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i36 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i36++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_i40 = 0;
    c8_sL5XorzCIaSBHWWbjdvvUSD_i42 = 0;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i44 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i44 < 4;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i44++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_m_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i40 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i36] = 0.0;
      c8_sL5XorzCIaSBHWWbjdvvUSD_i50 = 0;
      for (c8_sL5XorzCIaSBHWWbjdvvUSD_i53 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i53 <
           4; c8_sL5XorzCIaSBHWWbjdvvUSD_i53++) {
        c8_sL5XorzCIaSBHWWbjdvvUSD_m_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i40 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i36] +=
          c8_sL5XorzCIaSBHWWbjdvvUSD_dHdx[c8_sL5XorzCIaSBHWWbjdvvUSD_i50 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i36] *
          c8_sL5XorzCIaSBHWWbjdvvUSD_e_P[c8_sL5XorzCIaSBHWWbjdvvUSD_i53 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i42];
        c8_sL5XorzCIaSBHWWbjdvvUSD_i50 += 2;
      }

      c8_sL5XorzCIaSBHWWbjdvvUSD_i40 += 2;
      c8_sL5XorzCIaSBHWWbjdvvUSD_i42 += 4;
    }
  }

  c8_sL5XorzCIaSBHWWbjdvvUSD_i39 = 0;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i41 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i41 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i41++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_i48 = 0;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i49 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i49 < 4;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i49++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_i49 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i39] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_dHdx[c8_sL5XorzCIaSBHWWbjdvvUSD_i48 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i41];
      c8_sL5XorzCIaSBHWWbjdvvUSD_i48 += 2;
    }

    c8_sL5XorzCIaSBHWWbjdvvUSD_i39 += 4;
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i47 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i47 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i47++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_i52 = 0;
    c8_sL5XorzCIaSBHWWbjdvvUSD_i54 = 0;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i57 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i57 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i57++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[c8_sL5XorzCIaSBHWWbjdvvUSD_i52 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i47] = 0.0;
      c8_sL5XorzCIaSBHWWbjdvvUSD_i63 = 0;
      for (c8_sL5XorzCIaSBHWWbjdvvUSD_i66 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i66 <
           4; c8_sL5XorzCIaSBHWWbjdvvUSD_i66++) {
        c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[c8_sL5XorzCIaSBHWWbjdvvUSD_i52 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i47] +=
          c8_sL5XorzCIaSBHWWbjdvvUSD_m_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i63 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i47] *
          c8_sL5XorzCIaSBHWWbjdvvUSD_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_i66 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i54];
        c8_sL5XorzCIaSBHWWbjdvvUSD_i63 += 2;
      }

      c8_sL5XorzCIaSBHWWbjdvvUSD_i52 += 2;
      c8_sL5XorzCIaSBHWWbjdvvUSD_i54 += 4;
    }
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i51 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i51 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i51++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_i59 = 0;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i61 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i61 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i61++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_p_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i59 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i51] = 0.0;
      c8_sL5XorzCIaSBHWWbjdvvUSD_i69 = 0;
      for (c8_sL5XorzCIaSBHWWbjdvvUSD_i70 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i70 <
           2; c8_sL5XorzCIaSBHWWbjdvvUSD_i70++) {
        c8_sL5XorzCIaSBHWWbjdvvUSD_p_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i59 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i51] +=
          c8_sL5XorzCIaSBHWWbjdvvUSD_dHdv[c8_sL5XorzCIaSBHWWbjdvvUSD_i69 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i51] *
          c8_sL5XorzCIaSBHWWbjdvvUSD_zcov[c8_sL5XorzCIaSBHWWbjdvvUSD_i70 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i59];
        c8_sL5XorzCIaSBHWWbjdvvUSD_i69 += 2;
      }

      c8_sL5XorzCIaSBHWWbjdvvUSD_i59 += 2;
    }
  }

  c8_sL5XorzCIaSBHWWbjdvvUSD_i58 = 0;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i60 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i60 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i60++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_i65 = 0;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i68 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i68 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i68++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_zcov[c8_sL5XorzCIaSBHWWbjdvvUSD_i68 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i58] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_dHdv[c8_sL5XorzCIaSBHWWbjdvvUSD_i65 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i60];
      c8_sL5XorzCIaSBHWWbjdvvUSD_i65 += 2;
    }

    c8_sL5XorzCIaSBHWWbjdvvUSD_i58 += 2;
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i64 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i64 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i64++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_i72 = 0;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i73 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i73 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i73++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_q_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i72 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i64] = 0.0;
      c8_sL5XorzCIaSBHWWbjdvvUSD_i76 = 0;
      for (c8_sL5XorzCIaSBHWWbjdvvUSD_i78 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i78 <
           2; c8_sL5XorzCIaSBHWWbjdvvUSD_i78++) {
        c8_sL5XorzCIaSBHWWbjdvvUSD_q_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i72 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i64] +=
          c8_sL5XorzCIaSBHWWbjdvvUSD_p_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i76 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i64] *
          c8_sL5XorzCIaSBHWWbjdvvUSD_zcov[c8_sL5XorzCIaSBHWWbjdvvUSD_i78 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i72];
        c8_sL5XorzCIaSBHWWbjdvvUSD_i76 += 2;
      }

      c8_sL5XorzCIaSBHWWbjdvvUSD_i72 += 2;
    }
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i71 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i71 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i71++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i71] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_q_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i71];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i75 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i75 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i75++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[c8_sL5XorzCIaSBHWWbjdvvUSD_i75] +=
      c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i75];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i77 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i77 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i77++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i77] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_c_x[c8_sL5XorzCIaSBHWWbjdvvUSD_i77];
  }

  CV_EML_FCN(0, 2);
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i80 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i80 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i80++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_dv2[c8_sL5XorzCIaSBHWWbjdvvUSD_i80] = 0.0;
  }

  c8_sL5XorzCIaSBHWWbjdvvUSD_radarMeasurementFcn_invoke(chartInstance,
    c8_sL5XorzCIaSBHWWbjdvvUSD_imvec, c8_sL5XorzCIaSBHWWbjdvvUSD_dv2,
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec);
  c8_sL5XorzCIaSBHWWbjdvvUSD_j_x = c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[1];
  c8_sL5XorzCIaSBHWWbjdvvUSD_k_x = c8_sL5XorzCIaSBHWWbjdvvUSD_j_x;
  c8_sL5XorzCIaSBHWWbjdvvUSD_l_x = c8_sL5XorzCIaSBHWWbjdvvUSD_k_x;
  c8_sL5XorzCIaSBHWWbjdvvUSD_m_x = c8_sL5XorzCIaSBHWWbjdvvUSD_l_x;
  c8_sL5XorzCIaSBHWWbjdvvUSD_t_y = muDoubleScalarAbs
    (c8_sL5XorzCIaSBHWWbjdvvUSD_m_x);
  c8_sL5XorzCIaSBHWWbjdvvUSD_n_x = 0.0;
  c8_sL5XorzCIaSBHWWbjdvvUSD_o_x = c8_sL5XorzCIaSBHWWbjdvvUSD_n_x;
  c8_sL5XorzCIaSBHWWbjdvvUSD_p_x = c8_sL5XorzCIaSBHWWbjdvvUSD_o_x;
  c8_sL5XorzCIaSBHWWbjdvvUSD_u_y = muDoubleScalarAbs
    (c8_sL5XorzCIaSBHWWbjdvvUSD_p_x);
  c8_sL5XorzCIaSBHWWbjdvvUSD_d = c8_sL5XorzCIaSBHWWbjdvvUSD_t_y +
    c8_sL5XorzCIaSBHWWbjdvvUSD_u_y;
  c8_sL5XorzCIaSBHWWbjdvvUSD_q_x = c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[0];
  c8_sL5XorzCIaSBHWWbjdvvUSD_r_x = c8_sL5XorzCIaSBHWWbjdvvUSD_q_x;
  c8_sL5XorzCIaSBHWWbjdvvUSD_s_x = c8_sL5XorzCIaSBHWWbjdvvUSD_r_x;
  c8_sL5XorzCIaSBHWWbjdvvUSD_t_x = c8_sL5XorzCIaSBHWWbjdvvUSD_s_x;
  c8_sL5XorzCIaSBHWWbjdvvUSD_v_y = muDoubleScalarAbs
    (c8_sL5XorzCIaSBHWWbjdvvUSD_t_x);
  c8_sL5XorzCIaSBHWWbjdvvUSD_u_x = 0.0;
  c8_sL5XorzCIaSBHWWbjdvvUSD_v_x = c8_sL5XorzCIaSBHWWbjdvvUSD_u_x;
  c8_sL5XorzCIaSBHWWbjdvvUSD_w_x = c8_sL5XorzCIaSBHWWbjdvvUSD_v_x;
  c8_sL5XorzCIaSBHWWbjdvvUSD_w_y = muDoubleScalarAbs
    (c8_sL5XorzCIaSBHWWbjdvvUSD_w_x);
  c8_sL5XorzCIaSBHWWbjdvvUSD_b_d = c8_sL5XorzCIaSBHWWbjdvvUSD_v_y +
    c8_sL5XorzCIaSBHWWbjdvvUSD_w_y;
  if (c8_sL5XorzCIaSBHWWbjdvvUSD_d > c8_sL5XorzCIaSBHWWbjdvvUSD_b_d) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_r1 = 1;
    c8_sL5XorzCIaSBHWWbjdvvUSD_r2 = 0;
  } else {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_r1 = 0;
    c8_sL5XorzCIaSBHWWbjdvvUSD_r2 = 1;
  }

  c8_sL5XorzCIaSBHWWbjdvvUSD_x_x =
    c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[c8_sL5XorzCIaSBHWWbjdvvUSD_r2];
  c8_sL5XorzCIaSBHWWbjdvvUSD_x_y =
    c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[c8_sL5XorzCIaSBHWWbjdvvUSD_b_r1];
  c8_sL5XorzCIaSBHWWbjdvvUSD_y_x = c8_sL5XorzCIaSBHWWbjdvvUSD_x_x;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y_y = c8_sL5XorzCIaSBHWWbjdvvUSD_x_y;
  c8_sL5XorzCIaSBHWWbjdvvUSD_a21 = c8_sL5XorzCIaSBHWWbjdvvUSD_y_x /
    c8_sL5XorzCIaSBHWWbjdvvUSD_y_y;
  c8_sL5XorzCIaSBHWWbjdvvUSD_a22 = c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[2 +
    c8_sL5XorzCIaSBHWWbjdvvUSD_r2] - c8_sL5XorzCIaSBHWWbjdvvUSD_a21 *
    c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[2 + c8_sL5XorzCIaSBHWWbjdvvUSD_b_r1];
  if ((c8_sL5XorzCIaSBHWWbjdvvUSD_a22 == 0.0) ||
      (c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[c8_sL5XorzCIaSBHWWbjdvvUSD_b_r1] == 0.0))
  {
    c8_sL5XorzCIaSBHWWbjdvvUSD_warning(chartInstance);
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_k = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_k < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_k++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_ab_x =
      c8_sL5XorzCIaSBHWWbjdvvUSD_Pxy[c8_sL5XorzCIaSBHWWbjdvvUSD_k];
    c8_sL5XorzCIaSBHWWbjdvvUSD_ab_y =
      c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[c8_sL5XorzCIaSBHWWbjdvvUSD_b_r1];
    c8_sL5XorzCIaSBHWWbjdvvUSD_bb_x = c8_sL5XorzCIaSBHWWbjdvvUSD_ab_x;
    c8_sL5XorzCIaSBHWWbjdvvUSD_bb_y = c8_sL5XorzCIaSBHWWbjdvvUSD_ab_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_c_z = c8_sL5XorzCIaSBHWWbjdvvUSD_bb_x /
      c8_sL5XorzCIaSBHWWbjdvvUSD_bb_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_k +
      (c8_sL5XorzCIaSBHWWbjdvvUSD_b_r1 << 2)] = c8_sL5XorzCIaSBHWWbjdvvUSD_c_z;
    c8_sL5XorzCIaSBHWWbjdvvUSD_cb_x = c8_sL5XorzCIaSBHWWbjdvvUSD_Pxy[4 +
      c8_sL5XorzCIaSBHWWbjdvvUSD_k] -
      c8_sL5XorzCIaSBHWWbjdvvUSD_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_k +
      (c8_sL5XorzCIaSBHWWbjdvvUSD_b_r1 << 2)] * c8_sL5XorzCIaSBHWWbjdvvUSD_Pyy[2
      + c8_sL5XorzCIaSBHWWbjdvvUSD_b_r1];
    c8_sL5XorzCIaSBHWWbjdvvUSD_cb_y = c8_sL5XorzCIaSBHWWbjdvvUSD_a22;
    c8_sL5XorzCIaSBHWWbjdvvUSD_db_x = c8_sL5XorzCIaSBHWWbjdvvUSD_cb_x;
    c8_sL5XorzCIaSBHWWbjdvvUSD_db_y = c8_sL5XorzCIaSBHWWbjdvvUSD_cb_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_d_z = c8_sL5XorzCIaSBHWWbjdvvUSD_db_x /
      c8_sL5XorzCIaSBHWWbjdvvUSD_db_y;
    c8_sL5XorzCIaSBHWWbjdvvUSD_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_k +
      (c8_sL5XorzCIaSBHWWbjdvvUSD_r2 << 2)] = c8_sL5XorzCIaSBHWWbjdvvUSD_d_z;
    c8_sL5XorzCIaSBHWWbjdvvUSD_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_k +
      (c8_sL5XorzCIaSBHWWbjdvvUSD_b_r1 << 2)] -=
      c8_sL5XorzCIaSBHWWbjdvvUSD_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_k +
      (c8_sL5XorzCIaSBHWWbjdvvUSD_r2 << 2)] * c8_sL5XorzCIaSBHWWbjdvvUSD_a21;
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i81 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i81 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i81++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_z[c8_sL5XorzCIaSBHWWbjdvvUSD_i81] -=
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_imvec[c8_sL5XorzCIaSBHWWbjdvvUSD_i81];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i82 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i82 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i82++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_i82] = 0.0;
    c8_sL5XorzCIaSBHWWbjdvvUSD_i85 = 0;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i86 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i86 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i86++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_i82] +=
        c8_sL5XorzCIaSBHWWbjdvvUSD_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_i85 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i82] *
        c8_sL5XorzCIaSBHWWbjdvvUSD_z[c8_sL5XorzCIaSBHWWbjdvvUSD_i86];
      c8_sL5XorzCIaSBHWWbjdvvUSD_i85 += 4;
    }
  }

  c8_sL5XorzCIaSBHWWbjdvvUSD_i83 = 0;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i84 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i84 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i84++) {
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i88 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i88 < 2;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i88++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_p_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i88 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i83] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_b_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_i88 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i83];
    }

    c8_sL5XorzCIaSBHWWbjdvvUSD_i83 += 2;
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i87 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i87 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i87++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_c_x[c8_sL5XorzCIaSBHWWbjdvvUSD_i87] +=
      c8_sL5XorzCIaSBHWWbjdvvUSD_p_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i87];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i89 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i89 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i89++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_i91 = 0;
    c8_sL5XorzCIaSBHWWbjdvvUSD_i92 = 0;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i93 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i93 < 4;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i93++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_eb_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i91 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i89] = 0.0;
      c8_sL5XorzCIaSBHWWbjdvvUSD_i98 = 0;
      for (c8_sL5XorzCIaSBHWWbjdvvUSD_i99 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i99 <
           2; c8_sL5XorzCIaSBHWWbjdvvUSD_i99++) {
        c8_sL5XorzCIaSBHWWbjdvvUSD_eb_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i91 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i89] +=
          c8_sL5XorzCIaSBHWWbjdvvUSD_gain[c8_sL5XorzCIaSBHWWbjdvvUSD_i98 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i89] *
          c8_sL5XorzCIaSBHWWbjdvvUSD_dHdx[c8_sL5XorzCIaSBHWWbjdvvUSD_i99 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i92];
        c8_sL5XorzCIaSBHWWbjdvvUSD_i98 += 4;
      }

      c8_sL5XorzCIaSBHWWbjdvvUSD_i91 += 4;
      c8_sL5XorzCIaSBHWWbjdvvUSD_i92 += 2;
    }
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i90 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i90 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i90++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_i95 = 0;
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i96 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i96 < 4;
         c8_sL5XorzCIaSBHWWbjdvvUSD_i96++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_fb_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i95 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i90] = 0.0;
      c8_sL5XorzCIaSBHWWbjdvvUSD_i100 = 0;
      for (c8_sL5XorzCIaSBHWWbjdvvUSD_i102 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i102 <
           4; c8_sL5XorzCIaSBHWWbjdvvUSD_i102++) {
        c8_sL5XorzCIaSBHWWbjdvvUSD_fb_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i95 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i90] +=
          c8_sL5XorzCIaSBHWWbjdvvUSD_eb_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i100 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i90] *
          c8_sL5XorzCIaSBHWWbjdvvUSD_e_P[c8_sL5XorzCIaSBHWWbjdvvUSD_i102 +
          c8_sL5XorzCIaSBHWWbjdvvUSD_i95];
        c8_sL5XorzCIaSBHWWbjdvvUSD_i100 += 4;
      }

      c8_sL5XorzCIaSBHWWbjdvvUSD_i95 += 4;
    }
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i94 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i94 < 16;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i94++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_e_P[c8_sL5XorzCIaSBHWWbjdvvUSD_i94] -=
      c8_sL5XorzCIaSBHWWbjdvvUSD_fb_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i94];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i97 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i97 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i97++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew[c8_sL5XorzCIaSBHWWbjdvvUSD_i97] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_c_x[c8_sL5XorzCIaSBHWWbjdvvUSD_i97];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i101 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i101 < 16;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i101++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_d_P[c8_sL5XorzCIaSBHWWbjdvvUSD_i101] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_e_P[c8_sL5XorzCIaSBHWWbjdvvUSD_i101];
  }

  _SFD_SYMBOL_SWITCH(6U, 12U);
  _SFD_EML_CALL(0U, chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent, -48);
  _SFD_SYMBOL_SCOPE_POP();
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i103 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i103 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i103++) {
    (*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_xNew)
      [c8_sL5XorzCIaSBHWWbjdvvUSD_i103] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew[c8_sL5XorzCIaSBHWWbjdvvUSD_i103];
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i104 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i104 < 16;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i104++) {
    (*chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_b_P)
      [c8_sL5XorzCIaSBHWWbjdvvUSD_i104] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_d_P[c8_sL5XorzCIaSBHWWbjdvvUSD_i104];
  }

  *chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_b_blockOrdering =
    c8_sL5XorzCIaSBHWWbjdvvUSD_d_blockOrdering;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U,
               chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_sfEvent);
}

static void initSimStructsc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_radarMeasurementFcn_invoke
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_x[4], real_T
   c8_sL5XorzCIaSBHWWbjdvvUSD_v[2], real_T c8_sL5XorzCIaSBHWWbjdvvUSD_y[2])
{
  _ssFcnCallExecArgInfo c8_sL5XorzCIaSBHWWbjdvvUSD_args[3];
  c8_sL5XorzCIaSBHWWbjdvvUSD_args[0U].dataPtr = (void *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_x;
  c8_sL5XorzCIaSBHWWbjdvvUSD_args[1U].dataPtr = (void *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_v;
  c8_sL5XorzCIaSBHWWbjdvvUSD_args[2U].dataPtr = (void *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  slcsInvokeSimulinkFunction(chartInstance->S, "radarMeasurementFcn",
    &c8_sL5XorzCIaSBHWWbjdvvUSD_args[0U]);
}

static void init_script_number_translation(uint32_T
  c8_sL5XorzCIaSBHWWbjdvvUSD_machineNumber, uint32_T
  c8_sL5XorzCIaSBHWWbjdvvUSD_chartNumber, uint32_T
  c8_sL5XorzCIaSBHWWbjdvvUSD_instanceNumber)
{
  (void)(c8_sL5XorzCIaSBHWWbjdvvUSD_machineNumber);
  (void)(c8_sL5XorzCIaSBHWWbjdvvUSD_chartNumber);
  (void)(c8_sL5XorzCIaSBHWWbjdvvUSD_instanceNumber);
}

static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  (void)c8_sL5XorzCIaSBHWWbjdvvUSD_inData;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_y, sf_mex_create("y", NULL, 0, 0U,
    1U, 0U, 2, 0, 0), false);
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData,
                c8_sL5XorzCIaSBHWWbjdvvUSD_y, false);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
}

static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_b_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i105;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_iv0[2];
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  (void)c8_sL5XorzCIaSBHWWbjdvvUSD_inData;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i105 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i105 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i105++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_iv0[c8_sL5XorzCIaSBHWWbjdvvUSD_i105] = 0;
  }

  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_y, sf_mex_createcellarray(2,
    c8_sL5XorzCIaSBHWWbjdvvUSD_iv0), false);
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData,
                c8_sL5XorzCIaSBHWWbjdvvUSD_y, false);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId)
{
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i106;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i107;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_iv1[2];
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_bv0[2];
  (void)chartInstance;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i106 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i106 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i106++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_iv1[c8_sL5XorzCIaSBHWWbjdvvUSD_i106] = 0;
  }

  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i107 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i107 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i107++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_bv0[c8_sL5XorzCIaSBHWWbjdvvUSD_i107] = false;
  }

  sf_mex_check_cell(c8_sL5XorzCIaSBHWWbjdvvUSD_parentId,
                    c8_sL5XorzCIaSBHWWbjdvvUSD_u, 2U,
                    c8_sL5XorzCIaSBHWWbjdvvUSD_iv1,
                    c8_sL5XorzCIaSBHWWbjdvvUSD_bv0);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_u);
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_extraArgs;
  const char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  emlrtMsgIdentifier c8_sL5XorzCIaSBHWWbjdvvUSD_thisId;
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  (void)c8_sL5XorzCIaSBHWWbjdvvUSD_outData;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_extraArgs = sf_mex_dup
    (c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
  c8_sL5XorzCIaSBHWWbjdvvUSD_identifier = c8_sL5XorzCIaSBHWWbjdvvUSD_varName;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = (const char *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fParent = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.bParentIsCell = false;
  c8_sL5XorzCIaSBHWWbjdvvUSD_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_sL5XorzCIaSBHWWbjdvvUSD_extraArgs), &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_extraArgs);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
}

static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_c_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
  c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB
    c8_sL5XorzCIaSBHWWbjdvvUSD_u;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i108;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_b_y = NULL;
  char_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_u[19];
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_c_u;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_c_y = NULL;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_d_u;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_d_y = NULL;
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_e_u;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_e_y = NULL;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_f_y = NULL;
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_f_u;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_g_y = NULL;
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_u =
    *(c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_inData;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_y, sf_mex_createstruct("structure",
    2, 1, 1), false);
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i108 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i108 < 19;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i108++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_u[c8_sL5XorzCIaSBHWWbjdvvUSD_i108] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_u.FcnName[c8_sL5XorzCIaSBHWWbjdvvUSD_i108];
  }

  c8_sL5XorzCIaSBHWWbjdvvUSD_b_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_b_y, sf_mex_create("y",
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_u, 10, 0U, 1U, 0U, 2, 1, 19), false);
  sf_mex_addfield(c8_sL5XorzCIaSBHWWbjdvvUSD_y, c8_sL5XorzCIaSBHWWbjdvvUSD_b_y,
                  "FcnName", "FcnName", 0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_c_u = c8_sL5XorzCIaSBHWWbjdvvUSD_u.IsSimulinkFcn;
  c8_sL5XorzCIaSBHWWbjdvvUSD_c_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_c_y, sf_mex_create("y",
    &c8_sL5XorzCIaSBHWWbjdvvUSD_c_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c8_sL5XorzCIaSBHWWbjdvvUSD_y, c8_sL5XorzCIaSBHWWbjdvvUSD_c_y,
                  "IsSimulinkFcn", "IsSimulinkFcn", 0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_d_u =
    c8_sL5XorzCIaSBHWWbjdvvUSD_u.NumberOfExtraArgumentInports;
  c8_sL5XorzCIaSBHWWbjdvvUSD_d_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_d_y, sf_mex_create("y",
    &c8_sL5XorzCIaSBHWWbjdvvUSD_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c8_sL5XorzCIaSBHWWbjdvvUSD_y, c8_sL5XorzCIaSBHWWbjdvvUSD_d_y,
                  "NumberOfExtraArgumentInports", "NumberOfExtraArgumentInports",
                  0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_e_u = c8_sL5XorzCIaSBHWWbjdvvUSD_u.HasJacobian;
  c8_sL5XorzCIaSBHWWbjdvvUSD_e_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_e_y, sf_mex_create("y",
    &c8_sL5XorzCIaSBHWWbjdvvUSD_e_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c8_sL5XorzCIaSBHWWbjdvvUSD_y, c8_sL5XorzCIaSBHWWbjdvvUSD_e_y,
                  "HasJacobian", "HasJacobian", 0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_f_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_f_y, sf_mex_create("y", NULL, 10, 0U,
    1U, 0U, 2, 0, 0), false);
  sf_mex_addfield(c8_sL5XorzCIaSBHWWbjdvvUSD_y, c8_sL5XorzCIaSBHWWbjdvvUSD_f_y,
                  "JacobianFcnName", "JacobianFcnName", 0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_f_u = c8_sL5XorzCIaSBHWWbjdvvUSD_u.HasAdditiveNoise;
  c8_sL5XorzCIaSBHWWbjdvvUSD_g_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_g_y, sf_mex_create("y",
    &c8_sL5XorzCIaSBHWWbjdvvUSD_f_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c8_sL5XorzCIaSBHWWbjdvvUSD_y, c8_sL5XorzCIaSBHWWbjdvvUSD_g_y,
                  "HasAdditiveNoise", "HasAdditiveNoise", 0);
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData,
                c8_sL5XorzCIaSBHWWbjdvvUSD_y, false);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
}

static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_d_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_u;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_u = *(real_T *)c8_sL5XorzCIaSBHWWbjdvvUSD_inData;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_y, sf_mex_create("y",
    &c8_sL5XorzCIaSBHWWbjdvvUSD_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData,
                c8_sL5XorzCIaSBHWWbjdvvUSD_y, false);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
}

static real_T c8_sL5XorzCIaSBHWWbjdvvUSD_b_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId)
{
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_d0;
  (void)chartInstance;
  sf_mex_import(c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, sf_mex_dup
                (c8_sL5XorzCIaSBHWWbjdvvUSD_u), &c8_sL5XorzCIaSBHWWbjdvvUSD_d0,
                1, 0, 0U, 0, 0U, 0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = c8_sL5XorzCIaSBHWWbjdvvUSD_d0;
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_u);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_y;
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_b_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_nargin;
  const char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  emlrtMsgIdentifier c8_sL5XorzCIaSBHWWbjdvvUSD_thisId;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_nargin = sf_mex_dup
    (c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
  c8_sL5XorzCIaSBHWWbjdvvUSD_identifier = c8_sL5XorzCIaSBHWWbjdvvUSD_varName;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = (const char *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fParent = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.bParentIsCell = false;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = c8_sL5XorzCIaSBHWWbjdvvUSD_b_emlrt_marshallIn
    (chartInstance, sf_mex_dup(c8_sL5XorzCIaSBHWWbjdvvUSD_nargin),
     &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_nargin);
  *(real_T *)c8_sL5XorzCIaSBHWWbjdvvUSD_outData = c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
}

static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_e_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i109;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_u[4];
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i109 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i109 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i109++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_u[c8_sL5XorzCIaSBHWWbjdvvUSD_i109] = (*(real_T (*)
      [4])c8_sL5XorzCIaSBHWWbjdvvUSD_inData)[c8_sL5XorzCIaSBHWWbjdvvUSD_i109];
  }

  c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_y, sf_mex_create("y",
    c8_sL5XorzCIaSBHWWbjdvvUSD_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData,
                c8_sL5XorzCIaSBHWWbjdvvUSD_y, false);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
}

static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_f_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i110;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i111;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i112;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_u[16];
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_i110 = 0;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i111 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i111 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i111++) {
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i112 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i112 <
         4; c8_sL5XorzCIaSBHWWbjdvvUSD_i112++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_u[c8_sL5XorzCIaSBHWWbjdvvUSD_i112 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i110] = (*(real_T (*)[16])
        c8_sL5XorzCIaSBHWWbjdvvUSD_inData)[c8_sL5XorzCIaSBHWWbjdvvUSD_i112 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i110];
    }

    c8_sL5XorzCIaSBHWWbjdvvUSD_i110 += 4;
  }

  c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_y, sf_mex_create("y",
    c8_sL5XorzCIaSBHWWbjdvvUSD_u, 0, 0U, 1U, 0U, 2, 4, 4), false);
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData,
                c8_sL5XorzCIaSBHWWbjdvvUSD_y, false);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
}

static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_g_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i113;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_u[2];
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i113 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i113 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i113++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_u[c8_sL5XorzCIaSBHWWbjdvvUSD_i113] = (*(real_T (*)
      [2])c8_sL5XorzCIaSBHWWbjdvvUSD_inData)[c8_sL5XorzCIaSBHWWbjdvvUSD_i113];
  }

  c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_y, sf_mex_create("y",
    c8_sL5XorzCIaSBHWWbjdvvUSD_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData,
                c8_sL5XorzCIaSBHWWbjdvvUSD_y, false);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
}

static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_h_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i114;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i115;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i116;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_u[4];
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_i114 = 0;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i115 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i115 < 2;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i115++) {
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i116 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i116 <
         2; c8_sL5XorzCIaSBHWWbjdvvUSD_i116++) {
      c8_sL5XorzCIaSBHWWbjdvvUSD_u[c8_sL5XorzCIaSBHWWbjdvvUSD_i116 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i114] = (*(real_T (*)[4])
        c8_sL5XorzCIaSBHWWbjdvvUSD_inData)[c8_sL5XorzCIaSBHWWbjdvvUSD_i116 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i114];
    }

    c8_sL5XorzCIaSBHWWbjdvvUSD_i114 += 2;
  }

  c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_y, sf_mex_create("y",
    c8_sL5XorzCIaSBHWWbjdvvUSD_u, 0, 0U, 1U, 0U, 2, 2, 2), false);
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData,
                c8_sL5XorzCIaSBHWWbjdvvUSD_y, false);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
}

static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_i_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_u;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_u = *(boolean_T *)c8_sL5XorzCIaSBHWWbjdvvUSD_inData;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_y, sf_mex_create("y",
    &c8_sL5XorzCIaSBHWWbjdvvUSD_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData,
                c8_sL5XorzCIaSBHWWbjdvvUSD_y, false);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_c_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew, const
   char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier, real_T
   c8_sL5XorzCIaSBHWWbjdvvUSD_y[4])
{
  emlrtMsgIdentifier c8_sL5XorzCIaSBHWWbjdvvUSD_thisId;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = (const char *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fParent = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.bParentIsCell = false;
  c8_sL5XorzCIaSBHWWbjdvvUSD_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew), &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId,
    c8_sL5XorzCIaSBHWWbjdvvUSD_y);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew);
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_d_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, real_T
   c8_sL5XorzCIaSBHWWbjdvvUSD_y[4])
{
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_dv3[4];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i117;
  (void)chartInstance;
  sf_mex_import(c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, sf_mex_dup
                (c8_sL5XorzCIaSBHWWbjdvvUSD_u), c8_sL5XorzCIaSBHWWbjdvvUSD_dv3,
                1, 0, 0U, 1, 0U, 1, 4);
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i117 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i117 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i117++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i117] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_dv3[c8_sL5XorzCIaSBHWWbjdvvUSD_i117];
  }

  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_u);
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_c_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew;
  const char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  emlrtMsgIdentifier c8_sL5XorzCIaSBHWWbjdvvUSD_thisId;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_y[4];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i118;
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew = sf_mex_dup
    (c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
  c8_sL5XorzCIaSBHWWbjdvvUSD_identifier = c8_sL5XorzCIaSBHWWbjdvvUSD_varName;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = (const char *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fParent = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.bParentIsCell = false;
  c8_sL5XorzCIaSBHWWbjdvvUSD_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew), &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId,
    c8_sL5XorzCIaSBHWWbjdvvUSD_y);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_b_xNew);
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i118 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i118 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i118++) {
    (*(real_T (*)[4])c8_sL5XorzCIaSBHWWbjdvvUSD_outData)
      [c8_sL5XorzCIaSBHWWbjdvvUSD_i118] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i118];
  }

  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_e_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_c_P, const char_T
   *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier, real_T c8_sL5XorzCIaSBHWWbjdvvUSD_y
   [16])
{
  emlrtMsgIdentifier c8_sL5XorzCIaSBHWWbjdvvUSD_thisId;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = (const char *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fParent = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.bParentIsCell = false;
  c8_sL5XorzCIaSBHWWbjdvvUSD_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_sL5XorzCIaSBHWWbjdvvUSD_c_P), &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId,
    c8_sL5XorzCIaSBHWWbjdvvUSD_y);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_c_P);
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_f_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, real_T
   c8_sL5XorzCIaSBHWWbjdvvUSD_y[16])
{
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_dv4[16];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i119;
  (void)chartInstance;
  sf_mex_import(c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, sf_mex_dup
                (c8_sL5XorzCIaSBHWWbjdvvUSD_u), c8_sL5XorzCIaSBHWWbjdvvUSD_dv4,
                1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i119 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i119 < 16;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i119++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i119] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_dv4[c8_sL5XorzCIaSBHWWbjdvvUSD_i119];
  }

  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_u);
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_d_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_c_P;
  const char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  emlrtMsgIdentifier c8_sL5XorzCIaSBHWWbjdvvUSD_thisId;
  real_T c8_sL5XorzCIaSBHWWbjdvvUSD_y[16];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i120;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i121;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i122;
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_c_P = sf_mex_dup
    (c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
  c8_sL5XorzCIaSBHWWbjdvvUSD_identifier = c8_sL5XorzCIaSBHWWbjdvvUSD_varName;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = (const char *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fParent = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.bParentIsCell = false;
  c8_sL5XorzCIaSBHWWbjdvvUSD_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_sL5XorzCIaSBHWWbjdvvUSD_c_P), &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId,
    c8_sL5XorzCIaSBHWWbjdvvUSD_y);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_c_P);
  c8_sL5XorzCIaSBHWWbjdvvUSD_i120 = 0;
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i121 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i121 < 4;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i121++) {
    for (c8_sL5XorzCIaSBHWWbjdvvUSD_i122 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i122 <
         4; c8_sL5XorzCIaSBHWWbjdvvUSD_i122++) {
      (*(real_T (*)[16])c8_sL5XorzCIaSBHWWbjdvvUSD_outData)
        [c8_sL5XorzCIaSBHWWbjdvvUSD_i122 + c8_sL5XorzCIaSBHWWbjdvvUSD_i120] =
        c8_sL5XorzCIaSBHWWbjdvvUSD_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i122 +
        c8_sL5XorzCIaSBHWWbjdvvUSD_i120];
    }

    c8_sL5XorzCIaSBHWWbjdvvUSD_i120 += 4;
  }

  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
}

static boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_g_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_c_blockOrdering,
   const char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier)
{
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  emlrtMsgIdentifier c8_sL5XorzCIaSBHWWbjdvvUSD_thisId;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = (const char *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fParent = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.bParentIsCell = false;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = c8_sL5XorzCIaSBHWWbjdvvUSD_h_emlrt_marshallIn
    (chartInstance, sf_mex_dup(c8_sL5XorzCIaSBHWWbjdvvUSD_c_blockOrdering),
     &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_c_blockOrdering);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_y;
}

static boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_h_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId)
{
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_b0;
  (void)chartInstance;
  sf_mex_import(c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, sf_mex_dup
                (c8_sL5XorzCIaSBHWWbjdvvUSD_u), &c8_sL5XorzCIaSBHWWbjdvvUSD_b0,
                1, 11, 0U, 0, 0U, 0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = c8_sL5XorzCIaSBHWWbjdvvUSD_b0;
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_u);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_y;
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_e_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_c_blockOrdering;
  const char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  emlrtMsgIdentifier c8_sL5XorzCIaSBHWWbjdvvUSD_thisId;
  boolean_T c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_c_blockOrdering = sf_mex_dup
    (c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
  c8_sL5XorzCIaSBHWWbjdvvUSD_identifier = c8_sL5XorzCIaSBHWWbjdvvUSD_varName;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = (const char *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fParent = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.bParentIsCell = false;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = c8_sL5XorzCIaSBHWWbjdvvUSD_h_emlrt_marshallIn
    (chartInstance, sf_mex_dup(c8_sL5XorzCIaSBHWWbjdvvUSD_c_blockOrdering),
     &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_c_blockOrdering);
  *(boolean_T *)c8_sL5XorzCIaSBHWWbjdvvUSD_outData =
    c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
}

const mxArray
  *sf_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_nameCaptureInfo = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_nameCaptureInfo = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_nameCaptureInfo, sf_mex_create(
    "nameCaptureInfo", NULL, 0, 0U, 1U, 0U, 2, 0, 1), false);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_nameCaptureInfo;
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_warning
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  static char_T c8_sL5XorzCIaSBHWWbjdvvUSD_cv0[7] = { 'w', 'a', 'r', 'n', 'i',
    'n', 'g' };

  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_b_y = NULL;
  static char_T c8_sL5XorzCIaSBHWWbjdvvUSD_cv1[7] = { 'm', 'e', 's', 's', 'a',
    'g', 'e' };

  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_c_y = NULL;
  static char_T c8_sL5XorzCIaSBHWWbjdvvUSD_msgID[27] = { 'C', 'o', 'd', 'e', 'r',
    ':', 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a',
    'r', 'M', 'a', 't', 'r', 'i', 'x' };

  (void)chartInstance;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_y, sf_mex_create("y",
    c8_sL5XorzCIaSBHWWbjdvvUSD_cv0, 10, 0U, 1U, 0U, 2, 1, 7), false);
  c8_sL5XorzCIaSBHWWbjdvvUSD_b_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_b_y, sf_mex_create("y",
    c8_sL5XorzCIaSBHWWbjdvvUSD_cv1, 10, 0U, 1U, 0U, 2, 1, 7), false);
  c8_sL5XorzCIaSBHWWbjdvvUSD_c_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_c_y, sf_mex_create("y",
    c8_sL5XorzCIaSBHWWbjdvvUSD_msgID, 10, 0U, 1U, 0U, 2, 1, 27), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "feval", 0U, 2U, 14,
                    c8_sL5XorzCIaSBHWWbjdvvUSD_y, 14, sf_mex_call_debug
                    (sfGlobalDebugInstanceStruct, "feval", 1U, 2U, 14,
                     c8_sL5XorzCIaSBHWWbjdvvUSD_b_y, 14,
                     c8_sL5XorzCIaSBHWWbjdvvUSD_c_y));
}

static const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_j_sf_marshallOut(void
  *chartInstanceVoid, void *c8_sL5XorzCIaSBHWWbjdvvUSD_inData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_u;
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_u = *(int32_T *)c8_sL5XorzCIaSBHWWbjdvvUSD_inData;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = NULL;
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_y, sf_mex_create("y",
    &c8_sL5XorzCIaSBHWWbjdvvUSD_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData,
                c8_sL5XorzCIaSBHWWbjdvvUSD_y, false);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayOutData;
}

static int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId)
{
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i123;
  (void)chartInstance;
  sf_mex_import(c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, sf_mex_dup
                (c8_sL5XorzCIaSBHWWbjdvvUSD_u), &c8_sL5XorzCIaSBHWWbjdvvUSD_i123,
                1, 6, 0U, 0, 0U, 0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = c8_sL5XorzCIaSBHWWbjdvvUSD_i123;
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_u);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_y;
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_f_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_b_sfEvent;
  const char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  emlrtMsgIdentifier c8_sL5XorzCIaSBHWWbjdvvUSD_thisId;
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_b_sfEvent = sf_mex_dup
    (c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
  c8_sL5XorzCIaSBHWWbjdvvUSD_identifier = c8_sL5XorzCIaSBHWWbjdvvUSD_varName;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = (const char *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fParent = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.bParentIsCell = false;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = c8_sL5XorzCIaSBHWWbjdvvUSD_i_emlrt_marshallIn
    (chartInstance, sf_mex_dup(c8_sL5XorzCIaSBHWWbjdvvUSD_b_sfEvent),
     &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_b_sfEvent);
  *(int32_T *)c8_sL5XorzCIaSBHWWbjdvvUSD_outData = c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
}

static c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB
  c8_sL5XorzCIaSBHWWbjdvvUSD_j_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId)
{
  c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB
    c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  emlrtMsgIdentifier c8_sL5XorzCIaSBHWWbjdvvUSD_thisId;
  static const char * c8_sL5XorzCIaSBHWWbjdvvUSD_fieldNames[6] = { "FcnName",
    "IsSimulinkFcn", "NumberOfExtraArgumentInports", "HasJacobian",
    "JacobianFcnName", "HasAdditiveNoise" };

  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fParent =
    c8_sL5XorzCIaSBHWWbjdvvUSD_parentId;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.bParentIsCell = false;
  sf_mex_check_struct(c8_sL5XorzCIaSBHWWbjdvvUSD_parentId,
                      c8_sL5XorzCIaSBHWWbjdvvUSD_u, 6,
                      c8_sL5XorzCIaSBHWWbjdvvUSD_fieldNames, 0U, NULL);
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = "FcnName";
  c8_sL5XorzCIaSBHWWbjdvvUSD_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c8_sL5XorzCIaSBHWWbjdvvUSD_u, "FcnName", "FcnName", 0)),
    &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId, c8_sL5XorzCIaSBHWWbjdvvUSD_y.FcnName);
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = "IsSimulinkFcn";
  c8_sL5XorzCIaSBHWWbjdvvUSD_y.IsSimulinkFcn =
    c8_sL5XorzCIaSBHWWbjdvvUSD_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c8_sL5XorzCIaSBHWWbjdvvUSD_u, "IsSimulinkFcn",
                     "IsSimulinkFcn", 0)), &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId);
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = "NumberOfExtraArgumentInports";
  c8_sL5XorzCIaSBHWWbjdvvUSD_y.NumberOfExtraArgumentInports =
    c8_sL5XorzCIaSBHWWbjdvvUSD_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c8_sL5XorzCIaSBHWWbjdvvUSD_u,
                     "NumberOfExtraArgumentInports",
                     "NumberOfExtraArgumentInports", 0)),
    &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId);
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = "HasJacobian";
  c8_sL5XorzCIaSBHWWbjdvvUSD_y.HasJacobian =
    c8_sL5XorzCIaSBHWWbjdvvUSD_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c8_sL5XorzCIaSBHWWbjdvvUSD_u, "HasJacobian", "HasJacobian",
                     0)), &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId);
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = "JacobianFcnName";
  c8_sL5XorzCIaSBHWWbjdvvUSD_l_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c8_sL5XorzCIaSBHWWbjdvvUSD_u, "JacobianFcnName",
                     "JacobianFcnName", 0)), &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId);
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = "HasAdditiveNoise";
  c8_sL5XorzCIaSBHWWbjdvvUSD_y.HasAdditiveNoise =
    c8_sL5XorzCIaSBHWWbjdvvUSD_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c8_sL5XorzCIaSBHWWbjdvvUSD_u, "HasAdditiveNoise",
                     "HasAdditiveNoise", 0)), &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_u);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_y;
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_k_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, char_T
   c8_sL5XorzCIaSBHWWbjdvvUSD_y[19])
{
  char_T c8_sL5XorzCIaSBHWWbjdvvUSD_cv2[19];
  int32_T c8_sL5XorzCIaSBHWWbjdvvUSD_i124;
  (void)chartInstance;
  sf_mex_import(c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, sf_mex_dup
                (c8_sL5XorzCIaSBHWWbjdvvUSD_u), c8_sL5XorzCIaSBHWWbjdvvUSD_cv2,
                1, 10, 0U, 1, 0U, 2, 1, 19);
  for (c8_sL5XorzCIaSBHWWbjdvvUSD_i124 = 0; c8_sL5XorzCIaSBHWWbjdvvUSD_i124 < 19;
       c8_sL5XorzCIaSBHWWbjdvvUSD_i124++) {
    c8_sL5XorzCIaSBHWWbjdvvUSD_y[c8_sL5XorzCIaSBHWWbjdvvUSD_i124] =
      c8_sL5XorzCIaSBHWWbjdvvUSD_cv2[c8_sL5XorzCIaSBHWWbjdvvUSD_i124];
  }

  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_u);
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_l_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId)
{
  (void)chartInstance;
  sf_mex_import(c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, sf_mex_dup
                (c8_sL5XorzCIaSBHWWbjdvvUSD_u), NULL, 1, 10, 0U, 1, 0U, 2, 0, 0);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_u);
}

static void c8_sL5XorzCIaSBHWWbjdvvUSD_g_sf_marshallIn(void *chartInstanceVoid,
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData, const char_T
  *c8_sL5XorzCIaSBHWWbjdvvUSD_varName, void *c8_sL5XorzCIaSBHWWbjdvvUSD_outData)
{
  const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_b_pM;
  const char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  emlrtMsgIdentifier c8_sL5XorzCIaSBHWWbjdvvUSD_thisId;
  c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB
    c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
    chartInstanceVoid;
  c8_sL5XorzCIaSBHWWbjdvvUSD_b_pM = sf_mex_dup
    (c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
  c8_sL5XorzCIaSBHWWbjdvvUSD_identifier = c8_sL5XorzCIaSBHWWbjdvvUSD_varName;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = (const char *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fParent = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.bParentIsCell = false;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = c8_sL5XorzCIaSBHWWbjdvvUSD_j_emlrt_marshallIn
    (chartInstance, sf_mex_dup(c8_sL5XorzCIaSBHWWbjdvvUSD_b_pM),
     &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId);
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_b_pM);
  *(c8_sL5XorzCIaSBHWWbjdvvUSD_sUAIaS8TPWuzgg7KvQbnkvB *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_outData = c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_mxArrayInData);
}

static uint8_T c8_sL5XorzCIaSBHWWbjdvvUSD_m_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray
   *c8_sL5XorzCIaSBHWWbjdvvUSD_b_is_active_c8_sL5XorzCIaSBHWWbjdvvUS, const
   char_T *c8_sL5XorzCIaSBHWWbjdvvUSD_identifier)
{
  uint8_T c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  emlrtMsgIdentifier c8_sL5XorzCIaSBHWWbjdvvUSD_thisId;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fIdentifier = (const char *)
    c8_sL5XorzCIaSBHWWbjdvvUSD_identifier;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.fParent = NULL;
  c8_sL5XorzCIaSBHWWbjdvvUSD_thisId.bParentIsCell = false;
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = c8_sL5XorzCIaSBHWWbjdvvUSD_n_emlrt_marshallIn
    (chartInstance, sf_mex_dup
     (c8_sL5XorzCIaSBHWWbjdvvUSD_b_is_active_c8_sL5XorzCIaSBHWWbjdvvUS),
     &c8_sL5XorzCIaSBHWWbjdvvUSD_thisId);
  sf_mex_destroy
    (&c8_sL5XorzCIaSBHWWbjdvvUSD_b_is_active_c8_sL5XorzCIaSBHWWbjdvvUS);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_y;
}

static uint8_T c8_sL5XorzCIaSBHWWbjdvvUSD_n_emlrt_marshallIn
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance, const mxArray *c8_sL5XorzCIaSBHWWbjdvvUSD_u, const
   emlrtMsgIdentifier *c8_sL5XorzCIaSBHWWbjdvvUSD_parentId)
{
  uint8_T c8_sL5XorzCIaSBHWWbjdvvUSD_y;
  uint8_T c8_sL5XorzCIaSBHWWbjdvvUSD_u0;
  (void)chartInstance;
  sf_mex_import(c8_sL5XorzCIaSBHWWbjdvvUSD_parentId, sf_mex_dup
                (c8_sL5XorzCIaSBHWWbjdvvUSD_u), &c8_sL5XorzCIaSBHWWbjdvvUSD_u0,
                1, 3, 0U, 0, 0U, 0);
  c8_sL5XorzCIaSBHWWbjdvvUSD_y = c8_sL5XorzCIaSBHWWbjdvvUSD_u0;
  sf_mex_destroy(&c8_sL5XorzCIaSBHWWbjdvvUSD_u);
  return c8_sL5XorzCIaSBHWWbjdvvUSD_y;
}

static void init_dsm_address_info
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address
  (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
   *chartInstance)
{
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_fEmlrtCtx = (void *)sfrtGetEmlrtCtx
    (chartInstance->S);
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_x = (real_T (*)[4])
    ssGetInputPortSignal_wrapper(chartInstance->S, 0);
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_xNew = (real_T (*)[4])
    ssGetOutputPortSignal_wrapper(chartInstance->S, 1);
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_P = (real_T (*)[16])
    ssGetInputPortSignal_wrapper(chartInstance->S, 1);
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_b_P = (real_T (*)[16])
    ssGetOutputPortSignal_wrapper(chartInstance->S, 2);
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_yMeas = (real_T (*)[2])
    ssGetInputPortSignal_wrapper(chartInstance->S, 2);
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_R = (real_T (*)[4])
    ssGetInputPortSignal_wrapper(chartInstance->S, 3);
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_uMeas = (real_T *)
    ssGetInputPortSignal_wrapper(chartInstance->S, 4);
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_blockOrdering = (boolean_T *)
    ssGetInputPortSignal_wrapper(chartInstance->S, 5);
  chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_b_blockOrdering = (boolean_T *)
    ssGetOutputPortSignal_wrapper(chartInstance->S, 3);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_get_check_sum(mxArray
  *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3078900515U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2170061050U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2090034413U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2719643810U);
}

mxArray*
  sf_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_get_post_codegen_info(void);
mxArray
  *sf_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_get_autoinheritance_info
  (void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("1FfrySwIiTJVLfzEZ106VD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(4);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(2);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(4);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo =
      sf_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray
  *sf_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_third_party_uses_info
  (void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_jit_fallback_info
  (void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("late");
  mxArray *fallbackReason = mxCreateString("client_server");
  mxArray *hiddenFallbackType = mxCreateString("");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("radarMeasurementFcn");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray
  *sf_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_updateBuildInfo_args_info
  (void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray*
  sf_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString(
      "pWmdzi6mm0GphZfPjnM2LE");
    mwSize exp_dims[2] = { 1, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);

    {
      mxArray* mxFcnName = mxCreateString("radarMeasurementFcn");
      mxSetCell(mxExportedFunctionsUsedByThisChart, 0, mxFcnName);
    }

    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray
  *sf_get_sim_state_info_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[8],T\"P\",},{M[1],M[21],T\"blockOrdering\",},{M[1],M[5],T\"xNew\",},{M[8],M[0],T\"is_active_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
      *chartInstance =
      (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _sharedTrackingLibraryMachineNumber_,
           8,
           1,
           1,
           0,
           10,
           0,
           0,
           0,
           0,
           0,
           &chartInstance->chartNumber,
           &chartInstance->instanceNumber,
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_sharedTrackingLibraryMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_sharedTrackingLibraryMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _sharedTrackingLibraryMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"x");
          _SFD_SET_DATA_PROPS(1,1,1,0,"P");
          _SFD_SET_DATA_PROPS(2,1,1,0,"yMeas");
          _SFD_SET_DATA_PROPS(3,1,1,0,"R");
          _SFD_SET_DATA_PROPS(4,1,1,0,"uMeas");
          _SFD_SET_DATA_PROPS(5,1,1,0,"blockOrdering");
          _SFD_SET_DATA_PROPS(6,2,0,1,"xNew");
          _SFD_SET_DATA_PROPS(7,2,0,1,"P");
          _SFD_SET_DATA_PROPS(8,2,0,1,"blockOrdering");
          _SFD_SET_DATA_PROPS(9,10,0,0,"pM");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,5,0,6,0,0,1,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1511);
        _SFD_CV_INIT_EML_FCN(0,1,"",289,-1,324);
        _SFD_CV_INIT_EML_FCN(0,2,"",361,-1,418);
        _SFD_CV_INIT_EML_FCN(0,3,"",659,-1,702);
        _SFD_CV_INIT_EML_FCN(0,4,"",755,-1,820);
        _SFD_CV_INIT_EML_IF(0,1,0,169,188,428,480);
        _SFD_CV_INIT_EML_IF(0,1,1,240,262,330,427);
        _SFD_CV_INIT_EML_IF(0,1,2,481,498,915,957);
        _SFD_CV_INIT_EML_IF(0,1,3,503,522,838,914);
        _SFD_CV_INIT_EML_IF(0,1,4,598,620,712,833);
        _SFD_CV_INIT_EML_IF(0,1,5,1160,1182,1342,1507);

        {
          static int caseStart[] = { 1077, 1002, 1037 };

          static int caseExprEnd[] = { 1086, 1008, 1043 };

          _SFD_CV_INIT_EML_SWITCH(0,1,0,959,998,1113,3,&(caseStart[0]),
            &(caseExprEnd[0]));
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4U;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_e_sf_marshallOut,
            (MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4U;
          dimVector[1]= 4U;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_f_sf_marshallOut,
            (MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2U;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_g_sf_marshallOut,
            (MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 2U;
          dimVector[1]= 2U;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_h_sf_marshallOut,
            (MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_d_sf_marshallOut,
          (MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_i_sf_marshallOut,
          (MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 4U;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_e_sf_marshallOut,
            (MexInFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4U;
          dimVector[1]= 4U;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_f_sf_marshallOut,
            (MexInFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_d_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(8,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_i_sf_marshallOut,
          (MexInFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_e_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_c_sf_marshallOut,
          (MexInFcnForType)c8_sL5XorzCIaSBHWWbjdvvUSD_g_sf_marshallIn);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _sharedTrackingLibraryMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
      *chartInstance =
      (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, (void *)
          chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_x);
        _SFD_SET_DATA_VALUE_PTR(6U, (void *)
          chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_xNew);
        _SFD_SET_DATA_VALUE_PTR(1U, (void *)
          chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_P);
        _SFD_SET_DATA_VALUE_PTR(7U, (void *)
          chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_b_P);
        _SFD_SET_DATA_VALUE_PTR(2U, (void *)
          chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_yMeas);
        _SFD_SET_DATA_VALUE_PTR(3U, (void *)
          chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_R);
        _SFD_SET_DATA_VALUE_PTR(4U, (void *)
          chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_uMeas);
        _SFD_SET_DATA_VALUE_PTR(5U, (void *)
          chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_blockOrdering);
        _SFD_SET_DATA_VALUE_PTR(9U, (void *)
          &chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_pM);
        _SFD_SET_DATA_VALUE_PTR(8U, (void *)
          chartInstance->c8_sL5XorzCIaSBHWWbjdvvUSD_b_blockOrdering);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sL5XorzCIaSBHWWbjdvvUSD";
}

static void
  sf_opaque_initialize_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary(void
  *chartInstanceVar)
{
  chart_debug_initialization
    (((SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct*)
      chartInstanceVar)->S,0);
  initialize_params_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
    ((SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct*)
     chartInstanceVar);
  initialize_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
    ((SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_enable_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (void *chartInstanceVar)
{
  enable_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
    ((SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_disable_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (void *chartInstanceVar)
{
  disable_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
    ((SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_gateway_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (void *chartInstanceVar)
{
  sf_gateway_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
    ((SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct*)
     chartInstanceVar);
}

static const mxArray*
  sf_opaque_get_sim_state_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SimStruct* S)
{
  return get_sim_state_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
    ((SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)
     sf_get_chart_instance_ptr(S));    /* raw sim ctx */
}

static void
  sf_opaque_set_sim_state_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SimStruct* S, const mxArray *st)
{
  set_sim_state_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
    ((SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct*)
     sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S =
      ((SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct*)
       chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_sharedTrackingLibrary_optimization_info();
    }

    finalize_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
      ((SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct*)
       chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
    ((SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct*)
     chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void
  mdlProcessParameters_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
      ((SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct*)
       sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary
  (SimStruct *S)
{
  /* Actual parameters from chart:
     pM
   */
  const char_T *rtParamNames[] = { "pM" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));

  /* Set overwritable ports for inplace optimization */
  ssSetInputPortOverWritable(S, 1, 1);
  ssSetOutputPortOverwritesInputPort(S, 2, 1);
  ssSetInputPortOverWritable(S, 5, 1);
  ssSetOutputPortOverwritesInputPort(S, 3, 5);
  ssSetInputPortDirectFeedThrough(S, 0, 1);
  ssSetInputPortDirectFeedThrough(S, 1, 1);
  ssSetInputPortDirectFeedThrough(S, 2, 1);
  ssSetInputPortDirectFeedThrough(S, 3, 1);
  ssSetInputPortDirectFeedThrough(S, 4, 1);
  ssSetInputPortDirectFeedThrough(S, 5, 1);
  ssSetStatesModifiedOnlyInUpdate(S, 0);
  ssSetBlockIsPurelyCombinatorial_wrapper(S, 0);
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_sharedTrackingLibrary_optimization_info
      (sim_mode_is_rtw_gen(S), sim_mode_is_modelref_sim(S), sim_mode_is_external
       (S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,8);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetSupportedForRowMajorCodeGen(S, 1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,8,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 8);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,8);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,8,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,8,3);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=3; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 6; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,8);
    sf_register_codegen_names_for_scoped_functions_defined_by_chart(S);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3078900515U));
  ssSetChecksum1(S,(2170061050U));
  ssSetChecksum2(S,(2090034413U));
  ssSetChecksum3(S,(2719643810U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,0);
}

static void mdlRTW_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary(SimStruct *
  S)
{
  SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct
    *chartInstance;
  chartInstance =
    (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct *)utMalloc
    (sizeof(SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof
         (SFc8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibraryInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  if (ssGetSampleTime(S, 0) == CONTINUOUS_SAMPLE_TIME && ssGetOffsetTime(S, 0) ==
      0 && ssGetNumContStates(ssGetRootSS(S)) > 0) {
    sf_error_out_about_continuous_sample_time_with_persistent_vars(S);
  }

  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW =
    mdlRTW_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary;
  chartInstance->chartInfo.mdlStart =
    mdlStart_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartEventFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary(chartInstance);
}

void c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_method_dispatcher
  (SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c8_sL5XorzCIaSBHWWbjdvvUSD_sharedTrackingLibrary_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
