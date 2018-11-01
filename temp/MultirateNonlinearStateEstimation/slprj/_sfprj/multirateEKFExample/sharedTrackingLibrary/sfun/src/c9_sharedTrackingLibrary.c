/* Include files */

#include "sharedTrackingLibrary_sfun.h"
#include "c9_sharedTrackingLibrary.h"
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
static const char * c9_debug_family_names[10] = { "extraArgs", "pS", "nargin",
  "nargout", "x", "P", "Q", "uState", "unused", "xNew" };

/* Function Declarations */
static void initialize_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance);
static void initialize_params_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance);
static void enable_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance);
static void disable_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance);
static void c9_update_debugger_state_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance);
static void set_sim_state_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance, const mxArray *c9_st);
static void finalize_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance);
static void sf_gateway_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance);
static void mdl_start_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance);
static void initSimStructsc9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance);
static void c9_stateTransitionFcn_invoke
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance, real_T c9_b_x[4],
   real_T c9_w[2], real_T c9_xNext[4]);
static void c9_stateTransitionJacobianFcn_invoke
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance, real_T c9_b_x[4],
   real_T c9_w[2], real_T c9_A[16], real_T c9_G[8]);
static void init_script_number_translation(uint32_T c9_machineNumber, uint32_T
  c9_chartNumber, uint32_T c9_instanceNumber);
static const mxArray *c9_sf_marshallOut(void *chartInstanceVoid, void *c9_inData);
static void c9_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static const mxArray *c9_b_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static const mxArray *c9_c_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static real_T c9_b_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static const mxArray *c9_d_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static const mxArray *c9_e_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static const mxArray *c9_f_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static const mxArray *c9_g_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static void c9_c_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_b_xNew, const char_T *c9_identifier, real_T
  c9_y[4]);
static void c9_d_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[4]);
static void c9_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static void c9_e_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_c_P, const char_T *c9_identifier, real_T
  c9_y[16]);
static void c9_f_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[16]);
static void c9_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static const mxArray *c9_h_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static int32_T c9_g_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static void c9_h_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  c9_sGeqiQRKfMYBjhkuVHSNkYD *c9_y);
static void c9_i_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  char_T c9_y[18]);
static boolean_T c9_j_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct *
  chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_k_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  char_T c9_y[26]);
static void c9_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static uint8_T c9_l_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_b_is_active_c9_sharedTrackingLibrary, const
  char_T *c9_identifier);
static uint8_T c9_m_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void init_dsm_address_info(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc9_sharedTrackingLibrary(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c9_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c9_is_active_c9_sharedTrackingLibrary = 0U;
}

static void initialize_params_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance)
{
  const mxArray *c9_m0 = NULL;
  static const char * c9_fieldNames[6] = { "FcnName", "IsSimulinkFcn",
    "NumberOfExtraArgumentInports", "JacobianFcnName", "HasJacobian",
    "HasAdditiveNoise" };

  const mxArray *c9_mxField;
  c9_sGeqiQRKfMYBjhkuVHSNkYD c9_r0;
  c9_m0 = sf_mex_get_sfun_param(chartInstance->S, 0U, 1U);
  sf_mex_check_bus_parameter(c9_m0, 0, NULL, 6, c9_fieldNames, "pS",
    "sGeqiQRKfMYBjhkuVHSNkYD");
  c9_mxField = sf_mex_getfield(c9_m0, "FcnName", "pS", 0);
  sf_mex_import_named("pS", sf_mex_dup(c9_mxField), c9_r0.FcnName, 1, 10, 0U, 1,
                      0U, 2, 1, 18);
  c9_mxField = sf_mex_getfield(c9_m0, "IsSimulinkFcn", "pS", 0);
  sf_mex_import_named("pS", sf_mex_dup(c9_mxField), &c9_r0.IsSimulinkFcn, 1, 11,
                      0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "NumberOfExtraArgumentInports", "pS", 0);
  sf_mex_import_named("pS", sf_mex_dup(c9_mxField),
                      &c9_r0.NumberOfExtraArgumentInports, 1, 0, 0U, 0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "JacobianFcnName", "pS", 0);
  sf_mex_import_named("pS", sf_mex_dup(c9_mxField), c9_r0.JacobianFcnName, 1, 10,
                      0U, 1, 0U, 2, 1, 26);
  c9_mxField = sf_mex_getfield(c9_m0, "HasJacobian", "pS", 0);
  sf_mex_import_named("pS", sf_mex_dup(c9_mxField), &c9_r0.HasJacobian, 1, 0, 0U,
                      0, 0U, 0);
  c9_mxField = sf_mex_getfield(c9_m0, "HasAdditiveNoise", "pS", 0);
  sf_mex_import_named("pS", sf_mex_dup(c9_mxField), &c9_r0.HasAdditiveNoise, 1,
                      11, 0U, 0, 0U, 0);
  sf_mex_destroy(&c9_m0);
  chartInstance->c9_pS = c9_r0;
}

static void enable_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c9_update_debugger_state_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance)
{
  const mxArray *c9_st;
  const mxArray *c9_y = NULL;
  const mxArray *c9_b_y = NULL;
  const mxArray *c9_c_y = NULL;
  uint8_T c9_hoistedGlobal;
  const mxArray *c9_d_y = NULL;
  c9_st = NULL;
  c9_st = NULL;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_createcellmatrix(3, 1), false);
  c9_b_y = NULL;
  sf_mex_assign(&c9_b_y, sf_mex_create("y", *chartInstance->c9_b_P, 0, 0U, 1U,
    0U, 2, 4, 4), false);
  sf_mex_setcell(c9_y, 0, c9_b_y);
  c9_c_y = NULL;
  sf_mex_assign(&c9_c_y, sf_mex_create("y", *chartInstance->c9_xNew, 0, 0U, 1U,
    0U, 1, 4), false);
  sf_mex_setcell(c9_y, 1, c9_c_y);
  c9_hoistedGlobal = chartInstance->c9_is_active_c9_sharedTrackingLibrary;
  c9_d_y = NULL;
  sf_mex_assign(&c9_d_y, sf_mex_create("y", &c9_hoistedGlobal, 3, 0U, 0U, 0U, 0),
                false);
  sf_mex_setcell(c9_y, 2, c9_d_y);
  sf_mex_assign(&c9_st, c9_y, false);
  return c9_st;
}

static void set_sim_state_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance, const mxArray *c9_st)
{
  const mxArray *c9_u;
  real_T c9_dv0[16];
  int32_T c9_i0;
  real_T c9_dv1[4];
  int32_T c9_i1;
  chartInstance->c9_doneDoubleBufferReInit = true;
  c9_u = sf_mex_dup(c9_st);
  c9_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c9_u, 0)), "P",
                        c9_dv0);
  for (c9_i0 = 0; c9_i0 < 16; c9_i0++) {
    (*chartInstance->c9_b_P)[c9_i0] = c9_dv0[c9_i0];
  }

  c9_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c9_u, 1)),
                        "xNew", c9_dv1);
  for (c9_i1 = 0; c9_i1 < 4; c9_i1++) {
    (*chartInstance->c9_xNew)[c9_i1] = c9_dv1[c9_i1];
  }

  chartInstance->c9_is_active_c9_sharedTrackingLibrary = c9_l_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c9_u, 2)),
     "is_active_c9_sharedTrackingLibrary");
  sf_mex_destroy(&c9_u);
  c9_update_debugger_state_c9_sharedTrackingLibrary(chartInstance);
  sf_mex_destroy(&c9_st);
}

static void finalize_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance)
{
  int32_T c9_i2;
  int32_T c9_i3;
  int32_T c9_i4;
  real_T c9_hoistedGlobal;
  boolean_T c9_b_hoistedGlobal;
  int32_T c9_i5;
  int32_T c9_i6;
  real_T c9_b_x[4];
  int32_T c9_i7;
  real_T c9_c_P[16];
  real_T c9_b_uState;
  real_T c9_b_Q[4];
  boolean_T c9_b_unused;
  uint32_T c9_debug_family_var_map[10];
  c9_sGeqiQRKfMYBjhkuVHSNkYD c9_b_pS;
  real_T c9_nargin = 5.0;
  real_T c9_nargout = 2.0;
  real_T c9_b_xNew[4];
  real_T c9_d_P[16];
  static c9_sGeqiQRKfMYBjhkuVHSNkYD c9_r1 = { { 's', 't', 'a', 't', 'e', 'T',
      'r', 'a', 'n', 's', 'i', 't', 'i', 'o', 'n', 'F', 'c', 'n' },/* FcnName */
    true,                              /* IsSimulinkFcn */
    0.0,                               /* NumberOfExtraArgumentInports */
    { 's', 't', 'a', 't', 'e', 'T', 'r', 'a', 'n', 's', 'i', 't', 'i', 'o', 'n',
      'J', 'a', 'c', 'o', 'b', 'i', 'a', 'n', 'F', 'c', 'n' },/* JacobianFcnName */
    1.0,                               /* HasJacobian */
    false                              /* HasAdditiveNoise */
  };

  int32_T c9_i8;
  int32_T c9_i9;
  int32_T c9_i10;
  int32_T c9_i11;
  real_T c9_c_Q[4];
  int32_T c9_i12;
  real_T c9_c_x[4];
  int32_T c9_i13;
  real_T c9_y[16];
  real_T c9_fState[4];
  int32_T c9_i14;
  real_T c9_dv2[2];
  real_T c9_varargout_1[16];
  real_T c9_varargout_2[8];
  int32_T c9_i15;
  int32_T c9_i16;
  int32_T c9_i17;
  int32_T c9_i18;
  int32_T c9_i19;
  int32_T c9_i20;
  int32_T c9_i21;
  real_T c9_b_y[16];
  int32_T c9_i22;
  int32_T c9_i23;
  int32_T c9_i24;
  int32_T c9_i25;
  int32_T c9_i26;
  int32_T c9_i27;
  real_T c9_b[16];
  int32_T c9_i28;
  int32_T c9_i29;
  int32_T c9_i30;
  int32_T c9_i31;
  int32_T c9_i32;
  int32_T c9_i33;
  int32_T c9_i34;
  int32_T c9_i35;
  int32_T c9_i36;
  int32_T c9_i37;
  real_T c9_c_y[8];
  int32_T c9_i38;
  int32_T c9_i39;
  int32_T c9_i40;
  int32_T c9_i41;
  int32_T c9_i42;
  real_T c9_b_b[8];
  int32_T c9_i43;
  int32_T c9_i44;
  int32_T c9_i45;
  int32_T c9_i46;
  int32_T c9_i47;
  real_T c9_dv3[2];
  int32_T c9_i48;
  int32_T c9_i49;
  int32_T c9_i50;
  int32_T c9_i51;
  int32_T c9_i52;
  int32_T c9_i53;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c9_sfEvent);
  _SFD_DATA_RANGE_CHECK((real_T)*chartInstance->c9_unused, 4U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c9_uState, 3U);
  for (c9_i2 = 0; c9_i2 < 4; c9_i2++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c9_Q)[c9_i2], 2U);
  }

  for (c9_i3 = 0; c9_i3 < 16; c9_i3++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c9_P)[c9_i3], 1U);
  }

  for (c9_i4 = 0; c9_i4 < 4; c9_i4++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c9_x)[c9_i4], 0U);
  }

  chartInstance->c9_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c9_sfEvent);
  c9_hoistedGlobal = *chartInstance->c9_uState;
  c9_b_hoistedGlobal = *chartInstance->c9_unused;
  for (c9_i5 = 0; c9_i5 < 4; c9_i5++) {
    c9_b_x[c9_i5] = (*chartInstance->c9_x)[c9_i5];
  }

  for (c9_i6 = 0; c9_i6 < 16; c9_i6++) {
    c9_c_P[c9_i6] = (*chartInstance->c9_P)[c9_i6];
  }

  for (c9_i7 = 0; c9_i7 < 4; c9_i7++) {
    c9_b_Q[c9_i7] = (*chartInstance->c9_Q)[c9_i7];
  }

  c9_b_uState = c9_hoistedGlobal;
  c9_b_unused = c9_b_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 11U, c9_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(NULL, 0U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_b_pS, 1U, c9_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargin, 2U, c9_c_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargout, 3U, c9_c_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_b_x, 4U, c9_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_c_P, 5U, c9_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_b_Q, 6U, c9_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_b_uState, 7U, c9_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_b_unused, 8U, c9_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_b_xNew, 9U, c9_d_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_d_P, MAX_uint32_T, c9_e_sf_marshallOut,
    c9_d_sf_marshallIn);
  c9_b_pS = c9_r1;
  for (c9_i8 = 0; c9_i8 < 16; c9_i8++) {
    c9_d_P[c9_i8] = c9_c_P[c9_i8];
  }

  _SFD_SYMBOL_SWITCH(5U, 10U);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 9);
  CV_EML_IF(0, 1, 0, c9_b_pS.IsSimulinkFcn);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 10);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 11);
  CV_EML_IF(0, 1, 1, c9_b_pS.HasAdditiveNoise);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 14);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 19);
  CV_EML_IF(0, 1, 2, true);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 20);
  CV_EML_IF(0, 1, 3, c9_b_pS.IsSimulinkFcn);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 21);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 22);
  CV_EML_IF(0, 1, 4, c9_b_pS.HasAdditiveNoise);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 25);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 34);
  CV_EML_SWITCH(0, 1, 0, 1);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 36);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 43);
  for (c9_i9 = 0; c9_i9 < 4; c9_i9++) {
    c9_b_xNew[c9_i9] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 44);
  CV_EML_IF(0, 1, 5, c9_b_pS.HasAdditiveNoise);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 48);
  for (c9_i10 = 0; c9_i10 < 4; c9_i10++) {
    c9_c_Q[c9_i10] = c9_b_Q[c9_i10];
  }

  for (c9_i11 = 0; c9_i11 < 4; c9_i11++) {
    c9_c_x[c9_i11] = c9_b_x[c9_i11];
  }

  for (c9_i12 = 0; c9_i12 < 16; c9_i12++) {
    c9_y[c9_i12] = c9_d_P[c9_i12];
  }

  for (c9_i13 = 0; c9_i13 < 4; c9_i13++) {
    c9_fState[c9_i13] = c9_c_x[c9_i13];
  }

  CV_EML_FCN(0, 4);
  for (c9_i14 = 0; c9_i14 < 2; c9_i14++) {
    c9_dv2[c9_i14] = 0.0;
  }

  c9_stateTransitionJacobianFcn_invoke(chartInstance, c9_fState, c9_dv2,
    c9_varargout_1, c9_varargout_2);
  for (c9_i15 = 0; c9_i15 < 4; c9_i15++) {
    c9_i17 = 0;
    for (c9_i19 = 0; c9_i19 < 4; c9_i19++) {
      c9_b_y[c9_i17 + c9_i15] = 0.0;
      c9_i23 = 0;
      for (c9_i24 = 0; c9_i24 < 4; c9_i24++) {
        c9_b_y[c9_i17 + c9_i15] += c9_varargout_1[c9_i23 + c9_i15] * c9_y[c9_i24
          + c9_i17];
        c9_i23 += 4;
      }

      c9_i17 += 4;
    }
  }

  c9_i16 = 0;
  for (c9_i18 = 0; c9_i18 < 4; c9_i18++) {
    c9_i21 = 0;
    for (c9_i22 = 0; c9_i22 < 4; c9_i22++) {
      c9_b[c9_i22 + c9_i16] = c9_varargout_1[c9_i21 + c9_i18];
      c9_i21 += 4;
    }

    c9_i16 += 4;
  }

  for (c9_i20 = 0; c9_i20 < 4; c9_i20++) {
    c9_i26 = 0;
    for (c9_i27 = 0; c9_i27 < 4; c9_i27++) {
      c9_y[c9_i26 + c9_i20] = 0.0;
      c9_i33 = 0;
      for (c9_i34 = 0; c9_i34 < 4; c9_i34++) {
        c9_y[c9_i26 + c9_i20] += c9_b_y[c9_i33 + c9_i20] * c9_b[c9_i34 + c9_i26];
        c9_i33 += 4;
      }

      c9_i26 += 4;
    }
  }

  for (c9_i25 = 0; c9_i25 < 4; c9_i25++) {
    c9_i29 = 0;
    c9_i31 = 0;
    for (c9_i32 = 0; c9_i32 < 2; c9_i32++) {
      c9_c_y[c9_i29 + c9_i25] = 0.0;
      c9_i38 = 0;
      for (c9_i41 = 0; c9_i41 < 2; c9_i41++) {
        c9_c_y[c9_i29 + c9_i25] += c9_varargout_2[c9_i38 + c9_i25] *
          c9_c_Q[c9_i41 + c9_i31];
        c9_i38 += 4;
      }

      c9_i29 += 4;
      c9_i31 += 2;
    }
  }

  c9_i28 = 0;
  for (c9_i30 = 0; c9_i30 < 4; c9_i30++) {
    c9_i36 = 0;
    for (c9_i37 = 0; c9_i37 < 2; c9_i37++) {
      c9_b_b[c9_i37 + c9_i28] = c9_varargout_2[c9_i36 + c9_i30];
      c9_i36 += 4;
    }

    c9_i28 += 2;
  }

  for (c9_i35 = 0; c9_i35 < 4; c9_i35++) {
    c9_i40 = 0;
    c9_i42 = 0;
    for (c9_i43 = 0; c9_i43 < 4; c9_i43++) {
      c9_b_y[c9_i40 + c9_i35] = 0.0;
      c9_i45 = 0;
      for (c9_i47 = 0; c9_i47 < 2; c9_i47++) {
        c9_b_y[c9_i40 + c9_i35] += c9_c_y[c9_i45 + c9_i35] * c9_b_b[c9_i47 +
          c9_i42];
        c9_i45 += 4;
      }

      c9_i40 += 4;
      c9_i42 += 2;
    }
  }

  for (c9_i39 = 0; c9_i39 < 16; c9_i39++) {
    c9_y[c9_i39] += c9_b_y[c9_i39];
  }

  for (c9_i44 = 0; c9_i44 < 4; c9_i44++) {
    c9_fState[c9_i44] = c9_c_x[c9_i44];
  }

  CV_EML_FCN(0, 2);
  for (c9_i46 = 0; c9_i46 < 2; c9_i46++) {
    c9_dv3[c9_i46] = 0.0;
  }

  c9_stateTransitionFcn_invoke(chartInstance, c9_fState, c9_dv3, c9_c_x);
  for (c9_i48 = 0; c9_i48 < 4; c9_i48++) {
    c9_b_xNew[c9_i48] = c9_c_x[c9_i48];
  }

  for (c9_i49 = 0; c9_i49 < 16; c9_i49++) {
    c9_d_P[c9_i49] = c9_y[c9_i49];
  }

  _SFD_SYMBOL_SWITCH(5U, 10U);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, -48);
  _SFD_SYMBOL_SCOPE_POP();
  for (c9_i50 = 0; c9_i50 < 4; c9_i50++) {
    (*chartInstance->c9_xNew)[c9_i50] = c9_b_xNew[c9_i50];
  }

  for (c9_i51 = 0; c9_i51 < 16; c9_i51++) {
    (*chartInstance->c9_b_P)[c9_i51] = c9_d_P[c9_i51];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c9_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  for (c9_i52 = 0; c9_i52 < 4; c9_i52++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c9_xNew)[c9_i52], 5U);
  }

  for (c9_i53 = 0; c9_i53 < 16; c9_i53++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c9_b_P)[c9_i53], 6U);
  }
}

static void mdl_start_c9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance)
{
  sim_mode_is_external(chartInstance->S);
}

static void initSimStructsc9_sharedTrackingLibrary
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c9_stateTransitionFcn_invoke
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance, real_T c9_b_x[4],
   real_T c9_w[2], real_T c9_xNext[4])
{
  _ssFcnCallExecArgInfo c9_args[3];
  c9_args[0U].dataPtr = (void *)c9_b_x;
  c9_args[1U].dataPtr = (void *)c9_w;
  c9_args[2U].dataPtr = (void *)c9_xNext;
  slcsInvokeSimulinkFunction(chartInstance->S, "stateTransitionFcn", &c9_args[0U]);
}

static void c9_stateTransitionJacobianFcn_invoke
  (SFc9_sharedTrackingLibraryInstanceStruct *chartInstance, real_T c9_b_x[4],
   real_T c9_w[2], real_T c9_A[16], real_T c9_G[8])
{
  _ssFcnCallExecArgInfo c9_args[4];
  c9_args[0U].dataPtr = (void *)c9_b_x;
  c9_args[1U].dataPtr = (void *)c9_w;
  c9_args[2U].dataPtr = (void *)c9_A;
  c9_args[3U].dataPtr = (void *)c9_G;
  slcsInvokeSimulinkFunction(chartInstance->S, "stateTransitionJacobianFcn",
    &c9_args[0U]);
}

static void init_script_number_translation(uint32_T c9_machineNumber, uint32_T
  c9_chartNumber, uint32_T c9_instanceNumber)
{
  (void)(c9_machineNumber);
  (void)(c9_chartNumber);
  (void)(c9_instanceNumber);
}

static const mxArray *c9_sf_marshallOut(void *chartInstanceVoid, void *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  const mxArray *c9_y = NULL;
  int32_T c9_i54;
  int32_T c9_iv0[2];
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  (void)c9_inData;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_y = NULL;
  for (c9_i54 = 0; c9_i54 < 2; c9_i54++) {
    c9_iv0[c9_i54] = 0;
  }

  sf_mex_assign(&c9_y, sf_mex_createcellarray(2, c9_iv0), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static void c9_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  int32_T c9_i55;
  int32_T c9_i56;
  int32_T c9_iv1[2];
  boolean_T c9_bv0[2];
  (void)chartInstance;
  for (c9_i55 = 0; c9_i55 < 2; c9_i55++) {
    c9_iv1[c9_i55] = 0;
  }

  for (c9_i56 = 0; c9_i56 < 2; c9_i56++) {
    c9_bv0[c9_i56] = false;
  }

  sf_mex_check_cell(c9_parentId, c9_u, 2U, c9_iv1, c9_bv0);
  sf_mex_destroy(&c9_u);
}

static void c9_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_extraArgs;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  (void)c9_outData;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_extraArgs = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = (const char *)c9_identifier;
  c9_thisId.fParent = NULL;
  c9_thisId.bParentIsCell = false;
  c9_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_extraArgs), &c9_thisId);
  sf_mex_destroy(&c9_extraArgs);
  sf_mex_destroy(&c9_mxArrayInData);
}

static const mxArray *c9_b_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData;
  c9_sGeqiQRKfMYBjhkuVHSNkYD c9_u;
  const mxArray *c9_y = NULL;
  int32_T c9_i57;
  const mxArray *c9_b_y = NULL;
  char_T c9_b_u[18];
  boolean_T c9_c_u;
  const mxArray *c9_c_y = NULL;
  real_T c9_d_u;
  const mxArray *c9_d_y = NULL;
  int32_T c9_i58;
  const mxArray *c9_e_y = NULL;
  char_T c9_e_u[26];
  real_T c9_f_u;
  const mxArray *c9_f_y = NULL;
  boolean_T c9_g_u;
  const mxArray *c9_g_y = NULL;
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_mxArrayOutData = NULL;
  c9_u = *(c9_sGeqiQRKfMYBjhkuVHSNkYD *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c9_i57 = 0; c9_i57 < 18; c9_i57++) {
    c9_b_u[c9_i57] = c9_u.FcnName[c9_i57];
  }

  c9_b_y = NULL;
  sf_mex_assign(&c9_b_y, sf_mex_create("y", c9_b_u, 10, 0U, 1U, 0U, 2, 1, 18),
                false);
  sf_mex_addfield(c9_y, c9_b_y, "FcnName", "FcnName", 0);
  c9_c_u = c9_u.IsSimulinkFcn;
  c9_c_y = NULL;
  sf_mex_assign(&c9_c_y, sf_mex_create("y", &c9_c_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c9_y, c9_c_y, "IsSimulinkFcn", "IsSimulinkFcn", 0);
  c9_d_u = c9_u.NumberOfExtraArgumentInports;
  c9_d_y = NULL;
  sf_mex_assign(&c9_d_y, sf_mex_create("y", &c9_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c9_y, c9_d_y, "NumberOfExtraArgumentInports",
                  "NumberOfExtraArgumentInports", 0);
  for (c9_i58 = 0; c9_i58 < 26; c9_i58++) {
    c9_e_u[c9_i58] = c9_u.JacobianFcnName[c9_i58];
  }

  c9_e_y = NULL;
  sf_mex_assign(&c9_e_y, sf_mex_create("y", c9_e_u, 10, 0U, 1U, 0U, 2, 1, 26),
                false);
  sf_mex_addfield(c9_y, c9_e_y, "JacobianFcnName", "JacobianFcnName", 0);
  c9_f_u = c9_u.HasJacobian;
  c9_f_y = NULL;
  sf_mex_assign(&c9_f_y, sf_mex_create("y", &c9_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c9_y, c9_f_y, "HasJacobian", "HasJacobian", 0);
  c9_g_u = c9_u.HasAdditiveNoise;
  c9_g_y = NULL;
  sf_mex_assign(&c9_g_y, sf_mex_create("y", &c9_g_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c9_y, c9_g_y, "HasAdditiveNoise", "HasAdditiveNoise", 0);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static const mxArray *c9_c_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData;
  real_T c9_u;
  const mxArray *c9_y = NULL;
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_mxArrayOutData = NULL;
  c9_u = *(real_T *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static real_T c9_b_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  real_T c9_y;
  real_T c9_d0;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_d0, 1, 0, 0U, 0, 0U, 0);
  c9_y = c9_d0;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_nargin;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y;
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_nargin = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = (const char *)c9_identifier;
  c9_thisId.fParent = NULL;
  c9_thisId.bParentIsCell = false;
  c9_y = c9_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_nargin), &c9_thisId);
  sf_mex_destroy(&c9_nargin);
  *(real_T *)c9_outData = c9_y;
  sf_mex_destroy(&c9_mxArrayInData);
}

static const mxArray *c9_d_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData;
  int32_T c9_i59;
  const mxArray *c9_y = NULL;
  real_T c9_u[4];
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_mxArrayOutData = NULL;
  for (c9_i59 = 0; c9_i59 < 4; c9_i59++) {
    c9_u[c9_i59] = (*(real_T (*)[4])c9_inData)[c9_i59];
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static const mxArray *c9_e_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData;
  int32_T c9_i60;
  int32_T c9_i61;
  const mxArray *c9_y = NULL;
  int32_T c9_i62;
  real_T c9_u[16];
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_mxArrayOutData = NULL;
  c9_i60 = 0;
  for (c9_i61 = 0; c9_i61 < 4; c9_i61++) {
    for (c9_i62 = 0; c9_i62 < 4; c9_i62++) {
      c9_u[c9_i62 + c9_i60] = (*(real_T (*)[16])c9_inData)[c9_i62 + c9_i60];
    }

    c9_i60 += 4;
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 2, 4, 4), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static const mxArray *c9_f_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData;
  int32_T c9_i63;
  int32_T c9_i64;
  const mxArray *c9_y = NULL;
  int32_T c9_i65;
  real_T c9_u[4];
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_mxArrayOutData = NULL;
  c9_i63 = 0;
  for (c9_i64 = 0; c9_i64 < 2; c9_i64++) {
    for (c9_i65 = 0; c9_i65 < 2; c9_i65++) {
      c9_u[c9_i65 + c9_i63] = (*(real_T (*)[4])c9_inData)[c9_i65 + c9_i63];
    }

    c9_i63 += 2;
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 2, 2, 2), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static const mxArray *c9_g_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData;
  boolean_T c9_u;
  const mxArray *c9_y = NULL;
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_mxArrayOutData = NULL;
  c9_u = *(boolean_T *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static void c9_c_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_b_xNew, const char_T *c9_identifier, real_T
  c9_y[4])
{
  emlrtMsgIdentifier c9_thisId;
  c9_thisId.fIdentifier = (const char *)c9_identifier;
  c9_thisId.fParent = NULL;
  c9_thisId.bParentIsCell = false;
  c9_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_b_xNew), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_b_xNew);
}

static void c9_d_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[4])
{
  real_T c9_dv4[4];
  int32_T c9_i66;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv4, 1, 0, 0U, 1, 0U, 1, 4);
  for (c9_i66 = 0; c9_i66 < 4; c9_i66++) {
    c9_y[c9_i66] = c9_dv4[c9_i66];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_b_xNew;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y[4];
  int32_T c9_i67;
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_b_xNew = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = (const char *)c9_identifier;
  c9_thisId.fParent = NULL;
  c9_thisId.bParentIsCell = false;
  c9_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_b_xNew), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_b_xNew);
  for (c9_i67 = 0; c9_i67 < 4; c9_i67++) {
    (*(real_T (*)[4])c9_outData)[c9_i67] = c9_y[c9_i67];
  }

  sf_mex_destroy(&c9_mxArrayInData);
}

static void c9_e_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_c_P, const char_T *c9_identifier, real_T
  c9_y[16])
{
  emlrtMsgIdentifier c9_thisId;
  c9_thisId.fIdentifier = (const char *)c9_identifier;
  c9_thisId.fParent = NULL;
  c9_thisId.bParentIsCell = false;
  c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_c_P), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_c_P);
}

static void c9_f_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[16])
{
  real_T c9_dv5[16];
  int32_T c9_i68;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv5, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c9_i68 = 0; c9_i68 < 16; c9_i68++) {
    c9_y[c9_i68] = c9_dv5[c9_i68];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_c_P;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y[16];
  int32_T c9_i69;
  int32_T c9_i70;
  int32_T c9_i71;
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_c_P = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = (const char *)c9_identifier;
  c9_thisId.fParent = NULL;
  c9_thisId.bParentIsCell = false;
  c9_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_c_P), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_c_P);
  c9_i69 = 0;
  for (c9_i70 = 0; c9_i70 < 4; c9_i70++) {
    for (c9_i71 = 0; c9_i71 < 4; c9_i71++) {
      (*(real_T (*)[16])c9_outData)[c9_i71 + c9_i69] = c9_y[c9_i71 + c9_i69];
    }

    c9_i69 += 4;
  }

  sf_mex_destroy(&c9_mxArrayInData);
}

const mxArray *sf_c9_sharedTrackingLibrary_get_eml_resolved_functions_info(void)
{
  const mxArray *c9_nameCaptureInfo = NULL;
  c9_nameCaptureInfo = NULL;
  sf_mex_assign(&c9_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c9_nameCaptureInfo;
}

static const mxArray *c9_h_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData;
  int32_T c9_u;
  const mxArray *c9_y = NULL;
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_mxArrayOutData = NULL;
  c9_u = *(int32_T *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static int32_T c9_g_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  int32_T c9_y;
  int32_T c9_i72;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_i72, 1, 6, 0U, 0, 0U, 0);
  c9_y = c9_i72;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_b_sfEvent;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  int32_T c9_y;
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_b_sfEvent = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = (const char *)c9_identifier;
  c9_thisId.fParent = NULL;
  c9_thisId.bParentIsCell = false;
  c9_y = c9_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_b_sfEvent),
    &c9_thisId);
  sf_mex_destroy(&c9_b_sfEvent);
  *(int32_T *)c9_outData = c9_y;
  sf_mex_destroy(&c9_mxArrayInData);
}

static void c9_h_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  c9_sGeqiQRKfMYBjhkuVHSNkYD *c9_y)
{
  emlrtMsgIdentifier c9_thisId;
  static const char * c9_fieldNames[6] = { "FcnName", "IsSimulinkFcn",
    "NumberOfExtraArgumentInports", "JacobianFcnName", "HasJacobian",
    "HasAdditiveNoise" };

  c9_thisId.fParent = c9_parentId;
  c9_thisId.bParentIsCell = false;
  sf_mex_check_struct(c9_parentId, c9_u, 6, c9_fieldNames, 0U, NULL);
  c9_thisId.fIdentifier = "FcnName";
  c9_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c9_u,
    "FcnName", "FcnName", 0)), &c9_thisId, c9_y->FcnName);
  c9_thisId.fIdentifier = "IsSimulinkFcn";
  c9_y->IsSimulinkFcn = c9_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "IsSimulinkFcn", "IsSimulinkFcn", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "NumberOfExtraArgumentInports";
  c9_y->NumberOfExtraArgumentInports = c9_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getfield(c9_u, "NumberOfExtraArgumentInports",
    "NumberOfExtraArgumentInports", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "JacobianFcnName";
  c9_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c9_u,
    "JacobianFcnName", "JacobianFcnName", 0)), &c9_thisId, c9_y->JacobianFcnName);
  c9_thisId.fIdentifier = "HasJacobian";
  c9_y->HasJacobian = c9_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "HasJacobian", "HasJacobian", 0)), &c9_thisId);
  c9_thisId.fIdentifier = "HasAdditiveNoise";
  c9_y->HasAdditiveNoise = c9_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c9_u, "HasAdditiveNoise", "HasAdditiveNoise", 0)),
    &c9_thisId);
  sf_mex_destroy(&c9_u);
}

static void c9_i_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  char_T c9_y[18])
{
  char_T c9_cv0[18];
  int32_T c9_i73;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_cv0, 1, 10, 0U, 1, 0U, 2, 1,
                18);
  for (c9_i73 = 0; c9_i73 < 18; c9_i73++) {
    c9_y[c9_i73] = c9_cv0[c9_i73];
  }

  sf_mex_destroy(&c9_u);
}

static boolean_T c9_j_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct *
  chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  boolean_T c9_y;
  boolean_T c9_b0;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_b0, 1, 11, 0U, 0, 0U, 0);
  c9_y = c9_b0;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_k_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  char_T c9_y[26])
{
  char_T c9_cv1[26];
  int32_T c9_i74;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_cv1, 1, 10, 0U, 1, 0U, 2, 1,
                26);
  for (c9_i74 = 0; c9_i74 < 26; c9_i74++) {
    c9_y[c9_i74] = c9_cv1[c9_i74];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_b_pS;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  c9_sGeqiQRKfMYBjhkuVHSNkYD c9_y;
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)chartInstanceVoid;
  c9_b_pS = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = (const char *)c9_identifier;
  c9_thisId.fParent = NULL;
  c9_thisId.bParentIsCell = false;
  c9_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_b_pS), &c9_thisId, &c9_y);
  sf_mex_destroy(&c9_b_pS);
  *(c9_sGeqiQRKfMYBjhkuVHSNkYD *)c9_outData = c9_y;
  sf_mex_destroy(&c9_mxArrayInData);
}

static uint8_T c9_l_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_b_is_active_c9_sharedTrackingLibrary, const
  char_T *c9_identifier)
{
  uint8_T c9_y;
  emlrtMsgIdentifier c9_thisId;
  c9_thisId.fIdentifier = (const char *)c9_identifier;
  c9_thisId.fParent = NULL;
  c9_thisId.bParentIsCell = false;
  c9_y = c9_m_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c9_b_is_active_c9_sharedTrackingLibrary), &c9_thisId);
  sf_mex_destroy(&c9_b_is_active_c9_sharedTrackingLibrary);
  return c9_y;
}

static uint8_T c9_m_emlrt_marshallIn(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  uint8_T c9_y;
  uint8_T c9_u0;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_u0, 1, 3, 0U, 0, 0U, 0);
  c9_y = c9_u0;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void init_dsm_address_info(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc9_sharedTrackingLibraryInstanceStruct
  *chartInstance)
{
  chartInstance->c9_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c9_x = (real_T (*)[4])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c9_xNew = (real_T (*)[4])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c9_P = (real_T (*)[16])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c9_Q = (real_T (*)[4])ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c9_uState = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c9_unused = (boolean_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c9_b_P = (real_T (*)[16])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
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

void sf_c9_sharedTrackingLibrary_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4082043433U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2913487301U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2097203908U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2401299196U);
}

mxArray* sf_c9_sharedTrackingLibrary_get_post_codegen_info(void);
mxArray *sf_c9_sharedTrackingLibrary_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("uWejKQAjC0E65GdpLd4UjE");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

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
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(2);
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
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
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

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo =
      sf_c9_sharedTrackingLibrary_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c9_sharedTrackingLibrary_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c9_sharedTrackingLibrary_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("late");
  mxArray *fallbackReason = mxCreateString("client_server");
  mxArray *hiddenFallbackType = mxCreateString("");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("stateTransitionFcn");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c9_sharedTrackingLibrary_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c9_sharedTrackingLibrary_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString(
      "BBhvhWLLR0k1JKhhGKyAbB");
    mwSize exp_dims[2] = { 2, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);

    {
      mxArray* mxFcnName = mxCreateString("stateTransitionFcn");
      mxSetCell(mxExportedFunctionsUsedByThisChart, 0, mxFcnName);
    }

    {
      mxArray* mxFcnName = mxCreateString("stateTransitionJacobianFcn");
      mxSetCell(mxExportedFunctionsUsedByThisChart, 1, mxFcnName);
    }

    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c9_sharedTrackingLibrary(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[8],T\"P\",},{M[1],M[13],T\"xNew\",},{M[8],M[0],T\"is_active_c9_sharedTrackingLibrary\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c9_sharedTrackingLibrary_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc9_sharedTrackingLibraryInstanceStruct *chartInstance =
      (SFc9_sharedTrackingLibraryInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _sharedTrackingLibraryMachineNumber_,
           9,
           1,
           1,
           0,
           8,
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
          _SFD_SET_DATA_PROPS(2,1,1,0,"Q");
          _SFD_SET_DATA_PROPS(3,1,1,0,"uState");
          _SFD_SET_DATA_PROPS(4,1,1,0,"unused");
          _SFD_SET_DATA_PROPS(5,2,0,1,"xNew");
          _SFD_SET_DATA_PROPS(6,2,0,1,"P");
          _SFD_SET_DATA_PROPS(7,10,0,0,"pS");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1584);
        _SFD_CV_INIT_EML_FCN(0,1,"",298,-1,345);
        _SFD_CV_INIT_EML_FCN(0,2,"",386,-1,455);
        _SFD_CV_INIT_EML_FCN(0,3,"",708,-1,755);
        _SFD_CV_INIT_EML_FCN(0,4,"",812,-1,881);
        _SFD_CV_INIT_EML_IF(0,1,0,162,181,465,521);
        _SFD_CV_INIT_EML_IF(0,1,1,245,267,351,464);
        _SFD_CV_INIT_EML_IF(0,1,2,522,539,980,1026);
        _SFD_CV_INIT_EML_IF(0,1,3,544,563,899,979);
        _SFD_CV_INIT_EML_IF(0,1,4,643,665,765,894);
        _SFD_CV_INIT_EML_IF(0,1,5,1230,1252,1409,1580);

        {
          static int caseStart[] = { 1147, 1071, 1106 };

          static int caseExprEnd[] = { 1156, 1077, 1112 };

          _SFD_CV_INIT_EML_SWITCH(0,1,0,1028,1067,1183,3,&(caseStart[0]),
            &(caseExprEnd[0]));
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4U;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4U;
          dimVector[1]= 4U;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 2U;
          dimVector[1]= 2U;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_f_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_g_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 4U;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_d_sf_marshallOut,(MexInFcnForType)
            c9_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4U;
          dimVector[1]= 4U;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_e_sf_marshallOut,(MexInFcnForType)
            c9_d_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(7,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_b_sf_marshallOut,(MexInFcnForType)c9_f_sf_marshallIn);
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
    SFc9_sharedTrackingLibraryInstanceStruct *chartInstance =
      (SFc9_sharedTrackingLibraryInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, (void *)chartInstance->c9_x);
        _SFD_SET_DATA_VALUE_PTR(5U, (void *)chartInstance->c9_xNew);
        _SFD_SET_DATA_VALUE_PTR(1U, (void *)chartInstance->c9_P);
        _SFD_SET_DATA_VALUE_PTR(2U, (void *)chartInstance->c9_Q);
        _SFD_SET_DATA_VALUE_PTR(3U, (void *)chartInstance->c9_uState);
        _SFD_SET_DATA_VALUE_PTR(4U, (void *)chartInstance->c9_unused);
        _SFD_SET_DATA_VALUE_PTR(6U, (void *)chartInstance->c9_b_P);
        _SFD_SET_DATA_VALUE_PTR(7U, (void *)&chartInstance->c9_pS);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "soBm4wuxY6nhdNo4dLNwMxC";
}

static void sf_opaque_initialize_c9_sharedTrackingLibrary(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc9_sharedTrackingLibraryInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c9_sharedTrackingLibrary
    ((SFc9_sharedTrackingLibraryInstanceStruct*) chartInstanceVar);
  initialize_c9_sharedTrackingLibrary((SFc9_sharedTrackingLibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c9_sharedTrackingLibrary(void *chartInstanceVar)
{
  enable_c9_sharedTrackingLibrary((SFc9_sharedTrackingLibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c9_sharedTrackingLibrary(void *chartInstanceVar)
{
  disable_c9_sharedTrackingLibrary((SFc9_sharedTrackingLibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c9_sharedTrackingLibrary(void *chartInstanceVar)
{
  sf_gateway_c9_sharedTrackingLibrary((SFc9_sharedTrackingLibraryInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c9_sharedTrackingLibrary(SimStruct*
  S)
{
  return get_sim_state_c9_sharedTrackingLibrary
    ((SFc9_sharedTrackingLibraryInstanceStruct *)sf_get_chart_instance_ptr(S));/* raw sim ctx */
}

static void sf_opaque_set_sim_state_c9_sharedTrackingLibrary(SimStruct* S, const
  mxArray *st)
{
  set_sim_state_c9_sharedTrackingLibrary
    ((SFc9_sharedTrackingLibraryInstanceStruct*)sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c9_sharedTrackingLibrary(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc9_sharedTrackingLibraryInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_sharedTrackingLibrary_optimization_info();
    }

    finalize_c9_sharedTrackingLibrary((SFc9_sharedTrackingLibraryInstanceStruct*)
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
  initSimStructsc9_sharedTrackingLibrary
    ((SFc9_sharedTrackingLibraryInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c9_sharedTrackingLibrary(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c9_sharedTrackingLibrary
      ((SFc9_sharedTrackingLibraryInstanceStruct*)sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c9_sharedTrackingLibrary(SimStruct *S)
{
  /* Actual parameters from chart:
     pS
   */
  const char_T *rtParamNames[] = { "pS" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));

  /* Set overwritable ports for inplace optimization */
  ssSetInputPortOverWritable(S, 1, 1);
  ssSetOutputPortOverwritesInputPort(S, 2, 1);
  ssSetInputPortDirectFeedThrough(S, 0, 1);
  ssSetInputPortDirectFeedThrough(S, 1, 1);
  ssSetInputPortDirectFeedThrough(S, 2, 1);
  ssSetInputPortDirectFeedThrough(S, 3, 1);
  ssSetInputPortDirectFeedThrough(S, 4, 1);
  ssSetStatesModifiedOnlyInUpdate(S, 0);
  ssSetBlockIsPurelyCombinatorial_wrapper(S, 0);
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_sharedTrackingLibrary_optimization_info
      (sim_mode_is_rtw_gen(S), sim_mode_is_modelref_sim(S), sim_mode_is_external
       (S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,9);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetSupportedForRowMajorCodeGen(S, 1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,9,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 9);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,9);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,9,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,9,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,9);
    sf_register_codegen_names_for_scoped_functions_defined_by_chart(S);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3954953432U));
  ssSetChecksum1(S,(4270942044U));
  ssSetChecksum2(S,(3174876185U));
  ssSetChecksum3(S,(4167380323U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,0);
}

static void mdlRTW_c9_sharedTrackingLibrary(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c9_sharedTrackingLibrary(SimStruct *S)
{
  SFc9_sharedTrackingLibraryInstanceStruct *chartInstance;
  chartInstance = (SFc9_sharedTrackingLibraryInstanceStruct *)utMalloc(sizeof
    (SFc9_sharedTrackingLibraryInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc9_sharedTrackingLibraryInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  if (ssGetSampleTime(S, 0) == CONTINUOUS_SAMPLE_TIME && ssGetOffsetTime(S, 0) ==
      0 && ssGetNumContStates(ssGetRootSS(S)) > 0) {
    sf_error_out_about_continuous_sample_time_with_persistent_vars(S);
  }

  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c9_sharedTrackingLibrary;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c9_sharedTrackingLibrary;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c9_sharedTrackingLibrary;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c9_sharedTrackingLibrary;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c9_sharedTrackingLibrary;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c9_sharedTrackingLibrary;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c9_sharedTrackingLibrary;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c9_sharedTrackingLibrary;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c9_sharedTrackingLibrary;
  chartInstance->chartInfo.mdlStart = mdlStart_c9_sharedTrackingLibrary;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c9_sharedTrackingLibrary;
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
  mdl_start_c9_sharedTrackingLibrary(chartInstance);
}

void c9_sharedTrackingLibrary_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c9_sharedTrackingLibrary(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c9_sharedTrackingLibrary(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c9_sharedTrackingLibrary(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c9_sharedTrackingLibrary_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
