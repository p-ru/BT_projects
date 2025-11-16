/*
 * trajGen.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "trajGen".
 *
 * Model version              : 1.8
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C++ source code generated on : Sun Nov 16 22:33:09 2025
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef trajGen_h_
#define trajGen_h_
#include <cmath>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "slros_initialize.h"
#include "coder_posix_time.h"
#include "trajGen_types.h"
#include "coder_array.h"

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include <cstring>
#include "zero_crossing_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
struct B_trajGen_T {
  coder::array<real_T,2> r;
  coder::array<real_T,2> R_data;
  coder::array<real_T,2> R_data_m;
  coder::array<real_T,2> b_A;
  coder::array<real_T,1> r1;
  coder::array<real_T,1> r2;
  coder::array<real_T,1> constraints_data;
  coder::array<real_T,1> tau;
  coder::array<real_T,1> B;
  coder::array<real_T,1> work;
  coder::array<real_T,1> vn1;
  coder::array<real_T,1> vn2;
  coder::array<int32_T,2> jpvt;
  SL_Bus_trajGen_nav_msgs_Odometry In1;/* '<S168>/In1' */
  SL_Bus_trajGen_nav_msgs_Odometry rtb_SourceBlock_o2_cl;
  real_T M[64];
  real_T AInv[64];
  real_T Q[64];
  real_T Qsum[64];
  real_T R_tmp[64];
  real_T AInv_k[64];
  real_T AInv_c[64];
  real_T c_A[64];
  real_T c_A_b[64];
  real_T A0[32];
  real_T AT[32];
  real_T ppMatrix[24];
  real_T constraints[24];
  real_T upperleft[16];
  real_T lowerright[16];
  real_T Q_p[16];
  real_T lowerright_c[16];
  real_T lowerright_f[16];
  real_T c_A_g[16];
  real_T waypoints[12];
  int8_T M1[64];
  int8_T Mcontinuity[64];
  real_T b_p[8];
  real_T x[8];
  SL_Bus_trajGen_geometry_msgs_Twist BusAssignment;/* '<S3>/Bus Assignment' */
  real_T rtb_Transpose_tmp[6];
  real_T dv[6];
  real_T waypoints_g[6];
  real_T dv1[6];
  int32_T ipiv[8];
  int32_T b_ipiv[8];
  int32_T fixedBCIdx_data[8];
  int32_T freeBCIdx_data[8];
  int32_T tmp_data[8];
  SL_Bus_trajGen_geometry_msgs_Point In1_c;/* '<S5>/In1' */
  SL_Bus_trajGen_geometry_msgs_Point rtb_SourceBlock_o2_c_m;
  real_T Transpose1[3];                /* '<S3>/Transpose1' */
  char_T b_zeroDelimTopic[23];
  real_T dv2[2];
  real_T timePoints[2];                /* '<S3>/MATLAB Function' */
  int8_T tmp_data_n[8];
  boolean_T x_tmp[8];
  real_T delT;
  real_T FilterCoefficient;            /* '<S49>/Filter Coefficient' */
  real_T Sum1;                         /* '<S3>/Sum1' */
  real_T FilterCoefficient_m;          /* '<S101>/Filter Coefficient' */
  real_T Sum2;                         /* '<S3>/Sum2' */
  real_T FilterCoefficient_g;          /* '<S153>/Filter Coefficient' */
  real_T tmp1;
  real_T prod;
  real_T lowerright_l;
  real_T lowerright_j;
  real_T lowerright_d;
  real_T normA;
  real_T absrexk;
  real_T temp;
  real_T s;
  real_T smax;
  real_T s_g;
  real_T wj;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T smax_l;
  real_T scale_d;
  real_T absxk_d;
  real_T t_l;
  real_T smax_o;
  real_T a;
  real_T smax_b;
  int32_T R_size[2];
  int32_T R_size_p[2];
  int32_T b_k;
  int32_T loop_ub;
  int32_T b;
  int32_T c;
  int32_T e;
  int32_T nz;
  int32_T row;
  int32_T col;
  int32_T powerTerm;
  int32_T loop_ub_n;
  int32_T waypoints_tmp;
  int32_T tmp_size_idx_0;
  int32_T iter;
  int32_T kase;
  int32_T jump;
  int32_T j;
  int32_T rankA;
  int32_T mn;
  int32_T m;
  int32_T i;
  int32_T kAcol;
  int32_T k;
  int32_T ii;
  int32_T nmi;
};

/* Block states (default storage) for system '<Root>' */
struct DW_trajGen_T {
  shared_uav_rst_sluav_internal_T obj;
                                 /* '<S3>/Minimum Jerk Polynomial Trajectory' */
  ros_slros_internal_block_GetP_T obj_d;/* '<Root>/Get Parameter' */
  ros_slroscpp_internal_block_P_T obj_k;/* '<S1>/SinkBlock' */
  ros_slroscpp_internal_block_S_T obj_o;/* '<S4>/SourceBlock' */
  ros_slroscpp_internal_block_S_T obj_e;/* '<S2>/SourceBlock' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S3>/Discrete-Time Integrator' */
  real_T Integrator_DSTATE;            /* '<S46>/Integrator' */
  real_T Filter_DSTATE;                /* '<S41>/Filter' */
  real_T Integrator_DSTATE_p;          /* '<S98>/Integrator' */
  real_T Filter_DSTATE_m;              /* '<S93>/Filter' */
  real_T Integrator_DSTATE_g;          /* '<S150>/Integrator' */
  real_T Filter_DSTATE_d;              /* '<S145>/Filter' */
  real_T freq;                   /* '<S3>/Minimum Jerk Polynomial Trajectory' */
  real_T startPosition[3];             /* '<S3>/MATLAB Function1' */
  boolean_T objisempty;                /* '<S4>/SourceBlock' */
  boolean_T objisempty_n;        /* '<S3>/Minimum Jerk Polynomial Trajectory' */
  boolean_T freq_not_empty;      /* '<S3>/Minimum Jerk Polynomial Trajectory' */
  boolean_T startPosition_not_empty;   /* '<S3>/MATLAB Function1' */
  boolean_T objisempty_j;              /* '<S2>/SourceBlock' */
  boolean_T objisempty_l;              /* '<S1>/SinkBlock' */
  boolean_T objisempty_l0;             /* '<Root>/Get Parameter' */
};

/* Zero-crossing (trigger) state */
struct PrevZCX_trajGen_T {
  ZCSigState Subsystem_Reset_ZCE;      /* '<Root>/Subsystem' */
};

/* Real-time Model Data Structure */
struct tag_RTM_trajGen_T {
  const char_T *errorStatus;
};

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_trajGen_T trajGen_B;

#ifdef __cplusplus

}

#endif

/* Block states (default storage) */
extern struct DW_trajGen_T trajGen_DW;

/* Zero-crossing (trigger) state */
extern PrevZCX_trajGen_T trajGen_PrevZCX;

#ifdef __cplusplus

extern "C"
{

#endif

  /* Model entry point functions */
  extern void trajGen_initialize(void);
  extern void trajGen_step(void);
  extern void trajGen_terminate(void);

#ifdef __cplusplus

}

#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_trajGen_T *const trajGen_M;

#ifdef __cplusplus

}

#endif

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'trajGen'
 * '<S1>'   : 'trajGen/Publish'
 * '<S2>'   : 'trajGen/Subscribe'
 * '<S3>'   : 'trajGen/Subsystem'
 * '<S4>'   : 'trajGen/position reading'
 * '<S5>'   : 'trajGen/Subscribe/Enabled Subsystem'
 * '<S6>'   : 'trajGen/Subsystem/MATLAB Function'
 * '<S7>'   : 'trajGen/Subsystem/MATLAB Function1'
 * '<S8>'   : 'trajGen/Subsystem/X'
 * '<S9>'   : 'trajGen/Subsystem/Y'
 * '<S10>'  : 'trajGen/Subsystem/Z'
 * '<S11>'  : 'trajGen/Subsystem/control output'
 * '<S12>'  : 'trajGen/Subsystem/X/Anti-windup'
 * '<S13>'  : 'trajGen/Subsystem/X/D Gain'
 * '<S14>'  : 'trajGen/Subsystem/X/External Derivative'
 * '<S15>'  : 'trajGen/Subsystem/X/Filter'
 * '<S16>'  : 'trajGen/Subsystem/X/Filter ICs'
 * '<S17>'  : 'trajGen/Subsystem/X/I Gain'
 * '<S18>'  : 'trajGen/Subsystem/X/Ideal P Gain'
 * '<S19>'  : 'trajGen/Subsystem/X/Ideal P Gain Fdbk'
 * '<S20>'  : 'trajGen/Subsystem/X/Integrator'
 * '<S21>'  : 'trajGen/Subsystem/X/Integrator ICs'
 * '<S22>'  : 'trajGen/Subsystem/X/N Copy'
 * '<S23>'  : 'trajGen/Subsystem/X/N Gain'
 * '<S24>'  : 'trajGen/Subsystem/X/P Copy'
 * '<S25>'  : 'trajGen/Subsystem/X/Parallel P Gain'
 * '<S26>'  : 'trajGen/Subsystem/X/Reset Signal'
 * '<S27>'  : 'trajGen/Subsystem/X/Saturation'
 * '<S28>'  : 'trajGen/Subsystem/X/Saturation Fdbk'
 * '<S29>'  : 'trajGen/Subsystem/X/Sum'
 * '<S30>'  : 'trajGen/Subsystem/X/Sum Fdbk'
 * '<S31>'  : 'trajGen/Subsystem/X/Tracking Mode'
 * '<S32>'  : 'trajGen/Subsystem/X/Tracking Mode Sum'
 * '<S33>'  : 'trajGen/Subsystem/X/Tsamp - Integral'
 * '<S34>'  : 'trajGen/Subsystem/X/Tsamp - Ngain'
 * '<S35>'  : 'trajGen/Subsystem/X/postSat Signal'
 * '<S36>'  : 'trajGen/Subsystem/X/preInt Signal'
 * '<S37>'  : 'trajGen/Subsystem/X/preSat Signal'
 * '<S38>'  : 'trajGen/Subsystem/X/Anti-windup/Passthrough'
 * '<S39>'  : 'trajGen/Subsystem/X/D Gain/Internal Parameters'
 * '<S40>'  : 'trajGen/Subsystem/X/External Derivative/Error'
 * '<S41>'  : 'trajGen/Subsystem/X/Filter/Disc. Forward Euler Filter'
 * '<S42>'  : 'trajGen/Subsystem/X/Filter ICs/Internal IC - Filter'
 * '<S43>'  : 'trajGen/Subsystem/X/I Gain/Internal Parameters'
 * '<S44>'  : 'trajGen/Subsystem/X/Ideal P Gain/Passthrough'
 * '<S45>'  : 'trajGen/Subsystem/X/Ideal P Gain Fdbk/Disabled'
 * '<S46>'  : 'trajGen/Subsystem/X/Integrator/Discrete'
 * '<S47>'  : 'trajGen/Subsystem/X/Integrator ICs/Internal IC'
 * '<S48>'  : 'trajGen/Subsystem/X/N Copy/Disabled'
 * '<S49>'  : 'trajGen/Subsystem/X/N Gain/Internal Parameters'
 * '<S50>'  : 'trajGen/Subsystem/X/P Copy/Disabled'
 * '<S51>'  : 'trajGen/Subsystem/X/Parallel P Gain/Internal Parameters'
 * '<S52>'  : 'trajGen/Subsystem/X/Reset Signal/Disabled'
 * '<S53>'  : 'trajGen/Subsystem/X/Saturation/Passthrough'
 * '<S54>'  : 'trajGen/Subsystem/X/Saturation Fdbk/Disabled'
 * '<S55>'  : 'trajGen/Subsystem/X/Sum/Sum_PID'
 * '<S56>'  : 'trajGen/Subsystem/X/Sum Fdbk/Disabled'
 * '<S57>'  : 'trajGen/Subsystem/X/Tracking Mode/Disabled'
 * '<S58>'  : 'trajGen/Subsystem/X/Tracking Mode Sum/Passthrough'
 * '<S59>'  : 'trajGen/Subsystem/X/Tsamp - Integral/TsSignalSpecification'
 * '<S60>'  : 'trajGen/Subsystem/X/Tsamp - Ngain/Passthrough'
 * '<S61>'  : 'trajGen/Subsystem/X/postSat Signal/Forward_Path'
 * '<S62>'  : 'trajGen/Subsystem/X/preInt Signal/Internal PreInt'
 * '<S63>'  : 'trajGen/Subsystem/X/preSat Signal/Forward_Path'
 * '<S64>'  : 'trajGen/Subsystem/Y/Anti-windup'
 * '<S65>'  : 'trajGen/Subsystem/Y/D Gain'
 * '<S66>'  : 'trajGen/Subsystem/Y/External Derivative'
 * '<S67>'  : 'trajGen/Subsystem/Y/Filter'
 * '<S68>'  : 'trajGen/Subsystem/Y/Filter ICs'
 * '<S69>'  : 'trajGen/Subsystem/Y/I Gain'
 * '<S70>'  : 'trajGen/Subsystem/Y/Ideal P Gain'
 * '<S71>'  : 'trajGen/Subsystem/Y/Ideal P Gain Fdbk'
 * '<S72>'  : 'trajGen/Subsystem/Y/Integrator'
 * '<S73>'  : 'trajGen/Subsystem/Y/Integrator ICs'
 * '<S74>'  : 'trajGen/Subsystem/Y/N Copy'
 * '<S75>'  : 'trajGen/Subsystem/Y/N Gain'
 * '<S76>'  : 'trajGen/Subsystem/Y/P Copy'
 * '<S77>'  : 'trajGen/Subsystem/Y/Parallel P Gain'
 * '<S78>'  : 'trajGen/Subsystem/Y/Reset Signal'
 * '<S79>'  : 'trajGen/Subsystem/Y/Saturation'
 * '<S80>'  : 'trajGen/Subsystem/Y/Saturation Fdbk'
 * '<S81>'  : 'trajGen/Subsystem/Y/Sum'
 * '<S82>'  : 'trajGen/Subsystem/Y/Sum Fdbk'
 * '<S83>'  : 'trajGen/Subsystem/Y/Tracking Mode'
 * '<S84>'  : 'trajGen/Subsystem/Y/Tracking Mode Sum'
 * '<S85>'  : 'trajGen/Subsystem/Y/Tsamp - Integral'
 * '<S86>'  : 'trajGen/Subsystem/Y/Tsamp - Ngain'
 * '<S87>'  : 'trajGen/Subsystem/Y/postSat Signal'
 * '<S88>'  : 'trajGen/Subsystem/Y/preInt Signal'
 * '<S89>'  : 'trajGen/Subsystem/Y/preSat Signal'
 * '<S90>'  : 'trajGen/Subsystem/Y/Anti-windup/Passthrough'
 * '<S91>'  : 'trajGen/Subsystem/Y/D Gain/Internal Parameters'
 * '<S92>'  : 'trajGen/Subsystem/Y/External Derivative/Error'
 * '<S93>'  : 'trajGen/Subsystem/Y/Filter/Disc. Forward Euler Filter'
 * '<S94>'  : 'trajGen/Subsystem/Y/Filter ICs/Internal IC - Filter'
 * '<S95>'  : 'trajGen/Subsystem/Y/I Gain/Internal Parameters'
 * '<S96>'  : 'trajGen/Subsystem/Y/Ideal P Gain/Passthrough'
 * '<S97>'  : 'trajGen/Subsystem/Y/Ideal P Gain Fdbk/Disabled'
 * '<S98>'  : 'trajGen/Subsystem/Y/Integrator/Discrete'
 * '<S99>'  : 'trajGen/Subsystem/Y/Integrator ICs/Internal IC'
 * '<S100>' : 'trajGen/Subsystem/Y/N Copy/Disabled'
 * '<S101>' : 'trajGen/Subsystem/Y/N Gain/Internal Parameters'
 * '<S102>' : 'trajGen/Subsystem/Y/P Copy/Disabled'
 * '<S103>' : 'trajGen/Subsystem/Y/Parallel P Gain/Internal Parameters'
 * '<S104>' : 'trajGen/Subsystem/Y/Reset Signal/Disabled'
 * '<S105>' : 'trajGen/Subsystem/Y/Saturation/Passthrough'
 * '<S106>' : 'trajGen/Subsystem/Y/Saturation Fdbk/Disabled'
 * '<S107>' : 'trajGen/Subsystem/Y/Sum/Sum_PID'
 * '<S108>' : 'trajGen/Subsystem/Y/Sum Fdbk/Disabled'
 * '<S109>' : 'trajGen/Subsystem/Y/Tracking Mode/Disabled'
 * '<S110>' : 'trajGen/Subsystem/Y/Tracking Mode Sum/Passthrough'
 * '<S111>' : 'trajGen/Subsystem/Y/Tsamp - Integral/TsSignalSpecification'
 * '<S112>' : 'trajGen/Subsystem/Y/Tsamp - Ngain/Passthrough'
 * '<S113>' : 'trajGen/Subsystem/Y/postSat Signal/Forward_Path'
 * '<S114>' : 'trajGen/Subsystem/Y/preInt Signal/Internal PreInt'
 * '<S115>' : 'trajGen/Subsystem/Y/preSat Signal/Forward_Path'
 * '<S116>' : 'trajGen/Subsystem/Z/Anti-windup'
 * '<S117>' : 'trajGen/Subsystem/Z/D Gain'
 * '<S118>' : 'trajGen/Subsystem/Z/External Derivative'
 * '<S119>' : 'trajGen/Subsystem/Z/Filter'
 * '<S120>' : 'trajGen/Subsystem/Z/Filter ICs'
 * '<S121>' : 'trajGen/Subsystem/Z/I Gain'
 * '<S122>' : 'trajGen/Subsystem/Z/Ideal P Gain'
 * '<S123>' : 'trajGen/Subsystem/Z/Ideal P Gain Fdbk'
 * '<S124>' : 'trajGen/Subsystem/Z/Integrator'
 * '<S125>' : 'trajGen/Subsystem/Z/Integrator ICs'
 * '<S126>' : 'trajGen/Subsystem/Z/N Copy'
 * '<S127>' : 'trajGen/Subsystem/Z/N Gain'
 * '<S128>' : 'trajGen/Subsystem/Z/P Copy'
 * '<S129>' : 'trajGen/Subsystem/Z/Parallel P Gain'
 * '<S130>' : 'trajGen/Subsystem/Z/Reset Signal'
 * '<S131>' : 'trajGen/Subsystem/Z/Saturation'
 * '<S132>' : 'trajGen/Subsystem/Z/Saturation Fdbk'
 * '<S133>' : 'trajGen/Subsystem/Z/Sum'
 * '<S134>' : 'trajGen/Subsystem/Z/Sum Fdbk'
 * '<S135>' : 'trajGen/Subsystem/Z/Tracking Mode'
 * '<S136>' : 'trajGen/Subsystem/Z/Tracking Mode Sum'
 * '<S137>' : 'trajGen/Subsystem/Z/Tsamp - Integral'
 * '<S138>' : 'trajGen/Subsystem/Z/Tsamp - Ngain'
 * '<S139>' : 'trajGen/Subsystem/Z/postSat Signal'
 * '<S140>' : 'trajGen/Subsystem/Z/preInt Signal'
 * '<S141>' : 'trajGen/Subsystem/Z/preSat Signal'
 * '<S142>' : 'trajGen/Subsystem/Z/Anti-windup/Passthrough'
 * '<S143>' : 'trajGen/Subsystem/Z/D Gain/Internal Parameters'
 * '<S144>' : 'trajGen/Subsystem/Z/External Derivative/Error'
 * '<S145>' : 'trajGen/Subsystem/Z/Filter/Disc. Forward Euler Filter'
 * '<S146>' : 'trajGen/Subsystem/Z/Filter ICs/Internal IC - Filter'
 * '<S147>' : 'trajGen/Subsystem/Z/I Gain/Internal Parameters'
 * '<S148>' : 'trajGen/Subsystem/Z/Ideal P Gain/Passthrough'
 * '<S149>' : 'trajGen/Subsystem/Z/Ideal P Gain Fdbk/Disabled'
 * '<S150>' : 'trajGen/Subsystem/Z/Integrator/Discrete'
 * '<S151>' : 'trajGen/Subsystem/Z/Integrator ICs/Internal IC'
 * '<S152>' : 'trajGen/Subsystem/Z/N Copy/Disabled'
 * '<S153>' : 'trajGen/Subsystem/Z/N Gain/Internal Parameters'
 * '<S154>' : 'trajGen/Subsystem/Z/P Copy/Disabled'
 * '<S155>' : 'trajGen/Subsystem/Z/Parallel P Gain/Internal Parameters'
 * '<S156>' : 'trajGen/Subsystem/Z/Reset Signal/Disabled'
 * '<S157>' : 'trajGen/Subsystem/Z/Saturation/Passthrough'
 * '<S158>' : 'trajGen/Subsystem/Z/Saturation Fdbk/Disabled'
 * '<S159>' : 'trajGen/Subsystem/Z/Sum/Sum_PID'
 * '<S160>' : 'trajGen/Subsystem/Z/Sum Fdbk/Disabled'
 * '<S161>' : 'trajGen/Subsystem/Z/Tracking Mode/Disabled'
 * '<S162>' : 'trajGen/Subsystem/Z/Tracking Mode Sum/Passthrough'
 * '<S163>' : 'trajGen/Subsystem/Z/Tsamp - Integral/TsSignalSpecification'
 * '<S164>' : 'trajGen/Subsystem/Z/Tsamp - Ngain/Passthrough'
 * '<S165>' : 'trajGen/Subsystem/Z/postSat Signal/Forward_Path'
 * '<S166>' : 'trajGen/Subsystem/Z/preInt Signal/Internal PreInt'
 * '<S167>' : 'trajGen/Subsystem/Z/preSat Signal/Forward_Path'
 * '<S168>' : 'trajGen/position reading/Enabled Subsystem'
 */
#endif                                 /* trajGen_h_ */
