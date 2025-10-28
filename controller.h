

#ifndef controller_h_
#define controller_h_
#ifndef controller_COMMON_INCLUDES_
#define controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#endif                                 /* controller_COMMON_INCLUDES_ */

#include "controller_types.h"
#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T FilterCoefficient;            /* '<S40>/Filter Coefficient' */
  real_T FilterCoefficient_c;          /* '<S90>/Filter Coefficient' */
} B_controller_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Filter_CSTATE;                /* '<S32>/Filter' */
  real_T Integrator_CSTATE;            /* '<S37>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S82>/Filter' */
  real_T Integrator_CSTATE_i;          /* '<S87>/Integrator' */
} X_controller_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Filter_CSTATE;                /* '<S32>/Filter' */
  real_T Integrator_CSTATE;            /* '<S37>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S82>/Filter' */
  real_T Integrator_CSTATE_i;          /* '<S87>/Integrator' */
} XDot_controller_T;

/* State disabled  */
typedef struct {
  boolean_T Filter_CSTATE;             /* '<S32>/Filter' */
  boolean_T Integrator_CSTATE;         /* '<S37>/Integrator' */
  boolean_T Filter_CSTATE_f;           /* '<S82>/Filter' */
  boolean_T Integrator_CSTATE_i;       /* '<S87>/Integrator' */
} XDis_controller_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T Sum;                    /* '<S1>/Sum' */
  const real_T Sum1;                   /* '<S1>/Sum1' */
  const real_T DerivativeGain;         /* '<S30>/Derivative Gain' */
  const real_T IntegralGain;           /* '<S34>/Integral Gain' */
  const real_T Sum2;                   /* '<S1>/Sum2' */
  const real_T Sum3;                   /* '<S1>/Sum3' */
  const real_T DerivativeGain_b;       /* '<S80>/Derivative Gain' */
  const real_T IntegralGain_p;         /* '<S84>/Integral Gain' */
} ConstB_controller_T;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* Real-time Model Data Structure */
struct tag_RTM_controller_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_controller_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_controller_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[4];
  real_T odeF[4][4];
  ODE4_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
extern B_controller_T controller_B;

/* Continuous states (default storage) */
extern X_controller_T controller_X;

/* Disabled states (default storage) */
extern XDis_controller_T controller_XDis;
extern const ConstB_controller_T controller_ConstB;/* constant block i/o */

/* Model entry point functions */
extern void controller_initialize(void);
extern void controller_step(void);
extern void controller_terminate(void);

/* Real-time Model object */
extern RT_MODEL_controller_T *const controller_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S42>/Proportional Gain' : Unused code path elimination
 * Block '<S46>/Sum' : Unused code path elimination
 * Block '<S92>/Proportional Gain' : Unused code path elimination
 * Block '<S96>/Sum' : Unused code path elimination
 * Block '<S1>/Gain' : Eliminated nontunable gain of 1
 * Block '<S1>/Gain1' : Eliminated nontunable gain of 1
 */

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
 * '<Root>' : 'controller'
 * '<S1>'   : 'controller/pitch_control'
 * '<S2>'   : 'controller/pitch_control/PID Controller'
 * '<S3>'   : 'controller/pitch_control/PID Controller1'
 * '<S4>'   : 'controller/pitch_control/PID Controller/Anti-windup'
 * '<S5>'   : 'controller/pitch_control/PID Controller/D Gain'
 * '<S6>'   : 'controller/pitch_control/PID Controller/External Derivative'
 * '<S7>'   : 'controller/pitch_control/PID Controller/Filter'
 * '<S8>'   : 'controller/pitch_control/PID Controller/Filter ICs'
 * '<S9>'   : 'controller/pitch_control/PID Controller/I Gain'
 * '<S10>'  : 'controller/pitch_control/PID Controller/Ideal P Gain'
 * '<S11>'  : 'controller/pitch_control/PID Controller/Ideal P Gain Fdbk'
 * '<S12>'  : 'controller/pitch_control/PID Controller/Integrator'
 * '<S13>'  : 'controller/pitch_control/PID Controller/Integrator ICs'
 * '<S14>'  : 'controller/pitch_control/PID Controller/N Copy'
 * '<S15>'  : 'controller/pitch_control/PID Controller/N Gain'
 * '<S16>'  : 'controller/pitch_control/PID Controller/P Copy'
 * '<S17>'  : 'controller/pitch_control/PID Controller/Parallel P Gain'
 * '<S18>'  : 'controller/pitch_control/PID Controller/Reset Signal'
 * '<S19>'  : 'controller/pitch_control/PID Controller/Saturation'
 * '<S20>'  : 'controller/pitch_control/PID Controller/Saturation Fdbk'
 * '<S21>'  : 'controller/pitch_control/PID Controller/Sum'
 * '<S22>'  : 'controller/pitch_control/PID Controller/Sum Fdbk'
 * '<S23>'  : 'controller/pitch_control/PID Controller/Tracking Mode'
 * '<S24>'  : 'controller/pitch_control/PID Controller/Tracking Mode Sum'
 * '<S25>'  : 'controller/pitch_control/PID Controller/Tsamp - Integral'
 * '<S26>'  : 'controller/pitch_control/PID Controller/Tsamp - Ngain'
 * '<S27>'  : 'controller/pitch_control/PID Controller/postSat Signal'
 * '<S28>'  : 'controller/pitch_control/PID Controller/preSat Signal'
 * '<S29>'  : 'controller/pitch_control/PID Controller/Anti-windup/Passthrough'
 * '<S30>'  : 'controller/pitch_control/PID Controller/D Gain/Internal Parameters'
 * '<S31>'  : 'controller/pitch_control/PID Controller/External Derivative/Error'
 * '<S32>'  : 'controller/pitch_control/PID Controller/Filter/Cont. Filter'
 * '<S33>'  : 'controller/pitch_control/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S34>'  : 'controller/pitch_control/PID Controller/I Gain/Internal Parameters'
 * '<S35>'  : 'controller/pitch_control/PID Controller/Ideal P Gain/Passthrough'
 * '<S36>'  : 'controller/pitch_control/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S37>'  : 'controller/pitch_control/PID Controller/Integrator/Continuous'
 * '<S38>'  : 'controller/pitch_control/PID Controller/Integrator ICs/Internal IC'
 * '<S39>'  : 'controller/pitch_control/PID Controller/N Copy/Disabled'
 * '<S40>'  : 'controller/pitch_control/PID Controller/N Gain/Internal Parameters'
 * '<S41>'  : 'controller/pitch_control/PID Controller/P Copy/Disabled'
 * '<S42>'  : 'controller/pitch_control/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S43>'  : 'controller/pitch_control/PID Controller/Reset Signal/Disabled'
 * '<S44>'  : 'controller/pitch_control/PID Controller/Saturation/Passthrough'
 * '<S45>'  : 'controller/pitch_control/PID Controller/Saturation Fdbk/Disabled'
 * '<S46>'  : 'controller/pitch_control/PID Controller/Sum/Sum_PID'
 * '<S47>'  : 'controller/pitch_control/PID Controller/Sum Fdbk/Disabled'
 * '<S48>'  : 'controller/pitch_control/PID Controller/Tracking Mode/Disabled'
 * '<S49>'  : 'controller/pitch_control/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S50>'  : 'controller/pitch_control/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S51>'  : 'controller/pitch_control/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S52>'  : 'controller/pitch_control/PID Controller/postSat Signal/Forward_Path'
 * '<S53>'  : 'controller/pitch_control/PID Controller/preSat Signal/Forward_Path'
 * '<S54>'  : 'controller/pitch_control/PID Controller1/Anti-windup'
 * '<S55>'  : 'controller/pitch_control/PID Controller1/D Gain'
 * '<S56>'  : 'controller/pitch_control/PID Controller1/External Derivative'
 * '<S57>'  : 'controller/pitch_control/PID Controller1/Filter'
 * '<S58>'  : 'controller/pitch_control/PID Controller1/Filter ICs'
 * '<S59>'  : 'controller/pitch_control/PID Controller1/I Gain'
 * '<S60>'  : 'controller/pitch_control/PID Controller1/Ideal P Gain'
 * '<S61>'  : 'controller/pitch_control/PID Controller1/Ideal P Gain Fdbk'
 * '<S62>'  : 'controller/pitch_control/PID Controller1/Integrator'
 * '<S63>'  : 'controller/pitch_control/PID Controller1/Integrator ICs'
 * '<S64>'  : 'controller/pitch_control/PID Controller1/N Copy'
 * '<S65>'  : 'controller/pitch_control/PID Controller1/N Gain'
 * '<S66>'  : 'controller/pitch_control/PID Controller1/P Copy'
 * '<S67>'  : 'controller/pitch_control/PID Controller1/Parallel P Gain'
 * '<S68>'  : 'controller/pitch_control/PID Controller1/Reset Signal'
 * '<S69>'  : 'controller/pitch_control/PID Controller1/Saturation'
 * '<S70>'  : 'controller/pitch_control/PID Controller1/Saturation Fdbk'
 * '<S71>'  : 'controller/pitch_control/PID Controller1/Sum'
 * '<S72>'  : 'controller/pitch_control/PID Controller1/Sum Fdbk'
 * '<S73>'  : 'controller/pitch_control/PID Controller1/Tracking Mode'
 * '<S74>'  : 'controller/pitch_control/PID Controller1/Tracking Mode Sum'
 * '<S75>'  : 'controller/pitch_control/PID Controller1/Tsamp - Integral'
 * '<S76>'  : 'controller/pitch_control/PID Controller1/Tsamp - Ngain'
 * '<S77>'  : 'controller/pitch_control/PID Controller1/postSat Signal'
 * '<S78>'  : 'controller/pitch_control/PID Controller1/preSat Signal'
 * '<S79>'  : 'controller/pitch_control/PID Controller1/Anti-windup/Passthrough'
 * '<S80>'  : 'controller/pitch_control/PID Controller1/D Gain/Internal Parameters'
 * '<S81>'  : 'controller/pitch_control/PID Controller1/External Derivative/Error'
 * '<S82>'  : 'controller/pitch_control/PID Controller1/Filter/Cont. Filter'
 * '<S83>'  : 'controller/pitch_control/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S84>'  : 'controller/pitch_control/PID Controller1/I Gain/Internal Parameters'
 * '<S85>'  : 'controller/pitch_control/PID Controller1/Ideal P Gain/Passthrough'
 * '<S86>'  : 'controller/pitch_control/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S87>'  : 'controller/pitch_control/PID Controller1/Integrator/Continuous'
 * '<S88>'  : 'controller/pitch_control/PID Controller1/Integrator ICs/Internal IC'
 * '<S89>'  : 'controller/pitch_control/PID Controller1/N Copy/Disabled'
 * '<S90>'  : 'controller/pitch_control/PID Controller1/N Gain/Internal Parameters'
 * '<S91>'  : 'controller/pitch_control/PID Controller1/P Copy/Disabled'
 * '<S92>'  : 'controller/pitch_control/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S93>'  : 'controller/pitch_control/PID Controller1/Reset Signal/Disabled'
 * '<S94>'  : 'controller/pitch_control/PID Controller1/Saturation/Passthrough'
 * '<S95>'  : 'controller/pitch_control/PID Controller1/Saturation Fdbk/Disabled'
 * '<S96>'  : 'controller/pitch_control/PID Controller1/Sum/Sum_PID'
 * '<S97>'  : 'controller/pitch_control/PID Controller1/Sum Fdbk/Disabled'
 * '<S98>'  : 'controller/pitch_control/PID Controller1/Tracking Mode/Disabled'
 * '<S99>'  : 'controller/pitch_control/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S100>' : 'controller/pitch_control/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S101>' : 'controller/pitch_control/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S102>' : 'controller/pitch_control/PID Controller1/postSat Signal/Forward_Path'
 * '<S103>' : 'controller/pitch_control/PID Controller1/preSat Signal/Forward_Path'
 */
#endif                                 /* controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
