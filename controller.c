

#include "controller.h"
#include "controller_private.h"

/* Block signals (default storage) */
B_controller_T controller_B;

/* Continuous states */
X_controller_T controller_X;

/* Disabled State Vector */
XDis_controller_T controller_XDis;

/* Real-time model */
static RT_MODEL_controller_T controller_M_;
RT_MODEL_controller_T *const controller_M = &controller_M_;

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 4;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  controller_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  controller_step();
  controller_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  controller_step();
  controller_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  controller_step();
  controller_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void controller_step(void)
{
  if (rtmIsMajorTimeStep(controller_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&controller_M->solverInfo,
                          ((controller_M->Timing.clockTick0+1)*
      controller_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(controller_M)) {
    controller_M->Timing.t[0] = rtsiGetT(&controller_M->solverInfo);
  }

  /* Gain: '<S40>/Filter Coefficient' incorporates:
   *  Integrator: '<S32>/Filter'
   *  Sum: '<S32>/SumD'
   */
  controller_B.FilterCoefficient = (controller_ConstB.DerivativeGain -
    controller_X.Filter_CSTATE) * 664.682083275505;

  /* Gain: '<S90>/Filter Coefficient' incorporates:
   *  Integrator: '<S82>/Filter'
   *  Sum: '<S82>/SumD'
   */
  controller_B.FilterCoefficient_c = (controller_ConstB.DerivativeGain_b -
    controller_X.Filter_CSTATE_f) * 664.682083275505;
  if (rtmIsMajorTimeStep(controller_M)) {
    rt_ertODEUpdateContinuousStates(&controller_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++controller_M->Timing.clockTick0;
    controller_M->Timing.t[0] = rtsiGetSolverStopTime(&controller_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.2s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.2, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      controller_M->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void controller_derivatives(void)
{
  XDot_controller_T *_rtXdot;
  _rtXdot = ((XDot_controller_T *) controller_M->derivs);

  /* Derivatives for Integrator: '<S32>/Filter' */
  _rtXdot->Filter_CSTATE = controller_B.FilterCoefficient;

  /* Derivatives for Integrator: '<S37>/Integrator' */
  _rtXdot->Integrator_CSTATE = controller_ConstB.IntegralGain;

  /* Derivatives for Integrator: '<S82>/Filter' */
  _rtXdot->Filter_CSTATE_f = controller_B.FilterCoefficient_c;

  /* Derivatives for Integrator: '<S87>/Integrator' */
  _rtXdot->Integrator_CSTATE_i = controller_ConstB.IntegralGain_p;
}

/* Model initialize function */
void controller_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&controller_M->solverInfo,
                          &controller_M->Timing.simTimeStep);
    rtsiSetTPtr(&controller_M->solverInfo, &rtmGetTPtr(controller_M));
    rtsiSetStepSizePtr(&controller_M->solverInfo,
                       &controller_M->Timing.stepSize0);
    rtsiSetdXPtr(&controller_M->solverInfo, &controller_M->derivs);
    rtsiSetContStatesPtr(&controller_M->solverInfo, (real_T **)
                         &controller_M->contStates);
    rtsiSetNumContStatesPtr(&controller_M->solverInfo,
      &controller_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&controller_M->solverInfo,
      &controller_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&controller_M->solverInfo,
      &controller_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&controller_M->solverInfo,
      &controller_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&controller_M->solverInfo, (boolean_T**)
      &controller_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&controller_M->solverInfo, (&rtmGetErrorStatus
      (controller_M)));
    rtsiSetRTModelPtr(&controller_M->solverInfo, controller_M);
  }

  rtsiSetSimTimeStep(&controller_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&controller_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&controller_M->solverInfo, false);
  controller_M->intgData.y = controller_M->odeY;
  controller_M->intgData.f[0] = controller_M->odeF[0];
  controller_M->intgData.f[1] = controller_M->odeF[1];
  controller_M->intgData.f[2] = controller_M->odeF[2];
  controller_M->intgData.f[3] = controller_M->odeF[3];
  controller_M->contStates = ((X_controller_T *) &controller_X);
  controller_M->contStateDisabled = ((XDis_controller_T *) &controller_XDis);
  controller_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&controller_M->solverInfo, (void *)&controller_M->intgData);
  rtsiSetSolverName(&controller_M->solverInfo,"ode4");
  rtmSetTPtr(controller_M, &controller_M->Timing.tArray[0]);
  controller_M->Timing.stepSize0 = 0.2;

  /* InitializeConditions for Integrator: '<S32>/Filter' */
  controller_X.Filter_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S37>/Integrator' */
  controller_X.Integrator_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S82>/Filter' */
  controller_X.Filter_CSTATE_f = 0.0;

  /* InitializeConditions for Integrator: '<S87>/Integrator' */
  controller_X.Integrator_CSTATE_i = 0.0;
}

/* Model terminate function */
void controller_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
