#ifndef PROJECT_JPOSUSERPARAMETERS_H
#define PROJECT_JPOSUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class JPosUserParameters : public ControlParameters {
public:
  JPosUserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(tau_ff),
        INIT_PARAMETER(kp),
        INIT_PARAMETER(kd),
        INIT_PARAMETER(zero),
        INIT_PARAMETER(calibrate),
        INIT_PARAMETER(pos_x),
        INIT_PARAMETER(pos_y),
        INIT_PARAMETER(pos_z),
        INIT_PARAMETER(h),
        INIT_PARAMETER(hu),
        INIT_PARAMETER(hl)
      {}

  DECLARE_PARAMETER(double, tau_ff);
  DECLARE_PARAMETER(double, kp);
  DECLARE_PARAMETER(double, kd);
  DECLARE_PARAMETER(double, zero);
  DECLARE_PARAMETER(double, calibrate);
  DECLARE_PARAMETER(float, pos_x);
  DECLARE_PARAMETER(float, pos_y);
  DECLARE_PARAMETER(float, pos_z);
  DECLARE_PARAMETER(float, h);
  DECLARE_PARAMETER(float, hu);
  DECLARE_PARAMETER(float, hl);
};

#endif //PROJECT_JPOSUSERPARAMETERS_H
