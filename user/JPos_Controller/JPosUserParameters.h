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
 
        INIT_PARAMETER(h),
        INIT_PARAMETER(hu),
        INIT_PARAMETER(hl),
        
        INIT_PARAMETER(lamda),
        INIT_PARAMETER(time),
        INIT_PARAMETER(x_start),
        INIT_PARAMETER(x_end),
        INIT_PARAMETER(z_height),
        INIT_PARAMETER(z_start)
      {}

  DECLARE_PARAMETER(double, tau_ff);
  DECLARE_PARAMETER(double, kp);
  DECLARE_PARAMETER(double, kd);
  DECLARE_PARAMETER(double, zero);
  DECLARE_PARAMETER(double, calibrate);

  DECLARE_PARAMETER(float, h);
  DECLARE_PARAMETER(float, hu);
  DECLARE_PARAMETER(float, hl);

  DECLARE_PARAMETER(double, lamda);   // 摆动周期占比
  DECLARE_PARAMETER(double, time);    // 周期时间
  DECLARE_PARAMETER(double, x_start); // x 轴起始点
  DECLARE_PARAMETER(double, x_end);   // x 轴终点
  DECLARE_PARAMETER(double, z_height);// 抬腿高度
  DECLARE_PARAMETER(double, z_start); // z 轴起点
};

#endif //PROJECT_JPOSUSERPARAMETERS_H
