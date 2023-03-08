#ifndef JPOS_CONTROLLER
#define JPOS_CONTROLLER

#include <RobotController.h>
#include "JPosUserParameters.h"

#include <iostream>
//Eigen is not used in this project , however it is reserved for enhanced extensibility
#include"QuadrupedInverseKinematic.h"


//#define JPOS_JointTest
#define JPOS_Tracking

class JPos_Controller:public RobotController{
  public:
    JPos_Controller():RobotController(),_jpos_ini(cheetah::num_act_joint){
    _jpos_ini.setZero();
    }
    virtual ~JPos_Controller(){}

    virtual void initializeController(){}
    virtual void runController();
    virtual void updateVisualization(){}
    virtual ControlParameters* getUserControlParameters() {
      return &userParameters;
    }
  private:
        vector<double> x; // 存储 x 坐标
        vector<double> z; // 存储 z 坐标
        double x_t = 0;
        double z_t = 0;
        double x_last = 0;
        double z_last = 0;
        double dt = 1;  //  t - t_last
        double gamma = 0;
        double beta = 0;
        double alpha = 0;
  protected:
    DVec<float> _jpos_ini;
  JPosUserParameters userParameters;
};

#endif
