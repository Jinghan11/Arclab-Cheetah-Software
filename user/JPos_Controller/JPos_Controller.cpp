#include "JPos_Controller.hpp"




void JPos_Controller::runController(){
  
  Mat3<float> kpMat;
  Mat3<float> kdMat;
  //kpMat << 20, 0, 0, 0, 20, 0, 0, 0, 20;
  //kdMat << 2.1, 0, 0, 0, 2.1, 0, 0, 0, 2.1;
  kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
  kdMat <<  userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;

  static int iter(0);
  ++iter;
 

  //Reading current joint position
  if(iter < 10){
    for(int leg(0); leg<4; ++leg){
      for(int jidx(0); jidx<3; ++jidx){
        _jpos_ini[3*leg+jidx] = _legController->datas[leg].q[jidx];
      }
    }
  }

  _legController->_maxTorque = 2;
  _legController->_legsEnabled = true;
  _legController->_zeroEncoders = false;

  #ifdef JPOS_JointTest
  //Every joint is controled by sin function
  if(userParameters.calibrate > 0.4) {
    _legController->_calibrateEncoders = userParameters.calibrate;
  } else {
    if(userParameters.zero > 0.5) {
      _legController->_zeroEncoders = true;
    } else {
      _legController->_zeroEncoders = false;

      for(int leg(0); leg<4; ++leg){
        for(int jidx(0); jidx<3; ++jidx){
          float pos = std::sin(.001f * iter);
          _legController->commands[leg].qDes[jidx] = pos;
          _legController->commands[leg].qdDes[jidx] = 0.;
          _legController->commands[leg].tauFeedForward[jidx] = userParameters.tau_ff;
        }
        _legController->commands[leg].kpJoint = kpMat;
        _legController->commands[leg].kdJoint = kdMat;
      }
    }
  }
  #endif

  /*
  if(iter%200 ==0){
  printf("value 1, 2: %f, %f\n", userParameters.testValue, userParameters.testValue2);
  printf("leg 0: qDes of Ab is %f\n", _legController->commands[0].qDes[0]);
  printf("leg 1: qDes of Ab is %f\n", _legController->commands[1].qDes[0]);
  printf("leg 2: qDes of Ab is %f\n", _legController->commands[2].qDes[0]);
  printf("leg 3: qDes of Ab is %f\n", _legController->commands[3].qDes[0]);
  }
  */

#ifdef JPOS_Tracking
    QuadrupedInverseKinematic *p=new QuadrupedInverseKinematic;

    p->set_leg_length(userParameters.h, userParameters.hu, userParameters.hl);
        if (iter >= 0 && iter <= userParameters.lamda * userParameters.time)
        {
            double sigma = 2 * M_PI * iter / (userParameters.lamda * userParameters.time);
            x_t = (userParameters.x_end - userParameters.x_start) * ((sigma - sin(sigma)) / (2 * M_PI)) + userParameters.x_start;
            z_t = userParameters.z_height * (1 - cos(sigma)) / 2 + userParameters.z_start;
            p->set_toe_position(x_t,0,z_t);
            p->calc_dyz();
            p->calc_lyz();
            p->calc_L_gamma();
            p->calc_R_gamma();
            p->calc_lxz();
            p->calc_n();
            beta  = p->calc_beta();
            alpha = p->calc_alpha();
            for(int leg(0); leg<4; ++leg){
              for(int jidx(0); jidx<3; ++jidx){
                _legController->commands[leg].qDes[0] = gamma;
                _legController->commands[leg].qDes[1] = beta;
                _legController->commands[leg].qDes[2] = alpha;
                _legController->commands[leg].qdDes[jidx] = 0.;
                _legController->commands[leg].tauFeedForward[jidx] = userParameters.tau_ff;
            }
              _legController->commands[leg].kpJoint = kpMat;
              _legController->commands[leg].kdJoint = kdMat;
            }    
        }
        else if (iter > userParameters.lamda * userParameters.time && iter < userParameters.time)
        {
            x_t = x_last - (x_last - userParameters.x_start) / ((userParameters.time - iter) / dt);
            z_t = z_last;
            p->set_toe_position(x_t,0,z_t);
            p->calc_dyz();
            p->calc_lyz();
            p->calc_L_gamma();
            p->calc_R_gamma();
            p->calc_lxz();
            p->calc_n();
            beta  = p->calc_beta();
            alpha = p->calc_alpha();
            for(int leg(0); leg<4; ++leg){
              for(int jidx(0); jidx<3; ++jidx){
                _legController->commands[leg].qDes[0] = gamma;
                _legController->commands[leg].qDes[1] = beta;
                _legController->commands[leg].qDes[2] = alpha;
                _legController->commands[leg].qdDes[jidx] = 0.;
                _legController->commands[leg].tauFeedForward[jidx] = userParameters.tau_ff;
              }
              _legController->commands[leg].kpJoint = kpMat;
              _legController->commands[leg].kdJoint = kdMat;

            }
        }else{
            for(int leg(0); leg<4; ++leg){
              for(int jidx(0); jidx<3; ++jidx){
                _legController->commands[leg].qDes[0] = gamma;
                _legController->commands[leg].qDes[1] = beta;
                _legController->commands[leg].qDes[2] = alpha;
                _legController->commands[leg].qdDes[jidx] = 0.;
                _legController->commands[leg].tauFeedForward[jidx] = userParameters.tau_ff;
            }
              _legController->commands[leg].kpJoint = kpMat;
              _legController->commands[leg].kdJoint = kdMat;
            }
        }
        x_last = x_t;
        z_last = z_t;
        //printf("[Jpos] beta is %f\n", beta);     
        //printf("[Jpos] alpha is %f\n", alpha);   
    
    delete p;
    p = nullptr;
    //system("pause");
    /*
    for(int leg(0); leg<4; ++leg){
        _legController->commands[leg].qDes[0] = gamma;
        _legController->commands[leg].qDes[1] = beta;
        _legController->commands[leg].qDes[2] = alpha;
    }
    */
#endif
}
