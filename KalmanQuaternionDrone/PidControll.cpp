#include "PidControll.h";
#include "ConstSetting.h";
#include <Arduino.h>;

void PidControll::calcRotateDiff(float *controlQ,float *difAngle){
  float nom = sqrt(1-pow(controlQ[0],2));
  float deltaAng=2*acos(controlQ[0]);
  difAngle[0]=180*deltaAng*controlQ[1]/nom/3.141592;
  difAngle[1]=180*deltaAng*controlQ[2]/nom/3.141592;
  difAngle[2]=180*deltaAng*controlQ[3]/nom/3.141592;
}

void PidControll::calcPid(float *difAngle,float *pidPitchYawRoll){
  for(i=0;i<3;i++){
    pid_I[i]+=Ki*K*difAngle[i];
    pidPitchYawRoll[i]=Kp*K*difAngle[i]+(Kd*(K*difAngle[i]-prevAngle[i])/DT)+pid_I[i];
    prevAngle[i]=K*difAngle[i];
  }
}