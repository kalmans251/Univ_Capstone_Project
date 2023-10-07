#include "PidControll.h";
#include <Arduino.h>;

void PidControll::calcRotateDiff(float *controlQ,float *difAngle){
  float nom = sqrt(1-square(controlQ[0]));
  float deltaAng=2*acos(controlQ[0]);
  difAngle[0]=180*deltaAng*controlQ[1]/nom/3.141592;
  difAngle[1]=180*deltaAng*controlQ[2]/nom/3.141592;
  difAngle[2]=180*deltaAng*controlQ[3]/nom/3.141592;
}