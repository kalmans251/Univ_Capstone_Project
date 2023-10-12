#ifndef _PIDCONTROLL_h
#define _PIDCONTROLL_h

class PidControll{
  public:
    void calcRotateDiff(float *controlQ,float *difAngle);
    void calcPid(float *difAngle,float *pidPitchYawRoll);
    float K=1;
    float Kp=1;
    float Ki=1;
    float Kd=1;
  private:
    float prevAngle[3]={0,0,0};
    float pid_I[3]={0,0,0};
    int i;
};


#endif