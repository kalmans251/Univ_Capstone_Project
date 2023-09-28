#ifndef _PRINTVQ_h
#define _PRINTVQ_h

class PrintVQ
{
  public:
    void printVec(float *vec); //Vector를 정형화 된 형태로 시리얼에 print
    void printQuat(float *quat); //Quaternion 을 정형화 된 형태로 시리얼에 print
};


#endif