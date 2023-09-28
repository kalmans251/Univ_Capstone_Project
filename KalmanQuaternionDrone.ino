#include "MatrixCalc.h";
#include "PrintVQ.h";
#include "ConstSetting.h";
#include "QuaternionMeasure.h";
#include <Wire.h>;

MatrixCalc matCalc;
PrintVQ printVQ;
QuaternionMeasure Qm;

uint32_t LoopTimer;

float A[4][4];  //자이로 센서로 부터 받은 각가속도 회전 행렬
float Ainv[4][4]; // 자이로 센서 각가속도 역행렬
float Pk[4][4]={{0.1,0,0,0},{0,0.1,0,0},{0,0,0.1,0},{0,0,0,0.1}}; //공분산오차
float Pkinv[4][4]; //공부산 오차의 역행렬
float Xk[4]={1,0,0,0}; //추정값(칼만필터 적용된 자세 쿼터니언)
float R[4]={0.1,0.1,0.1,0.1}; //가속도와 지자기 센서의 표준편차 제곱
float Q[4][4]={{0.1,0,0,0},{0,0.1,0,0},{0,0,0.1,0},{0,0,0,0.1}}; //자이로 센서의 표준편차 제곱
float K[4][4]; // 칼만Gain 
float magXYZ[3]; // 지자기 센서값
float magQ[4]; // 지자기 회전 쿼터니언
float accXYZ[3]; // 가속도 센서값
float accQuat[4]; // 가속도 회전 쿼터니언
float measureQ[4]; // 관측 쿼터니언 (지자기 회전 쿼터니언과 가속도 회전 쿼터니언의 조합)

void setup() {
  Serial.begin(9600); // 시리얼 포트 9600으로 설정.
  Wire.setClock(400000); // 센서 클럭 설정
  Wire.begin(); //와이어 시작
  Qm.magnetometerSetup(); //Magnetometer 기본 세팅
  matCalc.gyroSetup(); //자이로 켈리브레이션과 세팅
  Qm.accSetup();  //가속도 센서 세팅

  Qm.ConfigMod=0; // 1 로 설정시 지자기센서 와 가속도센서 켈리브레이션 작업환경으로 변경 , 0 는 일반적인 실행
}

void loop() {
  Qm.getAccXYZ(accXYZ); //가속도 계산
  Qm.getMagXYZ(magXYZ); //자기장 센서 계산
  Qm.getQuatFromAcc(accXYZ,accQuat); //가속도 센서값을 계산하여 가속도 회전 쿼터니언 계산
  matCalc.rotateVectorQuaternion(magXYZ,accQuat); //가속도 쿼터니언 회전으로 자기장 센서값 회전하여 보정
  Qm.getQuatFromMagConfigrated(magXYZ,magQ); // 보정을 끝마친 자기장센서값을 통해 자기장 회전 쿼터니언 추출 
  matCalc.quaternionMultiplication(accQuat,magQ,measureQ); //가속도 회전 쿼터니언과 자기장 회전 쿼터니언을 합쳐 최종 관측쿼터니언을 추출. (MeasureMent 추출 마무리). 

  matCalc.getRotationMatFromGyro(A); // 자이로 센서에서 받은값을 회전 행렬 A로 갱신
  matCalc.calcQuaternionState(A,Xk); //Xk A행렬로 dt시간동안의 회전 
  matCalc.inverse_matrix_4x4(A,Ainv); //Ainv 생성
  matCalc.matrixMultiplicationResultBack(A,Pk); // 4x4 행렬곱 Pk로 갱신
  matCalc.matrixMultiplicationResultFront(Pk,Ainv); //4x4 행렬곱 Pk로 갱신
  matCalc.sumMatrix(Pk,Q); // Pk 와 자이로 센서의 분산치 Q 합

  matCalc.inverse_matrix_4x4(R,Pk,Pkinv); // 가속도 자이로 쿼터니언의 분산치 R 를 적용한 Pk의 역행렬 계산
  matCalc.matrixMultiplicationResultOther(Pk,Pkinv,K); // 칼만 Gain 계산!
  matCalc.calcXkMeasureKalmanGain(Xk,measureQ,K); //Xk 와 Mesure 을 뺀후 칼만필터 곱 한 후 다시 Xk 합
  matCalc.calcCovarianceError(Pk,K); //공분산 오차 Pk 갱신

  printVQ.printQuat(Xk); //Xk 쿼터니언을 출력

  while(micros()-LoopTimer < DT*1000000){ //적분 타이밍을 맞추기위해 루프.
  //Serial.println(micros()-LoopTimer);
  };
  LoopTimer=micros();
}
