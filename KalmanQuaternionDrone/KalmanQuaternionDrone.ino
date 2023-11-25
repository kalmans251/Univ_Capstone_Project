#include "MatrixCalc.h";
#include "PrintVQ.h";
#include "ConstSetting.h";
#include "QuaternionMeasure.h";
#include "PidControll.h";
#include <Wire.h>;


//===================================================================== // pwm 주파수 변경 low level programming 으로 구현 예시

// #define PWM_PIN 6

// void setup() {
//   // Set the PWM frequency for pin 6 (TC4)
//   configurePWMFrequency(PWM_PIN, 10000);  // Set the desired frequency (e.g., 10 kHz)
// }

// void loop() {
//   // Your main code here
// }

// void configurePWMFrequency(int pin, int frequency) {
//   // Map pin to timer/counter
//   const uint8_t timer = g_APinDescription[pin].ulPWMChannel;
  
//   // Set the PWM frequency
//   uint16_t prescaler = 1;  // Adjust prescaler as needed
//   REG_GCLK_CLKCTRL = GCLK_CLKCTRL_ID(TC4_GCLK_ID) | // Generic Clock Timer 4
//                      GCLK_CLKCTRL_CLKEN |           // Enable GCLK
//                      GCLK_CLKCTRL_GEN_GCLK0;        // Use GCLK0

//   REG_TC4_CTRLA &= ~TC_CTRLA_ENABLE;  // Disable TC4
//   while (REG_TC4_STATUS & TC_STATUS_SYNCBUSY);  // Wait for synchronization

//   REG_TC4_COUNT16_CC0 = F_CPU / (prescaler * frequency) - 1;
//   while (REG_TC4_STATUS & TC_STATUS_SYNCBUSY);  // Wait for synchronization

//   REG_TC4_CTRLA |= TC_CTRLA_ENABLE;  // Enable TC4
//   while (REG_TC4_STATUS & TC_STATUS_SYNCBUSY);  // Wait for synchronization
// }

//=====================================================================  // pwm duty 변경 low level programming 으로 구현 예시

// #define PWM_PIN 6

// void setup() {
//   // Initialize timer for PWM
//   configurePWM();
// }

// void loop() {
//   // Change duty cycle from 0 to 255 and back
//   for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
//     setPWMDutyCycle(PWM_PIN, dutyCycle);
//     delay(10);  // Delay for a short time to observe changes
//   }

//   for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
//     setPWMDutyCycle(PWM_PIN, dutyCycle);
//     delay(10);  // Delay for a short time to observe changes
//   }
// }

// void configurePWM() {
//   // Map pin to timer/counter
//   const uint8_t timer = g_APinDescription[PWM_PIN].ulPWMChannel;

//   // Set the PWM frequency (using a prescaler of 1, adjust as needed)
//   REG_GCLK_CLKCTRL = GCLK_CLKCTRL_ID(TC4_GCLK_ID) | 
//                      GCLK_CLKCTRL_CLKEN | 
//                      GCLK_CLKCTRL_GEN_GCLK0;

//   REG_TC4_CTRLA &= ~TC_CTRLA_ENABLE;  
//   while (REG_TC4_STATUS & TC_STATUS_SYNCBUSY);

//   REG_TC4_CTRLA = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ;
//   while (REG_TC4_STATUS & TC_STATUS_SYNCBUSY);

//   REG_TC4_CTRLA |= TC_CTRLA_ENABLE;
//   while (REG_TC4_STATUS & TC_STATUS_SYNCBUSY);
// }

// void setPWMDutyCycle(int pin, int dutyCycle) {
//   const uint8_t timer = g_APinDescription[pin].ulPWMChannel;

//   REG_TC4_CTRLA &= ~TC_CTRLA_ENABLE;
//   while (REG_TC4_STATUS & TC_STATUS_SYNCBUSY);

//   REG_TC4_COUNT16_CC0 = map(dutyCycle, 0, 4095, 0, REG_TC4_COUNT16_CC0_MAX);
//   while (REG_TC4_STATUS & TC_STATUS_SYNCBUSY);

//   REG_TC4_CTRLA |= TC_CTRLA_ENABLE;
//   while (REG_TC4_STATUS & TC_STATUS_SYNCBUSY);
// }
//=====================================================================


MatrixCalc matCalc; //MatrixCalc 클레스의 인스턴스 생성
PrintVQ printVQ; //PrintVQ 클래스의 인스턴스 생성
QuaternionMeasure Qm; //QuaternionMeasure 클래스의 인스턴스 생성
PidControll pidControl;
uint32_t LoopTimer; // 루프 타이머

uint16_t ch1_time,ch2_time,ch3_time,ch4_time;
uint16_t ch1_time_save,ch2_time_save,ch3_time_save,ch4_time_save;
int ch1_State=0,ch2_State=0,ch3_State=0,ch4_State=0;
uint16_t receive_PitchYawRollThrot[4];

float A[4][4];  //자이로 센서로 부터 받은 각가속도 회전 행렬
float Ainv[4][4]; // 자이로 센서 각가속도 역행렬
float Pk[4][4]={{0.1,0,0,0},{0,0.1,0,0},{0,0,0.1,0},{0,0,0,0.1}}; //공분산오차
float Pkinv[4][4]; //공부산 오차의 역행렬
float Xk[4]={1,0,0,0}; //추정값(칼만필터 적용된 자세 쿼터니언)
float R[4]={0.00001,0.00001,0.00001,0.001}; //가속도와 지자기 센서의 표준편차 제곱
float Q[4][4]={{0.000001,0,0,0},{0,0.000001,0,0},{0,0,0.000001,0},{0,0,0,0.000001}}; //자이로 센서의 표준편차 제곱
float K[4][4]; // 칼만Gain 
float magXYZ[3]; // 지자기 센서값
float magQ[4]; // 지자기 회전 쿼터니언
float accXYZ[3]; // 가속도 센서값
float accQuat[4]; // 가속도 회전 쿼터니언
float measureQ[4]; // 관측 쿼터니언 (지자기 회전 쿼터니언과 가속도 회전 쿼터니언의 조합)
float controlQ[4]; //컨트롤 쿼터니언 (dsired자세로 이동하기위한 쿼터니언 정보)
float desiredQ[4]={1,0,0,0}; // 목표 자세 쿼터니언(드론의 목표 자세를 나타내는 쿼터니언. 조종기와 연동해야 한다.)
float difAngle[3]; // 목표 자세까지의 필요한 각 변위 x,y,z
float pidPitchYawRoll[3];
float yawA[4][4];
float yawQuat[4];
float yawRate;
int dP;
int dR;
float nom;
int mode;
float theta;
float quatPitchRoll[4]={1,0,0,0};
float finalDesiredQ[4];
int setDesiredQ=0;
float moter2,moter3,moter4,moter5;

void ch1_yaw(){
  ch1_time = micros();
  if(ch1_State ==0){
    if(digitalRead(0)==LOW){
      ch1_State=0;
      return;
    }
    ch1_State=1;
    ch1_time_save=ch1_time;
  }else if(ch1_State==1){
    ch1_State=0;
    receive_PitchYawRollThrot[1]=ch1_time-ch1_time_save;
  }
}

void ch2_pitch(){
  ch2_time = micros();
  if(ch2_State ==0){
    if(digitalRead(1)==LOW){
      ch2_State=0;
      return;
    }
    ch2_State=1;
    ch2_time_save=ch2_time;
  }else if(ch2_State==1){
    ch2_State=0;
    receive_PitchYawRollThrot[0]=ch2_time-ch2_time_save;
  }
}

void ch3_throt(){
  ch3_time = micros();
  if(ch3_State ==0){
    if(digitalRead(6)==LOW){
      ch3_State=0;
      return;
    }
    ch3_State=1;
    ch3_time_save=ch3_time;
  }else if(ch3_State==1){
    ch3_State=0;
    receive_PitchYawRollThrot[3]=ch3_time-ch3_time_save;
  }
}

void ch4_roll(){
  ch4_time = micros();
  if(ch4_State ==0){
    if(digitalRead(8)==LOW){
      ch4_State=0;
      return;
    }
    ch4_State=1;
    ch4_time_save=ch4_time;
  }else if(ch4_State==1){
    ch4_State=0;
    receive_PitchYawRollThrot[2]=ch4_time-ch4_time_save;
  }
}
//===================================
// void moter_pwm_send(){

// }

//===================================
void setup() {
  Qm.ConfigMod=0;
  Serial.begin(115200); // 시리얼 포트 9600으로 설정. *
  Wire.setClock(400000); // 센서 클럭 설정 *
  Wire.begin(); //와이어 시작 *
  Qm.magnetometerSetup(); //Magnetometer 기본 세팅 *
  matCalc.gyroSetup(); //자이로 켈리브레이션과 세팅 *
  Qm.accSetup();  //가속도 센서 세팅 *
  
  pinMode(0,INPUT);
  attachInterrupt(digitalPinToInterrupt(0), ch1_yaw, CHANGE);
  pinMode(1,INPUT);
  attachInterrupt(digitalPinToInterrupt(1), ch2_pitch, CHANGE);
  pinMode(6,INPUT);
  attachInterrupt(digitalPinToInterrupt(6), ch3_throt, CHANGE);
  pinMode(8,INPUT);
  attachInterrupt(digitalPinToInterrupt(8), ch4_roll, CHANGE);

  // analogWriteFrequency(2,250);// Right Up = 2 pin , Right Down = 3pin , Left Up= 5pin Left Down =4pin
  // analogWriteFrequency(3,250);
  // analogWriteFrequency(4,250);
  // analogWriteFrequency(5,250);
  analogWriteResolution(12);
  // attachInterrupt(digitalPinToInterrupt(0), moter_pwm_send, RISING); 

  Serial.print("준비끝");
  delay(600);
  Serial.print("시작");
  delay(400);
}
void loop() {
  

  Qm.getAccXYZ(accXYZ); //가속도 계산 *
  Qm.getMagXYZ(magXYZ); //자기장 센서 계산 *
  Qm.getQuatFromAcc(accXYZ,accQuat); //가속도 센서값을 계산하여 가속도 회전 쿼터니언 계산 *
  matCalc.rotateVectorQuaternion(magXYZ,accQuat); //가속도 쿼터니언 회전으로 자기장 센서값 회전하여 보정 *
  Qm.getQuatFromMagConfigrated(magXYZ,magQ); // 보정을 끝마친 자기장센서값을 통해 자기장 회전 쿼터니언 추출 
  if(setDesiredQ == 0){
    matCalc.quaternionMultiplication(desiredQ,magQ,desiredQ);
    setDesiredQ=1;
  }
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

  //printVQ.printQuat(Xk); //Xk 쿼터니언을 시리얼 모니터에 출력

  // PID제어 초석.
  // matCalc.getControlQuaternion(desiredQ,Xk,controlQ); //컨트롤 쿼터니언 계산
  // printVQ.printQuat(controlQ);
  // pidControl.calcRotateDiff(controlQ,difAngle);
  // printVQ.printVec(difAngle);
  // pidControl.calcPid(difAngle,pidPitchYawRoll);
  
  // 조종기 수신.
  // Serial.print("Ch1(YAW): ");
  // Serial.print(receive_PitchYawRollThrot[1]);
  // Serial.println("");
  // Serial.print("Ch2(PITCH): ");
  // Serial.print(receive_PitchYawRollThrot[0]);
  // Serial.println("");
  // Serial.print("Ch3(THROT): ");
  // Serial.print(receive_PitchYawRollThrot[3]);
  // Serial.println("");
  // Serial.print("Ch4(ROLL): ");
  // Serial.print(receive_PitchYawRollThrot[2]);
  // Serial.println("");

  //=======================================
  // 조종기 Yaw값을 사용하여 가상 목표자세 쿼터니언의 z축 쿼터니언 회전
  yawRate=PI*(45/500.)*(1500-receive_PitchYawRollThrot[1])/180.*DT*1/2;
  yawA[0][0]=1;
  yawA[0][1]=0;
  yawA[0][2]=0;
  yawA[0][3]=-yawRate;

  yawA[1][0]=0;
  yawA[1][1]=1;
  yawA[1][2]=-yawRate;
  yawA[1][3]=0;

  yawA[2][0]=0;
  yawA[2][1]=yawRate;
  yawA[2][2]=1;
  yawA[2][3]=0;

  yawA[3][0]=yawRate;
  yawA[3][1]=0;
  yawA[3][2]=0;
  yawA[3][3]=1;
  matCalc.calcQuaternionState(yawA,desiredQ);

  //=======================================
  // 가상 목표자세 쿼터니언에 Pitch,Roll 적용
  dP=receive_PitchYawRollThrot[2]-1500;
  dR=receive_PitchYawRollThrot[0]-1500;

  nom=sqrt(pow(dP,2)+pow(dR,2));
  theta = (30/500.)*(PI/180)*nom;
  quatPitchRoll[0]=cos(theta/2.);
  quatPitchRoll[1]=(dP/nom)*sin(theta/2.);
  quatPitchRoll[2]=(dR/nom)*sin(theta/2.);

  matCalc.quaternionMultiplication(quatPitchRoll,desiredQ,finalDesiredQ);

  //printVQ.printQuat(finalDesiredQ); //모니터에 출력

  //=======================================
  // PID제어 초석.
  matCalc.getControlQuaternion(finalDesiredQ,Xk,controlQ); //컨트롤 쿼터니언 계산
  printVQ.printQuat(controlQ); //모니터에 출력
  pidControl.calcRotateDiff(controlQ,difAngle); // 쿼터니언을 각도로 전환 [dgree]
  printVQ.printVec(difAngle); //모니터에 출력
  pidControl.calcPid(difAngle,pidPitchYawRoll);  //PID 계산


  //=======================================


  //=======================================

  
  while(micros()-LoopTimer < DT*1000000){ //적분 타이밍을 맞추기위해 루프.
  //Serial.println(micros()-LoopTimer);
  };
  LoopTimer=micros();

      
  
}
