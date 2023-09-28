#ifndef _QUATERNIONMEASURE_h
#define _QUATERNIONMEASURE_h

class QuaternionMeasure
{
  public:

  //지자기 센서 기본세팅과 출력.
    void magnetometerSetup();
    void getMagXYZ(float *magXYZ);

  //가속도 센서 기본세팅과 출력.
    void accSetup();
    void getAccXYZ(float *accXYZ);
    
  //가속도 센서를 사용한 쿼터니언 회전 추출
    void getQuatFromAcc(float *accXYZ,float *accQ);
  
  //지자기 센서로부터 받은 회전 쿼터니언 계산.
   void getQuatFromMagConfigrated(float *magXYZ_config,float *magQ);

  //켈리브레이션 모드 설정 1=True, 0=False
    int ConfigMod;

  private:
    int status_1; //지자기 데이터 유효 상태 체크
    const int MPU_ADDR = 0x68; //자이로,가속도 센서 주소
    const int MAG_ADDR = 0x0C; //지자기 센서 주소
    float maxXYZ[3]={-999,-999,-999};
    float minXYZ[3]={999,999,999};
};

#endif