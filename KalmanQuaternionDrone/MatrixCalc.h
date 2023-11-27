#ifndef _MATRIXCALC_h
#define _MATRIXCALC_h


class MatrixCalc
{

  public:
    void rotateVectorQuaternion(float *vec,float *quat); //백터를 쿼터니언으로 회전하여 백터 갱신.
    void quaternionMultiplication(float *quat1,float *quat2,float *resultQ); //쿼터니언 곱 resultQ 갱신.
    void getControlQuaternion(float *quat1,float *quat2,float *resultQ);
    void gyroSetup(); //자이로 셋업
    void getRotationMatFromGyro(float A[][4]); //자이로 센서로부터 얻은 회전 행렬. 결과는 A로 갱신
    void calcQuaternionState(float A[][4],float *Xk); // 결과는 Xk로 갱신.
    void matrixMultiplicationResultBack(float mat1[][4],float mat2[][4]); //두개의 4x4 행렬 끼리의 곱셈 , 결과는 mat2로 갱신.
    void matrixMultiplicationResultFront(float mat1[][4],float mat2[][4]); //두개의 4x4 행렬 끼리의 곱셈 , 결과는 mat1로 갱신.
    void matrixMultiplicationResultOther(float mat1[][4],float mat2[][4],float mat3[][4]); //mat1,mat2 행렬의 곰셈 , 결과는 mat3로 갱신
    void changeToComplexConjugate(float *quat); // 쿼터니언의 켤레복소수로의 변환
    void inverse_matrix_4x4(float *R,float matrix[][4], float resultMat[][4]); // 공분산 R을 포함하는 4x4행렬의 역행렬.
    void inverse_matrix_4x4(float matrix[][4], float resultMat[][4]); // 4x4 행렬의 역행렬.
    void sumMatrix(float mat1[][4],float mat2[][4]); // 두 4x4행렬의 합
    void calcXkMeasureKalmanGain(float *Xk,float *measureQ,float kalmanGain[][4]); // // 칼만필터의 최종단계로 StateMent와 MeasureMent를 KalmanGain의 값을 기반으로 최종 추정 쿼터니언을 계산
    void calcCovarianceError(float Pk[][4],float K[][4]); // 오차 공분산 행렬 계산
    float temp[3]; // 데이터 보관함
    
  private:
    float vx,vy,vz;
    int RateCalibrationNumber; // 자이로 켈리브레이션 횟수
    float RateCalibrationx,RateCalibrationy,RateCalibrationz; // 자이로 켈리브레이션 차이값
    float Result[4][4]; // 데이터 보관함
    float room[4]; // 데이터 보관함
    
    float a_arr[16], inv[16], invout[16], det; //데이터 보관함
    int i, j; //for 문을 위한 데이터 보관함

};

#endif