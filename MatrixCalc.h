#ifndef _MATRIXCALC_h
#define _MATRIXCALC_h

class MatrixCalc
{
  public:
    void rotateVectorQuaternion(float *vec,float *quat); //백터를 쿼터니언으로 회전하여 백터 갱신.
    void quaternionMultiplication(float *quat1,float *quat2,float *resultQ); //쿼터니언 곱 resultQ 갱신.
    void gyroSetup();
    void getRotationMatFromGyro(float A[][4]); //자이로 센서로부터 얻은 회전 행렬. 결과는 A로 갱신
    void calcQuaternionState(float A[][4],float *Xk); // 결과는 Xk로 갱신.
    void matrixMultiplicationResultBack(float mat1[][4],float mat2[][4]); //4x4 행렬 끼리의 곱셈 , 결과는 mat2로 갱신.
    void matrixMultiplicationResultFront(float mat1[][4],float mat2[][4]);
    void matrixMultiplicationResultOther(float mat1[][4],float mat2[][4],float mat3[][4]);
    void changeToComplexConjugate(float *quat);
    void inverse_matrix_4x4(float *R,float matrix[][4], float resultMat[][4]);
    void inverse_matrix_4x4(float matrix[][4], float resultMat[][4]);
    void sumMatrix(float mat1[][4],float mat2[][4]);
    void calcXkMeasureKalmanGain(float *Xk,float *measureQ,float kalmanGain[][4]);
    void calcCovarianceError(float Pk[][4],float K[][4]);
  private:
    float vx,vy,vz;
    int RateCalibrationNumber;
    float RateCalibrationx,RateCalibrationy,RateCalibrationz;
    float Result[4][4];
    float room[4];
    float temp[3];
    float a_arr[16], inv[16], invout[16], det;
    int i, j;
};


#endif