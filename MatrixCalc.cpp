#include "MatrixCalc.h";
#include "ConstSetting.h";
#include <Arduino.h>;
#include <Wire.h>;

void MatrixCalc::rotateVectorQuaternion(float *vec, float *quat){ // (방향 벡터, 회전 쿼터니언) 으로 회전한 벡터를 계산.
  vx= (quat[0] * quat[0] + quat[1] * quat[1] - quat[2] * quat[2] - quat[3] * quat[3]) * vec[0] + 2 * (quat[1] * quat[2] - quat[0] * quat[3]) * vec[1] + 2 * (quat[1] * quat[3] + quat[0] * quat[2]) * vec[2];
	vy = 2 * (quat[1] * quat[2] + quat[0] * quat[3]) * vec[0] + (quat[0] * quat[0] - quat[1] * quat[1] + quat[2] * quat[2] - quat[3] * quat[3]) * vec[1] + 2 * (quat[2] * quat[3] - quat[0] * quat[1]) * vec[2];
	vz= 2 * (quat[1] * quat[3] - quat[0] * quat[2]) * vec[0] + 2 * (quat[2] * quat[3] + quat[0] * quat[1]) * vec[1] + (quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3]) * vec[2];
  vec[0]=vx;
  vec[1]=vy;
  vec[2]=vz;
}

void MatrixCalc::quaternionMultiplication(float *quat1, float *quat2,float *resultQ){ // 쿼터니언 회전끼리의 곱
	resultQ[0] = quat1[0] * quat2[0] - quat1[1] * quat2[1] - quat1[2] * quat2[2] - quat1[3] * quat2[3];
	resultQ[1] = quat1[0] * quat2[1] + quat1[1] * quat2[0] + quat1[2] * quat2[3] - quat1[3] * quat2[2];
	resultQ[2] = quat1[0] * quat2[2] - quat1[1] * quat2[3] + quat1[2] * quat2[0] + quat1[3] * quat2[1];
	resultQ[3] = quat1[0] * quat2[3] + quat1[1] * quat2[2] - quat1[2] * quat2[1] + quat1[3] * quat2[0];

}

void MatrixCalc::gyroSetup(){ //자이로 센서값의 켈리브레이션. 
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  for(RateCalibrationNumber=0;RateCalibrationNumber<2000;RateCalibrationNumber++){
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);
    RateCalibrationx += (float)(Wire.read()<<8 | Wire.read())/65.5;
    RateCalibrationy += (float)(Wire.read()<<8 | Wire.read())/65.5;
    RateCalibrationz += (float)(Wire.read()<<8 | Wire.read())/65.5;
    delay(1);
  }
  RateCalibrationx/=RateCalibrationNumber;
  RateCalibrationy/=RateCalibrationNumber;
  RateCalibrationz/=RateCalibrationNumber;
}

void MatrixCalc::getRotationMatFromGyro(float (*A)[4]){ // 자이로 센서로부터 4X4 회전 행렬 추출(쿼터니언 방식)

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  temp[0]=PI*((float)(Wire.read()<<8 | Wire.read())/65.5-RateCalibrationx)/180;
  temp[1]=PI*((float)(Wire.read()<<8 | Wire.read())/65.5-RateCalibrationy)/180;
  temp[2]=PI*((float)(Wire.read()<<8 | Wire.read())/65.5-RateCalibrationz)/180;
  
  A[0][0]=1;
  A[0][1]=-temp[0]*DT*1/2;
  A[0][2]=-temp[1]*DT*1/2;
  A[0][3]=-temp[2]*DT*1/2;

  A[1][0]=temp[0]*DT*1/2;
  A[1][1]=1;
  A[1][2]=-temp[2]*DT*1/2;
  A[1][3]=temp[1]*DT*1/2;

  A[2][0]=temp[1]*DT*1/2;
  A[2][1]=temp[2]*DT*1/2;
  A[2][2]=1;
  A[2][3]=-temp[0]*DT*1/2;

  A[3][0]=temp[2]*DT*1/2;
  A[3][1]=-temp[1]*DT*1/2;
  A[3][2]=temp[0]*DT*1/2;
  A[3][3]=1;
}

void MatrixCalc::calcQuaternionState(float A[][4],float *Xk){ // 회전행렬과 쿼터니언을 곱한 StateMent 쿼터니언 계산.
  room[0]=A[0][0]*Xk[0]+A[0][1]*Xk[1]+A[0][2]*Xk[2]+A[0][3]*Xk[3];
  room[1]=A[1][0]*Xk[0]+A[1][1]*Xk[1]+A[1][2]*Xk[2]+A[1][3]*Xk[3];
  room[2]=A[2][0]*Xk[0]+A[2][1]*Xk[1]+A[2][2]*Xk[2]+A[2][3]*Xk[3];
  room[3]=A[3][0]*Xk[0]+A[3][1]*Xk[1]+A[3][2]*Xk[2]+A[3][3]*Xk[3];
  float nom=sqrt(square(room[0])+square(room[1])+square(room[2])+square(room[3]));
  Xk[0]=room[0]/nom;
  Xk[1]=room[1]/nom;
  Xk[2]=room[2]/nom;
  Xk[3]=room[3]/nom;
}

void MatrixCalc::matrixMultiplicationResultBack(float mat1[][4],float mat2[][4]){ // 4x4 행렬의 곱 (결과치는 mat2로 갱신)
  Result[0][0]= mat1[0][0]*mat2[0][0]  +mat1[0][1]*mat2[1][0]  +mat1[0][2]*mat2[2][0]  +mat1[0][3]*mat2[3][0]; //[0][0]
  Result[0][1]= mat1[0][0]*mat2[0][1]  +mat1[0][1]*mat2[1][1]  +mat1[0][2]*mat2[2][1]  +mat1[0][3]*mat2[3][1]; //[0][1]
  Result[0][2]= mat1[0][0]*mat2[0][2]  +mat1[0][1]*mat2[1][2]  +mat1[0][2]*mat2[2][2]  +mat1[0][3]*mat2[3][2]; //[0][2]
  Result[0][3]= mat1[0][0]*mat2[0][3]  +mat1[0][1]*mat2[1][3]  +mat1[0][2]*mat2[2][3]  +mat1[0][3]*mat2[3][3]; //[0][3]
  Result[1][0]= mat1[1][0]*mat2[0][0]  +mat1[1][1]*mat2[1][0]  +mat1[1][2]*mat2[2][0]  +mat1[1][3]*mat2[3][0]; //[1][0]
  Result[1][1]= mat1[1][0]*mat2[0][1]  +mat1[1][1]*mat2[1][1]  +mat1[1][2]*mat2[2][1]  +mat1[1][3]*mat2[3][1];//[1][1]
  Result[1][2]= mat1[1][0]*mat2[0][2]  +mat1[1][1]*mat2[1][2]  +mat1[1][2]*mat2[2][2]  +mat1[1][3]*mat2[3][2]; //[1][2]
  Result[1][3]= mat1[1][0]*mat2[0][3]  +mat1[1][1]*mat2[1][3]  +mat1[1][2]*mat2[2][3]  +mat1[1][3]*mat2[3][3]; //[1][3]
  Result[2][0]= mat1[2][0]*mat2[0][0]  +mat1[2][1]*mat2[1][0]  +mat1[2][2]*mat2[2][0]  +mat1[2][3]*mat2[3][0]; //[2][0]
  Result[2][1]= mat1[2][0]*mat2[0][1]  +mat1[2][1]*mat2[1][1]  +mat1[2][2]*mat2[2][1]  +mat1[2][3]*mat2[3][1]; //[2][1]
  Result[2][2]= mat1[2][0]*mat2[0][2]  +mat1[2][1]*mat2[1][2]  +mat1[2][2]*mat2[2][2]  +mat1[2][3]*mat2[3][2]; //[2][2]
  Result[2][3]= mat1[2][0]*mat2[0][3]  +mat1[2][1]*mat2[1][3]  +mat1[2][2]*mat2[2][3]  +mat1[2][3]*mat2[3][3]; //[2][3]
  Result[3][0]= mat1[3][0]*mat2[0][0]  +mat1[3][1]*mat2[1][0]  +mat1[3][2]*mat2[2][0]  +mat1[3][3]*mat2[3][0]; //[3][0]
  Result[3][1]= mat1[3][0]*mat2[0][1]  +mat1[3][1]*mat2[1][1]  +mat1[3][2]*mat2[2][1]  +mat1[3][3]*mat2[3][1]; //[3][1]
  Result[3][2]= mat1[3][0]*mat2[0][2]  +mat1[3][1]*mat2[1][2]  +mat1[3][2]*mat2[2][2]  +mat1[3][3]*mat2[3][2]; //[3][2]
  Result[3][3]= mat1[3][0]*mat2[0][3]  +mat1[3][1]*mat2[1][3]  +mat1[3][2]*mat2[2][3]  +mat1[3][3]*mat2[3][3]; //[3][3]

  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      mat2[i][j]=Result[i][j];
    }
  }
  
}

void MatrixCalc::matrixMultiplicationResultFront(float mat1[][4],float mat2[][4]){ // 4x4 행렬의 곱 (결과치는 mat1로 갱신)
  Result[0][0]= mat1[0][0]*mat2[0][0]  +mat1[0][1]*mat2[1][0]  +mat1[0][2]*mat2[2][0]  +mat1[0][3]*mat2[3][0]; //[0][0]
  Result[0][1]= mat1[0][0]*mat2[0][1]  +mat1[0][1]*mat2[1][1]  +mat1[0][2]*mat2[2][1]  +mat1[0][3]*mat2[3][1]; //[0][1]
  Result[0][2]= mat1[0][0]*mat2[0][2]  +mat1[0][1]*mat2[1][2]  +mat1[0][2]*mat2[2][2]  +mat1[0][3]*mat2[3][2]; //[0][2]
  Result[0][3]= mat1[0][0]*mat2[0][3]  +mat1[0][1]*mat2[1][3]  +mat1[0][2]*mat2[2][3]  +mat1[0][3]*mat2[3][3]; //[0][3]
  Result[1][0]= mat1[1][0]*mat2[0][0]  +mat1[1][1]*mat2[1][0]  +mat1[1][2]*mat2[2][0]  +mat1[1][3]*mat2[3][0]; //[1][0]
  Result[1][1]= mat1[1][0]*mat2[0][1]  +mat1[1][1]*mat2[1][1]  +mat1[1][2]*mat2[2][1]  +mat1[1][3]*mat2[3][1];//[1][1]
  Result[1][2]= mat1[1][0]*mat2[0][2]  +mat1[1][1]*mat2[1][2]  +mat1[1][2]*mat2[2][2]  +mat1[1][3]*mat2[3][2]; //[1][2]
  Result[1][3]= mat1[1][0]*mat2[0][3]  +mat1[1][1]*mat2[1][3]  +mat1[1][2]*mat2[2][3]  +mat1[1][3]*mat2[3][3]; //[1][3]
  Result[2][0]= mat1[2][0]*mat2[0][0]  +mat1[2][1]*mat2[1][0]  +mat1[2][2]*mat2[2][0]  +mat1[2][3]*mat2[3][0]; //[2][0]
  Result[2][1]= mat1[2][0]*mat2[0][1]  +mat1[2][1]*mat2[1][1]  +mat1[2][2]*mat2[2][1]  +mat1[2][3]*mat2[3][1]; //[2][1]
  Result[2][2]= mat1[2][0]*mat2[0][2]  +mat1[2][1]*mat2[1][2]  +mat1[2][2]*mat2[2][2]  +mat1[2][3]*mat2[3][2]; //[2][2]
  Result[2][3]= mat1[2][0]*mat2[0][3]  +mat1[2][1]*mat2[1][3]  +mat1[2][2]*mat2[2][3]  +mat1[2][3]*mat2[3][3]; //[2][3]
  Result[3][0]= mat1[3][0]*mat2[0][0]  +mat1[3][1]*mat2[1][0]  +mat1[3][2]*mat2[2][0]  +mat1[3][3]*mat2[3][0]; //[3][0]
  Result[3][1]= mat1[3][0]*mat2[0][1]  +mat1[3][1]*mat2[1][1]  +mat1[3][2]*mat2[2][1]  +mat1[3][3]*mat2[3][1]; //[3][1]
  Result[3][2]= mat1[3][0]*mat2[0][2]  +mat1[3][1]*mat2[1][2]  +mat1[3][2]*mat2[2][2]  +mat1[3][3]*mat2[3][2]; //[3][2]
  Result[3][3]= mat1[3][0]*mat2[0][3]  +mat1[3][1]*mat2[1][3]  +mat1[3][2]*mat2[2][3]  +mat1[3][3]*mat2[3][3]; //[3][3]

  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      mat1[i][j]=Result[i][j];
    }
  }
}

void MatrixCalc::matrixMultiplicationResultOther(float mat1[][4],float mat2[][4],float mat3[][4]){ // 4x4 행렬의 곱 (결과치는 mat3로 갱신)
  Result[0][0]= mat1[0][0]*mat2[0][0]  +mat1[0][1]*mat2[1][0]  +mat1[0][2]*mat2[2][0]  +mat1[0][3]*mat2[3][0]; //[0][0]
  Result[0][1]= mat1[0][0]*mat2[0][1]  +mat1[0][1]*mat2[1][1]  +mat1[0][2]*mat2[2][1]  +mat1[0][3]*mat2[3][1]; //[0][1]
  Result[0][2]= mat1[0][0]*mat2[0][2]  +mat1[0][1]*mat2[1][2]  +mat1[0][2]*mat2[2][2]  +mat1[0][3]*mat2[3][2]; //[0][2]
  Result[0][3]= mat1[0][0]*mat2[0][3]  +mat1[0][1]*mat2[1][3]  +mat1[0][2]*mat2[2][3]  +mat1[0][3]*mat2[3][3]; //[0][3]
  Result[1][0]= mat1[1][0]*mat2[0][0]  +mat1[1][1]*mat2[1][0]  +mat1[1][2]*mat2[2][0]  +mat1[1][3]*mat2[3][0]; //[1][0]
  Result[1][1]= mat1[1][0]*mat2[0][1]  +mat1[1][1]*mat2[1][1]  +mat1[1][2]*mat2[2][1]  +mat1[1][3]*mat2[3][1];//[1][1]
  Result[1][2]= mat1[1][0]*mat2[0][2]  +mat1[1][1]*mat2[1][2]  +mat1[1][2]*mat2[2][2]  +mat1[1][3]*mat2[3][2]; //[1][2]
  Result[1][3]= mat1[1][0]*mat2[0][3]  +mat1[1][1]*mat2[1][3]  +mat1[1][2]*mat2[2][3]  +mat1[1][3]*mat2[3][3]; //[1][3]
  Result[2][0]= mat1[2][0]*mat2[0][0]  +mat1[2][1]*mat2[1][0]  +mat1[2][2]*mat2[2][0]  +mat1[2][3]*mat2[3][0]; //[2][0]
  Result[2][1]= mat1[2][0]*mat2[0][1]  +mat1[2][1]*mat2[1][1]  +mat1[2][2]*mat2[2][1]  +mat1[2][3]*mat2[3][1]; //[2][1]
  Result[2][2]= mat1[2][0]*mat2[0][2]  +mat1[2][1]*mat2[1][2]  +mat1[2][2]*mat2[2][2]  +mat1[2][3]*mat2[3][2]; //[2][2]
  Result[2][3]= mat1[2][0]*mat2[0][3]  +mat1[2][1]*mat2[1][3]  +mat1[2][2]*mat2[2][3]  +mat1[2][3]*mat2[3][3]; //[2][3]
  Result[3][0]= mat1[3][0]*mat2[0][0]  +mat1[3][1]*mat2[1][0]  +mat1[3][2]*mat2[2][0]  +mat1[3][3]*mat2[3][0]; //[3][0]
  Result[3][1]= mat1[3][0]*mat2[0][1]  +mat1[3][1]*mat2[1][1]  +mat1[3][2]*mat2[2][1]  +mat1[3][3]*mat2[3][1]; //[3][1]
  Result[3][2]= mat1[3][0]*mat2[0][2]  +mat1[3][1]*mat2[1][2]  +mat1[3][2]*mat2[2][2]  +mat1[3][3]*mat2[3][2]; //[3][2]
  Result[3][3]= mat1[3][0]*mat2[0][3]  +mat1[3][1]*mat2[1][3]  +mat1[3][2]*mat2[2][3]  +mat1[3][3]*mat2[3][3]; //[3][3]

  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      mat3[i][j]=Result[i][j];
    }
  }
  
}

void MatrixCalc::changeToComplexConjugate(float *quat){ // 켤레복소수로의 전환
  quat[1]*=-1.;
  quat[2]*=-1.;
  quat[3]*=-1.;
}

void MatrixCalc::sumMatrix(float mat1[][4],float mat2[][4]){ // 4x4행렬의 합
  for(i=0;i<4;i++){
    for(j=0;j<4;j++){
      mat1[i][j]=mat1[i][j]+mat2[i][j];
    }
  }
}

void MatrixCalc::calcXkMeasureKalmanGain(float *Xk,float *measureQ,float kalmanGain[][4]){ // 칼만필터의 최종단계로 StateMent와 MeasureMent를 KalmanGain의 값으로 최종 추정 쿼터니언을 계산

  room[0]=measureQ[0]-Xk[0];
  room[1]=measureQ[1]-Xk[1];
  room[2]=measureQ[2]-Xk[2];
  room[3]=measureQ[3]-Xk[3];

  Xk[0]+=kalmanGain[0][0]*room[0]+kalmanGain[0][1]*room[1]+kalmanGain[0][2]*room[2]+kalmanGain[0][3]*room[3];
  Xk[1]+=kalmanGain[1][0]*room[0]+kalmanGain[1][1]*room[1]+kalmanGain[1][2]*room[2]+kalmanGain[1][3]*room[3];
  Xk[2]+=kalmanGain[2][0]*room[0]+kalmanGain[2][1]*room[1]+kalmanGain[2][2]*room[2]+kalmanGain[2][3]*room[3];
  Xk[3]+=kalmanGain[3][0]*room[0]+kalmanGain[3][1]*room[1]+kalmanGain[3][2]*room[2]+kalmanGain[3][3]*room[3];

}

void MatrixCalc::calcCovarianceError(float Pk[][4],float K[][4]){ //오차 공분산 행렬 계산
  Result[0][0]= K[0][0]*Pk[0][0]  +K[0][1]*Pk[1][0]  +K[0][2]*Pk[2][0]  +K[0][3]*Pk[3][0]; //[0][0]
  Result[0][1]= K[0][0]*Pk[0][1]  +K[0][1]*Pk[1][1]  +K[0][2]*Pk[2][1]  +K[0][3]*Pk[3][1]; //[0][1]
  Result[0][2]= K[0][0]*Pk[0][2]  +K[0][1]*Pk[1][2]  +K[0][2]*Pk[2][2]  +K[0][3]*Pk[3][2]; //[0][2]
  Result[0][3]= K[0][0]*Pk[0][3]  +K[0][1]*Pk[1][3]  +K[0][2]*Pk[2][3]  +K[0][3]*Pk[3][3]; //[0][3]
  Result[1][0]= K[1][0]*Pk[0][0]  +K[1][1]*Pk[1][0]  +K[1][2]*Pk[2][0]  +K[1][3]*Pk[3][0]; //[1][0]
  Result[1][1]= K[1][0]*Pk[0][1]  +K[1][1]*Pk[1][1]  +K[1][2]*Pk[2][1]  +K[1][3]*Pk[3][1];//[1][1]
  Result[1][2]= K[1][0]*Pk[0][2]  +K[1][1]*Pk[1][2]  +K[1][2]*Pk[2][2]  +K[1][3]*Pk[3][2]; //[1][2]
  Result[1][3]= K[1][0]*Pk[0][3]  +K[1][1]*Pk[1][3]  +K[1][2]*Pk[2][3]  +K[1][3]*Pk[3][3]; //[1][3]
  Result[2][0]= K[2][0]*Pk[0][0]  +K[2][1]*Pk[1][0]  +K[2][2]*Pk[2][0]  +K[2][3]*Pk[3][0]; //[2][0]
  Result[2][1]= K[2][0]*Pk[0][1]  +K[2][1]*Pk[1][1]  +K[2][2]*Pk[2][1]  +K[2][3]*Pk[3][1]; //[2][1]
  Result[2][2]= K[2][0]*Pk[0][2]  +K[2][1]*Pk[1][2]  +K[2][2]*Pk[2][2]  +K[2][3]*Pk[3][2]; //[2][2]
  Result[2][3]= K[2][0]*Pk[0][3]  +K[2][1]*Pk[1][3]  +K[2][2]*Pk[2][3]  +K[2][3]*Pk[3][3]; //[2][3]
  Result[3][0]= K[3][0]*Pk[0][0]  +K[3][1]*Pk[1][0]  +K[3][2]*Pk[2][0]  +K[3][3]*Pk[3][0]; //[3][0]
  Result[3][1]= K[3][0]*Pk[0][1]  +K[3][1]*Pk[1][1]  +K[3][2]*Pk[2][1]  +K[3][3]*Pk[3][1]; //[3][1]
  Result[3][2]= K[3][0]*Pk[0][2]  +K[3][1]*Pk[1][2]  +K[3][2]*Pk[2][2]  +K[3][3]*Pk[3][2]; //[3][2]
  Result[3][3]= K[3][0]*Pk[0][3]  +K[3][1]*Pk[1][3]  +K[3][2]*Pk[2][3]  +K[3][3]*Pk[3][3]; //[3][3]

  for(i=0;i<4;i++){
    for(j=0;j<4;j++){
      Pk[i][j]-=Result[i][j];
    }
  }

}

void MatrixCalc::inverse_matrix_4x4(float *R,float matrix[][4], float resultMat[][4]){ // 4x4 행렬의 역행렬 계산 (3개 입력버전)

    for (j = 0; j < 4; j++)
    {
        for (i = 0; i < 4; i++)
        {
            if(j==i){
              a_arr[i + 4 * j] = matrix[j][i]+R[i];
        }
    }

    inv[0] = a_arr[5] * a_arr[10] * a_arr[15] -
        a_arr[5] * a_arr[11] * a_arr[14] -
        a_arr[9] * a_arr[6] * a_arr[15] +
        a_arr[9] * a_arr[7] * a_arr[14] +
        a_arr[13] * a_arr[6] * a_arr[11] -
        a_arr[13] * a_arr[7] * a_arr[10];

    inv[4] = -a_arr[4] * a_arr[10] * a_arr[15] +
        a_arr[4] * a_arr[11] * a_arr[14] +
        a_arr[8] * a_arr[6] * a_arr[15] -
        a_arr[8] * a_arr[7] * a_arr[14] -
        a_arr[12] * a_arr[6] * a_arr[11] +
        a_arr[12] * a_arr[7] * a_arr[10];

    inv[8] = a_arr[4] * a_arr[9] * a_arr[15] -
        a_arr[4] * a_arr[11] * a_arr[13] -
        a_arr[8] * a_arr[5] * a_arr[15] +
        a_arr[8] * a_arr[7] * a_arr[13] +
        a_arr[12] * a_arr[5] * a_arr[11] -
        a_arr[12] * a_arr[7] * a_arr[9];

    inv[12] = -a_arr[4] * a_arr[9] * a_arr[14] +
        a_arr[4] * a_arr[10] * a_arr[13] +
        a_arr[8] * a_arr[5] * a_arr[14] -
        a_arr[8] * a_arr[6] * a_arr[13] -
        a_arr[12] * a_arr[5] * a_arr[10] +
        a_arr[12] * a_arr[6] * a_arr[9];

    inv[1] = -a_arr[1] * a_arr[10] * a_arr[15] +
        a_arr[1] * a_arr[11] * a_arr[14] +
        a_arr[9] * a_arr[2] * a_arr[15] -
        a_arr[9] * a_arr[3] * a_arr[14] -
        a_arr[13] * a_arr[2] * a_arr[11] +
        a_arr[13] * a_arr[3] * a_arr[10];

    inv[5] = a_arr[0] * a_arr[10] * a_arr[15] -
        a_arr[0] * a_arr[11] * a_arr[14] -
        a_arr[8] * a_arr[2] * a_arr[15] +
        a_arr[8] * a_arr[3] * a_arr[14] +
        a_arr[12] * a_arr[2] * a_arr[11] -
        a_arr[12] * a_arr[3] * a_arr[10];

    inv[9] = -a_arr[0] * a_arr[9] * a_arr[15] +
        a_arr[0] * a_arr[11] * a_arr[13] +
        a_arr[8] * a_arr[1] * a_arr[15] -
        a_arr[8] * a_arr[3] * a_arr[13] -
        a_arr[12] * a_arr[1] * a_arr[11] +
        a_arr[12] * a_arr[3] * a_arr[9];

    inv[13] = a_arr[0] * a_arr[9] * a_arr[14] -
        a_arr[0] * a_arr[10] * a_arr[13] -
        a_arr[8] * a_arr[1] * a_arr[14] +
        a_arr[8] * a_arr[2] * a_arr[13] +
        a_arr[12] * a_arr[1] * a_arr[10] -
        a_arr[12] * a_arr[2] * a_arr[9];

    inv[2] = a_arr[1] * a_arr[6] * a_arr[15] -
        a_arr[1] * a_arr[7] * a_arr[14] -
        a_arr[5] * a_arr[2] * a_arr[15] +
        a_arr[5] * a_arr[3] * a_arr[14] +
        a_arr[13] * a_arr[2] * a_arr[7] -
        a_arr[13] * a_arr[3] * a_arr[6];

    inv[6] = -a_arr[0] * a_arr[6] * a_arr[15] +
        a_arr[0] * a_arr[7] * a_arr[14] +
        a_arr[4] * a_arr[2] * a_arr[15] -
        a_arr[4] * a_arr[3] * a_arr[14] -
        a_arr[12] * a_arr[2] * a_arr[7] +
        a_arr[12] * a_arr[3] * a_arr[6];

    inv[10] = a_arr[0] * a_arr[5] * a_arr[15] -
        a_arr[0] * a_arr[7] * a_arr[13] -
        a_arr[4] * a_arr[1] * a_arr[15] +
        a_arr[4] * a_arr[3] * a_arr[13] +
        a_arr[12] * a_arr[1] * a_arr[7] -
        a_arr[12] * a_arr[3] * a_arr[5];

    inv[14] = -a_arr[0] * a_arr[5] * a_arr[14] +
        a_arr[0] * a_arr[6] * a_arr[13] +
        a_arr[4] * a_arr[1] * a_arr[14] -
        a_arr[4] * a_arr[2] * a_arr[13] -
        a_arr[12] * a_arr[1] * a_arr[6] +
        a_arr[12] * a_arr[2] * a_arr[5];

    inv[3] = -a_arr[1] * a_arr[6] * a_arr[11] +
        a_arr[1] * a_arr[7] * a_arr[10] +
        a_arr[5] * a_arr[2] * a_arr[11] -
        a_arr[5] * a_arr[3] * a_arr[10] -
        a_arr[9] * a_arr[2] * a_arr[7] +
        a_arr[9] * a_arr[3] * a_arr[6];

    inv[7] = a_arr[0] * a_arr[6] * a_arr[11] -
        a_arr[0] * a_arr[7] * a_arr[10] -
        a_arr[4] * a_arr[2] * a_arr[11] +
        a_arr[4] * a_arr[3] * a_arr[10] +
        a_arr[8] * a_arr[2] * a_arr[7] -
        a_arr[8] * a_arr[3] * a_arr[6];

    inv[11] = -a_arr[0] * a_arr[5] * a_arr[11] +
        a_arr[0] * a_arr[7] * a_arr[9] +
        a_arr[4] * a_arr[1] * a_arr[11] -
        a_arr[4] * a_arr[3] * a_arr[9] -
        a_arr[8] * a_arr[1] * a_arr[7] +
        a_arr[8] * a_arr[3] * a_arr[5];

    inv[15] = a_arr[0] * a_arr[5] * a_arr[10] -
        a_arr[0] * a_arr[6] * a_arr[9] -
        a_arr[4] * a_arr[1] * a_arr[10] +
        a_arr[4] * a_arr[2] * a_arr[9] +
        a_arr[8] * a_arr[1] * a_arr[6] -
        a_arr[8] * a_arr[2] * a_arr[5];

    det = a_arr[0] * inv[0] + a_arr[1] * inv[4] + a_arr[2] * inv[8] + a_arr[3] * inv[12];

    if (det == 0)
        return false;

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
    {
        invout[i] = inv[i] * det;
    }

    for (j = 0; j < 4; j++)
    {
        for (i = 0; i < 4; i++)
        {
            resultMat[j][i] = invout[i + 4 * j];
        }
    }

  }





void MatrixCalc::inverse_matrix_4x4(float matrix[][4], float resultMat[][4]){  // 4x4행렬의 역행렬 계산 (2개 입력버전)

    for (j = 0; j < 4; j++)
    {
        for (i = 0; i < 4; i++)
        {
            a_arr[i + 4 * j] = matrix[j][i];
        }
    }

    inv[0] = a_arr[5] * a_arr[10] * a_arr[15] -
        a_arr[5] * a_arr[11] * a_arr[14] -
        a_arr[9] * a_arr[6] * a_arr[15] +
        a_arr[9] * a_arr[7] * a_arr[14] +
        a_arr[13] * a_arr[6] * a_arr[11] -
        a_arr[13] * a_arr[7] * a_arr[10];

    inv[4] = -a_arr[4] * a_arr[10] * a_arr[15] +
        a_arr[4] * a_arr[11] * a_arr[14] +
        a_arr[8] * a_arr[6] * a_arr[15] -
        a_arr[8] * a_arr[7] * a_arr[14] -
        a_arr[12] * a_arr[6] * a_arr[11] +
        a_arr[12] * a_arr[7] * a_arr[10];

    inv[8] = a_arr[4] * a_arr[9] * a_arr[15] -
        a_arr[4] * a_arr[11] * a_arr[13] -
        a_arr[8] * a_arr[5] * a_arr[15] +
        a_arr[8] * a_arr[7] * a_arr[13] +
        a_arr[12] * a_arr[5] * a_arr[11] -
        a_arr[12] * a_arr[7] * a_arr[9];

    inv[12] = -a_arr[4] * a_arr[9] * a_arr[14] +
        a_arr[4] * a_arr[10] * a_arr[13] +
        a_arr[8] * a_arr[5] * a_arr[14] -
        a_arr[8] * a_arr[6] * a_arr[13] -
        a_arr[12] * a_arr[5] * a_arr[10] +
        a_arr[12] * a_arr[6] * a_arr[9];

    inv[1] = -a_arr[1] * a_arr[10] * a_arr[15] +
        a_arr[1] * a_arr[11] * a_arr[14] +
        a_arr[9] * a_arr[2] * a_arr[15] -
        a_arr[9] * a_arr[3] * a_arr[14] -
        a_arr[13] * a_arr[2] * a_arr[11] +
        a_arr[13] * a_arr[3] * a_arr[10];

    inv[5] = a_arr[0] * a_arr[10] * a_arr[15] -
        a_arr[0] * a_arr[11] * a_arr[14] -
        a_arr[8] * a_arr[2] * a_arr[15] +
        a_arr[8] * a_arr[3] * a_arr[14] +
        a_arr[12] * a_arr[2] * a_arr[11] -
        a_arr[12] * a_arr[3] * a_arr[10];

    inv[9] = -a_arr[0] * a_arr[9] * a_arr[15] +
        a_arr[0] * a_arr[11] * a_arr[13] +
        a_arr[8] * a_arr[1] * a_arr[15] -
        a_arr[8] * a_arr[3] * a_arr[13] -
        a_arr[12] * a_arr[1] * a_arr[11] +
        a_arr[12] * a_arr[3] * a_arr[9];

    inv[13] = a_arr[0] * a_arr[9] * a_arr[14] -
        a_arr[0] * a_arr[10] * a_arr[13] -
        a_arr[8] * a_arr[1] * a_arr[14] +
        a_arr[8] * a_arr[2] * a_arr[13] +
        a_arr[12] * a_arr[1] * a_arr[10] -
        a_arr[12] * a_arr[2] * a_arr[9];

    inv[2] = a_arr[1] * a_arr[6] * a_arr[15] -
        a_arr[1] * a_arr[7] * a_arr[14] -
        a_arr[5] * a_arr[2] * a_arr[15] +
        a_arr[5] * a_arr[3] * a_arr[14] +
        a_arr[13] * a_arr[2] * a_arr[7] -
        a_arr[13] * a_arr[3] * a_arr[6];

    inv[6] = -a_arr[0] * a_arr[6] * a_arr[15] +
        a_arr[0] * a_arr[7] * a_arr[14] +
        a_arr[4] * a_arr[2] * a_arr[15] -
        a_arr[4] * a_arr[3] * a_arr[14] -
        a_arr[12] * a_arr[2] * a_arr[7] +
        a_arr[12] * a_arr[3] * a_arr[6];

    inv[10] = a_arr[0] * a_arr[5] * a_arr[15] -
        a_arr[0] * a_arr[7] * a_arr[13] -
        a_arr[4] * a_arr[1] * a_arr[15] +
        a_arr[4] * a_arr[3] * a_arr[13] +
        a_arr[12] * a_arr[1] * a_arr[7] -
        a_arr[12] * a_arr[3] * a_arr[5];

    inv[14] = -a_arr[0] * a_arr[5] * a_arr[14] +
        a_arr[0] * a_arr[6] * a_arr[13] +
        a_arr[4] * a_arr[1] * a_arr[14] -
        a_arr[4] * a_arr[2] * a_arr[13] -
        a_arr[12] * a_arr[1] * a_arr[6] +
        a_arr[12] * a_arr[2] * a_arr[5];

    inv[3] = -a_arr[1] * a_arr[6] * a_arr[11] +
        a_arr[1] * a_arr[7] * a_arr[10] +
        a_arr[5] * a_arr[2] * a_arr[11] -
        a_arr[5] * a_arr[3] * a_arr[10] -
        a_arr[9] * a_arr[2] * a_arr[7] +
        a_arr[9] * a_arr[3] * a_arr[6];

    inv[7] = a_arr[0] * a_arr[6] * a_arr[11] -
        a_arr[0] * a_arr[7] * a_arr[10] -
        a_arr[4] * a_arr[2] * a_arr[11] +
        a_arr[4] * a_arr[3] * a_arr[10] +
        a_arr[8] * a_arr[2] * a_arr[7] -
        a_arr[8] * a_arr[3] * a_arr[6];

    inv[11] = -a_arr[0] * a_arr[5] * a_arr[11] +
        a_arr[0] * a_arr[7] * a_arr[9] +
        a_arr[4] * a_arr[1] * a_arr[11] -
        a_arr[4] * a_arr[3] * a_arr[9] -
        a_arr[8] * a_arr[1] * a_arr[7] +
        a_arr[8] * a_arr[3] * a_arr[5];

    inv[15] = a_arr[0] * a_arr[5] * a_arr[10] -
        a_arr[0] * a_arr[6] * a_arr[9] -
        a_arr[4] * a_arr[1] * a_arr[10] +
        a_arr[4] * a_arr[2] * a_arr[9] +
        a_arr[8] * a_arr[1] * a_arr[6] -
        a_arr[8] * a_arr[2] * a_arr[5];

    det = a_arr[0] * inv[0] + a_arr[1] * inv[4] + a_arr[2] * inv[8] + a_arr[3] * inv[12];

    if (det == 0)
        return false;

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
    {
        invout[i] = inv[i] * det;
    }

    for (j = 0; j < 4; j++)
    {
        for (i = 0; i < 4; i++)
        {
            resultMat[j][i] = invout[i + 4 * j];
        }
    }

  }




