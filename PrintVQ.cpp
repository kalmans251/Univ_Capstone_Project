#include "PrintVQ.h";
#include <Arduino.h>;


void PrintVQ::printVec(float *vec){ //3개의 백터인자 시리얼에 출력
  Serial.print("Vec( ");
  Serial.print(vec[0]);
  Serial.print(" , ");
  Serial.print(vec[1]);
  Serial.print(" , ");
  Serial.print(vec[2]);
  Serial.println(" )");
}

void PrintVQ::printQuat(float *quat){ //4개의 쿼터니언인자 시리얼에 출력
  Serial.print("Quat( ");
  Serial.print(quat[0]);
  Serial.print(" , ");
  Serial.print(quat[1]);
  Serial.print(" , ");
  Serial.print(quat[2]);
  Serial.print(" , ");
  Serial.print(quat[3]);
  Serial.println(" )");
}
