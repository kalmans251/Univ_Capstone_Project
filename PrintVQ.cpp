#include "PrintVQ.h";
#include <Arduino.h>;


void PrintVQ::printVec(float *vec){
  Serial.print("Vec( ");
  Serial.print(vec[0]);
  Serial.print(" , ");
  Serial.print(vec[1]);
  Serial.print(" , ");
  Serial.print(vec[2]);
  Serial.println(" )");
}

void PrintVQ::printQuat(float *quat){
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
