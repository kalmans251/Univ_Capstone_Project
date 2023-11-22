#include "QuaternionMeasure.h";
#include "ConstSetting.h";
#include <Arduino.h>;
#include <Wire.h>;


void QuaternionMeasure::magnetometerSetup(){ // 지자기센서를 읽어들이기 위한 초반 세팅
  // Acc & Gyro Registers********************************************
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6A);   // USER CONTROL  
  Wire.write(0x00);   // 0x00 is reset value
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);   
  Wire.write(0x37);   //  IMU INT PIN CONFIG    
  Wire.write(0x02);   //  0x02 activate bypass in order to communicate with magnetometer
  Wire.endTransmission(true);
  delay(100);

  // Magnetometer Registers*****************************************
  Wire.beginTransmission(MAG_ADDR); 
  Wire.write(0x0B);   //  CONTROL 2
  Wire.write(0b01);   //  0 NORMAL OR 1 RESET
  Wire.endTransmission(true);
  delay(100);
  
  Wire.beginTransmission(MAG_ADDR);   //SLEEP MODE
  Wire.write(0x0A);   //  CONTROL 1
  Wire.write(0b00010000);  // 1 for 16 bit or 0 for 14 bit output, 0000 SLEEP MODE
  Wire.endTransmission(true);
  delay(100);
    
  Wire.beginTransmission(MAG_ADDR);   //ROM WRITE MODE
  Wire.write(0x0A);   //  CONTROL 1
  Wire.write(0b00011111); // 1 for 16 bit or 0 for 14 bit output, 1111 FUSE ROM ACCESS MODE
  Wire.endTransmission(true);
  delay(200);

  Wire.beginTransmission(MAG_ADDR);   //GET MAGNETIC SENSITIVITY DATA FOR CONVERTING RAW DATA
  Wire.write(0x10);     //  ASAX  
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 3 , true);  //GET SENSITIVITY ADJUSMENT VALUES STARTS AT ASAX
  Msens_x = Wire.read();    //GET X SENSITIVITY ADJUSMENT VALUE
  Msens_y = Wire.read();    //GET Y SENSITIVITY ADJUSMENT VALUE
  Msens_z = Wire.read();    //GET Z SENSITIVITY ADJUSMENT VALUE
  Serial.println(Msens_x);
  Serial.println(Msens_y);
  Serial.println(Msens_z);
  Wire.endTransmission(true);
  asax = (((Msens_x-128))/256.0f)+1.0f;
  asay = (((Msens_y-128))/256.0f)+1.0f;
  asaz = (((Msens_z-128))/256.0f)+1.0f;
  Serial.print("Mx Sensitivity: ");  Serial.println(asax);
  Serial.print("My Sensitivity: ");  Serial.println(asay);
  Serial.print("Mz Sensitivity: ");  Serial.println(asaz); 
  delay(200);

  Wire.beginTransmission(MAG_ADDR);   //SLEEP MODE
  Wire.write(0x0A);   //  CONTROL 1
  Wire.write(0b00010000);  // 1 for 16 bit or 0 for 14 bit output, 0000 SLEEP MODE
  Wire.endTransmission(true);
  delay(200);

  Wire.beginTransmission(MAG_ADDR);   //CONT MODE 2
  Wire.write(0x0A);
  Wire.write(0b00010110); // 1 for 16 bit or 0 for 14 bit output, 0110 FOR CONT MODE 2 (X Hz?) 
  Wire.endTransmission(true);
  delay(100);
}

void QuaternionMeasure::getMagXYZ(float *magXYZ){ // 지자기센서의 값을 간단히 보정하여 magXYZ의 주소로 정보주입
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 1 , true);   
  status_1 = Wire.read();
  Wire.endTransmission(true);
  
  if(status_1 == 0b11) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x03);
    Wire.endTransmission(false);
    
    if(ConfigMod==1){
      Wire.requestFrom(MAG_ADDR, 7 , true);
      magXYZ[1] =  (int16_t)(Wire.read() | Wire.read()<<8);
      magXYZ[0] =  (int16_t)(Wire.read() | Wire.read()<<8);  
      magXYZ[2] =  -(int16_t)(Wire.read() | Wire.read()<<8);
      Wire.endTransmission(true);
      
      if(maxXYZ[0] < magXYZ[0]){
        maxXYZ[0]=magXYZ[0];
      }
      if(maxXYZ[1] < magXYZ[1]){
       maxXYZ[1]=magXYZ[1];
      }
      if(maxXYZ[2] < magXYZ[2]){
        maxXYZ[2]=magXYZ[2];
      }
      if(minXYZ[0] > magXYZ[0]){
        minXYZ[0]=magXYZ[0];
      }
      if(minXYZ[1] > magXYZ[1]){
        minXYZ[1]=magXYZ[1];
      }
      if(minXYZ[2] > magXYZ[2]){
        minXYZ[2]=magXYZ[2];
      }
      Serial.print("( ");
      Serial.print(maxXYZ[0]);
      Serial.print(" , ");
      Serial.print(magXYZ[0]);
      Serial.print(" , ");
      Serial.print(minXYZ[0]);
      Serial.print(" ),");

      Serial.print("( ");
      Serial.print(maxXYZ[1]);
      Serial.print(" , ");
      Serial.print(minXYZ[1]);
      Serial.print(" ),");
      
      Serial.print("( ");
      Serial.print(maxXYZ[2]);
      Serial.print(" , ");
      Serial.print(minXYZ[2]);
      Serial.print(" ),");
    }
    else{
      Wire.requestFrom(MAG_ADDR, 7 , true);
      magXYZ[1] =  ((int16_t)(Wire.read() | Wire.read()<<8));
      magXYZ[0] =  ((int16_t)(Wire.read() | Wire.read()<<8));    
      magXYZ[2] =  -((int16_t)(Wire.read() | Wire.read()<<8));
      float x=(magXYZ[0]-(maxX+minX)/2)*((maxX-minX+maxY-minY+maxZ-minZ)/6)*(2/(maxX-minX));
      float y=(magXYZ[1]-(maxY+minY)/2)*((maxX-minX+maxY-minY+maxZ-minZ)/6)*(2/(maxY-minY));
      float z=(magXYZ[2]-(maxZ+minZ)/2)*((maxX-minX+maxY-minY+maxZ-minZ)/6)*(2/(maxZ-minZ));
      Wire.endTransmission(true);
      magXYZ[0]=x;
      magXYZ[1]=y;
      magXYZ[2]=z;
    }
  }
}

void QuaternionMeasure::accSetup(){ // 가속도 센서 세팅
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
}

void QuaternionMeasure::getAccXYZ(float *accXYZ){ // 가속도 센서의 값을 accXYZ주소로 주입
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  if(ConfigMod==1){
    accXYZ[0] = -((int16_t)(Wire.read()<<8 | Wire.read()))/4096.;
    accXYZ[1] = -((int16_t)(Wire.read()<<8 | Wire.read()))/4096.;
    accXYZ[2] = -((int16_t)(Wire.read()<<8 | Wire.read()))/4096.+0.08;
  }else{
    accXYZ[0] = -((int16_t)(Wire.read()<<8 | Wire.read()))/4096.;
    accXYZ[1] = -((int16_t)(Wire.read()<<8 | Wire.read()))/4096.;
    accXYZ[2] = -((int16_t)(Wire.read()<<8 | Wire.read()))/4096.+0.08;
    float nom=sqrt(pow(accXYZ[0],2)+pow(accXYZ[1],2)+pow(accXYZ[2],2));
    accXYZ[0]/=nom;
    accXYZ[1]/=nom;
    accXYZ[2]/=nom;
  }
  
}

void QuaternionMeasure::getQuatFromAcc(float *accXYZ,float *accQ){ // 가속도 센서로부터 쿼터니언 회전값 계산
  accQ[0]=sqrt((-accXYZ[2]+1)/2);
  accQ[1]=-accXYZ[1]/sqrt(2*(-accXYZ[2]+1));
  accQ[2]=accXYZ[0]/sqrt(2*(-accXYZ[2]+1));
  accQ[3]=0;
}

void QuaternionMeasure::getQuatFromMagConfigrated(float *magXYZ_config,float *magQ){ // 자기장센서로부터 쿼터니언 회전값 계산
  float nom=pow(magXYZ_config[0],2)+pow(magXYZ_config[1],2);
  if(magXYZ_config[0]>=0){
    magQ[3]=-magXYZ_config[1]/(sqrt(2*(nom+magXYZ_config[0]*sqrt(nom))));
    magQ[0]=sqrt(nom+magXYZ_config[0]*sqrt(nom))/sqrt(2*nom);
  }else{
    magQ[0]=-magXYZ_config[1]/(sqrt(2*(nom-magXYZ_config[0]*sqrt(nom))));
    magQ[3]=sqrt(nom-magXYZ_config[0]*sqrt(nom))/sqrt(2*nom);
  }

}