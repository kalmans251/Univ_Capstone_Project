#include "QuaternionMeasure.h";
#include "ConstSetting.h";
#include <Arduino.h>;
#include <Wire.h>;
#include <EEPROM.h>;

void QuaternionMeasure::magnetometerSetup(){
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
    
  Wire.beginTransmission(MAG_ADDR);   //CONT MODE 2
  Wire.write(0x0A);
  Wire.write(0b00010110); // 1 for 16 bit or 0 for 14 bit output, 0110 FOR CONT MODE 2 (X Hz?) 
  Wire.endTransmission(true);
  delay(100);
}

void QuaternionMeasure::getMagXYZ(float *magXYZ){
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
      magXYZ[1] =  Wire.read() | Wire.read()<<8;;
      magXYZ[0] =  Wire.read() | Wire.read()<<8;;    
      magXYZ[2] =  -(Wire.read() | Wire.read()<<8);;
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
      magXYZ[1] =  (Wire.read() | Wire.read()<<8);;
      magXYZ[0] =  (Wire.read() | Wire.read()<<8);;    
      magXYZ[2] =  -(Wire.read() | Wire.read()<<8);;
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

void QuaternionMeasure::accSetup(){
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
}

void QuaternionMeasure::getAccXYZ(float *accXYZ){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  if(ConfigMod==1){
    accXYZ[0] = -(Wire.read()<<8 | Wire.read())/4096.;
    accXYZ[1] = -(Wire.read()<<8 | Wire.read())/4096.;
    accXYZ[2] = -(Wire.read()<<8 | Wire.read())/4096.+0.08;
  }else{
    accXYZ[0] = -(Wire.read()<<8 | Wire.read())/4096.;
    accXYZ[1] = -(Wire.read()<<8 | Wire.read())/4096.;
    accXYZ[2] = -(Wire.read()<<8 | Wire.read())/4096.+0.08;
    float nom=sqrt(square(accXYZ[0])+square(accXYZ[1])+square(accXYZ[2]));
    accXYZ[0]/=nom;
    accXYZ[1]/=nom;
    accXYZ[2]/=nom;
  }
  
}

void QuaternionMeasure::getQuatFromAcc(float *accXYZ,float *accQ){
  accQ[0]=sqrt((-accXYZ[2]+1)/2);
  accQ[1]=-accXYZ[1]/sqrt(2*(-accXYZ[2]+1));
  accQ[2]=accXYZ[0]/sqrt(2*(-accXYZ[2]+1));
  accQ[3]=0;
}

void QuaternionMeasure::getQuatFromMagConfigrated(float *magXYZ_config,float *magQ){
  float nom=square(magXYZ_config[0])+square(magXYZ_config[1]);
  if(magXYZ_config[0]>=0){
    magQ[3]=-magXYZ_config[1]/(sqrt(2*(nom+magXYZ_config[0]*sqrt(nom))));
    magQ[0]=sqrt(nom+magXYZ_config[0]*sqrt(nom))/sqrt(2*nom);
  }else{
    magQ[0]=-magXYZ_config[1]/(sqrt(2*(nom-magXYZ_config[0]*sqrt(nom))));
    magQ[3]=sqrt(nom-magXYZ_config[0]*sqrt(nom))/sqrt(2*nom);
  }

}