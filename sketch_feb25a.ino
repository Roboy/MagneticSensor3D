#include <Wire.h>    
 
#define disk1 0x50
 
void setup(void)
{
  Serial.begin(115200);
  Wire.begin();  
  Serial.println("activating sensor");
  activateTLV(0b1011110, 0b00000101);
}
 
void loop(){
  delay(10);
  uint8_t data[3];
  while(digitalRead(SCL)){
  }
  readTLV(0b1011110, data);
  Serial.print(convertToMilliTesla(data[0]));
  Serial.print("\t");
  Serial.print(convertToMilliTesla(data[1]));
  Serial.print("\t");
  Serial.println(convertToMilliTesla(data[2]));
  }
 
void activateTLV(int deviceaddress, byte data) 
{
  Wire.beginTransmission(deviceaddress);
  Wire.write(0);
  Wire.write(data);
  Wire.endTransmission();
 
  delay(5);
}

float convertToMilliTesla(byte data){
  float mTs = 0;
  uint bitmask = 1;
  for(uint i=0; i<7; i++){
    mTs += (data&bitmask);
    bitmask <<= 1;
  }
  mTs -= (data&bitmask);
  return (mTs*1.56f);
}
 
void readTLV(int deviceaddress, uint8_t *data) 
{   
  Wire.requestFrom(deviceaddress,3); 
  data[0] = Wire.read(); 
  data[1] = Wire.read(); 
  data[2] = Wire.read(); 
}
