#include <Wire.h>    

byte deviceaddress = 0b1011110;
int devicepin = 255;
  
void setup(void)
{
  byte cfgdata[3];
  byte* factory_settings;
  byte deviceaddress = 0b1011110;
  int devicepin = 255;
 // start serial 
  Serial.begin(115200);
 // start wire - make sure you are powering the sensor with 3.3V, SCL and SDA are connected
  Wire.begin();  
  Serial.println("---------Activating sensor---------");
  factory_settings = initTLV(deviceaddress, cfgdata, devicepin);
  Serial.println("---------Done configuring---------");
}
 
void loop(){
  delay(10);
  uint8_t data[3];
  readTLV_B_MSB(deviceaddress, data);
  Serial.print(convertToMilliTesla(data[0]));
  Serial.print("\t");
  Serial.print(convertToMilliTesla(data[1]));
  Serial.print("\t");
  Serial.println(convertToMilliTesla(data[2]));
  }
 
byte* initTLV(byte deviceaddress, byte* data, int devicepin) 
{
  bool ADDR_pin;
  byte IICAddr;
  byte setaddr;
  byte defaultaddr;
  byte regdata[10];
  byte cfgdata[3];

  
  if (devicepin == 255){
    ADDR_pin = 1;
    IICAddr  = 0;
    setaddr  = 0b1011110;
    Serial.print("No device power pin selected, asuming device is on and continuing with the default address: ");
    Serial.print(setaddr,BIN);
    Serial.println(" .");
    Serial.println("Triggering general reset, to make sure device is configured correctly");
    Wire.beginTransmission(0);
    Wire.write(0);
    
  }else{
    digitalWrite(devicepin,LOW);  // Make sure device is off
    ADDR_pin = bitRead(deviceaddress,6);
    IICAddr  = (~bitRead(deviceaddress,4)<<1)|(~bitRead(deviceaddress,2));
    setaddr  = (ADDR_pin<<6)|(~bitRead(IICAddr,1)<<4)|(1<<3)|(~bitRead(IICAddr,0)<<2)|(~ADDR_pin);
  }

  if (setaddr != deviceaddress){
      Serial.println("Invalid device address! Please check and try again.");
      return NULL;
    }else{
      Serial.print("Configuring device with address: ");
      Serial.print(setaddr,BIN);
      Serial.print(" (ADDR_pin = ");
      Serial.print(ADDR_pin,BIN);
      Serial.print(", IICAddr = ");
      Serial.print(IICAddr,BIN);
      Serial.println(" )");
  }
  

  if (ADDR_pin != 1){
    digitalWrite(SDA,LOW);
    digitalWrite(devicepin,HIGH); // Power on device while SDA low to set ADDR bit to 0
    delay(1);                     // At least during 200us
    digitalWrite(SDA,HIGH);
    defaultaddr = 0b0011111;
  }else{
    digitalWrite(devicepin,HIGH);
    delay(1);
    defaultaddr = 0b1011110;
  }

  Serial.println("Backing up initial register config");
  int reg = 0;
  Wire.requestFrom(defaultaddr,(uint8_t) 10); // Beginning first read (for backup)
  while(Wire.available()){
    regdata[reg] = Wire.read();
    reg++;
  }
  if ( reg != 10 ){   //Todo: Handle this as a true error and retry!
    Serial.print("ERROR: Data could not be read correctly and may be incomplete! Continuing, nevertheless ...  (reg = ");
    Serial.print(reg,DEC);
    Serial.println(" )");
  }
  
  // Begin config
  // Static initial config for now
    cfgdata[0] = (IICAddr<<5)|(regdata[7]&0b00011000)|0b010;  // Last 3 bits: INT/FAST/LP
    cfgdata[1] = regdata[8];
    cfgdata[2] = (0b010<<5)|(regdata[9]&0b11111);             
    // First 3 bits: Enable temp/Low power interval/Parity test

    // Calculate parity bit
    bool parity = bitRead(cfgdata[0]+cfgdata[1]+cfgdata[2],0);
    Serial.print("Setting parity bit to ");
    Serial.println(parity,BIN);
    bitWrite(cfgdata[0],7,parity);

    // Write config
    Serial.println("Writing config now ...");
    configureTLV(defaultaddr, cfgdata);
  // End config
  
  Serial.println("Checking new config ...");
  reg = 0;
  Wire.requestFrom(setaddr,(uint8_t) 10);
  while(Wire.available()){
    regdata[reg] = Wire.read();
    reg++;
  }
  if ( reg != 10 ){   //Todo: Handle this as a true error and retry!
    Serial.print("ERROR: Data could not be read correctly and may be incomplete! Continuing, nevertheless ...  (reg = ");
    Serial.print(reg,DEC);
    Serial.println(" )");
  }
 
  return regdata;
}

void configureTLV(byte deviceaddress, byte* data)
{
  Wire.beginTransmission(deviceaddress);
  Wire.write(0);
  for (int i = 0; i < sizeof(data)-1; i++){
    Wire.write(data[i]);
  }
  Wire.endTransmission();  
}

float convertToMilliTesla(byte data){
  float mTs = 0;
  byte bitmask = 1;
  for(int i=0; i<7; i++){
    mTs += (data&bitmask);
    bitmask <<= 1;
  }
  mTs -= (data&bitmask);
  return (mTs*1.56f);
}
 
void readTLV_B_MSB(int deviceaddress, uint8_t *data) 
{   
  // Read the first 3 registers only. Corresponding to the 8bit MSB values of the magnetic field
  Wire.requestFrom(deviceaddress,3); 
  data[0] = Wire.read(); 
  data[1] = Wire.read(); 
  data[2] = Wire.read(); 
}
