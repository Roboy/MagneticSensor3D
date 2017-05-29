#include <Wire.h>    

#define ndevices 3                  // Allways set this first
                                    
// Look in the device's user manual for allowed addresses! (Table 6)
byte deviceaddress[ndevices] = {0b1011110, 0b0001111, 0b0001011};
int devicepin[ndevices] = {31,35,39};
  
void setup(void)
{
  byte initcfg[3];
  byte* factory_settings[ndevices];
  pinMode(0,OUTPUT);
  digitalWrite(0,HIGH);

  for (int i = 0; i < ndevices; i++){             // Activate high current outputs to power the sensors
    if (devicepin[i] != 255){                     // Not if it already is being powered
      pinMode(devicepin[i], OUTPUT);              // Be careful, use a level converter or a device with 3V3 outputs <- Lol fuck this guy
      digitalWrite(devicepin[i],LOW);             // Make sure device is off
    }
  }
  
  Serial.begin(115200);                           // Start serial coms
  Wire.begin();                                   // Init I2C
  Serial.println("Ok, initialized I2C bus");
  Serial.println("---------Activating sensors--------");
  delay(1);                                       // Letting things settle
  factory_settings[0] = initTLV(deviceaddress[0], initcfg, devicepin[0]);    // Write device address and initial config
  factory_settings[1] = initTLV(deviceaddress[1], initcfg, devicepin[1]);
  factory_settings[2] = initTLV(deviceaddress[2], initcfg, devicepin[2]);
  Serial.println("---------Done configuring---------");
}
 
void loop(){
  delay(10);
  uint8_t data[3];
  uint8_t data2[3];
  uint8_t data3[3];
  readTLV_B_MSB(deviceaddress[0], data);
  readTLV_B_MSB(deviceaddress[1], data2);
  readTLV_B_MSB(deviceaddress[2], data3);
  Serial.print(convertToMilliTesla(data[0]));
  Serial.print("\t");
  Serial.print(convertToMilliTesla(data[1]));
  Serial.print("\t");
  Serial.print(convertToMilliTesla(data[2]));
  Serial.print("\t");
  Serial.print(convertToMilliTesla(data2[0]));
  Serial.print("\t");
  Serial.print(convertToMilliTesla(data2[1]));
  Serial.print("\t");
  Serial.print(convertToMilliTesla(data2[2]));
  Serial.print("\t");
  Serial.print(convertToMilliTesla(data3[0]));
  Serial.print("\t");
  Serial.print(convertToMilliTesla(data3[1]));
  Serial.print("\t");
  Serial.println(convertToMilliTesla(data3[2]));
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
    Serial.println("Triggering general reset to make sure device is configured correctly");
    Wire.beginTransmission(0); // Clock in address 0 to reset 'ALL' devices
    Wire.write(0b11111111);    // Very, very important to keep SDA line up or else address changes to 0b0011111
    Wire.endTransmission();
    
  }else{
    ADDR_pin = bitRead(deviceaddress,6);
    IICAddr  = (!bitRead(deviceaddress,4)<<1)|(!bitRead(deviceaddress,2));
    setaddr  = (ADDR_pin<<6)|(!bitRead(IICAddr,1)<<4)|(1<<3)|(!bitRead(IICAddr,0)<<2)|(1<<1)|(!ADDR_pin);
  }

  if (setaddr != deviceaddress){
      Serial.println("Invalid device address! Please check and try again.");
      return NULL;
    }else{
      Serial.print("Configuring device on pin ");
      Serial.print(devicepin);
      Serial.print(" with address: ");
      Serial.print(setaddr,BIN);
      Serial.print(" (ADDR_pin = ");
      Serial.print(ADDR_pin,BIN);
      Serial.print(", IICAddr = ");
      Serial.print(IICAddr,BIN);
      Serial.println(" )");
  }
  

  if (ADDR_pin != 1){
    Wire.end();
    pinMode(SDA,OUTPUT);
    digitalWrite(SDA,LOW);
    Serial.print("Activating 'El cacharro' ");
    Serial.print(devicepin);
    Serial.println(" (SDA Low)");
    digitalWrite(devicepin,HIGH); // Power on device while SDA low to set ADDR bit to 0
    delay(1);                     // At least during 200us
    digitalWrite(SDA,HIGH);
    Wire.begin();
    defaultaddr = 0b0011111;
  }else{
    if(devicepin != 255){
      digitalWrite(devicepin,HIGH);
      Serial.print("Activating 'El cacharro' ");
      Serial.println(devicepin);
    }
    delay(1);
    defaultaddr = 0b1011110;
  }

  Serial.println("Backing up initial register config");
  int reg = 0;
//  Serial.println("But first checking correct address by bruteforcing the entire range. Fuck you angry pixies!");
//  for (byte tmpaddr = 1; tmpaddr <=127; tmpaddr++){
//    Wire.requestFrom(tmpaddr,(uint8_t) 10);
//    if(Wire.available()){
//      Serial.print("Address found: ");
//      Serial.println(tmpaddr,BIN);
//      break;
//    }
//  }
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

    // Calculate parity bit     (well doesen't work for now so fuck it)
    // bool parity = bitRead(cfgdata[0]+cfgdata[1]+cfgdata[2],0);
    // Serial.print("Setting parity bit to ");
    // Serial.println(parity,BIN);
    // bitWrite(cfgdata[0],7,parity);

    // Write config
    Serial.println("Writing config now ...");
    configureTLV(defaultaddr, cfgdata, sizeof(cfgdata));
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

void configureTLV(byte deviceaddress, byte* data, int count)
{
  Wire.beginTransmission(deviceaddress);
  Wire.write(0);
  for (int i = 0; i < count; i++){
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
