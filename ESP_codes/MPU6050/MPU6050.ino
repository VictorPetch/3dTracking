#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

//___MPU Configs_________________________________________________________
// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;
// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D6;
const uint8_t sda = D7;
// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384; // +- 2g
const uint16_t GyroScaleFactor = 131; //+- 250 deg/seg
// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;
int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

//___Wifi Configs__________________________________________________________
const char* ssid = "602";
const char* password =  "sophie602notredame";
const uint16_t port = 4242;
WiFiUDP Udp;
unsigned int localUdpPort = port; 

//___Packet Configs________________________________________________________
double Ax, Ay, Az, Gx, Gy, Gz,T;
char incomingPacket[255];  //buffer
char replyPacket[] = "123451234512345123451234512345222";
char msg[50] {};
int iterator = 0;

void setup() {
  Serial.begin(9600);
  Serial.read();
  Wire.begin(sda, scl); //begins I2C protocol
  MPU6050_Init(); //Configures all registors
  WiFi.begin(ssid, password); //Initialize wifi
  Serial.println("Connecting to wifi");
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.print("Wifi Connected, IP address: ");
  Serial.println(WiFi.localIP());
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);  
}
 void loop()
{ 

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    //__receive incoming UDP packets_____
    Serial.printf("\nReceived %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = 0;
    Serial.printf("UDP packet contents: %s\n", incomingPacket);
    Serial.println("Sending registers contents");
    len = 0;
    
    //__send back a reply________________
    //IPAddress host(192, 168, 1, 19);
    while(len == 0){
      //_____Read registers______________
      Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
      //divide each with their sensitivity scale factor
      Ax = (double)AccelX/AccelScaleFactor;
      Ay = (double)AccelY/AccelScaleFactor;
      Az = (double)AccelZ/AccelScaleFactor;
      Gx = (double)GyroX/GyroScaleFactor;
      Gy = (double)GyroY/GyroScaleFactor;
      Gz = (double)GyroZ/GyroScaleFactor;
      T = (double)Temperature/340+36.53; //temperature formula
      //msg = String(Ax)+','+String(Ay)+','+String(Az)+','+
      //      String(Gx)+','+String(Gy)+','+String(Gz);
      Serial.print(T);Serial.print(" | ");Serial.println(Ay,2);
      
      //____Serialize into json_______________
      const size_t capacity = JSON_ARRAY_SIZE(6) + JSON_OBJECT_SIZE(1);
      DynamicJsonDocument doc(capacity);
      JsonArray A = doc.createNestedArray("A");
      A.add(int(Ax*100));
      A.add(int(Ay*100));
      A.add(int(Az*100));
      A.add(int(Gx));
      A.add(int(Gy));
      A.add(int(Gz));
      
      //____Send the packets__________________
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      serializeJson(doc, Udp);
      //Udp.write(doc, capacity);
      Udp.endPacket();
      delay(50);

      //__Check if it's enough________________
      int packetSize = Udp.parsePacket();
      if (packetSize) {
        len = Udp.read(incomingPacket, 255);
        Serial.println("Stopped sending packets");
      }
    }
  }
}



//______________Funções__________________________________________
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  int error = Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
