#include <Arduino.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#include "EEPROM.h"
#include <mcp_can.h>
#include <SPI.h>

#include "Wire.h"

MPU6050 accelgyro;

MCP_CAN CAN(2);                                      // Set CS to pin 2

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} Offsets_t;

int16_t ax, ay, az;
int16_t gx, gy, gz;
Offsets_t Offsets,GOffsets;
int16_t sax, say, saz, sgx, sgy, sgz;
uint8_t CAN_Message[8], CAN_Message2[8];
uint8_t CANID, Status_Out;

#define LED_PIN 13

void Can_Init(void)
{
  EEPROM.get(50,CANID);
  if ((CANID ==0) || (CANID==0xFF))
  {
    CANID = 0x30;
  }
  if(CAN_OK == CAN.begin(CAN_1000KBPS))                   // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS init ok!");
    }
    else
    {
        Serial.println("CAN init fail");
        Serial.println("Init CAN BUS again...");
        delay(1000);
        Can_Init();
    }
    Serial.print("Using Can ID: 0x"); Serial.println(CANID,HEX);
}

void Send_Data_Serial(void)
{
  Serial.print((ax+Offsets.x)/16384.0,4); Serial.print("\t");
  Serial.print((ay+Offsets.y)/16384.0,4); Serial.print("\t");
  Serial.println((az+Offsets.z)/16384.0,4);
}


/// Default base CANID is 0x30, accelero data on 0x30, gyro data on 0x31.
/// See the attached DBC, Multipliers are based on 2G and 250deg/s ranges.
/// CAN Status byte:  bit 0 - Accelero Offsets Loaded
///                   bit 1 - Gyro Offsets Loaded
///                   bit 2 - Accelerometer Initialization Successful
///                   bit 3 - 0
///                   bit 4 - 0
///                   bit 5 - 0
///                   bit 6 - Offseting done in the actual power cycle
///                   bit 7 - Offseting reqest saved for next power cycle

void Store_CanID(uint8_t ID)
{
  Serial.println("Updating CANID in EEPROM...");
  EEPROM.write(50,ID);
}

void CAN_Send()
{
  //CANID = 0x30;
  digitalWrite(LED_PIN,LOW);
  CAN_Message[0]= ax%256; CAN_Message[1] = ax/256;
  CAN_Message[2]= ay%256; CAN_Message[3] = ay/256;
  CAN_Message[4]= az%256; CAN_Message[5] = az/256;
  CAN_Message[6] = Status_Out;

  CAN_Message2[0]= gx%256; CAN_Message2[1] = gx/256;
  CAN_Message2[2]= gy%256; CAN_Message2[3] = gy/256;
  CAN_Message2[4]= gz%256; CAN_Message2[5] = gz/256;
  CAN_Message2[6] = Status_Out;

  uint8_t Can_St = CAN.sendMsgBuf(CANID, 0, 7, CAN_Message);
  if (Can_St != CAN_OK)
  {
    digitalWrite(LED_PIN,HIGH);
    //Serial.print("Can Error:");Serial.println(Can_St);
  }
  Can_St = CAN.sendMsgBuf(CANID+1, 0, 7, CAN_Message2);
  if (Can_St != CAN_OK)
  {
    //Serial.print("Can Error:");Serial.println(Can_St);
  }
}

void Read_Acc (void)
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //Serial.println(millis());
}

void Calc_Offsets()
{
  for (int i=0;i<100;i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sax = sax + ax;   say = say + ay;     saz= (saz + (az-16384));
    sgx = sgx + gx;   sgy = sgy + gy;     sgz = sgz + gz;
  }
  Offsets.x = sax/100.0;     Offsets.y = say/100.0;     Offsets.z = saz/100.0;
  GOffsets.x = sgx/100.0;    GOffsets.y = sgy/100.0;    GOffsets.z = sgz/100.0;
  EEPROM.put(0, Offsets);
  EEPROM.put(60, GOffsets);
  Serial.print("ACC Offsets: "); Serial.print(Offsets.x); Serial.print("/");
  Serial.print(Offsets.y); Serial.print("/");Serial.println(Offsets.z);
  Serial.print("Gyro Offsets: "); Serial.print(GOffsets.x); Serial.print("/");
  Serial.print(GOffsets.y); Serial.print("/");Serial.println(GOffsets.z);
}

void Offs_Load(void)
{
  EEPROM.get(0, Offsets);
  Status_Out = Status_Out || 0b00000001;        //Flag Indicating Accelero Offsets Loaded
  EEPROM.get(60, GOffsets);
  Status_Out = Status_Out || 0b00000010;        //Flag Indicating Gyro Offsets Loaded
  if(EEPROM.read(52)==1) {
    Serial.println("Offseting...");
    Calc_Offsets();
    EEPROM.write(52, 0);
    Status_Out = Status_Out || 0b01000000;      //Flag indicating Offseting was done in this cycle
  }
}


/// To Initiate Offseting turn the sensor to the side of Y axis, power up the device
/// wait 20s and power off. Fix the sensor to the desired location, power on the device
/// offseting will be done and saved to the EEPROM.

void Offs_Check(void)
{
  uint8_t Valid;
  for (uint8_t i=0; i<100; i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if ((ay>10000) || (ay<-10000)) Valid++;
  }
  if (Valid>70)
  {
    EEPROM.put(52,1);
    Serial.println("Offset request for next startup...");
    Status_Out = (Status_Out || 0b10000000);                //Set flag to indicate Offseting request
  }
}

void setup()
{

    Wire.begin();

    Wire.setClock(400000L);

    Serial.begin(115200);

    // initialize Accelero
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    accelgyro.setIntDataReadyEnabled(true);

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    if (accelgyro.testConnection())
    {
      Status_Out = Status_Out || 0b00000100;                //Flag indicating Accelerometer connection OK
    }
    // use the code below to change accel/gyro offset values

    Serial.println("Updating internal sensor offsets...");

    Offs_Check();

    //Init CAN Interface

    Can_Init();
    //Store_CanID(CANID);

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    delay(1000);
    CAN_Send();                                             //Send One Message for STATUS info
}

void loop()
{

  if(accelgyro.getIntStatus()%2)    //poll 1ms Data Ready Interrupt on sensor
  {
      Read_Acc();                   // Read Values from Accelero
      //Send_Data_Serial();           // Accelero values to serial bus
      CAN_Send();                   // Send out CAN data
      //Serial.println(millis());
  }
}
