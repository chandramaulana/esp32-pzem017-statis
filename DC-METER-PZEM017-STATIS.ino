#include <AntaresESP32MQTT.h>
#include <ModbusMaster.h>

///////////////// Config Area ///////////////////
//-----------------------------------------------

#define WIFISSID "Telkom IoT"
#define PASSWORD "0987654321"

//#define ACCESSKEY "8b175a2ca85aa24e:11e5aae47db9ef5d"   // antares testing chandra
//#define projectName "monitoring-all"
//#define deviceName "device1"

#define ACCESSKEY "a08c0faa1b988b11:9ae97d4dee93a723"   // antares pomi
#define projectName "POMI"
#define deviceName "PM_POH_2"

const int publish_interval = 3000; // 5 second (send to antares)

float PZEMVoltage_cal = 1;
float PZEMCurrent_cal = 1;
float PZEMPower_cal = 1;
float PZEMEnergy_cal = 1;

float PZEMVoltage_offset = 0;
float PZEMCurrent_offset = 0;
float PZEMPower_offset = 0;
float PZEMEnergy_offset = 0;
//-----------------------------------------------
/////////////////////END/////////////////////////


#define p_button 0
#define p_button_led 27
#define PZEMSerial Serial2
#define MAX485_RE  14
#define MAX485_DE  14

static uint8_t pzemSlaveAddr = 0x01;
static uint16_t NewshuntAddr = 0x0000;

ModbusMaster node;

float PZEMVoltage, PZEMCurrent, PZEMPower, PZEMEnergy;

unsigned long startMillisPZEM;
unsigned long currentMillisPZEM;
const unsigned long periodPZEM = 1000;

unsigned long startMillisReadData;
unsigned long startMillis1;
float calibration_factor = 0;

AntaresESP32MQTT antares(ACCESSKEY);

void setup()
{
  delay(500);
  Serial.begin(115200);
  pinMode(p_button, INPUT);
  pinMode(p_button_led, OUTPUT);

  antares.setDebug(true);
  antares.wifiConnection(WIFISSID, PASSWORD);
  antares.setMqttServer();
  digitalWrite(p_button_led, HIGH);

  if (!digitalRead(p_button))
  {
    digitalWrite(p_button_led, HIGH);
    delay(3000);
    digitalWrite(p_button_led, LOW);

    if (!digitalRead(p_button))
    {
      resetEnergy();
      delay(1000);
      ESP.restart();
    }
  }
  init_pzem017();
}

void loop()
{
  read_pzem017();

  antares.checkMqttConnection();
  digitalWrite(p_button_led, HIGH);

  antares.add("V", String(PZEMVoltage, 2).toFloat());
  antares.add("Q", String(PZEMCurrent, 2).toFloat());
  antares.add("J", String(PZEMPower, 2).toFloat());
  antares.add("Wh", String(PZEMEnergy, 2).toFloat());
  antares.publish(projectName, deviceName);

  delay(10);
  digitalWrite(p_button_led, LOW);
  delay(publish_interval);
}


void preTransmission()
{
  if (millis() - startMillis1 > 5000)
  {
    digitalWrite(MAX485_RE, 1);
    digitalWrite(MAX485_DE, 1);
    delay(1);
  }
}

void postTransmission()
{
  if (millis() - startMillis1 > 5000)
  {
    delay(3);
    digitalWrite(MAX485_RE, 0);
    digitalWrite(MAX485_DE, 0);
  }
}

void setShunt(uint8_t slaveAddr)
{
  static uint8_t SlaveParameter = 0x06;
  static uint16_t registerAddress = 0x0003;
  uint16_t u16CRC = 0xFFFF;
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewshuntAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewshuntAddr));

  preTransmission();

  PZEMSerial.write(slaveAddr);
  PZEMSerial.write(SlaveParameter);
  PZEMSerial.write(highByte(registerAddress));
  PZEMSerial.write(lowByte(registerAddress));
  PZEMSerial.write(highByte(NewshuntAddr));
  PZEMSerial.write(lowByte(NewshuntAddr));
  PZEMSerial.write(lowByte(u16CRC));
  PZEMSerial.write(highByte(u16CRC));
  delay(10);
  postTransmission();
  delay(100);
}

void resetEnergy()
{
  uint16_t u16CRC = 0xFFFF;
  static uint8_t resetCommand = 0x42;
  uint8_t slaveAddr = pzemSlaveAddr;
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  preTransmission();
  PZEMSerial.write(slaveAddr);
  PZEMSerial.write(resetCommand);
  PZEMSerial.write(lowByte(u16CRC));
  PZEMSerial.write(highByte(u16CRC));
  delay(10);
  postTransmission();
  delay(100);
}

void changeAddress(uint8_t OldslaveAddr, uint8_t NewslaveAddr)
{
  static uint8_t SlaveParameter = 0x06;
  static uint16_t registerAddress = 0x0002;
  uint16_t u16CRC = 0xFFFF;
  u16CRC = crc16_update(u16CRC, OldslaveAddr);
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));
  preTransmission();
  PZEMSerial.write(OldslaveAddr);
  PZEMSerial.write(SlaveParameter);
  PZEMSerial.write(highByte(registerAddress));
  PZEMSerial.write(lowByte(registerAddress));
  PZEMSerial.write(highByte(NewslaveAddr));
  PZEMSerial.write(lowByte(NewslaveAddr));
  PZEMSerial.write(lowByte(u16CRC));
  PZEMSerial.write(highByte(u16CRC));
  delay(10);
  postTransmission();
  delay(100);
}

void init_pzem017()
{
  startMillis1 = millis();

  PZEMSerial.begin(9600, SERIAL_8N2, 16, 17);

  startMillisPZEM = millis();
  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  node.begin(pzemSlaveAddr, PZEMSerial);
  delay(1000);

  while (millis() - startMillis1 < 5000)
  {
    delay(500);
    Serial.print(".");
  }
  setShunt(pzemSlaveAddr);
  //changeAddress(0xF8, pzemSlaveAddr);
}

void read_pzem017()
{
  uint8_t result;
  result = node.readInputRegisters(0x0000, 6);
  if (result == node.ku8MBSuccess)
  {
    uint32_t tempdouble = 0x00000000;
    PZEMVoltage = node.getResponseBuffer(0x0000) / 100.0;
    PZEMCurrent = node.getResponseBuffer(0x0001) / 100.0;
    tempdouble =  (node.getResponseBuffer(0x0003) << 16) + node.getResponseBuffer(0x0002);
    PZEMPower = tempdouble / 10.0;
    tempdouble =  (node.getResponseBuffer(0x0005) << 16) + node.getResponseBuffer(0x0004);
    PZEMEnergy = tempdouble;


    ///calibrate offset and scaling////

    PZEMVoltage = (PZEMVoltage * PZEMVoltage_cal) + PZEMVoltage_offset;
    PZEMCurrent = (PZEMCurrent * PZEMCurrent_cal) + PZEMCurrent_offset;
    PZEMPower = (PZEMPower * PZEMPower_cal) + PZEMPower_offset;
    PZEMEnergy = (PZEMEnergy * PZEMEnergy_cal) + PZEMEnergy_offset;
  }
  else
  {
    PZEMVoltage = NAN;
    PZEMCurrent = NAN;
    PZEMPower = NAN;
    PZEMEnergy = NAN;
  }

  Serial.print("Vdc : "); Serial.print(PZEMVoltage); Serial.println(" V ");
  Serial.print("Idc : "); Serial.print(PZEMCurrent); Serial.println(" A ");
  Serial.print("Power : "); Serial.print(PZEMPower); Serial.println(" W ");
  Serial.print("Energy : "); Serial.print(PZEMEnergy, 3); Serial.println(" kWh ");
  Serial.println();
  Serial.println();
}
