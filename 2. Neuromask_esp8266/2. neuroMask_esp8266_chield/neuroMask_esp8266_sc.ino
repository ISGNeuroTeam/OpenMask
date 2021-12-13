#include <Wire.h>
#include <heartRate.h>
#include <MAX30105.h>
#include <Ticker.h>
#include <FaBo9Axis_MPU9250.h>
#include "EEPROM.h"
#include "DFRobot_CCS811.h"
#include "SparkFunBME280.h"
#include "Adafruit_HDC1000.h"
#include <Adafruit_NeoPixel.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <FIR.h>
FIR<int, 103> fir_lp;
FIR<int, 103> fir_lp_1;

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

#define DEBUG false
//#define RESET_EEPROM false // uncomment to clear eeprom

//#define WEMOS

#ifdef WEMOS
#define BUTTONPIN 13
#define POWERPIN  5
#else
#define BUTTONPIN D7
#define POWERPIN  D5
// sensor keys


#endif

bool SLEEPMODE  = false;

// eeprom adress for save port host
#define IP_ADDRESS 0x00
#define PORT_ADDRESS 0x100
#define ASAP_ADDRESS 0x200


//ESP32AnalogRead adc; // adc conversion with calibration
#define VOLTAGE_BAT_PIN  A0
#define VOLTAGE_KOEF (100 + 210)/210  //(R1+R2)/R2
uint8_t BATPERCENTVALUE  = 0;  // save percent batery level

#define LEDS_COUNT  2
#define LEDS_PIN  12

Adafruit_NeoPixel pixels(LEDS_COUNT, LEDS_PIN, NEO_GRB + NEO_KHZ800);

#define CO2 0x10
#define O2 0x11
#define T_BODY 0x12
#define HUM 0x13
#define TVOC 0x14
#define PRESS 0x15
#define T_OUT 0x16
#define CARDIO 0x17
#define AX 0x18
#define AY 0x19
#define AZ 0x1A
#define GX 0x1B
#define GY 0x1C
#define GZ 0x1D
#define MX 0x1E
#define MY 0x1F
#define MZ 0x20
#define SPO2_RED 0x21
#define SPO2_IR 0x22
#define SPO2_GREEN 0x23
#define SPO2 0x24
#define ALT 0x25
#define HR 0x26
#define STEP 0x27
#define BAT_PERCENT 0x28
#define TVOCEXT 0x29
#define CO2EXT 0x2A
#define HUMEXT 0x2B



// received commands keys

#define STARTSENDDATA     0x50
#define STOPSENDDATA      0x51
#define SETFREQENCY       0x52
#define WITHPROCESSING    0x53
#define GETCONFIGSENSORS  0x54
#define GETMACADRESS      0x55





WiFiManager wm;
WiFiManagerParameter custom_address; // global param ( for non blocking w params )
WiFiManagerParameter custom_port; // global param ( for non blocking w params )
WiFiManagerParameter asAP; // global param ( for non blocking w params )


static const uint16_t startOfPacket =  0xAAF0;
static const uint16_t endOfPacket =  0xAAF1;

#define SENSCOUNT 22
#define DEFAULT_FREQ_HZ 100


Ticker sender;

Ticker batreader;

Ticker batShower;


volatile uint8_t sensReadCount = 0;
uint8_t sensorsActive[SENSCOUNT];

MAX30105 particleSensor; // sensor init

FaBo9Axis imu;  // Gyroscop Acellerometr

DFRobot_CCS811 CCS811;  // tvoc co2
DFRobot_CCS811 CCS811EXT(&Wire, 0x5B);  // tvoc co2 external sensor

BME280 bme;
BME280_SensorMeasurements measurements; // presure  & Humidity sens

Adafruit_HDC1000 hdc = Adafruit_HDC1000(); // external & Humidity sens

#define SEALEVELPRESSURE_HPA (1013.25) // Average sea-level pressure from wiki

typedef struct {
  uint8_t sensorId;   // id sensor
  float value;   // value
} structSensor;  // sensor struct

typedef struct {
  uint16_t dataLength;  // length data without start stop bits and self
  uint16_t timeStamp[2]; // timesatmp milliseconds from boot
  structSensor sensors[SENSCOUNT];  // array of sensors struct
} structPacket;  // struct of packet to send client

structPacket packet;

typedef struct {
  char modulName[10];  // name
  void (*functionRead)(void);  // coomand to read data
  bool enable;  // enable or disable reading
} structDevice; // struct of modul with sesors

void imuSensRead();
void practicleRead();
void readSensors();
void TvocCo2Read();
void TvocCo2ReadExt();
void HumiPressRead();
void HumiExtRead();
void VoltageRead();

structDevice devices[] = {  {"Imu", imuSensRead,   true},
  {"SpO2", practicleRead, true},
  {"CO2Tvoc", TvocCo2Read, true},
  {"ExCO2Tvoc", TvocCo2ReadExt, true},
  {"HumiPress", HumiPressRead, true},
  {"HumiExt", HumiExtRead, true},
  {"Voltage", VoltageRead, true}
};

// WiFi network name and password:
//const char * networkName = "";
//const char * networkPswd = "";

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address

String udpAddress = "10.77.2.36";
int udpPort = 3333;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;


float filter(float inputVal) {
  //implement the IIR filter
  //reference: The Scientist and Engineer's Guide to Digital Signal Processing By Steven W. Smith, Ph.D.
  //  http://www.dspguide.com/ch19/2.htm
  //This is a "single pole" filter.

  //x is the decay between adjacent samples. Choose any number between 0 and 1 (except 1).
  const float x = 0.86;
  static float outputVal = 0; //make it static so it's stored for the function to use again the next time it's called

  outputVal = outputVal * x + inputVal * (1 - x);
  return outputVal;
}

void writeStringToEEPROM(int addrOffset, String strToWrite)
{
  byte len = strToWrite.length();
  EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++)
  {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
}
String readStringFromEEPROM(int addrOffset)
{
  int newStrLen = EEPROM.read(addrOffset);
  char data[newStrLen + 1];
  for (int i = 0; i < newStrLen; i++)
  {
    data[i] = EEPROM.read(addrOffset + 1 + i);
    //Serial.print(data[i]);
  }
  data[newStrLen] = '\0'; // !!! NOTE !!! Remove the space between the slash "/" and "0" (I've added a space because otherwise there is a display bug)
  return String(data);
}

void disableDeviceByName(char * modulName)
{
  for (int i = 0; i < sizeof(devices) / sizeof(structDevice); i++ )
  {

    if (!memcmp(devices[i].modulName, modulName, strlen(modulName)))
    {
#if DEBUG
      Serial.println("Modul " + String(modulName) + " disable");
#endif
      devices[i].enable = false;
    }
  }
}

void timerSenderStart(int freq) {
#if DEBUG
  Serial.printf("Timer value is: %f\n", float(1) / freq);
#endif
  sender.attach(float(1) / freq, readSensors);
}

void timerSenderStop()
{
  sender.detach();
}

void timerSenderChangeFreq(int freq) // change frequncy timer
{
  timerSenderStop();
  timerSenderStart(freq);
}

void addDataSensToPacket(uint8_t sensorId, float value) // add sensor data to packet
{
  int sensIndex = (packet.dataLength /  (sizeof(uint8_t) + sizeof(float))) % SENSCOUNT;

  packet.sensors[sensIndex].sensorId = sensorId;
  packet.sensors[sensIndex].value = value;
  packet.dataLength += (sizeof(uint8_t) + sizeof(float));

}

void clearPacket()  // clear packet
{

  memset(&packet, 0, sizeof(packet));
}



bool VoltageInit()
{
  //adc.attach(VOLTAGE_BAT_PIN);
  return true;

}


bool practicleSensInit()
{
  // Initialize sensor
  if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    disableDeviceByName("SpO2");
    return false;
  }

  //Setup to sense up to 18 inches, max LED brightness
  byte ledBrightness = 0x2A; //Options: 0=Off to 255=50mA
  byte sampleAverage = 2; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 3200; //Options:SENSCOUNT 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 69; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  // default value (byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);
  return true;
}

bool imuSensInit()
{
  if (imu.begin()) {
    return true;
  }
  else {
    Serial.println("Imu was not found. Please check wiring/power.");
    disableDeviceByName("Imu");

    return false;
  }
}

bool TvocCo2Init()
{
  if (CCS811.begin() != 0) {
    Serial.println("failed to init chip, please check if the chip connection is fine");
    disableDeviceByName("CO2Tvoc");
    return false;

  }
  else
  {
    return true;
  }
}

bool TvocCo2InitExt()
{
  if (CCS811EXT.begin() != 0) {
    Serial.println("failed to init ccs811 external chip, please check if the chip connection is fine");
    disableDeviceByName("ExCO2Tvoc");
    return false;
  }
  else
  {
    return true;
  }
}

bool HumiExtSensInit()
{
  if (!hdc.begin()) {
    Serial.println("Couldn't find sensor HDC100x , check wiring!");
    disableDeviceByName("HumiExt");
    return false;
  }
  return true;
}


bool pressureHumiSensInit()
{
  bme.setI2CAddress(0x77);
  if (bme.beginI2C() == false) {
    Serial.println("Could not find a valid BME285 sensor, check wiring!");
    disableDeviceByName("HumiPress");
    return false;
  }
  bme.setStandbyTime(6);
  bme.readAllMeasurements(&measurements);
  bme.setReferencePressure(101325);
  delay(10);
  return true;
}

void readBatPercent()
{
  float batVoltage = analogRead(A0);

  float max_adc = 730;
  float min_adc = 530;
  //Serial.println(batVoltage);


  //BATPERCENTVALUE = batVoltage;

  if (batVoltage > max_adc)
  {
    BATPERCENTVALUE = 100;
  }
  else if (batVoltage < min_adc)
  {
    BATPERCENTVALUE = 0;
  }
  else
  {
    BATPERCENTVALUE  = map(batVoltage, min_adc, max_adc, 0, 100);
  }

}

void VoltageRead()
{
  static uint32_t lastTimeSend = millis();
  if (millis() - lastTimeSend > 1000) // send each seconds
  {
    addDataSensToPacket(BAT_PERCENT, BATPERCENTVALUE);
    Serial.println(BATPERCENTVALUE);
    sensReadCount += 1;
    lastTimeSend = millis();
  }

}

void HumiExtRead()
{
  float hum, temp;
  if (hdc.readHumidityAndTemp(&hum, &temp))
  {
    addDataSensToPacket(HUMEXT, hum);
    sensReadCount += 1;
  }


}

void HumiPressRead()
{
  if (!bme.isMeasuring())
  {
    bme.readAllMeasurements(&measurements);

    float alt =  ((float) - 44330.77) * (pow(((float)measurements.pressure / (float)bme.getReferencePressure()), 0.190263) - (float)1);
    //Serial.println(alt);

    addDataSensToPacket(T_OUT, measurements.temperature);
    addDataSensToPacket(PRESS, measurements.pressure);
    addDataSensToPacket(HUM, measurements.humidity);
    addDataSensToPacket(ALT, alt);
    sensReadCount += 4;
  }


}



void TvocCo2Read() {
  if (CCS811.checkDataReady() == true) {
    float co2 = CCS811.getCO2PPM();
    float ppm = CCS811.getTVOCPPB();
    addDataSensToPacket(TVOC, ppm);
    addDataSensToPacket(CO2, co2);
    sensReadCount += 2;

  } else {


    return;
  }

}

void TvocCo2ReadExt() {

  if (CCS811EXT.checkDataReady() == true) {
    float co2 = CCS811EXT.getCO2PPM();
    float ppm = CCS811EXT.getTVOCPPB();
    addDataSensToPacket(TVOCEXT, ppm);
    addDataSensToPacket(CO2EXT, co2);
    sensReadCount += 2;

  } else {
    return;
  }

}

void imuSensRead()
{
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float temp;
  sensReadCount += 10;

  imu.readAccelXYZ(&ax, &ay, &az);
  imu.readGyroXYZ(&gx, &gy, &gz);
  imu.readMagnetXYZ(&mx, &my, &mz);
  imu.readTemperature(&temp);
  addDataSensToPacket(AX, ax);
  addDataSensToPacket(AY, ay);
  addDataSensToPacket(AZ, az);
  addDataSensToPacket(GX, gx);
  addDataSensToPacket(GY, gy);
  addDataSensToPacket(GZ, gz);
  addDataSensToPacket(MX, mx);
  addDataSensToPacket(MY, my);
  addDataSensToPacket(MZ, mz);
  addDataSensToPacket(T_BODY, temp);
#if DEBUG
  //Serial.println(ax);
#endif
#if DEBUG
  //Serial.println(gx);
#endif
#if DEBUG
  //Serial.println(temp);
#endif
}



float getBPM(float value)
{
  static long lastBeat = 0; //Time at which the last beat occurred

  if (checkForBeat(value) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    float beatsPerMinute = 60 / (delta / 1000.0);

    return filter(beatsPerMinute);

  }
  return -1;
}

void practicleRead() {
  static int counter = 0;
  particleSensor.check();
  if (!particleSensor.available())
  {

#if DEBUG
    Serial.println("no data");
#endif
    return;
  }
  float red, ir, green;

  red = particleSensor.getFIFORed();
  ir = particleSensor.getFIFOIR();
  green = particleSensor.getFIFOGreen();
  sensReadCount += 3;
  particleSensor.nextSample();
  addDataSensToPacket(SPO2_RED, red);
  addDataSensToPacket(SPO2_IR, ir);
  addDataSensToPacket(SPO2_GREEN, green);


  float clean_ir = fir_lp.processReading(ir);
  float clean_red = fir_lp_1.processReading(red);
  if (ir < 50000)
    return;
  float beatsPerMinute = getBPM(clean_ir);

  if (beatsPerMinute > 30 && beatsPerMinute < 220 )
  {

    addDataSensToPacket(HR, beatsPerMinute);
    sensReadCount++;

  }

}
uint8_t activeSensorCounter()
{
  uint8_t counter = 0;

  for (uint8_t i = 0; i < SENSCOUNT; i++)
  {
    if (sensorsActive[i] != 0)
    {
      counter++;
    }
  }
  return counter;
}
uint8_t scanModules()
{
  uint8_t counter = 0; // sensors counter

  if (imuSensInit())
  {
    sensorsActive[counter++] =  AX;
    sensorsActive[counter++] =  AY;
    sensorsActive[counter++] =  AZ;
    sensorsActive[counter++] =  GX;
    sensorsActive[counter++] =  GY;
    sensorsActive[counter++] =  GZ;
    sensorsActive[counter++] =  MX;
    sensorsActive[counter++] =  MY;
    sensorsActive[counter++] =  MZ;
    sensorsActive[counter++] =  T_BODY;
  }
  if (practicleSensInit())
  {
    sensorsActive[counter++] = SPO2_RED;
    sensorsActive[counter++] = SPO2_IR;
    sensorsActive[counter++] = SPO2_GREEN;
  }

  if (TvocCo2Init())
  {
    sensorsActive[counter++] = TVOC;
    sensorsActive[counter++] = CO2;
  }
  if (TvocCo2InitExt())
  {
    sensorsActive[counter++] = TVOCEXT;
    sensorsActive[counter++] = CO2EXT;
  }


  if (pressureHumiSensInit())
  {
    sensorsActive[counter++] = PRESS;
    sensorsActive[counter++] = HUM;
    sensorsActive[counter++] = T_OUT;
    sensorsActive[counter++] = ALT;
  }

  if (HumiExtSensInit())
  {
    sensorsActive[counter++] = HUMEXT;
  }
  if (VoltageInit())
  {
    sensorsActive[counter++] = BAT_PERCENT;
  }

  return counter;
}

String getParam(String name) {
  //read parameter from server, for customhmtl input
  String value;
  if (wm.server->hasArg(name)) {
    value = wm.server->arg(name);
  }
  return value;
}

void saveParamCallback() {

  //#if DEBUG
  Serial.println("[CALLBACK] saveParamCallback fired");
  Serial.println("PARAM hostip = " + getParam("hostip"));
  Serial.println("PARAM hostport = " + getParam("hostport"));
  Serial.println("PARAM asAP = " + getParam("asAP"));
  //#endif

  writeStringToEEPROM(IP_ADDRESS, getParam("hostip"));
  writeStringToEEPROM(PORT_ADDRESS, getParam("hostport"));
  writeStringToEEPROM(ASAP_ADDRESS, getParam("asAP"));
  EEPROM.commit();
}



//wifi event handler
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case WIFI_EVENT_SOFTAPMODE_STACONNECTED:
      {
#if DEBUG
        Serial.print("Client connected! IP address: ???");
#endif
        if (!connected)
          udp.begin(udpPort);
        connected = true;
        //send_sensors();
      }
      break;

    case WIFI_EVENT_STAMODE_GOT_IP:
      {
        //When connected set
#if DEBUG
        Serial.print("WiFi connected! IP address: ");
#endif
#if DEBUG
        Serial.println(WiFi.localIP());
#endif
        //initializes the UDP state
        //This initializes the transfer buffer
        udp.begin(udpPort);
        connected = true;
        send_sensors();
      }
      break;
    case WIFI_EVENT_STAMODE_DISCONNECTED:
      {
#if DEBUG
        Serial.println("WiFi lost connection");
#endif
        connected = false;
        break;
      }
    default: break;
  }
}

void startConfigAPServer()
{
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  //WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);

  WiFi.onEvent(WiFiEvent); //register event handler

  int customFieldLength = 40;
  const char* custom_text_str = "<label for='hostip'>Host IP</label><input type='text' name='hostip'><br>";
  const char* custom_port_str = "<br/><label for='hostport'>Host port</label><input type='number' name='hostport'><br>";
  const char* custom_asAP = "<br/><label for='asAP'>As Access Point</label><input type='checkbox' id='asAP' name='asAP'><br>";
  new (&custom_address) WiFiManagerParameter(custom_text_str); // custom html input
  new (&custom_port) WiFiManagerParameter(custom_port_str);
  new (&asAP) WiFiManagerParameter(custom_asAP);

  wm.addParameter(&custom_address);
  wm.addParameter(&custom_port);
  wm.addParameter(&asAP);
  wm.setSaveParamsCallback(saveParamCallback);

  std::vector<const char *> menu = {"wifi", "info", "param", "sep", "restart", "exit"};
  wm.setMenu(menu);

  // set dark theme
  wm.setClass("invert");


  wm.setConfigPortalBlocking(false); // disable blocking

  //wm.setConfigPortalTimeout(90);
  //wm.resetSettings();
  bool res;

  res = wm.autoConnect(); // auto generated AP name from chipid
  //res = wm.autoConnect("AutoConnectAP"); // anonymous ap
  //res = wm.autoConnect("AutoConnectAP", "password"); // password protected ap


  if (!res) {
    Serial.println("Configportal running");

  }
  else {
    //if you get here you have connected to the WiFi
    Serial.println("NeuroMask connected");

  }
}

void responseCommand(uint8_t * packet, int len)
{
  uint8_t commandId =  packet[2];
  float param;
  *((uint8_t *)(&param) + 3) = packet[3];
  *((uint8_t *)(&param) + 2) = packet[4];
  *((uint8_t *)(&param) + 1) = packet[5];
  *((uint8_t*)(&param) + 0)  = packet[6];

  switch (commandId)
  {
    case GETCONFIGSENSORS:
      send_sensors();
#if DEBUG
      Serial.print("Get config sensors command received");
#endif
      break;

    case STARTSENDDATA:
      timerSenderStart(DEFAULT_FREQ_HZ);
#if DEBUG
      Serial.print("Start sensors data sending command received");
#endif
      break;

    case STOPSENDDATA:
      timerSenderStop();
#if DEBUG
      Serial.print("Stop sensors data sending command received");
#endif
      break;

    case SETFREQENCY:
      timerSenderStart(param);
#if DEBUG
      Serial.printf("Set frequency data sending command received, freq=%f", param);
#endif
      break;

    case GETMACADRESS:
      sendMacAddress();
#if DEBUG
      Serial.print("Get mac address command received");
#endif
      break;


  }

}

void readIncomingUdp()
{
  if (connected)
  {

    int packetSize = udp.parsePacket();

    if (packetSize)
    {
      uint8_t incomingPacket[packetSize];  // buffer for incoming packets
      // receive incoming UDP packets
#if DEBUG
      Serial.printf("Received %d bytes from %s, port %d\n", packetSize, udp.remoteIP().toString().c_str(), udp.remotePort());
#endif
      int len = udp.read(incomingPacket, packetSize);
      udpAddress = udp.remoteIP().toString().c_str();
      udpPort = udp.remotePort();
      if (len == 9)
      {
#if DEBUG
        Serial.printf("Received command %02x and value %02x%02x%02x%02x\n", incomingPacket[2], incomingPacket[3], incomingPacket[4], incomingPacket[5], incomingPacket[6]);
#endif
        responseCommand(incomingPacket, len);
      }


#if DEBUG
      Serial.printf("UDP packet contents: %s\n", incomingPacket);
#endif
    }
  }
}

void offLedStatus()
{
  pixels.clear();
  pixels.show();
}

void showLedStatus()
{


  if (BATPERCENTVALUE > 50)
  {
    pixels.setPixelColor(0, pixels.Color(0, 200, 0));
    pixels.setPixelColor(1, pixels.Color(0, (BATPERCENTVALUE - 50) * 2, 0));
  }
  else if (BATPERCENTVALUE > 10)
  {
    pixels.setPixelColor(0, pixels.Color(0, BATPERCENTVALUE * 2, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  }
  else
  {
    pixels.setPixelColor(0, pixels.Color(BATPERCENTVALUE, 0, 0));
    pixels.setPixelColor(1, pixels.Color(BATPERCENTVALUE, 0, 0));
  }
  pixels.show();
  batShower.once(5, offLedStatus);
}


void lowPassFilterInit()
{
  int coef_lp[103] = { -67, -257, -146, -240, -276, -327, -368, -403, -426, -436, -431, -408, -369,
                       -313, -242, -159, -68, 26, 119, 204, 276, 331, 362, 368, 346, 296, 220, 121,
                       5, -121, -250, -372, -477, -557, -602, -605, -560, -464, -315, -116, 129, 414,
                       728, 1061, 1399, 1730, 2038, 2311, 2537, 2707, 2811, 2847, 2811, 2707, 2537, 2311,
                       2038, 1730, 1399, 1061, 728, 414, 129, -116, -315, -464, -560, -605, -602, -557,
                       -477, -372, -250, -121, 5, 121, 220, 296, 346, 368, 362, 331, 276, 204, 119, 26,
                       -68, -159, -242, -313, -369, -408, -431, -436, -426, -403, -368, -327, -276, -240,
                       -146, -257, -67
                     };
  fir_lp.setFilterCoeffs(coef_lp);
  fir_lp_1.setFilterCoeffs(coef_lp);

}

void setup() {
  pinMode(POWERPIN, OUTPUT);
  digitalWrite(POWERPIN, HIGH);

  // Initilize hardware serial:
  Serial.begin(115200);
#if DEBUG
  Serial.println("starting");
#endif

#ifdef WEMOS
  Wire.begin(SDA, SCL);
#endif
  //Wire.begin();
  EEPROM.begin(1000);
  delay(100);

  lowPassFilterInit();
  clearPacket(); // clear packet for sensors
  VoltageInit(); // init ADC for mesure analog input
  readBatPercent();

  batreader.attach(0.01, readBatPercent);  // start timer for mesure bat voltage

  pixels.begin();
  pixels.setBrightness(5);

  showLedStatus();


  pinMode(BUTTONPIN, INPUT);
  while (digitalRead(BUTTONPIN) == HIGH) {
    Serial.println("HIGH");
    delay(500);
    ESP.wdtFeed();
  }
  String isAP = readStringFromEEPROM(ASAP_ADDRESS);
  Serial.println("asAP param is: = " + isAP);
  if (isAP == "on")
  {
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    //WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);
    uint8_t mac[6];
    WiFi.macAddress(mac);
    String macId = String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);
    macId.toUpperCase();
    WiFi.onEvent(WiFiEvent); //register event handler
    WiFi.softAP(String("NeuroMaskAP_") + macId , "password");
    //Serial.println(String("NeuroMaskAP_") + String(mac[3], HEX) +String(mac[4], HEX) +String(mac[5], HEX));
    IPAddress myIP = WiFi.softAPIP();
  }
  else
  {
    startConfigAPServer();
    String ip = readStringFromEEPROM(IP_ADDRESS);  // read data from eeprom
    String port = readStringFromEEPROM(PORT_ADDRESS);  // read data from eeprom
    //#if DEBUG
    Serial.println("EEPROM PARAM hostip = " + ip);
    Serial.println("EEPROM PARAM hostport = " + String(port));
    //#endif
    if (ip != ""  && atoi(port.c_str()) > 0 && atoi(port.c_str()) < 65536)
    {
      udpAddress = ip;
      udpPort = atoi(port.c_str());

    }
  }



#if DEBUG
  Serial.println("PARAM hostip = " + udpAddress);
  Serial.println("PARAM hostport = " + String(udpPort));
#endif
  memset(sensorsActive, 0, SENSCOUNT); // clear array of activity sensors

  if (scanModules())
  {
    timerSenderStart(DEFAULT_FREQ_HZ);
#if DEBUG
    Serial.println("Timer start");
#endif
  }
  else
  {
#if DEBUG
    Serial.println("No sensors found check sensors or wires");
#endif
  }


}


void sendMacAddress()
{
  //only send data when connected
  if (connected) {

    uint8_t mac[6];
    WiFi.macAddress(mac);
    //Send a packet
    udp.beginPacket(udpAddress.c_str(), udpPort);

    udp.write((uint8_t *)&startOfPacket, sizeof(startOfPacket));
    udp.write(mac, 6);
    udp.write((uint8_t *)&endOfPacket, sizeof(endOfPacket));
    udp.endPacket();
    Serial.printf("Mac address sended %012x", mac);
  }
}

void send_sensors()
{
#if DEBUG
  Serial.printf("Total sensors is %d\n");
#endif

  //only send data when connected
  if (connected) {
    //Send a packet
    udp.beginPacket(udpAddress.c_str(), udpPort);
    //packet.timeStamp = millis();
    udp.write((uint8_t *)&startOfPacket, sizeof(startOfPacket));
    for (int i = 0; i < activeSensorCounter(); i++ )
    {
#if DEBUG
      Serial.print("0x");
      Serial.print(sensorsActive[i], HEX);
      Serial.print(' ');
#endif

      udp.write(sensorsActive[i]);

    }
    udp.write((uint8_t *)&endOfPacket, sizeof(endOfPacket));
    udp.endPacket();
#if DEBUG
    Serial.println( );
#endif
    Serial.println("Sensors config sended");
  }

}

void send_data()
{

  if (connected) {
    //Send a packet
    uint8_t mac[6];
    WiFi.macAddress(mac);



    udp.beginPacket(udpAddress.c_str(), udpPort);
    uint32_t c_time = millis();
    packet.timeStamp[1] = c_time >> 16;
    packet.timeStamp[0] = c_time;
    packet.dataLength += 4;  // time stamp
    packet.dataLength += 3;  // mac uniq of
#if DEBUG
    //Serial.println(packet.timeStamp );
#endif
#if DEBUG
    //Serial.println(packet.dataLength);
#endif
    udp.write((uint8_t *)&startOfPacket, sizeof(startOfPacket));
    udp.write((uint8_t *)&packet.dataLength, sizeof(packet.dataLength));
    udp.write(mac + 3, 3);
    udp.write((uint8_t *)&packet.timeStamp, sizeof(packet.timeStamp));
#if DEBUG_SEND
    Serial.printf("%u\t", c_time);
#endif
    for (int i = 0; i < sensReadCount; i++)
    {
#if DEBUG_SEND
      Serial.print(i);
#endif
#if DEBUG_SEND
      Serial.print("=>");
#endif

#if DEBUG_SEND
      Serial.print(packet.sensors[i].sensorId, HEX);
#endif
#if DEBUG_SEND
      Serial.print('=');
#endif
#if DEBUG_SEND
      Serial.print(packet.sensors[i].value);
#endif
#if DEBUG_SEND
      Serial.print(" ");
#endif
      udp.write(packet.sensors[i].sensorId);
      udp.write((uint8_t *)&packet.sensors[i].value, sizeof(packet.sensors[i].value));
    }
#if DEBUG_SEND
    Serial.println();
#endif
    udp.write((uint8_t *)&endOfPacket, sizeof(endOfPacket));
    udp.endPacket();

  }
  sensReadCount = 0;
  clearPacket();

}

void readSensors()
{
  uint32_t last_send = millis();
  //float freq = 1000 /

#if DEBUG
  Serial.printf("Start time is %u\n", last_send);
#endif

  for (uint8_t i = 0; i < sizeof(devices) / sizeof(structDevice); i++)
  {
    if (devices[i].enable == true)
    {
      (*devices[i].functionRead)();
#if DEBUG
      Serial.printf("Time read %d is %u \n", i, millis() - last_send);
#endif
    }
  }
  if (sensReadCount)
  {
    send_data();
  }

}

void readButtonState()
{
  if (digitalRead(BUTTONPIN) == HIGH)
  {
    //Serial.println("BUTTONPIN HIGH");
    uint32_t startPress = millis();
    while (digitalRead(BUTTONPIN) == HIGH) {
      ESP.wdtFeed();
    }
    //Serial.println("BUTTONPIN LOW");

    int pressTime = millis() - startPress;
    if ( pressTime > 1000 && pressTime <= 5000)
    {
      Serial.print("Reset....time:" + String(pressTime));

      digitalWrite(POWERPIN, LOW);
      delay(500);
      ESP.restart();
    }
    else if (pressTime > 5000)
    {
      Serial.print("Hard reset....");
      wm.resetSettings();
      writeStringToEEPROM(ASAP_ADDRESS, String("off"));
      EEPROM.commit();
      ESP.restart();
    }
    else
    {
      showLedStatus();
    }
  }
}

void loop() {
  String inputString;
  while (Serial.available()) {

    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inputString == "restart") {
      Serial.print("restarting....");
      ESP.restart();

      break;
    }
    if (inputString == "reset") {
      Serial.print("hard reset....");
      wm.resetSettings();
      writeStringToEEPROM(ASAP_ADDRESS, "off");
      EEPROM.commit();
      ESP.restart();

      break;
    }
    delayMicroseconds(100);

  }
  readIncomingUdp();
  readButtonState();
  wm.process();

}
