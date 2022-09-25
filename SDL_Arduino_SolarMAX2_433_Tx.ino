// SDL_Arduino_SolarMAX2_433_Tx
// SwitchDoc Labs December 2020
//
// #define TXDEBUG
#undef TXDEBUG

#include <JeeLib.h>

#include "MemoryFree.h"

// Define pin on Grove Mini Pro Plus to control load viaSoar Power Manger
#define LOAD_CONTROL_PIN 2

// unique ID of this solarmax system - change if you have multiple solarmax systems
#define SOLARMAXID 1

// the three channels of the INA3221 named for INA3221 Solar Power Controller channels (www.switchdoc.com)
#define LOAD_CHANNEL 1
#define SOLAR_CELL_CHANNEL 2
#define BATTERY_CHANNEL 3

#define LED 13

// Software version
#define SOFTWAREVERSION 15

// Which SolarMax Protocol
#define SOLARMAXPROTOCOL 1

// WeatherSenseProtocol of 10 is SolarMAX2 LeadAcid   BatV > 8V
// #define WeatherSenseProtocol 8
int WeatherSenseProtocol = 10;

// Number of milliseconds between data out
#define SLEEPCYCLE 14500

#include "Crc16.h"

//Crc 16 library (XModem)
Crc16 crc;

ISR(WDT_vect) {
  Sleepy::watchdogEvent();
}

#include <RH_ASK.h>

#include <avr/sleep.h>
#include <avr/power.h>

#include "SDL_Arduino_INA3221.h"

SDL_Arduino_INA3221 INA3221;

#include "XClosedCube_HDC1080.h"

XClosedCube_HDC1080 hdc1080;

#define WATCHDOG        5

#define TXPIN 8
#define RXPIN 9

RH_ASK driver(2000, RXPIN, TXPIN);

unsigned long MessageCount = 0UL;

#include "avr/pgmspace.h"
// #include <Time.h>
#include <TimeLib.h>

#include <Wire.h>

typedef enum  {
  NO_INTERRUPT,
  IGNORE_INTERRUPT,
  SLEEP_INTERRUPT,
  RAIN_INTERRUPT,
  ANEMOMETER_INTERRUPT,
  ALARM_INTERRUPT,
  REBOOT
} wakestate;

// Device Present State Variables
bool INA3221_Present = false;
bool HDC1080_Present = false;

byte byteBuffer[200]; // contains string to be sent to RX unit

// State Variables
long TimeStamp = 0;
float temperature = 0.0;
float humidity = 0.0;
float BatteryVoltage = 0.0;
float BatteryCurrent = 0.0;
float LoadVoltage = 0.0;
float LoadCurrent = 0.0;
float SolarPanelVoltage = 0.0;
float SolarPanelCurrent = 0.0;
unsigned long AuxA = 0UL;

// AuxA has state information
// coded in the long integer
// 00000000 00000000 00000000 000X ABCD
// X - undefined (0)

// A = undefined (0)
// B = 5V Load Power state
// C = 1 - Ten Minute Power Off / On Timer
// D = 1 - First Time On Timer On, 0 First Time Timer Off

// Timers
// the ten minute timer is used so we don't turn on and turn off right away on loss of sunlight on a well discharged battery.
// If the computer turns the Load power off, you can't turn it on again for 10 minutes.  This hysterisis prevents slamming the Load on and off under heavy loads (which will pull the battery down quickly if there is not much charge)
unsigned long TenMinuteTimerDuration = 0UL; // 0 means disabled - used for preventing slamming off and on - once it turns off, it waits 10 minutes to try to turn on again.
unsigned long TenMinuteTimerStart = 0UL;

// FirstTimeTimerDuration
unsigned long FirstTimeTimerDuration = 0UL; // 0 means disabled
unsigned long FirstTimeTimerStart = 0UL;

int protocolBufferCount = 0;

wakestate wakeState;  // who woke us up?

bool Alarm_State_1 = false;
bool Alarm_State_2 = false;

long nextSleepLength = 0L;

int convert4ByteLongVariables(int bufferCount, long myVariable)
{
  union {
    long a;
    unsigned char bytes[4];
  } thing;

  thing.a = myVariable;

  for (int i = 0; i < 4; i++)
  {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }

  return bufferCount;
}

int convert4ByteFloatVariables(int bufferCount, float myVariable)
{
  union {
    float a;
    unsigned char bytes[4];
  } thing;

  thing.a = myVariable;

  for (int i = 0; i < 4; i++)
  {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }

  return bufferCount;
}

int convert2ByteVariables(int bufferCount, int myVariable)
{
  union {
    int a;
    unsigned char bytes[2];
  } thing;

  thing.a = myVariable;

  byteBuffer[bufferCount] = thing.bytes[0];
  bufferCount++;
  byteBuffer[bufferCount] = thing.bytes[1];
  bufferCount++;

#if defined(TXDEBUG)
  Serial.println(F("-------"));
  Serial.println(thing.bytes[0]);
  Serial.println(thing.bytes[1]);
  Serial.println(F("------"));
#endif

  return bufferCount;
}

int convert1ByteVariables(int bufferCount, int myVariable)
{
  byteBuffer[bufferCount] = (byte) myVariable;
  bufferCount++;
  return bufferCount;
}

int checkSum(int bufferCount)
{
  unsigned short checksumValue;

  // calculate checksum
  checksumValue = crc.XModemCrc(byteBuffer, 0, 59);

#if defined(TXDEBUG)
  Serial.print(F("crc = 0x"));
  Serial.println(checksumValue, HEX);
#endif

  byteBuffer[bufferCount] = checksumValue >> 8;
  bufferCount++;
  byteBuffer[bufferCount] = checksumValue & 0xFF;
  bufferCount++;

  return bufferCount;
}

int buildProtocolMessage()
{
  int bufferCount;

  bufferCount = 0;
  bufferCount = convert4ByteLongVariables(bufferCount, MessageCount);

  byteBuffer[bufferCount] = SOLARMAXID; // SolarMAX ID
  bufferCount++;

  // WeatherSenseProtocol of 8 is SolarMAX LiPo   BatV < 8V
  // WeatherSenseProtocol of 10 is SolarMAX2 LeadAcid   BatV > 8V

  byteBuffer[bufferCount] = WeatherSenseProtocol; // Type of WeatherSense System
  bufferCount++;

  byteBuffer[bufferCount] = SOLARMAXPROTOCOL; // SolarMAX protocol
  bufferCount++;

  byteBuffer[bufferCount] = SOFTWAREVERSION; // Software Version
  bufferCount++;

  bufferCount = convert4ByteFloatVariables(bufferCount, LoadVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, temperature),
  bufferCount = convert4ByteFloatVariables(bufferCount, humidity);

  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, LoadCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelCurrent);

  bufferCount = convert4ByteLongVariables(bufferCount, AuxA);

  Serial.print("AuxA=");
  Serial.println(AuxA, HEX);

  // protocolBufferCount = bufferCount + 2;
  // bufferCount = convert1ByteVariables(bufferCount, protocolBufferCount);
  // bufferCount = checkSum(bufferCount);

  return bufferCount;
}

void printStringBuffer()
{
  int bufferLength = protocolBufferCount;

  for (int i = 0; i < bufferLength; i++)
  {
    Serial.print(F("i="));
    Serial.print(i);
    Serial.print(F(" | "));
    Serial.println(byteBuffer[i], HEX);
  }
}

void return2Digits(char returnString[], char *buffer2, int digits)
{
  if (digits < 10)
    sprintf(returnString, "0%i", digits);
  else
    sprintf(returnString, "%i", digits);
  strcpy(returnString, buffer2);
}

void ResetWatchdog()
{
  digitalWrite(WATCHDOG, LOW);
  delay(200);
  digitalWrite(WATCHDOG, HIGH);

#if defined(TXDEBUG)
  Serial.println(F("Watchdog Reset - Patted the Dog"));
#endif
}

bool statusLoadPower = false;

void setupLoadPower()
{
  digitalWrite(LOAD_CONTROL_PIN, HIGH);
  pinMode(LOAD_CONTROL_PIN, OUTPUT);
  LoadPowerSetOn();
  statusLoadPower = true;
}

void setupLoadPowerOff()
{
  digitalWrite(LOAD_CONTROL_PIN, LOW);
  pinMode(LOAD_CONTROL_PIN, OUTPUT);
  LoadPowerSetOff();
  statusLoadPower = false;
}

void LoadPowerOff()
{
  LoadPowerSetOff();
  statusLoadPower = false;

  TenMinuteTimerDuration = 10L * 60L * 1000L;
  TenMinuteTimerStart = millis();

  AuxA = AuxA | 0x02; // Coded as Bit A (bit 1)

#if defined(TXDEBUG)
  Serial.println(F("FirstTimeTimer Reset"));
#endif

  //FirstTimeTimerDuration = 0;  // reset  timer
  //AuxA = AuxA & 0xFFFFFFFE; // Coded as Bit B (bit 0)

#if defined(TXDEBUG)
  Serial.print(F("Ten Minute Time Start Set at: "));
  Serial.println(TenMinuteTimerStart);
  Serial.print(F(" millisecs"));
#endif
}

void LoadPowerOn()
{
#if defined(TXDEBUG)
  Serial.println(F("Checking Ten Minute Timer inside LoadPowerOn"));
  Serial.print(F("millis() = "));
  Serial.println(millis());
  Serial.print(F("TenMinuteTimerStart = "));
  Serial.println(TenMinuteTimerStart);
  Serial.print(F("TenMinuteTimerDuration (0 = Disabled) = "));
  Serial.println(TenMinuteTimerDuration);
#endif

  if ((millis() - FirstTimeTimerStart) > FirstTimeTimerDuration) {  // reset first time start
    AuxA = AuxA & 0xFFFFFFFE; // Coded as Bit B (bit 0)
  }

  if (((millis() - TenMinuteTimerStart) > TenMinuteTimerDuration) && ((millis() - FirstTimeTimerStart) > FirstTimeTimerDuration))  // avoids rollover problem and delays first turn on
  {
    /* if (FirstTimeTimerDuration == 0) // First Time Timer has not been set
      {
      FirstTimeTimerDuration = 10L * 60L * 1000L;
      FirstTimeTimerStart = millis();
      AuxA = AuxA | 0x01; // Coded as Bit B (bit 0)
      #if defined(TXDEBUG)
      Serial.println(F("Load Power ON REFUSED - Set FirstTime Timer"));
      #endif
      }
      else
    */
    {
      LoadPowerSetOn();
      statusLoadPower = true;
      TenMinuteTimerDuration = 0;  // reset Ten Minute timer
      AuxA = AuxA & 0xFFFFFFFD; // Coded as Bit A (bit 1)
      // Note   First time timer is left expired, reset in Power Off command
    }
  }
  else
  {
#if defined(TXDEBUG)
    Serial.println(F("Load Power ON REFUSED due to Ten Minute Timer or First Time Timer"));
#endif
  }
}

void LoadPowerSetOff()
{
  digitalWrite(LOAD_CONTROL_PIN, LOW);

#if defined(TXDEBUG)
  Serial.println(F("Load Power OFF"));
#endif

  AuxA = AuxA & 0xFFFFFFFB;   // Coded as bit Z (bit 2)
}

void LoadPowerSetOn()
{
  digitalWrite(LOAD_CONTROL_PIN, HIGH);

#if defined(TXDEBUG)
  Serial.println(F("Load Power ON"));
#endif

  AuxA = AuxA | 0x04;    // Coded as bit Z (bit 2)
}

void read_ina3221()
{       
    Serial.print(F("INA3221 Sensor Reading:"));
    Serial.println("");

    BatteryVoltage = INA3221.getBusVoltage_V(BATTERY_CHANNEL);
    BatteryCurrent = INA3221.getCurrent_mA(BATTERY_CHANNEL);

    Serial.print(F("  Battery Voltage:      ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
    Serial.print(F("  Battery Current:      ")); Serial.print(BatteryCurrent); Serial.println(F(" mA"));

    SolarPanelVoltage = INA3221.getBusVoltage_V(SOLAR_CELL_CHANNEL);
    SolarPanelCurrent = INA3221.getCurrent_mA(SOLAR_CELL_CHANNEL) * -1.0;

    Serial.print(F("  Solar Panel Voltage:  ")); Serial.print(SolarPanelVoltage); Serial.println(F(" V"));
    Serial.print(F("  Solar Panel Current:  ")); Serial.print(SolarPanelCurrent); Serial.println(F(" mA"));

    LoadVoltage = INA3221.getBusVoltage_V(LOAD_CHANNEL);
    LoadCurrent = INA3221.getCurrent_mA(LOAD_CHANNEL) * 0.75;

    Serial.print(F("  Load Voltage:         ")); Serial.print(LoadVoltage); Serial.println(" V");
    Serial.print(F("  Load Current:         ")); Serial.print(LoadCurrent); Serial.println(" mA");
    
    Serial.println(""); 
}

void read_hdc1080()
{
    Serial.print(F("HDC1080 Sensor Reading:"));
    Serial.println("");
  
    temperature = hdc1080.readTemperature();

    Serial.print(F("  Temperature (C): ")); Serial.println(temperature);

    humidity = hdc1080.readHumidity();

    Serial.print(F("  Humidity (%RH):  ")); Serial.println(humidity);

    Serial.println("");  
}

void setup()
{
  Serial.begin(115200);    // TXDEBUGging only

  AuxA = 0x00;

  // turn on Load Power for power check.
  setupLoadPowerOff();

  Serial.println(F(">>>>>>>>>><<<<<<<<<"));
  Serial.println(F("SolarMAX2 433MHz Tx"));
  Serial.println(F(">>>>>>>>>><<<<<<<<<"));

  if (!driver.init())
  {
    Serial.println(F("init failed"));
    while (1);
  }

  Serial.print("max message length=");
  Serial.println(driver.maxMessageLength());

  Serial.print(F("Software Version:"));
  Serial.println(SOFTWAREVERSION);

  Serial.println("");

  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);

  // setup initial values of variables
  wakeState = REBOOT;
  Alarm_State_1 = false;
  Alarm_State_2 = false;
  nextSleepLength = SLEEPCYCLE;
  TimeStamp = 0;
  temperature = 0.0;
  humidity = 0.0;
  BatteryVoltage = 0.0;
  BatteryCurrent = 0.0;
  LoadCurrent = 0.0;
  SolarPanelVoltage = 0.0;
  SolarPanelCurrent = 0.0;

  pinMode(WATCHDOG, OUTPUT);
  digitalWrite(WATCHDOG, HIGH);

  // Just to turn off LED on _1_ƒload
  //pinMode(WATCHDOG_2, OUTPUT);
  //digitalWrite(WATCHDOG_2, HIGH);

  Wire.begin();

  // test for INA3221_Present
  INA3221_Present = false;

  Serial.println(F(">>>>>>>>>><<<<<<<<<"));
  Serial.println(F("  Initialisation   "));
  Serial.println(F(">>>>>>>>>><<<<<<<<<"));
  Serial.println("");
  
  int MIDNumber;
  INA3221.begin();
  INA3221.wireReadRegister(0xFE, &MIDNumber);
  if (MIDNumber != 0x5449)
  {
    INA3221_Present = false;
    Serial.println(F("INA3221 Not Present"));
  }
  else
  {
    INA3221_Present = true;
    Serial.println(F("INA3221 Present"));
   
    read_ina3221();
    
    // Now set WeatherSenseProtocol based on measured voltage
    // if (BatteryVoltage < 5.0)
    // {
    //   WeatherSenseProtocol = 8;   // SolarMAX2 LiPo
    // }
    // else
    // {
    //   WeatherSenseProtocol = 10;   // SolarMAX2 Lead Acid (12V Battery)
    // }

    Serial.print(F("WeatherSenseProtocol: "));
    Serial.println(WeatherSenseProtocol);
    Serial.println("");
  }

  // now do the power up check on start.
  //
  //  if BV > 12V then turn on
  //
  //
#if defined(TXDEBUG)
  Serial.print(F("Battery Voltage: ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));  
#endif

#if defined(TXDEBUG)
  Serial.println(F("Checking for setup Load Power ON:"));
#endif
  if (WeatherSenseProtocol == 10)
  {
    if (BatteryVoltage > 12.0)// Test voltage
    {
      LoadPowerOn();
    }
    else
    {
      // set the firstTime Timer On for 10 minute delay
      FirstTimeTimerDuration = 10UL * 60UL * 1000UL;
      FirstTimeTimerStart = millis();
      AuxA = AuxA | 0x01; // Coded as Bit B (bit 0)
#if defined(TXDEBUG)
      Serial.println(F("  Load Power ON REFUSED - Set FirstTime Timer"));
#endif

#if defined(TXDEBUG)
      Serial.println(F("  Load Power NOT turned on due to Battery Voltage < 12.0V"));
      LoadPowerOff();
#endif
    }
  }

  if (WeatherSenseProtocol == 8)
  {
    if (BatteryVoltage > 3.9)
    {
      LoadPowerOn();
    }
    else
    {
      // set the firstTime Timer On for 10 minute delay
      FirstTimeTimerDuration = 10UL * 60UL * 1000UL;
      FirstTimeTimerStart = millis();
      AuxA = AuxA | 0x01; // Coded as Bit B (bit 0)
#if defined(TXDEBUG)
      Serial.println(F("  Load Power ON REFUSED - Set FirstTime Timer"));
#endif

#if defined(TXDEBUG)
      Serial.println(F("  Load Power NOT turned on due to Battery Voltage < 3.9V"));
#endif
    }
  }

#if defined(TXDEBUG)
  Serial.print(F("LoadPower Status: "));
  Serial.println(statusLoadPower);
#endif

  // Check for HDC1080
  HDC1080_Present = false;
  hdc1080.begin(0x40);
  int devID = hdc1080.readDeviceId();

  if (devID == 0x1050)
  {
    Serial.println("HDC1080 Present");
    HDC1080_Present = true;
    read_hdc1080();
  }
  else
  {
    Serial.println("HDC1080 Not Present");
    HDC1080_Present = false;
  }

  TenMinuteTimerDuration = 0;   // starts disabled
}


void loop()
{
  // Only send if source is SLEEP_INTERRUPT
#if defined(TXDEBUG)
  Serial.print(F("wakeState = "));
  Serial.println(wakeState);
#endif

  if ((wakeState == SLEEP_INTERRUPT) || (Alarm_State_1 == true)  || (Alarm_State_2 == true))
  {
    wakeState = NO_INTERRUPT;
    Alarm_State_1 = false;
    Alarm_State_2 = false;

    Serial.println(F(">>>>>>>>>><<<<<<<<<"));
    Serial.println(F("    Monitoring     "));
    Serial.println(F(">>>>>>>>>><<<<<<<<<"));
    Serial.println("");
    
    Serial.print(F("MessageCount = "));
    Serial.println(MessageCount);
    Serial.println("");

    if (HDC1080_Present)
    {
      read_hdc1080();
    }

    TimeStamp = millis();

    // if INA3221 present, read charge data
    if (INA3221_Present)
    {
      read_ina3221();
    }

#if defined(TXDEBUG)
    Serial.println(F("###############"));
    Serial.print(F("Load PowerUp/Down Check - WeatherSenseProtocol: "));
    Serial.println(WeatherSenseProtocol);
    Serial.print(F("DeviceID= "));
    Serial.println(SOLARMAXID);
    Serial.print(F("Battery Voltage:     ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
    Serial.print(F("Battery Current:     ")); Serial.print(BatteryCurrent); Serial.println(F(" mA"));
    Serial.print(F("Solar Panel Voltage: ")); Serial.print(SolarPanelVoltage); Serial.println(F(" V"));
    Serial.print(F("Solar Current:       ")); Serial.print(SolarPanelCurrent); Serial.println(F(" mA"));
    Serial.print(F("Load Voltage:        ")); Serial.print(LoadVoltage); Serial.println(F(" V"));
    Serial.print(F("Load Current:        ")); Serial.print(LoadCurrent); Serial.println(" mA");
    Serial.print(F("Currentmillis() =    "));
    Serial.println(millis());
    Serial.println();

    Serial.print(F("TenMinuteTimerStart = "));
    Serial.println(TenMinuteTimerStart);
    Serial.print(F("TenMinuteTimerDuration (0 = Disabled) = "));
    Serial.println(TenMinuteTimerDuration);

    if (TenMinuteTimerDuration == 0)
      Serial.println(F("TenMinute Timer = DISABLED"));
    else if ((millis() - TenMinuteTimerStart) > TenMinuteTimerDuration)
      Serial.println(F("TenMinuteTimer = EXPIRED"));
    else
      Serial.println(F("TenMinuteTimer = RUNNING"));
    Serial.println();

    Serial.print(F("FirstTimeTimerStart = "));
    Serial.println(FirstTimeTimerStart);
    Serial.print(F("FirstTimeTimerDuration (0 = Disabled) = "));
    Serial.println(FirstTimeTimerDuration);

    if (FirstTimeTimerDuration == 0)
      Serial.println(F("FirstTimeTimer = DISABLED"));
    else if ((millis() - FirstTimeTimerStart) > FirstTimeTimerDuration)
      Serial.println(F("FirstTimeTimer = EXPIRED"));
    else
      Serial.println(F("FirstTimeTimer = RUNNING"));
    Serial.println();

    Serial.print(F("AuxA State:"));

    Serial.print(AuxA);
    Serial.print(F(" "));
    Serial.println(AuxA, HEX);
    Serial.print(F("LoadPower Status:"));
    Serial.println(statusLoadPower);
    Serial.println(F("###############"));
#endif

    // Check for power off
    if (statusLoadPower == true)
    {
#if defined(TXDEBUG)
      Serial.println(F("Checking for Low Battery or Low Voltage"));
      Serial.print(F("Battery Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
      Serial.print(F("Load Voltage:     ")); Serial.print(LoadVoltage); Serial.println(F(" V"));
#endif
      if ((WeatherSenseProtocol == 10) )
      {
        if ((BatteryVoltage < 10.96) || ((LoadVoltage < 4.76) && (statusLoadPower == true)))  // 10.96 emperical
        {
          Serial.println(F("Low LEAD ACID battery or Load Voltage Power Off on 5V"));
          LoadPowerOff();
        }
      }
      else if ((WeatherSenseProtocol == 8))
      {
        if ((BatteryVoltage < 2.9) || ((LoadVoltage < 4.76) && (statusLoadPower == true)))  // 2.9 emperical
        {
          Serial.println(F("Low LIPO battery or Load Voltage Power Off on 5V"));
          LoadPowerOff();
        }
      }
    } // statusLoadPower = true
    else // statusLoadPower = false
    {
      // check to turn power on
      if ((WeatherSenseProtocol == 10) )
      {
#if defined(TXDEBUG)
        Serial.println(F("Checking for LoadPower On"));
#endif
        if ((BatteryVoltage > 11.8 ) && (SolarPanelCurrent > 100))
        {
          LoadPowerOn();
          delay(100);
          LoadVoltage = INA3221.getBusVoltage_V(LOAD_CHANNEL);
        }
        else
        {
          Serial.println(F("Load Power NOT turned on due to Battery Voltage < 11.8 V or Solar Panel Current < 100 mA"));
        }
      }
      if (WeatherSenseProtocol == 8)
      {
        if (((BatteryVoltage > 3.0) && (SolarPanelCurrent > 100.0)) || (BatteryVoltage > 3.9))
        {
          LoadPowerOn();
          delay(100);
          // Now check the voltage again
          LoadVoltage = INA3221.getBusVoltage_V(LOAD_CHANNEL);
        }
        else
        {
          Serial.println(F("Load Power NOT turned  on due to (Battery Voltage < 3.0 V or Solar Panel Current < 100 mA) ||  BV < 3.9)"));
        }
      }
    }

    // update 10 minute timer
    if ((millis() - TenMinuteTimerStart) > TenMinuteTimerDuration)
    {
      // expire 10 minute timer
      TenMinuteTimerDuration = 0;  // reset Ten Minute timer
      AuxA = AuxA & 0xFFFFFFFD; // Coded as Bit A (bit 1)
    }

    // Now send the message

    // write out the current protocol to message and send.
    int bufferLength;

    Serial.println(F("----------Sending packets----------"));
  
    bufferLength = buildProtocolMessage();

    // Send a message
    driver.send(byteBuffer, bufferLength);
  
    Serial.println(F("----------After Sending packet----------"));

    for (int i = 0; i < bufferLength; i++) {
      Serial.print(" ");
      if (byteBuffer[i] < 16)
      {
        Serial.print(F("0"));
      }
      Serial.print(byteBuffer[i], HEX);           //  write buffer to hardware serial port
    }
    Serial.println();

    if (!driver.waitPacketSent(6000))
    {
      Serial.println(F("Timeout on transmission"));
      // re-initialize board
      if (!driver.init())
      {
        Serial.println(F("init failed"));
        while (1);
      }
      Serial.println(F("----------Board Reinitialized----------"));
    }

    Serial.println(F("----------After Wait Sending packet----------"));
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);

    Serial.print(F("freeMemory()="));
    Serial.println(freeMemory());
    Serial.print(F("bufferlength="));
    Serial.println(bufferLength);

    MessageCount++;

    Serial.println(F("----------Packet Sent. Sleeping Now----------"));
  }

  if (wakeState != REBOOT) {
    wakeState = SLEEP_INTERRUPT;
  }
  unsigned long timeBefore;
  unsigned long timeAfter;
  timeBefore = millis();
#if defined(TXDEBUG)
  Serial.print(F("timeBeforeSleep = "));
  Serial.println(timeBefore);
#endif
  delay(100);

  //Sleepy::loseSomeTime(nextSleepLength);
  for (long i = 0; i < nextSleepLength / 16; ++i) {
    Sleepy::loseSomeTime(16);
  }

  wakeState = SLEEP_INTERRUPT;

#if defined(TXDEBUG)
  Serial.print(F("Awake now: "));
#endif
  timeAfter = millis();
#if defined(TXDEBUG)
  Serial.print(F("timeAfterSleep = "));
  Serial.println(timeAfter);

  Serial.print(F("SleepTime = "));
  Serial.println(timeAfter - timeBefore);

  Serial.print(F("Millis Time: "));
#endif
  unsigned long time;
  time = millis();
#if defined(TXDEBUG)
  //prints time since program started
  Serial.println(time / 1000.0);
  Serial.print(F("wakeState = "));
  Serial.println(wakeState);
#endif

  // Pat the WatchDog
  ResetWatchdog();
}
