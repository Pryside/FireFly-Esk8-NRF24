#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "RF24.h"
#include "VescUart.h"

// #define DEBUG

#define NUMBER_OF_VESCS 2 //1 for single Vesc, 2 for dual Vesc and so on

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.println(x)
#include "printf.h"
#else
#define DEBUG_PRINT(x)
#endif

// Defining the type of display used (128x32)
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

static unsigned char logo_bits[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x3c, 0x01, 0xe0, 0x00, 0x07, 0x70, 0x18, 0x0e, 0x30, 0x18, 0x0c, 0x98, 0x99, 0x19, 0x80, 0xff, 0x01, 0x04, 0xc3, 0x20, 0x0c, 0x99, 0x30, 0xec, 0xa5, 0x37, 0xec, 0xa5, 0x37, 0x0c, 0x99, 0x30, 0x04, 0xc3, 0x20, 0x80, 0xff, 0x01, 0x98, 0x99, 0x19, 0x30, 0x18, 0x0c, 0x70, 0x18, 0x0e, 0xe0, 0x00, 0x07, 0x80, 0x3c, 0x01, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static unsigned char signal_transmitting_bits[] = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x0f,
  0x33, 0x0f, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

static unsigned char signal_connected_bits[] = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x09,
  0x33, 0x09, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

static unsigned char signal_noconnection_bits[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x09,
  0x00, 0x09, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Defining struct to hold UART data.
struct vescValues
{
  float ampHours;
  float inpVoltage;
  long rpm;
  long tachometerAbs;
  float avgInputCurrent;
};

//NunchuckValues for sending
struct NunchuckValues
{
  byte ValY;
  bool upperButton;
  bool lowerButton;
  int checksum;
};

// Defining struct to hold stats
struct stats
{
  float total_energy;
  float total_distance;
  //float maxSpeed;
  //long maxRpm;
  //float minVoltage;
  //float maxVoltage;
};
//Variables to keep track of current stats
struct temp_stats
{
  float lastAh;
  float distance;
  float energy;
};

// Defining struct to hold setting values while remote is turned on.
struct settings
{
  byte triggerMode;
  byte batteryType;
  byte batteryCells;
  byte motorPoles;
  byte motorPulley;
  byte wheelPulley;
  byte wheelDiameter;
  bool useUart;
  int minHallValue;
  int centerHallValue;
  int maxHallValue;
};

// Defining variables for speed and distance calculation
float gearRatio;
float ratioRpmSpeed;
float ratioPulseDistance;

byte currentSetting = 0;
const byte numOfSettings = 11;

String settingPages[numOfSettings][2] = {
  {"Trigger", ""},
  {"Battery type", ""},
  {"Battery cells", "S"},
  {"Motor poles", ""},
  {"Motor pulley", "T"},
  {"Wheel pulley", "T"},
  {"Wheel diameter", "mm"},
  {"UART data", ""},
  {"Throttle min", ""},
  {"Throttle center", ""},
  {"Throttle max", ""}
};

// Setting rules format: default, min, max.
int settingRules[numOfSettings][3] {
  {0, 0, 3}, // 0 Killswitch, 1 cruise & 2 data toggle
  {0, 0, 1}, // 0 Li-ion & 1 LiPo
  {10, 0, 12},
  {14, 0, 250},
  {15, 0, 250},
  {40, 0, 250},
  {83, 0, 250},
  {1, 0, 1}, // Yes or no
  {190, 0, 1023},
  {512, 0, 1023},
  {890, 0, 1023}
};

struct vescValues data;
struct settings remoteSettings;
struct NunchuckValues sendData;
struct stats statistics;
struct temp_stats temp_statistics;

// Pin defination
const byte triggerPin = 4;
const int chargeMeasurePin = A1;
const int batteryMeasurePin = A2;
const int hallSensorPin = A3;

// Battery monitering
const float minVoltage = 3.2;
const float maxVoltage = 4.1;
const float refVoltage = 5.1; // Set to 4.5V if you are testing connected to USB, otherwise 5V (or the supply voltage)

// Defining variables for Hall Effect throttle.
short hallMeasurement, throttle;
byte hallCenterMargin = 4;

//For debouncing the switch
bool Trigger_laststate;
unsigned long Trigger_lastchange;
bool Trigger_output;

//Variables for EEPROM Statistics
unsigned long lastSave;
float old_distance;
bool timesaved = true;
bool init_stats = false;

//Cruise Control Variables
unsigned long cruise_lastPress;
int cruise_state = 0;            //0/1\2/3\4/5---0 3 presses quickly in a row activate cruise control

// Defining variables for NRF24 communication
bool connected = false;
short failCount;
const uint64_t pipe = 0xE6E6F0F0E1LL; // If you change the pipe, you will need to update it on the receiver to.
unsigned long lastTransmission;

// Defining variables for OLED display
char displayBuffer[20];
String displayString;
short displayData = 0;
unsigned long lastSignalBlink;
unsigned long lastDataRotation;

// Instantiating RF24 object for NRF24 communication
RF24 radio(9, 10);

// Defining variables for Settings menu
bool changeSettings = false;
bool changeSelectedSetting = false;

bool settingsLoopFlag = false;
bool settingsChangeFlag = false;
bool settingsChangeValueFlag = false;

void setup()
{
  // setDefaultEEPROMSettings(); // Call this function if you want to reset settings and stats

#ifdef DEBUG
  Serial.begin(9600);
#endif

  loadEEPROMSettings();

  pinMode(triggerPin, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT);
  pinMode(batteryMeasurePin, INPUT);

  u8g2.begin();

  drawStartScreen();


  if (!digitalRead(triggerPin))
  {
    changeSettings = true;
    drawTitleScreen("Remote Settings");
  }

  // Start radio communication
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe(pipe);

  initStatistics();

#ifdef DEBUG
  printf_begin();
  radio.printDetails();
#endif
}

void loop()
{
  debounceTrigger();

  calculateThrottlePosition();

  if (changeSettings == true)
  {
    // Use throttle and trigger to change settings
    controlSettingsMenu();
  }
  else
  {
    //CheckCruiseControl(); //currently disabled because its not fully developed yet
    // Use throttle and trigger to drive motors
    if (triggerActive())
    {
      throttle = throttle;
    }
    else
    {
      // 127 is the middle position - no throttle and no brake/reverse
      throttle = 127;
    }
    // Transmit to receiver

    transmitToVesc();
  }

  if (init_stats)
    EEPROMStatSaveCycle();

  // Call function to update display and LED
  updateMainDisplay();
}

void debounceTrigger() {
  if(Trigger_laststate != (!digitalRead(triggerPin))) {
    Trigger_laststate = (!digitalRead(triggerPin));

    Trigger_lastchange = millis();
  }

  if(Trigger_lastchange + 50 < millis()) {
    Trigger_output = Trigger_laststate;
  }
}

void controlSettingsMenu()
{
  if (triggerActive())
  {
    if (settingsChangeFlag == false)
    {

      // Save settings to EEPROM
      if (changeSelectedSetting == true)
      {
        updateEEPROMSettings();
      }

      changeSelectedSetting = !changeSelectedSetting;
      settingsChangeFlag = true;
    }
  }
  else
  {
    settingsChangeFlag = false;
  }

  if (hallMeasurement >= (remoteSettings.maxHallValue - 150) && settingsLoopFlag == false)
  {
    // Up
    if (changeSelectedSetting == true)
    {
      int val = getSettingValue(currentSetting) + 1;

      if (inRange(val, settingRules[currentSetting][1], settingRules[currentSetting][2]))
      {
        setSettingValue(currentSetting, val);
        settingsLoopFlag = true;
      }
    }
    else
    {
      if (currentSetting != 0)
      {
        currentSetting--;
        settingsLoopFlag = true;
      }
    }
  }
  else if (hallMeasurement <= (remoteSettings.minHallValue + 150) && settingsLoopFlag == false)
  {
    // Down
    if (changeSelectedSetting == true)
    {
      int val = getSettingValue(currentSetting) - 1;

      if (inRange(val, settingRules[currentSetting][1], settingRules[currentSetting][2]))
      {
        setSettingValue(currentSetting, val);
        settingsLoopFlag = true;
      }
    }
    else
    {
      if (currentSetting < (numOfSettings - 1))
      {
        currentSetting++;
        settingsLoopFlag = true;
      }
    }
  }
  else if (inRange(hallMeasurement, remoteSettings.centerHallValue - 50, remoteSettings.centerHallValue + 50))
  {
    settingsLoopFlag = false;
  }
}

void drawSettingNumber()
{
  // Position on OLED
  int x = 2;
  int y = 10;

  // Draw current setting number box
  u8g2.drawRFrame(x + 102, y - 10, 22, 32, 4);

  // Draw current setting number
  displayString = (String)(currentSetting + 1);
  displayString.toCharArray(displayBuffer, displayString.length() + 1);

  u8g2.setFont(u8g2_font_profont22_tn);
  u8g2.drawStr(x + 108, 22, displayBuffer);
}

void drawSettingsMenu()
{
  // Position on OLED
  int x = 0;
  int y = 10;

  // Draw setting title
  displayString = settingPages[currentSetting][0];
  displayString.toCharArray(displayBuffer, displayString.length() + 1);

  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr(x, y, displayBuffer);

  int val = getSettingValue(currentSetting);

  displayString = (String)val + "" + settingPages[currentSetting][1];
  displayString.toCharArray(displayBuffer, displayString.length() + 1);
  u8g2.setFont(u8g2_font_10x20_tr);

  if (changeSelectedSetting == true)
  {
    u8g2.drawStr(x + 10, y + 20, displayBuffer);
  }
  else
  {
    u8g2.drawStr(x, y + 20, displayBuffer);
  }
}

void EEPROMStatSaveCycle()
{

  calculateStats();
  float difference = statistics.total_distance - old_distance * 1.00;

  bool BoardShutdown=false;
  if((data.inpVoltage > 0.00) && (data.inpVoltage < 30.00)){
    BoardShutdown = true;
  }

  if (difference > 2.0) //When Kilometers increased by 2.0, data will be saved
  {
    lastSave = millis();
    old_distance = statistics.total_distance;
    updateEEPROMStats();
  }
  else if (( ((lastSave + 240000 < millis()) && difference > 1.0) || BoardShutdown ) && timesaved == false)
  { //After eBoard comes to a stop and is turned off, or after 2 minutes the data will also be saved one last time
    timesaved = true;
    old_distance = statistics.total_distance;
    updateEEPROMStats();
  }


  


}

void setDefaultEEPROMSettings()
{
  eraseStatistics();
  for (int i = 0; i < numOfSettings; i++)
  {
    setSettingValue(i, settingRules[i][0]);
  }

  updateEEPROMSettings();
}

void loadEEPROMSettings()
{
  // Load settings from EEPROM to custom struct
  EEPROM.get(0, remoteSettings);
  //Load Stats as well
  EEPROM.get(200, statistics);

  bool rewriteSettings = false;

  // Loop through all settings to check if everything is fine
  for (int i = 0; i < numOfSettings; i++)
  {
    int val = getSettingValue(i);

    if (!inRange(val, settingRules[i][1], settingRules[i][2]))
    {
      // Setting is damaged or never written. Rewrite default.
      rewriteSettings = true;
      setSettingValue(i, settingRules[i][0]);
    }
  }

  if (rewriteSettings == true)
  {
    updateEEPROMSettings();
    eraseStatistics();
  }
  else
  {
    // Calculate constants
    calculateRatios();
  }
}

void initStatistics()
{
  temp_statistics.distance = ratioPulseDistance * data.tachometerAbs;
  temp_statistics.energy += (temp_statistics.lastAh - data.ampHours) * data.inpVoltage;
  temp_statistics.lastAh = data.ampHours;
  old_distance = statistics.total_distance;
}

//when settings are corrupted or not yet written stats get erased too
void eraseStatistics()
{
  statistics.total_energy = 0;
  statistics.total_distance = 0;
  EEPROM.put(200, statistics);
}

void updateEEPROMStats()
{
  EEPROM.put(200, statistics);
  /*
    Serial.print(statistics.total_energy);
    Serial.print("   ");
    Serial.print(statistics.total_distance);
    Serial.print("   ");
    Serial.println(millis());
  */
}

// Write settings to the EEPROM then exiting settings menu.
void updateEEPROMSettings()
{
  EEPROM.put(0, remoteSettings);
  calculateRatios();
}

void calculateStats()
{
  float old_distanceL;
  old_distanceL = temp_statistics.distance;

  temp_statistics.distance = ratioPulseDistance * data.tachometerAbs;

  float old_energy;
  old_energy = temp_statistics.energy;

  temp_statistics.energy += (temp_statistics.lastAh - data.ampHours) * data.inpVoltage;
  temp_statistics.lastAh = data.ampHours;

  statistics.total_energy += temp_statistics.energy - old_energy;
  statistics.total_distance += temp_statistics.distance - old_distanceL;
}



// Update values used to calculate speed and distance travelled.
void calculateRatios()
{
  gearRatio = (float)remoteSettings.motorPulley / (float)remoteSettings.wheelPulley;

  ratioRpmSpeed = (gearRatio * 60 * (float)remoteSettings.wheelDiameter * 3.14156) / (((float)remoteSettings.motorPoles / 2) * 1000000); // ERPM to Km/h

  ratioPulseDistance = (gearRatio * (float)remoteSettings.wheelDiameter * 3.14156) / (((float)remoteSettings.motorPoles * 3) * 1000000); // Pulses to km travelled
}

// Get settings value by index (usefull when iterating through settings).
int getSettingValue(int index)
{
  int value;
  switch (index)
  {
    case 0:
      value = remoteSettings.triggerMode;
      break;
    case 1:
      value = remoteSettings.batteryType;
      break;
    case 2:
      value = remoteSettings.batteryCells;
      break;
    case 3:
      value = remoteSettings.motorPoles;
      break;
    case 4:
      value = remoteSettings.motorPulley;
      break;
    case 5:
      value = remoteSettings.wheelPulley;
      break;
    case 6:
      value = remoteSettings.wheelDiameter;
      break;
    case 7:
      value = remoteSettings.useUart;
      break;
    case 8:
      value = remoteSettings.minHallValue;
      break;
    case 9:
      value = remoteSettings.centerHallValue;
      break;
    case 10:
      value = remoteSettings.maxHallValue;
      break;
  }
  return value;
}

// Set a value of a specific setting by index.
void setSettingValue(int index, int value)
{
  switch (index)
  {
    case 0:
      remoteSettings.triggerMode = value;
      break;
    case 1:
      remoteSettings.batteryType = value;
      break;
    case 2:
      remoteSettings.batteryCells = value;
      break;
    case 3:
      remoteSettings.motorPoles = value;
      break;
    case 4:
      remoteSettings.motorPulley = value;
      break;
    case 5:
      remoteSettings.wheelPulley = value;
      break;
    case 6:
      remoteSettings.wheelDiameter = value;
      break;
    case 7:
      remoteSettings.useUart = value;
      break;
    case 8:
      remoteSettings.minHallValue = value;
      break;
    case 9:
      remoteSettings.centerHallValue = value;
      break;
    case 10:
      remoteSettings.maxHallValue = value;
      break;
  }
}

// Check if an integer is within a min and max value
bool inRange(int val, int minimum, int maximum)
{
  return ((minimum <= val) && (val <= maximum));
}

void CheckCruiseControl() //DOESNT GET USED
{
  
    if (millis() > cruise_lastPress + 700 && cruise_state > 0 && cruise_state < 5) //defines the amout of ms you have for activating the cruise control
    {
      cruise_state = 0;
    }

  if (Trigger_output)
  {
    if (cruise_state == 0) {
      cruise_lastPress = millis();
      cruise_state = 1;
    }    
    if (cruise_state == 2)
      cruise_state = 3;
    if (cruise_state == 4)
      cruise_state = 5;

  }
  else
  {
    if (cruise_state == 1)
      cruise_state = 2;
    if (cruise_state == 3)
      cruise_state = 4;
  }



  if (Trigger_output && cruise_state == 5)
  {
    sendData.lowerButton = true; //CRUISE CONTROL GETS ACTIVATED!
  }
  else if (cruise_state == 5)
  {
    sendData.lowerButton = false;
    cruise_state = 0; //CRUISE CONTROL GETS DEACTIVATED
  }

}

// Return true if trigger is activated, false otherwice
boolean triggerActive()
{
  if (Trigger_output)
    return true;
  else
    return false;
}

// Function used to transmit the throttle value, and receive the VESC realtime data.
void transmitToVesc()
{
  // Transmit once every 50 millisecond
  if (millis() - lastTransmission >= 50)
  {

    lastTransmission = millis();

    boolean sendSuccess = false;
    // Transmit the Nunchuck Values

    sendData.ValY = throttle;

    sendData.checksum = 0;
    sendData.checksum = sendData.ValY;
    if(sendData.upperButton ==  true) sendData.checksum++;
    if(sendData.lowerButton ==  true) sendData.checksum++;

    sendSuccess = radio.write(&sendData, sizeof(sendData));

    bool dataRecieving = false;
    // Listen for an acknowledgement reponse (return of VESC data).
    while (radio.isAckPayloadAvailable())
    {
      radio.read(&data, sizeof(data));
      dataRecieving = true;
    }
    if (dataRecieving && !init_stats)
    {
      initStatistics();
      init_stats = true;
    }

    if (sendSuccess == true)
    {
      // Transmission was a succes
      failCount = 0;
      sendSuccess = false;

      DEBUG_PRINT("Transmission succes");
    }
    else
    {
      // Transmission was not a succes
      failCount++;

      DEBUG_PRINT("Failed transmission");
    }

    // If lost more than 5 transmissions, we can assume that connection is lost.
    if (failCount < 5)
    {
      connected = true;
    }
    else
    {
      connected = false;
    }
  }
}

void calculateThrottlePosition()
{
  // Hall sensor reading can be noisy, lets make an average reading.
  int total = 0;
  for (int i = 0; i < 10; i++)
  {
    total += analogRead(hallSensorPin);
  }
  hallMeasurement = total / 10;

  DEBUG_PRINT((String)hallMeasurement);

  if (hallMeasurement >= remoteSettings.centerHallValue)
  {
    throttle = constrain(map(hallMeasurement, remoteSettings.centerHallValue, remoteSettings.maxHallValue, 127, 255), 127, 255);
  }
  else
  {
    throttle = constrain(map(hallMeasurement, remoteSettings.minHallValue, remoteSettings.centerHallValue, 0, 127), 0, 127);
  }

  // removeing center noise
  if (abs(throttle - 127) < hallCenterMargin)
  {
    throttle = 127;
  }
}

// Function used to indicate the remotes battery level.
int batteryLevel()
{
  float voltage = batteryVoltage();

  if (voltage <= minVoltage)
  {
    return 0;
  }
  else if (voltage >= maxVoltage)
  {
    return 100;
  }
  else
  {
    return (voltage - minVoltage) * 100 / (maxVoltage - minVoltage);
  }
}

// Function to calculate and return the remotes battery voltage.
float batteryVoltage()
{
  float batteryVoltage = 0.0;
  int total = 0;

  for (int i = 0; i < 10; i++)
  {
    total += analogRead(batteryMeasurePin);
  }

  batteryVoltage = (refVoltage / 1024.0) * ((float)total / 10.0);

  return batteryVoltage;
}

void updateMainDisplay()
{

  u8g2.firstPage();
  do
  {

    if (changeSettings == true)
    {
      drawSettingsMenu();
      drawSettingNumber();
    }
    else
    {
      drawThrottleBattery();
      drawPage();
      drawBatteryLevel();
      drawSignal();
    }

  } while (u8g2.nextPage());
}

void drawStartScreen()
{
  u8g2.firstPage();
  do
  {
    u8g2.drawXBM(4, 4, 24, 24, logo_bits);

    displayString = "Esk8 remote";
    displayString.toCharArray(displayBuffer, 12);
    u8g2.setFont(u8g2_font_helvR10_tr);
    u8g2.drawStr(34, 22, displayBuffer);
  } while (u8g2.nextPage());
  delay(1500);
}

void drawTitleScreen(String title)
{
  u8g2.firstPage();
  do
  {
    title.toCharArray(displayBuffer, 20);
    u8g2.setFont(u8g2_font_helvR10_tr);
    u8g2.drawStr(12, 20, displayBuffer);
  } while (u8g2.nextPage());
  delay(1500);
}

void drawPage()
{
  int decimals;
  float value;
  String suffix;
  String prefix;

  int first, last;

  int x = 0;
  int y = 16;

  // Rotate the realtime data each 4s.
  if ((millis() - lastDataRotation) >= 4000)
  {

    lastDataRotation = millis();
    displayData++;


    if( ((data.inpVoltage * data.avgInputCurrent) / 1000.00) > 0.5 ) {
      displayData=3;
    }
    else if(ratioRpmSpeed * data.rpm > 3) {
      if(displayData==0) displayData=2;
      else if (displayData==2) displayData=0;
      else displayData=0;
    }
    else {
      if (displayData == 0 || displayData == 3) displayData++;
    }

    
    if (displayData > 5)
    {
      displayData = 0;
    }
    
  }

  switch (displayData)
  {
    case 0:
      value = ratioRpmSpeed * data.rpm;
      suffix = "KMH";
      prefix = "SPEED";
      decimals = 1;
      break;
    case 1:
      value = ratioPulseDistance * data.tachometerAbs;
      suffix = "KM";
      prefix = "DISTANCE";
      decimals = 2;
      break;
    case 2:
      value = data.inpVoltage;
      suffix = "V";
      prefix = "BATTERY";
      decimals = 1;
      break;
    case 3:
      value = (data.inpVoltage * data.avgInputCurrent) / 1000.00 * NUMBER_OF_VESCS;
      suffix = "KW";
      prefix = "POWER";
      decimals = 2;
      break;
    case 4:
      value = statistics.total_distance / 100.00;
      suffix = "KM";
      prefix = "T. DIST";
      decimals = 2;
      break;
    case 5:
      value = statistics.total_energy / 1000.00 * -1.00 * NUMBER_OF_VESCS;
      suffix = "KWH";
      prefix = "T. ENERGY";
      decimals = 2;
      break;
  }

  // Display prefix (title)
  displayString = prefix;
  displayString.toCharArray(displayBuffer, 10);
  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr(x, y - 1, displayBuffer);

  // Split up the float value: a number, b decimals.
  first = abs(floor(value));
  last = value * pow(10, 3) - first * pow(10, 3);

  // Add leading zero
  if (first <= 9)
  {
    displayString = "0" + (String)first;
  }
  else
  {
    displayString = (String)first;
  }

  // Display numbers
  displayString.toCharArray(displayBuffer, 10);
  u8g2.setFont(u8g2_font_logisoso22_tn);
  u8g2.drawStr(x + 55, y + 13, displayBuffer);

  // Display decimals
  if(displayData != 4) displayString = "." + (String)last; //Disabled the . for the total_km
  displayString.toCharArray(displayBuffer, decimals + 2);
  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr(x + 86, y - 1, displayBuffer);

  // Display suffix
  displayString = suffix;
  displayString.toCharArray(displayBuffer, 10);
  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr(x + 86 + 2, y + 13, displayBuffer);
}

void drawThrottleBattery() //Draws Battery Level when not being used as Throttle
{
  int inputValue = throttle;
  int percent;

  if(throttle == 127) {
    if(remoteSettings.batteryType == 0) {
      if(remoteSettings.batteryCells*4.0 < data.inpVoltage){
        percent = map(data.inpVoltage*100, remoteSettings.batteryCells*400, remoteSettings.batteryCells*420, 80, 100);
      }
      else {
        percent = map(data.inpVoltage*100, remoteSettings.batteryCells*320, remoteSettings.batteryCells*400, 0, 80);
      }
    }
    if(remoteSettings.batteryType == 1) {
      if(remoteSettings.batteryCells*4.0 < data.inpVoltage){
        percent = map(data.inpVoltage*100, remoteSettings.batteryCells*400, remoteSettings.batteryCells*420, 85, 100);
      }
      else {
        percent = map(data.inpVoltage*100, remoteSettings.batteryCells*350, remoteSettings.batteryCells*400, 0, 85);
      }
    }
    inputValue = map(percent, 0, 100, 127, 255);
  }

  

  int x = 0;
  int y = 18;

  // Draw throttle
  u8g2.drawHLine(x, y, 52);
  u8g2.drawVLine(x, y, 10);
  u8g2.drawVLine(x + 52, y, 10);
  u8g2.drawHLine(x, y + 10, 5);
  u8g2.drawHLine(x + 52 - 4, y + 10, 5);

  if (throttle >= 127)
  {
    int width = map(inputValue, 127, 255, 0, 49);

    for (int i = 0; i < width; i++)
    {
      //if( (i % 2) == 0){
      u8g2.drawVLine(x + i + 2, y + 2, 7);
      //}
    }
  }
  else
  {
    int width = map(inputValue, 0, 126, 49, 0);
    for (int i = 0; i < width; i++)
    {
      //if( (i % 2) == 0){
      u8g2.drawVLine(x + 50 - i, y + 2, 7);
      //}
    }
  }
}

bool signalBlink = false;

void drawSignal()
{
  // Position on OLED
  int x = 114;
  int y = 17;

  if (connected == true)
  {
    if (triggerActive())
    {
      u8g2.drawXBM(x, y, 12, 12, signal_transmitting_bits);
    }
    else
    {
      u8g2.drawXBM(x, y, 12, 12, signal_connected_bits);
    }
  }
  else
  {
    if (millis() - lastSignalBlink > 500)
    {
      signalBlink = !signalBlink;
      lastSignalBlink = millis();
    }

    if (signalBlink == true)
    {
      u8g2.drawXBM(x, y, 12, 12, signal_connected_bits);
    }
    else
    {
      u8g2.drawXBM(x, y, 12, 12, signal_noconnection_bits);
    }
  }
}

void drawBatteryLevel()
{
  int level = batteryLevel();

  // Position on OLED
  int x = 108;
  int y = 4;

  u8g2.drawFrame(x + 2, y, 18, 9);
  u8g2.drawBox(x, y + 2, 2, 5);

  for (int i = 0; i < 5; i++)
  {
    int p = round((100 / 5) * i);
    if (p <= level)
    {
      u8g2.drawBox(x + 4 + (3 * i), y + 2, 2, 5);
    }
  }
}
