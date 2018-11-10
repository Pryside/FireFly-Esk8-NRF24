#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <VescUart.h>

VescUart UART;

struct vescValues {
  
  float ampHours;
  float inpVoltage;
  long rpm;
  long tachometerAbs;
  float avgInputCurrent;

};

struct NunchuckValues {
  byte ValY;
  bool upperButton;
  bool lowerButton;
  int checksum;
};

RF24 radio(7, 8); //Set CE and CSN Pins here!
const uint64_t pipe = 0xE6E6F0F0E1LL; //Set new pipline here!

bool recievedData = false;
uint32_t lastTimeReceived = 0;

int timeoutMax = 500;

struct NunchuckValues ControlValues;

struct vescValues data;
unsigned long lastDataCheck;

void setup() {
  Serial.begin(115200); //Serial 0 selected (for arduino Nano)

  UART.nunchuck.valueY = 127;
  UART.nunchuck.valueX = 127; //set to default value
  UART.nunchuck.lowerButton = false;
  UART.nunchuck.upperButton = false;


  radio.begin();
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, pipe);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

  while (!Serial) {
    ; //wait for Serial 0
  }
  UART.setSerialPort(&Serial); //Set serial 0
}

void loop() {
  getVescData();

  // If transmission is available
  if (radio.available())
  {
    // The next time a transmission is received on pipe, the data in gotByte will be sent back in the acknowledgement (this could later be changed to data from VESC!)
    radio.writeAckPayload(pipe, &data, sizeof(data));

    // Read the actual message
    radio.read(&ControlValues, sizeof(ControlValues));

    int counter_checksum=0;
    counter_checksum = ControlValues.ValY;
    if(ControlValues.upperButton == true) counter_checksum++;
    if(ControlValues.upperButton == true) counter_checksum++;
    if(ControlValues.checksum == counter_checksum)
    {
      recievedData = true;
    }
    
  }

  if (recievedData == true)
  {
    // A speed is received from the transmitter (remote).

    lastTimeReceived = millis();
    recievedData = false;

    //Write Data to VESC
    UART.nunchuck.valueY = ControlValues.ValY;
    UART.nunchuck.valueX = 127; //set to default value
    UART.nunchuck.lowerButton = ControlValues.lowerButton;
    UART.nunchuck.upperButton = ControlValues.upperButton;
  }
  else if ((millis() - lastTimeReceived) > timeoutMax)
  {
    //case of timeout set all to 0
    UART.nunchuck.valueY = 127;
    UART.nunchuck.valueX = 127; //set to default value
    UART.nunchuck.lowerButton = false;
    UART.nunchuck.upperButton = false;
  }
  UART.setNunchuckValues();
}

void getVescData() {

  if (millis() - lastDataCheck >= 250) {

    lastDataCheck = millis();

    if ( UART.getVescValues() ) {
      data.rpm = UART.data.rpm;
      data.inpVoltage = UART.data.inpVoltage;
      data.ampHours = UART.data.ampHours;
      data.tachometerAbs = UART.data.tachometerAbs;
      data.avgInputCurrent = UART.data.avgInputCurrent;
    }
else {
      data.ampHours = 0.0;
      data.inpVoltage = 0.0;
      data.rpm = 0;
      data.tachometerAbs = 0;
      data.avgInputCurrent = 0;
    }
  }
}
