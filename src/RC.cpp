#include <SPI.h>
#include <RF24.h>
#include <RC.h>

#define LED_BUILTIN 2  // Built-in LED is typically on GPIO2 (D4) on NodeMCU

RF24 radio(3, 15);
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 0;

unsigned long rc_last_received_time = 0;
unsigned long rc_timeout = 1000;

RC_Data_Package rc_data;
RC_Data_Package rc_data_previous;

void RC_DisplayData(){
  Serial.print("Joy1 X: ");
  Serial.print(rc_data.joy1_X);

  Serial.print(" | Joy1 Y: ");
  Serial.print(rc_data.joy1_Y);

  Serial.print(" | Joy1 Button: ");
  Serial.print(rc_data.joy1_Button);

  Serial.print(" | Joy2 X: ");
  Serial.print(rc_data.joy2_X);

  Serial.print(" | Joy2 Y: ");
  Serial.print(rc_data.joy2_Y);

  Serial.print(" | Joy2 Button: ");
  Serial.print(rc_data.joy2_Button);

  Serial.print(" | Pot 1: ");
  Serial.print(rc_data.slider1);

  Serial.print(" | Pot 2: ");
  Serial.println(rc_data.slider2);
}

void RC_ResetData(){
  rc_data.joy1_X = 127;
  rc_data.joy1_Y = 180;
  rc_data.joy1_Button = 0;

  rc_data.joy2_X = 127;
  rc_data.joy2_Y = 127;
  rc_data.joy2_Button = 0;

  rc_data.slider1 = 40;
  rc_data.slider2 = 25;

  rc_data.pushButton1 = 0;
  rc_data.pushButton2 = 0;
}

void SetupRC() {
  Serial.println(F("Initializing RF24..."));
  RC_ResetData();
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {
      delay(1000); // Prevent further execution
    }
  } else {
    Serial.println(F("radio hardware is ready!"));
    digitalWrite(LED_BUILTIN, HIGH);
  }
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS); // Set to 2 Mbps for faster communication
  radio.setPayloadSize(sizeof(rc_data));
  radio.setChannel(112);
  radio.openReadingPipe(0, address[radioNumber]);
  //radio.enableAckPayload();
  radio.startListening();
  Serial.println(F("RF24 setup complete."));
}


/*
void initializeHexPayload(){
  hex_data.current_sensor_value = 0;
}
*/
bool GetData(){  
  // This device is a RX node
  uint8_t pipe;
  if (radio.available(&pipe)) {
    uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
    radio.read(&rc_data, bytes);            // fetch payload from FIFO 
    //RC_DisplayData();me
    rc_last_received_time = millis();
  }
  if(millis() - rc_last_received_time >  rc_timeout) return false;
  return true;
}