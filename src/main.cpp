#include <Arduino.h>
#include <carState.h>

long elapsedTime = 0;
long loopStartTime = 0;

bool LED = 0;

void setup() {
  Serial.begin(9600);  // Serial communication at 115200 baud
  Wire.begin(); // Join the I2C bus as master
  SetupRC();
}

void loop() {
  elapsedTime = millis() - loopStartTime;
  loopStartTime = millis();
  bool connected = GetData();

  if (connected) {
    //RC_DisplayData();

    double joy1x = map(rc_data.joy1_X,0,254,-100,100);
    double joy1y = map(rc_data.joy1_Y,0,254,-100,100);

    double joy2x = map(rc_data.joy2_X,0,254,-100,100);
    double joy2y = map(rc_data.joy2_Y,0,254,-100,100);
    
    joy1TargetVector = Vector2(joy1x,joy1y);
    joy1TargetMagnitude = constrain(calculateHypotenuse(abs(joy1x),abs(joy1y)),0,100);   
    
    joy2TargetVector = Vector2(joy2x,joy2y);
    joy2TargetMagnitude = constrain(calculateHypotenuse(abs(joy2x),abs(joy2y)),0,100);  

    previousDistanceFromGround = distanceFromGround;
    distanceFromGround = distanceFromGroundBase + rc_data.slider1 * -1.7;
    distanceFromCenter = 170;
  }

  joy1CurrentVector = lerp(joy1CurrentVector, joy1TargetVector, 0.08);
  joy1CurrentMagnitude = lerp(joy1CurrentMagnitude, joy1TargetMagnitude, 0.08);

  joy2CurrentVector = lerp(joy2CurrentVector, joy2TargetVector, 0.12);
  joy2CurrentMagnitude = lerp(joy2CurrentMagnitude, joy2TargetMagnitude, 0.12);  

  if (abs(joy1CurrentMagnitude) >= 10 || abs(joy2CurrentMagnitude) >= 10) {
    carState();
    timeSinceLastInput = millis();
    return;
  }

}
