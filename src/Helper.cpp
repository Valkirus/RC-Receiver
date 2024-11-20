#include "Helper.h"

int totalGaits = 6;
Gait gaits[6] = {Tri,Wave,Ripple,Bi,Quad,Hop};

const int servo2040Address = 8; // I2C address of the Servo2040

float points = 1000;
int cycleProgress[6];
LegState legStates[6];
int standProgress = 0;

float standingDistanceAdjustment = 0;

float distanceFromGroundBase = -40;
float distanceFromGround = 0; 
float previousDistanceFromGround = 0;

float liftHeight = 70;
float landHeight = 10;
float strideOvershoot = 0;
float distanceFromCenter = 80;

float crabTargetForwardAmount = 0;
float crabForwardAmount = 0;

Vector2 joy1TargetVector = Vector2(0,0);
float joy1TargetMagnitude = 0;

Vector2 joy1CurrentVector = Vector2(0,0);
float joy1CurrentMagnitude = 0;

Vector2 joy2TargetVector = Vector2(0,0);
float joy2TargetMagnitude = 0;

Vector2 joy2CurrentVector = Vector2(0,0);
float joy2CurrentMagnitude = 0;

unsigned long timeSinceLastInput = 0;

float landingBuffer = 0;

int attackCooldown = 0;

Vector3 targetRot(180, 0, 180);

Gait currentGait = Tri;
Gait previousGait = Tri;

int currentGaitID = 0;

State currentState = Initialize;

Vector3 ControlPoints[10];
Vector3 RotateControlPoints[10];

Vector3 currentRot(180, 0, 180);

float strideMultiplier[6] = {1, 1, 1, -1, -1, -1};
float rotationMultiplier[6] = {-1, 0, 1, -1, 0 , 1};

const Vector3 offsets1 = {90,75,-45};
const Vector3 offsets2 = {93,75,-45};
const Vector3 offsets3 = {93,75,-48}; 
const Vector3 offsets4 = {87,80,-45};
const Vector3 offsets5 = {85,89,-45};
const Vector3 offsets6 = {93,110,80};
const Vector3 offsets[6] = {offsets1, offsets2, offsets3, offsets4, offsets5, offsets6};


const float a1 = 29;  //Coxa Length
const float a2 = 76; //Femur Length
const float a3 = 106; //Tibia Length   
float legLength = a1+a2+a3;

Vector3 currentPoints[6];
Vector3 cycleStartPoints[6];

float lerp(float a, float b, float f)
{
    return a * (1.0 - f) + (b * f);
}

Vector2 lerp(Vector2 a, Vector2 b, float f)
{
    return Vector2(lerp(a.x, b.x, f), lerp(a.y, b.y, f));
}

float calculateHypotenuse(float x, float y) {
  float result = sqrt(pow(x, 2) + pow(y, 2));
  return result;
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


Vector2 GetPointOnBezierCurve(Vector2* points, int numPoints, float t) {
  Vector2 pos;

  for (int i = 0; i < numPoints; i++) {
    float b = binomialCoefficient(numPoints - 1, i) * pow(1 - t, numPoints - 1 - i) * pow(t, i);
    pos.x += b * points[i].x;
    pos.y += b * points[i].y;
  }

  return pos;
}


Vector3 GetPointOnBezierCurve(Vector3* points, int numPoints, float t) {
  Vector3 pos;

  for (int i = 0; i < numPoints; i++) {
    float b = binomialCoefficient(numPoints - 1, i) * pow(1 - t, numPoints - 1 - i) * pow(t, i);
    pos.x += b * points[i].x;
    pos.y += b * points[i].y;
    pos.z += b * points[i].z;
  }

  return pos;
}

int binomialCoefficient(int n, int k) {
  int result = 1;

  // Calculate the binomial coefficient using the formula:
  // (n!) / (k! * (n - k)!)
  for (int i = 1; i <= k; i++) {
    result *= (n - (k - i));
    result /= i;
  }

  return result;
}

int angleToMicroseconds(double angle) {
  double val = 500.0 + (((2500.0 - 500.0) / 180.0) * angle);
  return (int)val;
}


void resetMovementVectors(){
  joy1CurrentVector = Vector2(0,0);
  joy1CurrentMagnitude = 0;

  joy2CurrentVector = Vector2(0,0);
  joy2CurrentMagnitude = 0;
}

void setCycleStartPoints(int leg){
  cycleStartPoints[leg] = currentPoints[leg];    
}

void setCycleStartPoints(){
  for(int i = 0; i < 6; i++){
    cycleStartPoints[i] = currentPoints[i]; 
  }     
}

void print_value(String name, float value, bool newLine){
  Serial.print(name + ": ");

  if(newLine)Serial.println(value);
  else Serial.print(value);
  
}

void print_value(String name, String value, bool newLine){
  Serial.print(name + ": ");
  if(newLine)Serial.println(value);
  else Serial.print(value);
}

void print_value(String name, Vector3 value, bool newLine){
  Serial.print(name + ": ");
  if(newLine)Serial.println(value.toString());
  else Serial.print(value.toString());
}

void sendLegData(uint8_t leg, uint16_t coxaMicroseconds, uint16_t femurMicroseconds, uint16_t tibiaMicroseconds) {
  Wire.beginTransmission(servo2040Address); // Start I2C transmission
  
  switch (leg)
  {
  case 0:
    Wire.write(leg);                         // Send the leg identifier (1 byte)
    Wire.write(1);                           // Send servo number 1
    Wire.write(coxaMicroseconds >> 8);       // MSB of coxa PWM value
    Wire.write(coxaMicroseconds & 0xFF);     // LSB of coxa PWM value

    Wire.write(2);                           // Send servo number 2
    Wire.write(femurMicroseconds >> 8);      // MSB of femur PWM value
    Wire.write(femurMicroseconds & 0xFF);    // LSB of femur PWM value

    Wire.write(3);                           // Send servo number 3
    Wire.write(tibiaMicroseconds >> 8);      // MSB of tibia PWM value
    Wire.write(tibiaMicroseconds & 0xFF);    // LSB of tibia PWM value
    break;

  case 1:
    Wire.write(leg);                         // Send the leg identifier (1 byte)
    Wire.write(4);                           // Send servo number 1
    Wire.write(coxaMicroseconds >> 8);       // MSB of coxa PWM value
    Wire.write(coxaMicroseconds & 0xFF);     // LSB of coxa PWM value

    Wire.write(5);                           // Send servo number 2
    Wire.write(femurMicroseconds >> 8);      // MSB of femur PWM value
    Wire.write(femurMicroseconds & 0xFF);    // LSB of femur PWM value

    Wire.write(6);                           // Send servo number 3
    Wire.write(tibiaMicroseconds >> 8);      // MSB of tibia PWM value
    Wire.write(tibiaMicroseconds & 0xFF);    // LSB of tibia PWM value
    break;

  case 2:
    Wire.write(leg);                         // Send the leg identifier (1 byte)
    Wire.write(7);                           // Send servo number 1
    Wire.write(coxaMicroseconds >> 8);       // MSB of coxa PWM value
    Wire.write(coxaMicroseconds & 0xFF);     // LSB of coxa PWM value

    Wire.write(8);                           // Send servo number 2
    Wire.write(femurMicroseconds >> 8);      // MSB of femur PWM value
    Wire.write(femurMicroseconds & 0xFF);    // LSB of femur PWM value

    Wire.write(9);                           // Send servo number 3
    Wire.write(tibiaMicroseconds >> 8);      // MSB of tibia PWM value
    Wire.write(tibiaMicroseconds & 0xFF);    // LSB of tibia PWM value
    break;

  case 3:
    Wire.write(leg);                         // Send the leg identifier (1 byte)
    Wire.write(10);                           // Send servo number 1
    Wire.write(coxaMicroseconds >> 8);       // MSB of coxa PWM value
    Wire.write(coxaMicroseconds & 0xFF);     // LSB of coxa PWM value

    Wire.write(11);                           // Send servo number 2
    Wire.write(femurMicroseconds >> 8);      // MSB of femur PWM value
    Wire.write(femurMicroseconds & 0xFF);    // LSB of femur PWM value

    Wire.write(12);                           // Send servo number 3
    Wire.write(tibiaMicroseconds >> 8);      // MSB of tibia PWM value
    Wire.write(tibiaMicroseconds & 0xFF);    // LSB of tibia PWM value
    break;

  case 4:
    Wire.write(leg);                         // Send the leg identifier (1 byte)
    Wire.write(13);                           // Send servo number 1
    Wire.write(coxaMicroseconds >> 8);       // MSB of coxa PWM value
    Wire.write(coxaMicroseconds & 0xFF);     // LSB of coxa PWM value

    Wire.write(14);                           // Send servo number 2
    Wire.write(femurMicroseconds >> 8);      // MSB of femur PWM value
    Wire.write(femurMicroseconds & 0xFF);    // LSB of femur PWM value

    Wire.write(15);                           // Send servo number 3
    Wire.write(tibiaMicroseconds >> 8);      // MSB of tibia PWM value
    Wire.write(tibiaMicroseconds & 0xFF);    // LSB of tibia PWM value
    break;
  
  case 5:
    Wire.write(leg);                         // Send the leg identifier (1 byte)
    Wire.write(16);                           // Send servo number 1
    Wire.write(coxaMicroseconds >> 8);       // MSB of coxa PWM value
    Wire.write(coxaMicroseconds & 0xFF);     // LSB of coxa PWM value

    Wire.write(17);                           // Send servo number 2
    Wire.write(femurMicroseconds >> 8);      // MSB of femur PWM value
    Wire.write(femurMicroseconds & 0xFF);    // LSB of femur PWM value

    Wire.write(18);                           // Send servo number 3
    Wire.write(tibiaMicroseconds >> 8);      // MSB of tibia PWM value
    Wire.write(tibiaMicroseconds & 0xFF);    // LSB of tibia PWM value
    break;
  
  default:
    break;
  }

  Wire.endTransmission();                  // End I2C transmission
}


void moveToPos(int leg, Vector3 pos){
  currentPoints[leg] = pos;
  
  float dis = Vector3(0,0,0).distanceTo(pos);
  if(dis > legLength){
    print_value("Point impossible to reach", pos, false);
    print_value("Distance",dis, true);
    return;
  }

  float x = pos.x;
  float y = pos.y;
  float z = pos.z;

  float o1 = offsets[leg].x;
  float o2 = offsets[leg].y;
  float o3 = offsets[leg].z;

  float theta1 = atan2(y,x) * (180 / PI) + o1; // base angle
  float l = sqrt(x*x + y*y); // x and y extension 
  float l1 = l - a1;
  float h = sqrt(l1*l1 + z*z);

  float phi1 = acos(constrain((pow(h,2) + pow(a2,2) - pow(a3,2)) / (2*h*a2),-1,1));
  float phi2 = atan2(z, l1);
  float theta2 = (phi1 + phi2) * 180 / PI + o2;
  float phi3 = acos(constrain((pow(a2,2) + pow(a3,2) - pow(h,2)) / (2*a2*a3),-1,1));
  float theta3 = 180 - (phi3 * 180 / PI) + o3;

  if (leg >= 3) {
    theta2 = -(theta2); // Invert femur angle
    theta3 = -(theta3 + 90); // Invert tibia angle
  }

  targetRot = Vector3(theta1,theta2,theta3);
  
  
  int coxaMicroseconds = angleToMicroseconds(targetRot.x);
  int femurMicroseconds = angleToMicroseconds(targetRot.y);
  int tibiaMicroseconds = angleToMicroseconds(targetRot.z);

  coxaMicroseconds = constrain(coxaMicroseconds, 500, 2500);
  femurMicroseconds = constrain(femurMicroseconds, 500, 2500);
  tibiaMicroseconds = constrain(tibiaMicroseconds, 500, 2500);

  sendLegData(leg, coxaMicroseconds, femurMicroseconds, tibiaMicroseconds);

  return; 
}