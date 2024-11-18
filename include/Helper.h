#include <vectors.h>
#include <Wire.h>

extern Vector2 joy1TargetVector;
extern float joy1TargetMagnitude;

extern Vector2 joy1CurrentVector;
extern float joy1CurrentMagnitude;

extern Vector2 joy2TargetVector;
extern float joy2TargetMagnitude;

extern Vector2 joy2CurrentVector;
extern float joy2CurrentMagnitude; 

extern float distanceFromGroundBase;
extern float distanceFromGround; 
extern  float previousDistanceFromGround;

extern float distanceFromCenter;

extern unsigned long timeSinceLastInput;

enum State {
  Initialize,
  Stand,
  Car,
  Crab,
  Calibrate,
  SlamAttack
};

enum Gait {
  Tri,
  Wave,
  Ripple,
  Bi,
  Quad,
  Hop  
};

enum LegState {
  Propelling,
  Lifting,
  Standing,
  Reset
};

extern State currentState;
extern Gait currentGait;
extern Gait previousGait;
extern int currentGaitID;

extern Vector3 ControlPoints[10];
extern Vector3 RotateControlPoints[10];

extern float strideMultiplier[6];
extern float rotationMultiplier[6];

extern float points;
extern int cycleProgress[6];
extern LegState legStates[6];
extern int standProgress;
extern Vector3 currentPoints[6];
extern Vector3 cycleStartPoints[6];

extern float liftHeight;
extern float landHeight;
extern float strideOvershoot;
extern float distanceFromCenter;

extern Vector3 targetRot;

float lerp(float a, float b, float f);
Vector2 lerp(Vector2 a, Vector2 b, float f);
float calculateHypotenuse(float x, float y);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

//Beizer
Vector2 GetPointOnBezierCurve(Vector2* points, int numPoints, float t);
Vector3 GetPointOnBezierCurve(Vector3* points, int numPoints, float t);
int binomialCoefficient(int n, int k);


int angleToMicroseconds(double angle);
//simple
void resetMovementVectors();
void setCycleStartPoints(int leg);
void setCycleStartPoints();

//print
void print_value(String name, float value, bool newLine);
void print_value(String name, String value, bool newLine);
void print_value(String name, Vector3 value, bool newLine);

void sendLegData(uint8_t leg, uint16_t coxaMicroseconds, uint16_t femurMicroseconds, uint16_t tibiaMicroseconds);


void moveToPos(int leg, Vector3 pos);