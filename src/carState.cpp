#include "carState.h"

float forwardAmount;
float turnAmount;
float  tArray[6];
int ControlPointsAmount = 0;
int RotateControlPointsAmount = 0;
float pushFraction = 3.0/6.0;
float speedMultiplier = 0.5;
float strideLengthMultiplier = 1.5;
float liftHeightMultiplier = 1.0;
float maxStrideLength = 200;
float maxSpeed = 100;
float legPlacementAngle = 56;

int leftSlider = 50;
float globalSpeedMultiplier = 0.55;
float globalRotationMultiplier = 0.55;

Vector3 getGaitPoint(int leg, float pushFraction){  
 

  float rotateStrideLength = joy2CurrentVector.x * globalRotationMultiplier;
  Vector2 v = joy1CurrentVector * Vector2(1,strideLengthMultiplier);
  v.y = constrain(v.y,-maxStrideLength/2, maxStrideLength/2);
  v = v * globalSpeedMultiplier;

  float weightSum = abs(forwardAmount) + abs(turnAmount);

  float t = tArray[leg];

  //if(leg == 0)print_value("cycleProgress[leg]",cycleProgress[leg]);
  
  
  //Propelling
  if(t < pushFraction){ 
    if(legStates[leg] != Propelling)setCycleStartPoints(leg);
    legStates[leg] = Propelling;

    ControlPoints[0] = cycleStartPoints[leg];
    ControlPoints[1] = Vector3(v.x * strideMultiplier[leg] + distanceFromCenter, -v.y * strideMultiplier[leg], distanceFromGround).rotate(legPlacementAngle * rotationMultiplier[leg], Vector2(distanceFromCenter,0));
    ControlPointsAmount = 2;    
    Vector3 straightPoint = GetPointOnBezierCurve(ControlPoints, ControlPointsAmount, mapFloat(t,0,pushFraction,0,1));

    RotateControlPoints[0] = cycleStartPoints[leg];
    RotateControlPoints[1] = { distanceFromCenter + 40, 0, distanceFromGround };
    RotateControlPoints[2] = { distanceFromCenter, rotateStrideLength, distanceFromGround };
    RotateControlPointsAmount = 3;    
    Vector3 rotatePoint = GetPointOnBezierCurve(RotateControlPoints, RotateControlPointsAmount, mapFloat(t,0,pushFraction,0,1));

    //if(leg == 0)print_value("pushing point",(straightPoint*abs(forwardAmount) + rotatePoint*abs(turnAmount))/ weightSum);

    return (straightPoint*abs(forwardAmount) + rotatePoint*abs(turnAmount))/ weightSum;
  }

  //Lifting
  else{
    if(legStates[leg] != Lifting)setCycleStartPoints(leg);
    legStates[leg] = Lifting;

    ControlPoints[0] = cycleStartPoints[leg];
    ControlPoints[1] = cycleStartPoints[leg] + Vector3(0,0,liftHeight * liftHeightMultiplier);
    ControlPoints[2] = Vector3(-v.x * strideMultiplier[leg] + distanceFromCenter, (v.y + strideOvershoot) * strideMultiplier[leg], distanceFromGround + landHeight).rotate(legPlacementAngle * rotationMultiplier[leg], Vector2(distanceFromCenter,0));
    ControlPoints[3] = Vector3(-v.x * strideMultiplier[leg] + distanceFromCenter, v.y * strideMultiplier[leg], distanceFromGround).rotate(legPlacementAngle * rotationMultiplier[leg], Vector2(distanceFromCenter,0));
    ControlPointsAmount = 4;
    Vector3 straightPoint = GetPointOnBezierCurve(ControlPoints, ControlPointsAmount, mapFloat(t,pushFraction,1,0,1));

    RotateControlPoints[0] = cycleStartPoints[leg];
    RotateControlPoints[1] = cycleStartPoints[leg] + Vector3(0,0,liftHeight * liftHeightMultiplier);
    RotateControlPoints[2] = { distanceFromCenter + 40, 0, distanceFromGround + liftHeight * liftHeightMultiplier};
    RotateControlPoints[3] = { distanceFromCenter, -(rotateStrideLength + strideOvershoot), distanceFromGround + landHeight};
    RotateControlPoints[4] = { distanceFromCenter, -rotateStrideLength, distanceFromGround};
    RotateControlPointsAmount = 5;
    Vector3 rotatePoint =  GetPointOnBezierCurve(RotateControlPoints, RotateControlPointsAmount, mapFloat(t,pushFraction,1,0,1));

    //if(leg == 0)print_value("lifting point",(straightPoint*abs(forwardAmount) + rotatePoint*abs(turnAmount))/ weightSum);

    return (straightPoint*abs(forwardAmount) + rotatePoint*abs(turnAmount))/ weightSum;
  }  
}

void carState() {
  leftSlider = (int)rc_data.slider2;
  globalSpeedMultiplier = (leftSlider + 10.0)*0.01;
  globalRotationMultiplier = map(rc_data.slider2,0,100,40,130) * 0.01;

  if (currentState != Car || previousGait != currentGait) {
    currentState = Car;

    //Initialize Leg States
    for(int i = 0; i < 6; i++){
      legStates[i] = Reset;
    }   

    switch (currentGait) {
      case Tri:
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 2);
        cycleProgress[2] = 0;
        cycleProgress[3] = (points / 2);
        cycleProgress[4] = 0;
        cycleProgress[5] = (points / 2);

        pushFraction = 3.1/6.0;
        speedMultiplier = 1;
        strideLengthMultiplier = 1.2;
        liftHeightMultiplier = 1.1;
        maxStrideLength = 240;
        maxSpeed = 200;
        break;

      case Wave:
        //Offsets
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 6);
        cycleProgress[2] = (points / 6)*2;
        cycleProgress[3] = (points / 6)*5;
        cycleProgress[4] = (points / 6)*4;
        cycleProgress[5] = (points / 6)*3;

        //Percentage Time On Ground
        pushFraction = 5.0/6.0; 

        speedMultiplier = 0.40;
        strideLengthMultiplier = 2;
        liftHeightMultiplier = 1.3;
        maxStrideLength = 150;
        maxSpeed = 160;
        break;

      case Ripple:
        //Offsets
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 6)*4;
        cycleProgress[2] = (points / 6)*2;
        cycleProgress[3] = (points / 6)*5;
        cycleProgress[4] = (points / 6);
        cycleProgress[5] = (points / 6)*3;

        //Percentage Time On Ground
        pushFraction = 3.2/6.0;


        speedMultiplier = 1;
        strideLengthMultiplier = 1.3;
        liftHeightMultiplier = 1;
        maxStrideLength = 220;
        maxSpeed = 200;
        break;

      case Bi:
        //Offsets
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 3);
        cycleProgress[2] = (points / 3)*2;
        cycleProgress[3] = 0;
        cycleProgress[4] = (points / 3);
        cycleProgress[5] = (points / 3)*2;

        //Percentage Time On Ground
        pushFraction = 2.1/6.0;

        
        speedMultiplier = 4;        
        strideLengthMultiplier = 1;
        liftHeightMultiplier = 1.8;
        maxStrideLength = 230;
        maxSpeed = 130;
        break;

      case Quad:
        //Offsets
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 3);
        cycleProgress[2] = (points / 3)*2;
        cycleProgress[3] = 0;
        cycleProgress[4] = (points / 3);
        cycleProgress[5] = (points / 3)*2;

        //Percentage Time On Ground
        pushFraction = 4.1/6.0;

        
        speedMultiplier = 1;        
        strideLengthMultiplier = 1.2;
        liftHeightMultiplier = 1.8;
        maxStrideLength = 220;
        maxSpeed = 200;
        break;

      case Hop:
        //Offsets
        cycleProgress[0] = 0;
        cycleProgress[1] = 0;
        cycleProgress[2] = 0;
        cycleProgress[3] = 0;
        cycleProgress[4] = 0;
        cycleProgress[5] = 0;

        //Percentage Time On Ground        
        pushFraction = 3/6.0;

        speedMultiplier = 1;
        strideLengthMultiplier = 1.6;
        liftHeightMultiplier = 2.5;
        maxStrideLength = 240;
        maxSpeed = 200;
        break;
    }      
  }

  
  
  for(int i = 0; i < 6; i++){
    tArray[i] = (float)cycleProgress[i] / points;    
  }  

  forwardAmount = joy1CurrentMagnitude;
  turnAmount = joy2CurrentVector.x;

  moveToPos(0, getGaitPoint(0, pushFraction));
  moveToPos(1, getGaitPoint(1, pushFraction));
  moveToPos(2, getGaitPoint(2, pushFraction));
  moveToPos(3, getGaitPoint(3, pushFraction));
  moveToPos(4, getGaitPoint(4, pushFraction));
  moveToPos(5, getGaitPoint(5, pushFraction));
  
  

  float progressChangeAmount = (max(abs(forwardAmount),abs(turnAmount))* speedMultiplier)*globalSpeedMultiplier ;

  
  progressChangeAmount = constrain(progressChangeAmount,0,maxSpeed*globalSpeedMultiplier);

  for(int i = 0; i < 6; i++){
    cycleProgress[i] += progressChangeAmount;

    if(cycleProgress[i] >= points){
      cycleProgress[i] = cycleProgress[i] - points;
    }
  } 
}

//Standing Control Points Array
Vector3 SCPA[6][10];

Vector3 standingStartPoints[6];      //the points the legs are at in the beginning of the standing state
Vector3 standingInBetweenPoints[6];  //the middle points of the bezier curves that the legs will follow to smoothly transition to the end points
Vector3 standingEndPoint;

int currentLegs[3] = { -1, -1, -1 };
int standLoops = 0;

void standingState() {
  bool moveAllAtOnce = false;
  bool highLift = false;
  setCycleStartPoints();
  standingEndPoint = Vector3(distanceFromCenter, 0, distanceFromGround + standingDistanceAdjustment);
  standLoops = 2;
  // We only set the starting, inbetween, and ending points one time, which is when we enter the standing state.
  if (currentState == Calibrate || currentState == Initialize || currentState == SlamAttack) moveAllAtOnce = true;
  if (currentState == SlamAttack) highLift = true;
  if (currentState != Stand) {
    
    set3HighestLeg();
    standLoops = 0;
    standProgress = 0;
    memcpy(standingStartPoints, currentPoints, sizeof(currentPoints[0]) * 6);
    currentState = Stand;

    // Calculate the inbetween and ending points
    for (int i = 0; i < 6; i++) {
      Vector3 inBetweenPoint = standingStartPoints[i];
      inBetweenPoint.x = (inBetweenPoint.x + standingEndPoint.x) / 2;
      inBetweenPoint.y = (inBetweenPoint.y + standingEndPoint.y) / 2;

      inBetweenPoint.z = ((inBetweenPoint.z + standingEndPoint.z) / 2);
      if(abs(inBetweenPoint.z - standingEndPoint.z) < 50 )inBetweenPoint.z += 50;
      if(highLift)inBetweenPoint.z += 150;

      standingInBetweenPoints[i] = inBetweenPoint;

      SCPA[i][0] = standingStartPoints[i];
      SCPA[i][1] = standingInBetweenPoints[i];
      SCPA[i][2] = standingEndPoint;
    }

    for(int i = 0; i < 6; i++){
      legStates[i] = Standing;
    } 
  }

  //update distance from ground constantly
  for (int i = 0; i < 6; i++) {
    SCPA[i][2] = standingEndPoint;
  }

  //readjusting. This takes about a second
  while(standLoops < 2){
    standProgress += 25;
    if(highLift){
      standProgress += 40 - 50 * ((float)standProgress / points);
    }

    float t = (float)standProgress / points;
    if (t > 1) {
      t = 1;
    }

    if(moveAllAtOnce){
      for (int i = 0; i < 6; i++) {
        moveToPos(i, GetPointOnBezierCurve(SCPA[i], 3, t));
      }

      if (standProgress > points) {
        standProgress = 0;
        standLoops = 2;
      }
    }

    else{
      for (int i = 0; i < 3; i++) {
        if (currentLegs[i] != -1) {
          moveToPos(currentLegs[i], GetPointOnBezierCurve(SCPA[currentLegs[i]], 3, t));
        }
      }

      if (standProgress > points) {
        standProgress = 0;
        standLoops ++;
        set3HighestLeg();
      }
    }
  }


  //constantly move to the standing end position
  for (int i = 0; i < 6; i++) {
    moveToPos(i, GetPointOnBezierCurve(SCPA[i], 3, 1));
  }
  return;
}

void set3HighestLeg() {

  currentLegs[0] = -1;
  currentLegs[1] = -1;
  currentLegs[2] = -1;
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 6; i++) {  //go through the legs
      //if the leg is already on the list of current legs, skip it
      if (currentLegs[0] == i || currentLegs[1] == i || currentLegs[2] == i) continue;

      //if the leg is already in position, dont add it
      if (currentPoints[i] == standingEndPoint) continue;

      //if the legs z is greater than the leg already there, add it
      if (currentLegs[j] == -1 || currentPoints[i].z > currentPoints[currentLegs[j]].z) {
        currentLegs[j] = i;
      }
    }
  }
}