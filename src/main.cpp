/**
 * This is a program that operates a CNC Drill Press.
 *
 * @author Jose Ramirez
 * @version 1.0
 * @date 05-06-2024
 * 
 * @note Future implementations: 
 *      - Add button for start and stop main void loop()
 *      - Add microSD card reading and writing functionality for holeData 
 */

#include <Arduino.h>
#include "AccelStepperWithDistance.h"

// Define Driver interface
const int DRIVER = 1;

// Define pin number for steppers
const int x_step_pin = 0;
const int x_dir_pin = 1;
const int y_step_pin = 2;
const int y_dir_pin = 3;
const int z_step_pin = 8;
const int z_dir_pin = 9;

// Define pins for motor
const int EN_motor_pin = 4; // Motor Enable pin
const int RPWM_pin = 5; // RPWM pin 
const int LPWM_pin = 6; /// LPWM pin

// Define pin numbers for home switches (limit switches)
const int home_switch = 7;

// Define pin for sensors
const int SENSOR_PIN = A0;  // Analog input pin connected to the sensor
const int CONTINUITY_PIN = A1;

// Create instances for three stepper motors
AccelStepperWithDistance stepperZ(DRIVER, z_step_pin, z_dir_pin); // X-Axis Stepper
AccelStepperWithDistance stepperY(DRIVER, y_step_pin, y_dir_pin); // Y-Axis Stepper
AccelStepperWithDistance stepperX(DRIVER, x_step_pin, x_dir_pin); // Z-Axis Stepper

/// @struct drillData Define Struct to store drillData
/// @brief Defines a struct to hold drill data: {float, float, float, String}
/// @param xLocation Defines a member float that stores the X- Coordinate Location in [mm].
/// @param yLocation Defines a member float that stores the Y- Coordinate Location in [mm].
/// @param drillDepth Defines a member float that stores the desired Drill depth in [mm].
/// @param feedType Defines a member String that stores the desired drill operation.
struct drillData {
  float xLocation;
  float yLocation;
  float drillDepth;
  String feedType;
};
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

// Declare and Define variables to be used in program. Make sure to update
const int totalNumOfHoles = 6;// Number of holes to be drilled. Used in Struct array declaration
const int SFM = 160; // Surface feet per minute, value from look-up table
const float drillDiameter = 0.188; // Drill diameter decimal in [inches]

const float X_offset = 12.7;
const float Y_offset = 25.445;

// Declare and define a hole array of drillData struct 
drillData hole[totalNumOfHoles]= {
  {-2.54,  -5.08, 11, "peck" },
  {0.00,    7.62, 11, "peck" },
  {12.7,    7.62, 11, "peck" },
  {-6.35,    7.62, 11, "peck" },
  {19.05,  10.16, 11, "peck" },
  {19.05,   0.00, 11, "peck" },
}; 
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

// Method to home a stepper motor using limit switches in series
void homeStepper(AccelStepperWithDistance& stepper, int homeSwitchPin) {
  Serial.print("Homing ");
  int initial_homing = -1;

  stepper.setMaxSpeed(500.0);        // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(1000.0);    // Set Acceleration of Stepper
  delay(50);
  // if switch is active, then deactivate it. else activate switch, then deactivate
  if(!digitalRead(homeSwitchPin)){
    // Make the Stepper move CW until the switch is Activated
    while (!digitalRead(homeSwitchPin)) {
      stepper.moveTo(initial_homing);   // Set the position to move to
      initial_homing--;                  // Decrease by 1 for the move if needed
      stepper.run();                    // Start moving the stepper
    }
  }
  
  delay(1000);
  // Back-Off the Switch
  initial_homing = 1;
  while (digitalRead(homeSwitchPin)) {
    stepper.moveTo(initial_homing);   // Set the position to move to
    initial_homing++;                  // Decrease by 1 for the move if needed
    stepper.run();                    // Start moving the steppery
  }
  
  stepper.setCurrentPosition(0);     // Set the current position as 0
  stepper.runToNewPosition(150);     // make sure backed off switch
  stepper.setCurrentPosition(0);
  Serial.println("completed.");
  delay(1000);
}

// Method to locate top of part using continuity test
void locateTopSurface(AccelStepperWithDistance& Zstepper, int continuityTestPin){
  int moveStep = 1;

  // move to bed center
  stepperX.runToNewDistance(90);
  stepperY.runToNewDistance(60);

  while(digitalRead(continuityTestPin) ){
    Zstepper.moveTo(moveStep);  // Set the position to move to
    Zstepper.run();             // Decrease by 1 for the move if needed
    moveStep++;                // Start moving the stepper
    delay(1);                  // delay
  }
  
  Zstepper.setCurrentPosition(0);     // Set the current position to sensor offset X
  Zstepper.runToNewDistance(-5);     // move enough to clear part
}

// Method to find the part edges using IR Sensor
void findPart(AccelStepperWithDistance& Xstepper, AccelStepperWithDistance& Ystepper, int continuityTestPin, int diameterDrill){
  int moveStep = 1;
  float offset = (-(diameterDrill*2.54))/2; // in millimeters
  
  stepperX.runToNewDistance(0);   // move x to start location
  stepperY.runToNewDistance(55);  // move Y to meet the part near center
  stepperZ.runToNewDistance(3);   //move Z drill bit down to touch part

  // locate X- corner
  while (digitalRead(continuityTestPin))
  {
    Xstepper.moveTo(moveStep);  // Set the position to move to
    Xstepper.run();             // Decrease by 1 for the move if needed
    moveStep++;                // Start moving the stepper
  }
  stepperX.setCurrentPosition(offset);     // Set the current position to sensor offset X

 // locate Y- corner
  moveStep = 1;
  stepperZ.runToNewDistance(-5);  // move Z drill bit up to clear the part
  stepperX.runToNewDistance(3);   // move X to 0 location over the part
  stepperY.runToNewDistance(0);   // move Y to start position
  stepperZ.runToNewDistance(4);   // move Z down to touch part


  while (digitalRead(continuityTestPin))
  {
    Ystepper.moveTo(moveStep);  // Set the position to move to
    Ystepper.run();             // Decrease by 1 for the move if needed
    moveStep++;                // Start moving the stepper
  }
  stepperY.setCurrentPosition(offset);  // Set the current position to sensor offset Y 25mm*1rev/4mm*800pulses/1rev
  
  // Go to the corner
  stepperZ.runToNewDistance(-5);
  stepperY.runToNewDistance(0);
  stepperX.runToNewDistance(0);
  
}

/// @brief Method function calculates Spindle speed 
/// @param cutterSize drill diameter in [inches].
/// @param cuttingSpeed Surface Feet Per minute from Lookup chart.
/// @return Required PWM value to Drive the Motor.
int calcSpindleSpeed(float cutterSize, int cuttingSpeed){
    // Example: speedMotorPWM = ((1,500*4)/12,000)*255 = 127.5 ; Note: analogWrite rounds up to nearest integer
  const int gearRatio = 4;
  const int maxMotorRPM = 12000;

  // Calculate required Spindle Speed for material
  int speedSpindle = (cuttingSpeed * (12 / 3.14159)) / cutterSize;
  Serial.print("Required RPM at spindle is: ");
  Serial.print(speedSpindle);
  Serial.println(" RPM");

  // Calculate required Motor duty cycle for PWM
  int speedMotorPWM = ((speedSpindle * 255 * gearRatio) / maxMotorRPM) ; // 255 is the highest value in PWM signal 100% speed
  Serial.println(speedMotorPWM);
  return speedMotorPWM;

}

/// @fn Method fucntion calculates Feed rate (Plunge speed).
/// @param cutterSize drill diameter in [inches].
/// @param cuttingSpeed Surface Feet Per minute from Lookup chart.
/// @return Speed in Steps/seconds
int calcFeedRate(float cutterSize, int cuttingSpeed){
   int speedSpindle = (cuttingSpeed * (12 / 3.14159)) / cutterSize;

   double rateFeed = (speedSpindle*(0.001*16)*2.54)/(60); // inches per second
   int rateFeedSteps = rateFeed*800/4; // steps/sec

   Serial.println(rateFeedSteps);

   return rateFeedSteps;
}

// Method to start the motor/spindle drive at the required PWM Value
void runSpindle(int enable, int LPWM, int RPWM, int speedPWM){
  
  analogWrite(enable, speedPWM); // Set motor speed
  digitalWrite(RPWM, LOW);   // Turn motor CW. HIGH = ON, LOW = OFF
  digitalWrite(LPWM, HIGH);   // Turns on motor CCW. HIGH = ON, LOW = OFF


}

// Method to Decrement and stop the Motor using PWM 
void stopSpindle(int enable, int LPWM, int RPWM, int speedPWM){

  for(speedPWM <= 255; speedPWM >= 0; speedPWM--) {
      analogWrite(enable, speedPWM); // Set motor speed
      digitalWrite(LPWM, HIGH);   // Turns on motor CCW. HIGH = ON, LOW = OFF
      digitalWrite(RPWM, LOW);   // Turn motor CW. HIGH = ON, LOW = OFF
      delay(5); // Short delay to see the speed decrease
    };
    
    digitalWrite(LPWM, LOW);   // Turns on motor CCW. HIGH = ON, LOW = OFF
    digitalWrite(RPWM, LOW);  // Turn motor CW. HIGH = ON, LOW = OFF
}

///////////////////////////////////////////////////////////////////////////////////////
/// 
/// @brief Arduino setup() Function.
/// 
///   This funciton initializes all required pins and homes the steppers at startup.
///  
//////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);

  // Initialize home switches
  pinMode(home_switch, INPUT_PULLUP);
  pinMode(EN_motor_pin, OUTPUT);   
  pinMode(LPWM_pin, OUTPUT);
  pinMode(RPWM_pin, OUTPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP); // Set the sensor pin as an input
  pinMode(CONTINUITY_PIN, INPUT_PULLUP);
  delay(5);

  // Welcome Message
  Serial.println("Welcome to CNC Drill Press Program- Group 3");

  // Home each stepper
  homeStepper(stepperZ, home_switch); // Z-axis
  homeStepper(stepperX, home_switch); // X-axis
  homeStepper(stepperY, home_switch); // Y-axis

  // Set values for AccelStepperWith Distances
  // X-Axis 
  stepperX.setMicroStep(1);
  stepperX.setStepsPerRotation(800);
  stepperX.setAnglePerRotation(360);
  stepperX.setDistancePerRotation(4);
  
  // y-Axis
  stepperY.setMicroStep(1);
  stepperY.setStepsPerRotation(800);
  stepperY.setAnglePerRotation(360);
  stepperY.setDistancePerRotation(4);

  // Z-Axis
  stepperZ.setMicroStep(1);
  stepperZ.setStepsPerRotation(800);
  stepperZ.setAnglePerRotation(360);
  stepperZ.setDistancePerRotation(4);

}



///////////////////////////////////////////////////////////////////////////////////////
/// 
/// @brief Main Arduino Loop() Function.
/// 
///   This loop executes all of the work for the drill operation. 
/// 
//////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // Declaring and initializing  variables for loop
  int rpmRequired = 0;
  int feedRate = 0;

  // Calculate (duty cycle PWM) for RPM required in drill operation
  rpmRequired = calcSpindleSpeed(drillDiameter, SFM);

  // Calculate feed rate steps/s
  feedRate= calcFeedRate(drillDiameter, SFM);

  // Set max speed and acceleration for all steppers for travel while drilling (slow)
  stepperY.setMaxSpeed(400);
  stepperY.setAcceleration(800); // do not call too often. check at the end to possibly delete some of these
  stepperX.setMaxSpeed(400);
  stepperX.setAcceleration(800);

  // Continuity test
  locateTopSurface(stepperZ, CONTINUITY_PIN);

  // Detect part edge
  findPart(stepperX, stepperY, CONTINUITY_PIN, drillDiameter);

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);

  stepperX.runToNewDistance(X_offset);
  stepperY.runToNewDistance(Y_offset);

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);

  stepperZ.setMaxSpeed(feedRate); //set feedrate travel speed
  stepperZ.setAcceleration(feedRate);

  // for totalNumOfHoles Drill Sequence 
  for (size_t i = 0; i < totalNumOfHoles; i++)
  {
    Serial.print("Hole # ");
    Serial.print(i+1);
    Serial.println(" Started");

    stepperX.runToNewDistance(hole[i].xLocation);
    stepperY.runToNewDistance(hole[i].yLocation);
    delay(100);

    if (hole[i].feedType = "plunge")
    {
      runSpindle(EN_motor_pin, LPWM_pin, RPWM_pin, rpmRequired);
      
      // Set the travel distance (depth of Hole)
      stepperZ.moveToDistance(hole[i].drillDepth);
      
      // Loop to move to position is reached
      while (stepperZ.distanceToGo() != 0)
      {
        stepperZ.run();
      }

      delay(100);
    }

    if (hole[i].feedType = "peck")
    {
      runSpindle(EN_motor_pin, LPWM_pin, RPWM_pin, rpmRequired);
      
      //while (stepperZ.getCurrentPositionDistance()!= hole[i].drillDepth) // may not need while loop
     // {
        for (int peck = 10; peck > 0; peck--)
        {
          // Set the travel distance (depth of Hole)
          stepperZ.moveToDistance(hole[i].drillDepth*(1/peck));
          
          // Loop to move to position is reached
          while (stepperZ.distanceToGo() != 0)
          {
            stepperZ.run();
          }
          // Back-Out Drill bit
          stepperZ.runToNewDistance(hole[i].drillDepth*(-1/peck));
        }

      //}
    }
    delay(100);
    stepperZ.runToNewPosition(-7); // Back-Off drill bit 
    stopSpindle(EN_motor_pin, LPWM_pin, RPWM_pin, rpmRequired);
    stepperZ.runToNewPosition(-20); // Clearance height
    Serial.print("  ->Hole # ");
    Serial.print(i+1);
    Serial.println(" Completed");
  }

  // Home each stepper
  homeStepper(stepperZ, home_switch); // Z-axis
  homeStepper(stepperX, home_switch); // X-axis
  homeStepper(stepperY, home_switch); // Y-axis

  Serial.println("Program Completed");
  for (;;); // loop halt   

}

