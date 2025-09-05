//Dual Curtain Scroll With PlexStep Boards
//By Vsev Krawczeniuk
//This is the working version as of September 4, 2025.

#include <DMXSerial.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#define SL // define which driver we are programming. Options: SL, CS, SR

#if defined (SL) //This section defines things specific to the SL driver
int startAddress = 1; // unit 1 is address 1, unit 2 is address 7, unit 3 is address 13
bool invDir = true; //SL is inverted
bool dualMotor = true; //SL uses two motors. This does affect performance when both run at the same time
unsigned int motor1limit = 1000; //careful with the motor limits, they interact with mapVals to limit the travel.
unsigned int motor2limit = 1000;
const int map1Val = 1000; //careful with these. See above. Lower mapVals make the curtains travel in further.  
const int map2Val = 520;
unsigned long maxSpd = 8000;  //sets max speed for all steppers. 
#define m1lim A3 //motor 1 top limit switch
#define m2lim A5 //motor 2 top limit switch
#endif

#if defined (CS) //This section defines things specific to the CS driver
int startAddress = 7; // unit 1 is address 1, unit 2 is address 7, unit 3 is address 13
bool invDir = true; //CS is inverted
bool dualMotor = false; //the center only has a single curtain. This must be false or it gets stuck in the motor 2 homing sequence
unsigned long motor1limit = 1000; //tested working motorLimit and mapVal, sends it in far enough at a decent speed
unsigned long  motor2limit = 1000;
const int map1Val = 512;
const int map2Val = 512; //left in to not break things. Doesnt do anything with CS
unsigned long maxSpd = 6000;  //sets max speed for all steppers
#define m1lim A3 //motor 1 top limit switch
#define m2lim A5 //motor 2 top limit switch
#endif

#if defined (SR) // This section defines things specific to the SR driver
int startAddress = 13; // unit 1 is address 1, unit 2 is address 7, unit 3 is address 13
bool invDir = false; // the motors are on the opposite side on SR, so they are not inverted
bool dualMotor = true;
unsigned long motor1limit = 1000; //motorLimits and mapVals are the opposite of SL since they're mirrored.
unsigned long motor2limit = 1000;
const int map1Val = 520;
const int map2Val = 1000;
unsigned long maxSpd = 8000;  //sets max speed for all steppers
#define m1lim A3 //motor 1 top limit switch
#define m2lim A5 //motor 2 top limit switch
#endif

int motor1ps_ad = startAddress; //address for motor 1 course
int motor1ps_fn_ad = startAddress + 1; //address for motor 1 fine
//int motor1sp_ad = startAddress + 2; //speed values not in use. Keeping current addresses to use the same fixture profile. Commented out for speed.
int motor2ps_ad = startAddress + 3; //address for motor 2 course
int motor2ps_fn_ad = startAddress + 4; //address for motor 2 fine
//int motor2sp_ad = startAddress + 5;

int mot1crs = 0; //initialize course values
int mot2crs = 0;

int mot2fn = 0; //initialize fine values
int mot1fn = 0;

long mot1pos = 0; //initialize position values
long mot2pos = 0;

long homeUpSpd = -3000; //how fast the curtain runs up to home
long homeDnSpd = 3000; //how fast the curtain runs down to home
const int startPos = 500; //how far the curtain runs out before beginning the homing sequence

bool m1homed = false; //turns true after homeMotor1 completes
bool m2homed = false; //turns true after homeMotor2 completes

bool atStartPos1 = false; //these don't do much, they're immediately set true after the first step of homeMotors rubns
bool atStartPos2 = false;

#define motRX 9 //UART on drivers not implemented
#define motTX 8 //
#define AmotCLK A0 //not used
#define BmotCLK 12 //

#define AstepPin 6
#define AdirPin 4

#define BstepPin 13
#define BdirPin 5

#define AenPin 10
#define BenPin 11

#define interface 1 // this sets AccelStepper to use motor drivers. Double check this if updating the library.

AccelStepper motor1 = AccelStepper(interface, AstepPin, AdirPin);
AccelStepper motor2 = AccelStepper(interface, BstepPin, BdirPin);
MultiStepper motors;

void setup() {

  DMXSerial.init(DMXReceiver);
  DMXSerial.maxChannel(24);

  pinMode(AenPin,OUTPUT);
  pinMode(BenPin,OUTPUT);

  motor1.setMaxSpeed(maxSpd);
  motor2.setMaxSpeed(maxSpd);

  motor1.setPinsInverted(invDir, false, false); //direction is set in the defines for each driver
  motor2.setPinsInverted(invDir, false, false); //direction, step, enable, 1-3 direction = true, 4,5 direction = false 

  motors.addStepper(motor1);
  motors.addStepper(motor2);


  pinMode(m1lim, INPUT_PULLUP);
  pinMode(m2lim, INPUT_PULLUP);
  pinMode(motRX, OUTPUT);
  pinMode(motTX, OUTPUT);
  pinMode(AmotCLK, OUTPUT);
  pinMode(BmotCLK, OUTPUT);
  digitalWrite(AenPin,LOW); 
  digitalWrite(BenPin,LOW); 
  digitalWrite(motRX, LOW);
  digitalWrite(motTX, LOW);
  digitalWrite(AmotCLK, LOW);
  digitalWrite(BmotCLK, LOW);
}

bool homeMotor1() {
  bool topLim1State = digitalRead(m1lim); // this first step is not working. I left it for future use, it makes no difference in this state
      motor1.setSpeed(homeDnSpd);
      motor1.moveTo(startPos); //this needs to run speed, probably for a set amount of time, using milis.
      atStartPos1 = true; // because this currently doesnt work, it is recommended to lower the curtains slightly before shut down.

    if (atStartPos1 == true){
          motor1.setSpeed(homeUpSpd);
          motor1.runSpeed();
      }
    if (topLim1State == LOW){
      motor1.stop();
      motor1.setCurrentPosition(0);
      m1homed = true;
    }
    return m1homed;
}

bool homeMotor2() {
  bool topLim2State = digitalRead(m2lim);

    if (atStartPos2 == false){
      motor1.setSpeed(homeDnSpd);
      motor1.moveTo(startPos);
      atStartPos2 = true;
    }
    if (atStartPos2 == true){
          motor2.setSpeed(homeUpSpd);
          motor2.runSpeed();
    }
    if (topLim2State == LOW){
      motor2.stop();
      motor2.setCurrentPosition(0);
      m2homed = true;
    }
    return m2homed;
}

void loop() {

 while (m1homed == false){
    homeMotor1();
  }
  if (dualMotor == true){
   while (m2homed == false){
    homeMotor2();
   }

      long allMotorPos[] = {0,0};
          mot1crs = DMXSerial.read(motor1ps_ad);
          mot1fn = DMXSerial.read(motor1ps_fn_ad);

          unsigned int motor1full = (mot1crs << 8) | mot1fn;
          unsigned long mot1val = map(motor1full, 0, map1Val, 0, motor1limit);
        allMotorPos[0] = mot1val;

          mot2crs = DMXSerial.read(motor2ps_ad);
          mot2fn = DMXSerial.read(motor2ps_fn_ad);

          unsigned int motor2full = (mot2crs << 8) | mot2fn;
          unsigned long mot2val = map(motor2full, 0, map2Val, 0, motor2limit);
        allMotorPos[1] = mot2val;

    motors.moveTo(allMotorPos);
      if (DMXSerial.dataUpdated()){
        motors.run();
      if (motor1.distanceToGo() == 0){
        motor1.stop();
      }      
      if (motor2.distanceToGo() == 0){
        motor2.stop();
      }
    }
  }

  if (dualMotor == false){

      long allMotorPos[] = {0,0};
          mot1crs = DMXSerial.read(motor1ps_ad);
          mot1fn = DMXSerial.read(motor1ps_fn_ad);

          unsigned int motor1full = (mot1crs << 8) | mot1fn;
          unsigned long mot1val = map(motor1full, 0, map1Val, 0, motor1limit);
          allMotorPos[0] = mot1val;

    motors.moveTo(allMotorPos);
      if (DMXSerial.dataUpdated()){
        motors.run();
      if (motor1.distanceToGo() == 0){
        motor1.stop();
      }      
    }
  }
}
