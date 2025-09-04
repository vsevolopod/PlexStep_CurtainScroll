//Dual Curtain Scroll With PlexStep Boards
//By Vsev Krawczeniuk

//worknotes: figure out the acceleration, figure out lower limit

#include <DMXSerial.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#define SL // define which driver we are programming. Options: SL, CS, SR
bool verbose = true; // prints diagnostic information to the serial monitor

#if defined (SL)
int startAddress = 1; // unit 1 is address 1, unit 2 is address 7, unit 3 is address 13
bool invDir = true;
bool dualMotor = true;
unsigned long motor1limit = 1000; //how far the motor is to travel
unsigned long motor2limit = 1000;
#endif

#if defined (CS)
int startAddress = 7; // unit 1 is address 1, unit 2 is address 7, unit 3 is address 13
bool invDir = true;
bool dualMotor = false;
unsigned long motor1limit = 80000; //how far the motor is to travel
unsigned long  motor2limit = 80000;
#endif

#if defined (SR)
int startAddress = 13; // unit 1 is address 1, unit 2 is address 7, unit 3 is address 13
bool invDir = false;
bool dualMotor = true;
unsigned long motor1limit = 90000; //how far the motor is to travel
unsigned long motor2limit = 80000;
#endif

int motor1ps_ad = startAddress;
int motor1ps_fn_ad = startAddress + 1;
int motor1sp_ad = startAddress + 2;
int motor2ps_ad = startAddress + 3;
int motor2ps_fn_ad = startAddress + 4;
int motor2sp_ad = startAddress + 5;

int mot1crs = 0;
int mot2crs = 0;

int mot2fn = 0;
int mot1fn = 0;

long mot1spd = 0;
long mot2spd = 0;

long mot1pos = 0;
long mot2pos = 0;

unsigned long maxSpd = 3000;  //sets max speed for all steppers
//unsigned long maxSpd2 = 5000;  //sets max speed for all steppers
long homeUpSpd = -1000; //how fast the curtain runs up to home
long homeDnSpd = 1000; //how fast the curtain runs down to home
const int startPos = 500; //how far the curtain runs out before beginning the homing sequence

bool m1homed = false;
bool m2homed = false;
bool atStartPos1 = false;
bool atStartPos2 = false;

#define motRX 9
#define motTX 8
#define AmotCLK A0
#define BmotCLK 12

#define AstepPin 6
#define AdirPin 4

#define BstepPin 13
#define BdirPin 5

#define AenPin 10
#define BenPin 11

#define m1lim A3 //motor 1 top limit switch
#define m2lim A5 //motor 2 top limit switch

#define interface 1

AccelStepper motor1 = AccelStepper(interface, AstepPin, AdirPin);
AccelStepper motor2 = AccelStepper(interface, BstepPin, BdirPin);
MultiStepper motors;



void setup() {
//if (verbose == true){
Serial.begin(19200);
//}

  DMXSerial.init(DMXReceiver);
  DMXSerial.maxChannel(192);

  pinMode(AenPin,OUTPUT);
  pinMode(BenPin,OUTPUT);

  motor1.setMaxSpeed(maxSpd);
  motor2.setMaxSpeed(maxSpd);

  motor1.setPinsInverted(invDir, false, false); //direction, step, enable, 1-3 direction = true, 4,5 direction = false 
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
  bool topLim1State = digitalRead(m1lim);
      motor1.setCurrentPosition(0);
  //long motor1pos = motor1.currentPosition();
    //if (atStartPos1 == false){
      Serial.println("Homing Motor 1");
      motor1.setSpeed(homeDnSpd);
      motor1.moveTo(startPos);
     motor1.runSpeedToPosition();
      atStartPos1 = true;

    if (atStartPos1 == true){
      Serial.println("at starting point 1");
          motor1.setSpeed(homeUpSpd);
          motor1.runSpeed();
      }
    if (topLim1State == LOW){
      motor1.stop();
      motor1.setCurrentPosition(0);
      m1homed = true;
      Serial.print("Motor 1 Homed");
    }
    return m1homed;
}

bool homeMotor2() {
  bool topLim2State = digitalRead(m2lim);
  //long motor1pos = motor1.currentPosition();
    if (atStartPos2 == false){
      Serial.println("Homing Motor 2");
      motor1.setSpeed(homeDnSpd);
      motor1.moveTo(startPos);
      atStartPos2 = true;
    }
    if (atStartPos2 == true){
      Serial.println("at starting point 2");
          motor2.setSpeed(homeUpSpd);
          motor2.runSpeed();
      }
    if (topLim2State == LOW){
      motor2.stop();
      motor2.setCurrentPosition(0);
      m2homed = true;
      Serial.print("Motor 2 Homed");
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
  }
   unsigned long lastPacket = DMXSerial.noDataSince();
       if (lastPacket < 5000) {

      long allMotorPos[] = {0,0};
          mot1crs = DMXSerial.read(motor1ps_ad);
          mot1fn = DMXSerial.read(motor1ps_fn_ad);
          unsigned int motor1full = (mot1crs << 8) | mot1fn;
          unsigned long mot1val = map(motor1full, 0, 255, 0, motor1limit);
          allMotorPos[0] = mot1val;

          mot1spd = DMXSerial.read(motor1sp_ad);

          mot2crs = DMXSerial.read(motor2ps_ad);
          mot2fn = DMXSerial.read(motor2ps_fn_ad);
          unsigned int motor2full = (mot2crs << 8) | mot2fn;
          unsigned long mot2val = map(motor2full, 0, 255, 0, motor2limit);
         allMotorPos[1] = mot2val;

          mot2spd = DMXSerial.read(motor2sp_ad);

      motor1.setSpeed(mot1spd); 
      motor2.setSpeed(mot2spd); 

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
      //Serial.print(allMotorPos[0]);
      //Serial.print("  |   ");
      //Serial.print(allMotorPos[1]);
        }
    }
