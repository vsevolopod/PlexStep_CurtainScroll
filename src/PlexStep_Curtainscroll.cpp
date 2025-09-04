//Dual Curtain Scroll With PlexStep Boards
//By Vsev Krawczeniuk

#include <DMXSerial.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#define CS // define which driver we are programming. Options: SL, CS, SR
bool verbose = true; // prints diagnostic information to the serial monitor

#if defined (SL)
int startAddress = 1; // unit 1 is address 1, unit 2 is address 7, unit 3 is address 13
bool invDir = true;
bool dualMotor = true;
unsigned long motor1limit = 90000; //how far the motor is to travel
unsigned long motor2limit = 80000;
#endif

#if defined (CS)
int startAddress = 7; // unit 1 is address 1, unit 2 is address 7, unit 3 is address 13
bool invDir = true;
bool dualMotor = false;
unsigned long motor1limit = 200000; //how far the motor is to travel
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

unsigned long motor1crs = 0;
unsigned long motor1fn = 0;
unsigned long motor2crs = 0;
unsigned long motor2fn = 0;

unsigned long motor1spd = 0;
unsigned long motor1pos = 0;
unsigned long motor2spd = 0;
unsigned long motor2pos = 0;


unsigned long motor1spin = 0;
unsigned long motor2spin = 0;

unsigned long maxSpd = 5000;  //sets max speed for all steppers
long homeUpSpd = -2000; //how fast the curtain runs up to home
long homeDnSpd = 2000; //how fast the curtain runs down to home
const int startPos = 1000; //how far the curtain runs out before beginning the homing sequence

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

#define motorInterfaceType 1

AccelStepper motor1 = AccelStepper(motorInterfaceType, AstepPin, AdirPin);
AccelStepper motor2 = AccelStepper(motorInterfaceType, BstepPin, BdirPin);


void setup() {
if (verbose == true){
  Serial.begin(9600);
}

  DMXSerial.init(DMXReceiver);
  DMXSerial.maxChannel(192);

  pinMode(AenPin,OUTPUT);
  pinMode(BenPin,OUTPUT);

  motor1.setMaxSpeed(maxSpd);
  motor2.setMaxSpeed(maxSpd);

  motor1.setPinsInverted(invDir, false, false); //direction, step, enable, 1-3 direction = true, 4,5 direction = false 
  motor2.setPinsInverted(invDir, false, false); //direction, step, enable, 1-3 direction = true, 4,5 direction = false 


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

void motor1control(){
  unsigned long lastPacket = DMXSerial.noDataSince();

     if (lastPacket < 5000) {
    motor1crs = DMXSerial.read(motor1ps_ad);
    motor1fn = DMXSerial.read(motor1ps_fn_ad);
    long motor1full = (motor1crs << 8) | motor1fn;
        motor1spin = DMXSerial.read(motor1sp_ad);
        motor1pos = motor1full;
        //motor1pos = map(motor1full, 0, 65535, 0, motor1limit);
        motor1spd = map(motor1spin, 0, 255, 0, maxSpd);
      motor1.setSpeed(motor1spd); 
      motor1.moveTo(motor1full);
      motor1.runSpeedToPosition();
  }
}
void motor2control(){
  unsigned long lastPacket = DMXSerial.noDataSince();

     if (lastPacket < 5000) {
    motor2crs = DMXSerial.read(motor2ps_ad);
    motor2fn = DMXSerial.read(motor2ps_fn_ad);
    unsigned int motor2full = (motor2crs << 8) | motor2fn;
        motor2spin = DMXSerial.read(motor2sp_ad);
        motor2pos = map(motor2full, 0, 65535, 0, motor2limit);
        motor2spd = map(motor2spin, 0, 255, 0, maxSpd);
      motor2.setSpeed(motor2spd); 
      motor2.moveTo(motor2full);
      motor2.runSpeedToPosition();
  }
}

bool homeMotor1() {
  bool topLim1State = digitalRead(m1lim);
  //long motor1pos = motor1.currentPosition();
    if (atStartPos1 == false){
      Serial.println("Homing Motor 1");
      motor1.setSpeed(homeDnSpd);
      motor1.moveTo(startPos);
      motor1.runSpeedToPosition();
      atStartPos1 = true;
    }
    if (atStartPos1 == true){
      Serial.println("at starting point");
          motor1.setSpeed(homeUpSpd);
          motor1.runSpeed();
      }
    if (topLim1State == LOW){
      motor1.stop();
      motor1.setCurrentPosition(0);
      m1homed = true;
      Serial.print("Motor 1 Homed");
      //Serial.println(motor1pos);
    }
  
    return m1homed;
}

bool homeMotor2() {
  bool topLim2State = digitalRead(m2lim);
  //long motor2pos = motor2.currentPosition();
    if (atStartPos2 == false){
      Serial.println("Homing Motor 2");
      motor2.setSpeed(homeDnSpd);
      motor2.moveTo(startPos);
      motor2.runSpeedToPosition();
      atStartPos2 = true;
    }
    if (atStartPos2 == true){
      Serial.println("Motor 2 at starting point");
          motor2.setSpeed(homeUpSpd);
          motor2.runSpeed();
      }
    if (topLim2State == LOW){
      motor2.stop();
      motor2.setCurrentPosition(0);
      m2homed = true;
      Serial.print("Motor 2 Homed");
      //Serial.println(motor2pos);
    }
    return m2homed;
}

void loop() {

 while (m1homed == false){
    homeMotor1();
  }
  if (m1homed == true){
    motor1control();
  }

  if (dualMotor == true){
   while (m2homed == false){
    homeMotor2();
 }
  if (m2homed == true){
    motor2control();
  }
}

  if (verbose == true){
    //Serial.print("motor positions:");
     // Serial.print('\n'); //newline
   // Serial.print("_________________");
     // Serial.print('\n'); //newline
    Serial.print("motor 1");
    Serial.print('\t');//prints tab
    Serial.print("|");
    Serial.print(" motor 2");
      Serial.print('\n');//prints newline
    Serial.print(motor1pos, DEC);
    Serial.print('\t');//prints tab
    Serial.print("|");//prints tab
    Serial.print(" ");//prints tab
    Serial.print(motor2pos, DEC);
    Serial.print('\n');
  }
} 
