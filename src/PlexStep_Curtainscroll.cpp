//DMX Fixture 1

#include <DMXSerial.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
//#include <EEPROM.h>

int startAddress = 7; // unit 1 is address 1, unit 2 is address 7, unit 3 is address 13

int motor1ps_ad = startAddress;
int motor1ps_fn_ad = startAddress + 1;
int motor1sp_ad = startAddress + 2;
int motor2ps_ad = startAddress + 3;
int motor2ps_fn_ad = startAddress + 4;
int motor2sp_ad = startAddress + 5;

int motor1crs = 0;
int motor1fn = 0;
int motor2crs = 0;
int motor2fn = 0;

unsigned long motor1spin = 0;
unsigned long motor2spin = 0;

unsigned long motor1limit = 80000; //how far the motor is to travel
unsigned long motor2limit = 80000;

unsigned long maxSpd = 5000;  //sets max speed for all steppers
long homeUpSpd = 1500; //how fast the curtain runs up to home
long homeDnSpd = -1500; //how fast the curtain runs down to home, must be negative
long startPos = -8000; //how far the curtain runs out before beginning the homing sequence

bool motorsHomed = false;
bool topHomed = false;
bool botHomed = false;
bool atStartPos = false;



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

#define AtopLim A2 //top limit switch
#define AbotLim A3 //bottom limit switch
#define BtopLim A4 //top limit switch
#define BbotLim A5 //bottom limit switch

#define motorInterfaceType 1

AccelStepper motor1 = AccelStepper(motorInterfaceType, AstepPin, AdirPin);
AccelStepper motor2 = AccelStepper(motorInterfaceType, BstepPin, BdirPin);


unsigned long motor1spd = 0;
unsigned long motor1pos = 0;
unsigned long motor2spd = 0;
unsigned long motor2pos = 0;


void setup() {

  Serial.begin(9600);

  DMXSerial.init(DMXReceiver);
  DMXSerial.maxChannel(192);

  pinMode(AenPin,OUTPUT);
  pinMode(BenPin,OUTPUT);

  motor1.setMaxSpeed(maxSpd);
  motor2.setMaxSpeed(maxSpd);

  motor1.setPinsInverted(true, false, false); //direction, step, enable, 1-3 direction = true, 4,5 direction = false 
  motor2.setPinsInverted(false, false, false); //direction, step, enable 


  pinMode(AtopLim, INPUT_PULLUP);
  pinMode(AbotLim, INPUT_PULLUP);
  pinMode(motRX, OUTPUT);
  pinMode(motTX, OUTPUT);
  pinMode(AmotCLK, OUTPUT);
  pinMode(BmotCLK, OUTPUT);
  digitalWrite(AenPin,LOW); // to make Stepper motor enable
  digitalWrite(BenPin,LOW); // to make Stepper motor enable
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
    unsigned int motor1full = (motor1crs << 8) | motor1fn;
        motor1spin = DMXSerial.read(motor1sp_ad);
        motor1pos = map(motor1full, 0, 65535, 0, motor1limit);
        motor1spd = map(motor1spin, 0, 255, 0, maxSpd);
      motor1.setSpeed(motor1spd); 
      motor1.moveTo(motor1full);
      motor1.runSpeedToPosition();
      //Serial.println(motor1pos);
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
     // Serial.println(motor2crs);
  }
}

bool homeMotor1() {
  bool topLimState = digitalRead(AtopLim);
  bool botLimState = digitalRead(AbotLim);
  motor1pos = motor1.currentPosition();

    if (botHomed == false){
     // Serial.println("homing bottom");
      motor1.setSpeed(homeDnSpd);
      motor1.run();
    }
    if (botLimState == LOW){
      motor1.stop();
      motor1.setCurrentPosition(0); 
      botHomed = true;
      Serial.print("bottom homed = ");
      Serial.println(botHomed);
    }
    if (botHomed == true && topHomed == false){
      motor1.setSpeed(homeUpSpd);
      motor1.run();
     // Serial.println("homing top");
       // Serial.println(motor1pos);
    }
    if (topLimState == LOW){
      //motor1.stop();
      motor1limit = motor1.currentPosition();
      topHomed = true;
     // Serial.print("top homed = ");
      Serial.println(motor1pos);
    }
    if(topHomed && botHomed == true){
      motorsHomed = true;
      Serial.print("we're home!");
    }
}

void loop() {

 //while (motorsHomed == false){
  //  homeMotor1();
 // }
 // if (motorsHomed == true){
    motor1control();
    motor2control();
 // }
 //   if (atStartPos == false){
//  }
} 
