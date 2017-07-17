//----------------------------------------------
//    Wire Library   - I2C Bus Communications
//----------------------------------------------
#include <Wire.h>
//    This is required for...
//    Magnetometer
//    PWM Controller  
//    OLED Display (optional)
//
//
//Thanks, Adafruit for the awesome Servo Driver Library!
#include <Adafruit_PWMServoDriver.h>
//
//OLED Display interface
#define _Digole_Serial_I2C_  //To tell compiler compile the special communication only, 
#include <DigoleSerial.h>
//
//Compass interface & calibration code
#include "compass.h"


#include <string.h>



//----------------------------------------------
//    SoftSerial Library   - For Serial IrDa
//----------------------------------------------
#include <SoftwareSerial.h>



//----------------------------------------------
//    EEProm Library   - For Compass Cal Vals
//----------------------------------------------
#include <EEPROM.h>



//This is what is required for the timer event support
//----------------------------------------------
//    Timer Library   - Timed Events
//----------------------------------------------
#include "Timer.h"








const uint8_t startscreenIV[1024] PROGMEM={
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,128,2,2,4,0,16,63,128,127,0,128,128,126,1,0,1
,255,2,2,4,0,16,0,64,0,128,128,128,195,1,0,1
,129,2,2,4,0,16,31,224,63,192,128,128,129,1,0,1
,129,2,2,4,0,16,48,32,96,64,128,128,129,1,1,255
,129,2,2,4,0,48,32,32,64,64,128,128,129,1,3,1
,255,3,252,4,15,224,63,192,127,128,255,0,255,1,2,1
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,1
,0,0,0,4,0,0,0,0,0,0,0,0,0,1,3,255
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,15,255,248,255,255,15,248,240,51,255,254,63,255,227,255,252
,31,255,251,255,255,159,243,248,119,255,254,255,255,243,255,254
,31,248,3,252,127,159,231,252,247,254,0,255,143,240,0,255
,31,252,7,248,127,159,239,254,247,255,1,255,15,240,0,255
,31,254,15,248,127,191,255,255,247,255,131,255,15,240,1,255
,15,255,31,240,127,63,252,255,227,255,199,254,15,224,1,254
,15,255,159,240,255,63,248,255,227,255,231,254,31,224,1,254
,7,255,223,224,255,127,224,127,225,255,247,252,31,224,3,254
,1,255,223,225,254,255,128,127,192,127,247,252,63,192,7,252
,0,255,223,225,254,254,0,63,128,63,247,252,63,192,15,248
,255,255,223,255,252,252,0,31,191,255,247,255,255,191,255,240
,255,255,31,255,249,240,0,31,191,255,199,255,255,63,255,224
,255,240,3,255,129,224,0,15,63,252,0,255,240,63,255,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};










// EEPROM addresses for persisted data
const int CompassXOffsetAddress = 0;
const int CompassYOffsetAddress = 8;
const int CompassZOffsetAddress = 16;

//This is the enable pin for the IrDa Serial Port
const int IrDaEnablePin = 5;
//This is the Rx/Tx pins for the IrDa Serial Port
#define RxIR A0
#define TxIR A1

//Piezo Speaker Pin
const int speakerPin = 4;

//Battery voltage divider
const int vRegMonPin = A2;




//Create the PWM Servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x78);

DigoleSerialDisp mydisp(&Wire,'\x27');

//Create the SoftSerial interface object
SoftwareSerial infraredSerial(RxIR, TxIR);

//This is what is required for the timer event support
Timer tmr;





//SoftSerial Support Variables
int intPipeCnt = 0;
String aryInput[4];
String temp;
const unsigned int MAX_INPUT = 50;






// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
//#define SERVOMIN  120 // this is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX  460 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN  140 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  430 // this is the 'maximum' pulse length count (out of 4096)
double ServoOffset;  //Find the zero center of the servo offset.  We want the positional output to reflect turning...
//
//
//Trying these settings with the Motor Controller
const uint16_t MOTORZERO = 0; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORREADY = 200; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORSTART = 292; // was 275...  this is the 'minimum' pulse length count (out of 4096)
//const uint16_t MOTORIDLE = 302; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORIDLE = 310; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORMAX = 320; // 250this is the 'maximum' pulse length count (out of 4096)
// our servo # counter
uint8_t servochannelnum = 1;
uint16_t LastMOTOR;
double targHeading = 90;
int8_t valDirection = 0;
int8_t dwellCount = 0;
int8_t dwlTotalCount = 0;
int8_t dwellTest = 3; // this is minimum number of times the maneuver test must pass before a position is considered "hit"
float heading=0;
float turnRate=0;
boolean valDirection2 = false;
boolean dirPassN = false;
boolean dirPassS = false;
boolean dirPassE = false;
boolean dirPassW = false;
double baseFoundHeading = -1;   //This is the base station heading value...  -1 is the unassigned default
const double searchHeadingIncrement = 30; // This is how much we will increment the search angle every XX seconds
const uint16_t maxListensBeforeHeadingChange = 700; // 1/10th of a second for each maneuver... 1 second for each increment
int8_t listenCount = 0;
double findTargetHeading = 0;   //This is the base station heading value...  -1 is the unassigned default
boolean findTarget = false;


boolean MotorRunning = false;
boolean cmgActive = false;
boolean CMGTestState = false;
boolean servoTestState = false;
int motorspeed = 0;    // variable to store the servo position 
const int motorinit = 165;    // variable to store the servo position 
const int motorlow = 200;    // variable to store the servo position
int pos = 0;    // variable to store the servo position 
int posLast = 280;    // variable to store the servo position 
int posCenter = 280;    // variable to store the servo position 
//int posLeft = 450;    //  variable to store the servo position 
//int posRight = 120;    // variable to store the servo position 
int posLeft = 440;    // variable to store the servo position 
int posRight = 130;    // variable to store the servo position 
int posDelay = 10;





//Cosmoneer PID settings
//#define propGAIN  .1 // .1 proportional gain (the bigger the number the harder the controller pushes)
#define propGAIN  .04 // .1 proportional gain (the bigger the number the harder the controller pushes)
//#define intGAIN  .07 // .05 integral gain  (the SMALLER the number (except for 0, which turns it off,)  the more quickly the controller reacts to load changes, but the greater the risk of oscillations.)
#define intGAIN  .2 // .05 integral gain  (the SMALLER the number (except for 0, which turns it off,)  the more quickly the controller reacts to load changes, but the greater the risk of oscillations.)
float intgError;
float lastError;
float dervError;
//#define dervGAIN  3 // .5 derivative gain  (the bigger the number  the more the controller dampens oscillations (to the point where performance can be hindered))
#define dervGAIN  20 // .5 derivative gain  (the bigger the number the more the controller dampens oscillations (to the point where performance can be hindered))








//OLED Display variables
#define LCDCol 16
#define LCDRow 4
#define LCDW 128
#define LCDMaxDelayCount 4
int LCDDelayCount;









boolean InitPower = false;    //This is the initial Power UP State...  This is zero if we have just powered up!  It will be one after we have powered completely up.
boolean SleepState = false;






/*
byte patterns[18] = {
 B10000000, 100,
 B01000000, 100,
 B00100000, 100,
 B00010000, 100,
 B00001000, 100,
 B00000100, 100,
 B00000010, 100,
 B00000001, 100
 };
 */
int LEDindex = 0;
int LEDState = 0;
int mirIndex = 0;
int patternCount = 0;   //sizeof(patterns) / 2;
int ShowLEDCount = 0;   
int pingpoingPoint = 0;
byte LEDOutput = 0;
byte LEDOutTemp = 0;
byte LEDOutHold = 0;
byte LEDSetup = 0;
byte StoredSetup = 0;
int demoLEDCount = 0;//1 = Cycle LEDs
int DelayCount = 0;




int idxSensorToRead = 0;


int value = 0; // set values you need to zero









void loop() {

  // if serial data available, process it
  processIncomingIrDa();


  //Run the timer events
  //tmr.update();
  CycleLEDs();
  Maneuver();
}





void setup() {
  //Before we do anything else, let the user know we are doing something!
  bleep(3, 25, 500, 80, 3, false);



  //Prepare the port so we can see debug messages!
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps



  initIrDa();

  //Initialize variables as needed
  setVariables();

  //Setup pins, as required
  setPins();


  //Start the I2C interface
  Wire.begin();

  //i2cScan();
  //delay(3000);






  initDisplay();
  initPWM();
  initESC();
  initCompass();
  
  mydisp.clearScreen(); //Clear screen
  mydisp.setRot180();
  


  //Set the "I am initialized" flag
  InitPower = true;    //We have completely powered up.  Now set the flag so we know we are already running
  //let the user know we are going into the Main Loop!
  //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay)
  bleep(1, 20, 3500, -150, 5, false);


  //Set the LED timer event
  //tmr.every(100, CycleLEDs);
  //Set the Compass timer event
  //tmr.every(500, ReadCompass2);
  //Set the Maneuver timer event
  //tmr.every(1500, Maneuver);
  //Set the Sensor timer event
  //tmr.every(1000, readSensors);
  //
  //tmr.every(100, Maneuver);


}



void initIrDa(){
  //Turn on the IrDa Transcoder
  pinMode(IrDaEnablePin, OUTPUT);
  digitalWrite(IrDaEnablePin, LOW);  //LOW is off, HIGH is on!
  delay(1000);  
  digitalWrite(IrDaEnablePin, HIGH);  //LOW is off, HIGH is on!
  delay(5000);
  infraredSerial.begin(9600);
}

void initDisplay(){
  mydisp.begin();
  delay(500);
  /*----------for text LCD adapter and graphic LCD adapter ------------*/
  mydisp.clearScreen(); //Clear screen
  delay(100);
  mydisp.displayConfig(0);
  mydisp.displayStartScreen(0);

  mydisp.drawBitmap(0,0,128,64,startscreenIV);
  delay(3000);

  //mydisp.setLCDColRow(16,7);  //This doesn't work for the 0.96" OLED Display!
  //mydisp.clearScreen(); //Clear screen
  delay(500);
  //mydisp.clearScreen(); //Clear screen
  //delay(100);
  mydisp.disableCursor(); //disable cursor, enable cursore use: enableCursor();
  //mydisp.undoRotation();
  mydisp.setRot180();
}

void initPWM(){
  initstatusbar();
  mydisp.print("Init PWM");
  Serial.println("Init PWM");
  pwm.reset();
  pwm.begin();
  pwm.setPWMFreq(50);  // 60 is the maximum PWM frequency for Servos, suggest using 50 instead
}

void initCompass(){
  initstatusbar();
  mydisp.print("Init Compass");
  Serial.println("Init Compass");
  LoadParameters();  //Load parameters from EEProm
  //compass_x_offset = -413.98;
  //compass_y_offset = 549.80;
  //compass_z_offset = 83.99;
  compass_init(2);

  //timerDelay(1000);
  delay(1000);


  initstatusbar();
  mydisp.print("Init Compass Tst");
  //
  CompassCalibrationCheck();
  //
  initstatusbar();
  mydisp.print("Test Complete");
}

void initESC(){
  initstatusbar();
  mydisp.print("Init ESC");
  //Use this sequence to reset the Min/Max settings on the Mx-3A ESC!!
  pwm.setPWM(0, 0, MOTORZERO);  //Set the PWM to 0%, instructing the ESC that we are at the ZERO position
  delay(7000);  //These timings are critical here...  Wait seven seconds
  pwm.setPWM(0, 0, MOTORMAX);   //Set the PWM to 100%
  delay(1000);  //These timings are critical here...
  pwm.setPWM(0, 0, MOTORSTART);
  delay(1000);  //These timings are critical here...
  pwm.setPWM(0, 0, MOTORREADY);
  delay(7000);  //These timings are critical here...
}





boolean GetCheckSum(){
  byte chkSum = 0;

  for (int i = 0; i < 4;i++){
    temp = aryInput[i];
    for (int j = 0; j < temp.length();j++){
      chkSum += byte(temp[j]);
    }
    chkSum += byte('|');
  }
  if (chkSum == byte('\n')){
    chkSum +=1;
  }
  if (String(chkSum) == aryInput[4]) {
    return true;
  }
  else
  {
    return false;
  }
}



void processIncomingIrDa(){
   String content = "";
   char character;

  //digitalWrite(13, HIGH);

  // if serial data available, process it
  while(infraredSerial.available() > 0) {
      character = infraredSerial.read();
      content.concat(character);
  }

  if (content != "") {
    //mydisp.clearScreen(); //Clear screen
    //initstatusbar();
    //mydisp.print(content); 
    //LEDSetup = B00110111;
    //ShowLEDCount = 5;
    //delay(50);
    
    char content2[content.length()];
    content.toCharArray(content2, content.length());
    char *i;
    char* command = strtok_r(content2, "|",&i);

    //initstatusbar();
    //mydisp.print(command); 
    //delay(2000);

    if (command == "T") {
      LEDSetup = B00110111;
      ShowLEDCount = 5;
      delay(50);
      command = strtok_r(NULL,":",&i);
      command = strtok_r(NULL,":",&i);
      initstatusbar();
      mydisp.print(command); 
      //delay(2000);
      findTargetHeading = atof(command);
      findTarget = true;
    }
    else
    {
      if (content == "?|01|NA") {
        baseFoundHeading = targHeading;
      }
    }

  }
  else
  {
    //mydisp.clearScreen(); //Clear screen
    //mydisp.drawStr(0, 0, "No Data"); //display string at: x=0, y=0
    //delay(100);
    //LEDSetup = B00110111;
    //LEDOutput = B10011001;
    //ShowLEDCount = 2;
  }

  //digitalWrite(13, LOW);
}  








void initstatusbar()
{
  mydisp.setTextPosAbs(0,59);
  mydisp.print("                ");
  mydisp.setTextPosAbs(0,59);
}





//----------------------------------------------------------------
//
//
//   This section contains Setup subroutines
//
//
//----------------------------------------------------------------
void setVariables() {
  uint32_t currentFrequency;
}

void setPins() {
  analogReference(INTERNAL);

  //This pin is used for system activity feedback to the user
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);



  //byte LEDSetup = 0;
  //1   = On/Off
  //2   = Pattern/All
  //4   = Single/Alternates
  //8   = PingPong/Forward
  //16  = Mirror/Entire
  //32  = Flip Direction
  //64  = Blink/Normal
  //128 = Invert/Normal
  LEDSetup = B00000001;
  LEDOutput = B00000000;  //All Off!
  setLEDArray(LSBFIRST, LEDOutput);
  LEDSetup = B00000001;





  //Just a small delay
  delay(100);
}






void CompassCalibrationCheck() {
  //This subroutine starts the motor
  //and checks to see if a turn will yield
  //all four compass point crossings
  double calLastHeading;
  double calHeadingDiff;
  uint16_t countRevolution = 0;


  Serial.println("Verify Compass Calibration:\t");
  digitalWrite(13, HIGH);

  //read the initial heading
  ReadCompass2();
  //set the last heading value
  calLastHeading = bearing;

  //center the servo...
  //warning, this won't be graceful if the servo
  //is somewhere else
  pwm.setPWM(1, 0, posCenter);
  //start the motor
  runMotor(MOTORMAX);
  //run the motor up to speed
  runMotor(MOTORMAX);

  //runMotor(MOTORSTART);
  //delay(3000);
  //runMotor(MOTORMAX);
  //slight delay
  delay(300);
  //turn towards the right, or a clockwise spin
  setServo(240);
  //posCenter = 340 
  //int posLeft = 530 
  //int posRight = 120 
  //slight delay
  delay(300); 



  //initstatusbar();
  //mydisp.print("Testing");
  //delay(1000); 
  for(uint16_t steps = 0; steps<10; steps++)     // steps goes from 0 to 10 
  {                                
    Serial.println("Sre:\t");
    for(uint16_t calsteps = 0; calsteps<10; calsteps++)     // calsteps goes from 0 to 10 
    {                                
      // waits 100ms before reading the next position
      digitalWrite(13, LOW);
      delay(100); 
      digitalWrite(13, HIGH);
      //read the current heading
      ReadCompass2();

      //Calculate the difference
      calHeadingDiff = bearing - calLastHeading;

      initstatusbar();
      mydisp.print("Testing: ");
      mydisp.print((int) round(calLastHeading)); //display string at: x=0, y=0
      mydisp.print("/"); //display string at: x=0, y=0
      mydisp.print((int) round(bearing)); //display string at: x=0, y=0

      if (calLastHeading < 90 && bearing > 90 && dirPassE == false) {
        dirPassE = true;
        bleep(1, 2, 3500, -500, 5, false);
      }
      if (calLastHeading < 180 && bearing > 180 && dirPassS == false) {
        dirPassS = true;
        bleep(1, 3, 3500, -500, 5, false);
      }
      if (calLastHeading < 270 && bearing > 270 && dirPassW == false) {
        dirPassW = true;
        bleep(1, 4, 3500, -500, 5, false);
      }
      if (calLastHeading < 360 && bearing > 0 && dirPassN == false) {
        dirPassN = true;
        bleep(1, 1, 3500, -500, 5, false);
      }
      if (dirPassE && dirPassS && dirPassW && dirPassN) {
        dirPassE = false;
        dirPassS = false;
        dirPassW = false;
        dirPassN = false;
        countRevolution +=1;
        mydisp.print(".");
        //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay)
        bleep(4, 20, 1000, 50, 1, false);
      }
      calLastHeading = bearing;
    } 
    //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay)
    //bleep(1, 20, 3500, -150, 10);
    bleep(1, 10, 3500, -300, 3, false);
  } 
  digitalWrite(13, LOW);


  if (countRevolution > 0) {
    bleep(3, 25, 500, 80, 3, false);
  }
  else
  {
    //calibration is needed!!
    bleep(1, 2, 3500, -2000, 10, false);
    pwm.setPWM(1, 0, posCenter);
    delay(500);
    pwm.setPWM(1, 0, 240);
    setServo(240);
    initstatusbar();
    mydisp.print("Calibrating!");
    compass_debug = 1;
    //compass_offset_calibration(3);
    compass_offset_calibration(2);
    SaveParameters();    //Save parameters after the calibration
  }
  dirPassE = false;
  dirPassS = false;
  dirPassW = false;
  dirPassN = false;

}















//----------------------------------------------------------------
//
//
//   This section contains the Contol Moment Gyro subroutines
//
//
//----------------------------------------------------------------
void Maneuver() {
  //This Sub calls the main sub, passing the appropriate values
  if (cmgActive == false) {
    cmgActive = true;




    if (baseFoundHeading == -1) {
      listenCount +=1;
      if (listenCount > maxListensBeforeHeadingChange) {
        targHeading += searchHeadingIncrement;
        if (targHeading >= 360) {
          targHeading = 0;
        }
        listenCount =0;
      }
    }
    else
    {
      if (findTarget == true) {
        targHeading = findTargetHeading;
      }
      else
      {
        targHeading = baseFoundHeading;
      }
    }





    setCMG2(targHeading, 100);
    cmgActive = false;

    //initstatusbar();
    //mydisp.print("Tgt: ");
    //mydisp.print(targHeading);


    double errHeading = abs(targHeading - bearing);

    dwlTotalCount +=1;
    // are we within range?
    if (errHeading > 0 && errHeading < 10) {
      //increment the dwell counter
      dwellCount += 1;



      //have we spent enough time at this heading?  If not, then exit the subroutine
      if (dwellCount < dwellTest) {
        //bleep(1, 1, 2000, -10, 5);
        return;
      }

      dwellTest = 10 ;
      
      findTarget = false;

/*
      //which heading have we successfully passed?
      switch ((int)targHeading) {
      case 90:    //East
        dirPassE = true;
        break;
      case 180:    //South
        dirPassS = true;
        break;
      case 270:    //West
        dirPassW = true;
        break;
      case 360:    //North
        dirPassN = true;
        break;
      }

      //move to the next heading
      if (valDirection2) {
        targHeading -= 90;
      }
      else {
        targHeading += 90;
      }

      //if the target heading is out of range, then adjust and bring it back into range
      if (targHeading > 360 || targHeading < 90) {
        valDirection2 = !valDirection2;
        if (targHeading > 360) {
          targHeading = 270;
        }
        else {
          targHeading = 180;
        }
        dirPassE = false;
        dirPassS = false;
        dirPassW = false;
        dirPassN = false;
        //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay)
        bleep(4, 20, 1000, 50, 1,false);
      }
      else {
        //decrease the dwell counter...  We missed it this time.
        dwellCount -= 1;
        //check to make sure we haven't gone below the lower range of the dwell counter
        if (dwellCount < 0) {
          dwellCount = 0;
        }
      }
*/
    }


    //decrease the dwell counter...  We missed it this time.
    dwellCount -= 1;
    //check to make sure we haven't gone below the lower range of the dwell counter
    if (dwellCount < 0) {
      dwellCount = 0;
    }
    
    if (dwlTotalCount >10) {
      dwlTotalCount = 0;
      dwellTest -= 1;
      if (dwellTest < 1) {
        dwellTest = 1;
      }
      //bleep(1, 1, 500, -10, 5, false);                                        
    }

  }
}



void setCMG2(double tgtYAW, uint16_t timeDomain) {
  //This routine is based on similar logic used in quadricopters and other systems
  //where a target is attempting to be reached...
  //This is a Parallel PID

  float lastBearing = bearing;
  float adjTurnBrake;
  float tempTurnRate=0;

  ReadCompass2();

  //http://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
  //Truth Table - Direction And Result Calculation
  //----------------------------------------------------
  //
  //
  //
  //  ABS(errHeading)  |  Direction  |  tgtYaw < finalHeading  |  > 180  |   Motor Spin  |  Examples: tgt - now
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------
  //      0 - 180      |    CW (1)   |            NO           |    NO   |       CW      |   90 - 30 = 60CW
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------
  //     180 - 360     |   CCW (-1)  |            NO           |   YES   |       CW      |  270 - 30 = 240CW  (shorter as 120CCW)
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------
  //      0 - 180      |   CCW (-1)  |           YES           |    NO   |       CW      |   30 - 90 = 60CCW
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------
  //     180 - 360     |    CW (1)   |           YES           |   YES   |       CW      |  30 - 270 = 240CW  (shorter as 120CCW)
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------

  //#define SERVOMIN  120 // this is the 'minimum' pulse length count (out of 4096)
  //#define SERVOMAX  530 // this is the 'maximum' pulse length count (out of 4096)
  //Kp = (530 - 120)/(180 - 0) = 10
  //Kp = (410)/(180) = 2.27




  double errHeading = abs(tgtYAW - bearing);

  tempTurnRate = (float) (lastBearing - bearing);
  //turnRate = turnRate * .25 + tempTurnRate * .75;
  //turnRate = turnRate * .70 + tempTurnRate * .30;
  turnRate = (turnRate * .50) + (tempTurnRate * .50);

  if (turnRate > 60)
  {
    if (lastBearing > 270 && bearing < 90)
    {
      turnRate = (360 - lastBearing) + bearing;
    }
    else if (lastBearing < 90 && bearing > 270)
    {
      turnRate = (360 - bearing) + lastBearing;
    }
  }
  turnRate = abs(turnRate);


  if (tgtYAW < bearing) {
    valDirection = -1;
  }
  else
  {
    valDirection = 1;
  }




  if (errHeading > 180) {
    //Motor must be spinning CCW
    //This result means the target position is closer if we spin in the opposite direction
    errHeading = 360 - errHeading;
    valDirection = valDirection * -1;
  }

  if (ShowLEDCount == 0) {
     setTargetHeadingLED((errHeading * valDirection));
  }



  float u = (PID(errHeading) / 7);
  if (u < 1 && u > .6) {
    u = 1;
  }

  //if ((turnRate/2) > 1) {
  //  adjTurnBrake = (u * 3);
  //}
  //else
  //{
  //  adjTurnBrake = (u * 3) * (turnRate);
  //}
  
  
  /*
  if (turnRate > 0) {
    adjTurnBrake = abs(u / turnRate);
  }

  if (errHeading < 30) {
    adjTurnBrake = 2;
  }
  */


  //was....  && errHeading < 7
  if (errHeading > 0 && errHeading < 15) {
    adjTurnBrake = 0;
    u = 0;
  }

  //We have to limit how frequently we update the OLED display,
  //since it can't keep up with our refresh rate
  if (LCDDelayCount > LCDMaxDelayCount)
  {
    mydisp.setTextPosAbs(0,59);
    mydisp.print("Srvo: ");
    mydisp.print(posLast);
    mydisp.print("/"); //display string at: x=0, y=0
    mydisp.print((posLast + (u * valDirection) - adjTurnBrake));
    mydisp.print("   ");
    //mydisp.setTextPosAbs(0,0);
    mydisp.drawStr(0, 0, "H/D/B: "); //display string at: x=0, y=0
    mydisp.print((int) round(bearing)); //display string at: x=0, y=0
    mydisp.print("/"); //display string at: x=0, y=0
    mydisp.print((int) dwellCount); //display string at: x=0, y=0
    mydisp.print("/"); //display string at: x=0, y=0
    mydisp.print((int) adjTurnBrake); //display string at: x=0, y=0
    mydisp.print("  "); //display string at: x=0, y=0
    mydisp.drawStr(0, 1, "T/E: "); //display string at: x=0, y=0
    mydisp.print((int) round(tgtYAW)); //display string at: x=0, y=0
    mydisp.print("/"); //display string at: x=0, y=0
    mydisp.print((int) round(errHeading)); //display string at: x=0, y=0
    mydisp.print("  "); //display string at: x=0, y=0
    mydisp.drawStr(0, 2, "R/Pd: "); //display string at: x=0, y=0
    mydisp.print((int) turnRate); //display string at: x=0, y=0
    //mydisp.print(turnRate); //display string at: x=0, y=0
    mydisp.print("/"); //display string at: x=0, y=0
    mydisp.print(u); //display string at: x=0, y=0
    //delay(70);
    LCDDelayCount=0;
  }
  else
  {
    LCDDelayCount+=1;
  }

  //mydisp.drawStr(0, 1, "Rate:  "); //display string at: x=0, y=0
  //mydisp.print(turnRate); //display string at: x=0, y=0
  //mydisp.drawStr(0, 3, "Bk:"); //display string at: x=0, y=0
  //mydisp.print(adjTurnBrake); //display string at: x=0, y=0
  //mydisp.drawStr(0, 2, "Bearing  "); //display string at: x=0, y=0
  //mydisp.print(bearing); //display string at: x=0, y=0



  if ((((int) u) > 1) || (turnRate > 0)) {
    //setServo(posLast + (u * valDirection) - adjTurnBrake);
    //setServo3(posLast + (u * valDirection) - adjTurnBrake);
    setServo3(posLast + (u * valDirection));
  }
}





float PID(float valHeading) {
  float result;

  //running sum of the error for the integral calculation
  intgError = (.75 * intgError) + valHeading; 
  dervError = valHeading - lastError;

  //-------------------------------------
  float Kp = propGAIN * valHeading;
  float Ki = intGAIN * intgError;
  float Kd = dervGAIN * dervError;

  //float s = timeDomain/1000;
  //result = Kp + (Ki * s) + (Kd * 1/s);
  result = Kp + Ki + Kd;
  lastError = valHeading;

  return result;

}






//----------------------------------------------------------------
//
//
//   This section contains the primary motor task subroutines
//
//
//----------------------------------------------------------------
void runMotor(uint16_t setMOTOR) {

  // Set the motor PWM Channel number
  servochannelnum=0;

  if (MotorRunning == false)
  {
    MotorRunning = true;
    pwm.setPWM(servochannelnum, 0, MOTORZERO);
    timerDelay(3000);
    pwm.setPWM(servochannelnum, 0, MOTORREADY);
    timerDelay(2000);
    pwm.setPWM(servochannelnum, 0, MOTORSTART);
    timerDelay(3000);
    LastMOTOR = MOTORSTART;
  }
  else
  {

    /*
    if (setMOTOR > LastMOTOR)
     {
     //Serial.println("Speeding Up...");
     //Serial.println(setMOTOR);  
     for (uint16_t pulselen = LastMOTOR; pulselen <= setMOTOR; pulselen++) {
     //Serial.println(pulselen);  
     LastMOTOR = pulselen;
     pwm.setPWM(servochannelnum, 0, pulselen);
     timerDelay(25);
     }
     }
     else if (setMOTOR < LastMOTOR)
     {
     //Serial.println("Slowing Down...");
     //Serial.println(setMOTOR);  
     bleep(3, 2, 3500, -2000, 10, false);
     for (uint16_t pulselen = LastMOTOR ; pulselen >= setMOTOR; pulselen--) {
     //Serial.println(pulselen);  
     LastMOTOR = pulselen;
     pwm.setPWM(servochannelnum, 0, pulselen);
     delay(5);
     }
     delay(3000);
     
     } */
    pwm.setPWM(servochannelnum, 0, setMOTOR);
    LastMOTOR = setMOTOR;
  }
}








//----------------------------------------------------------------
//
//
//   This section contains the Servo subroutines for performing turns
//
//
//----------------------------------------------------------------
void setServo2(int sumPosition) {
  boolean reCenter = false;


  if (posLast < posRight || posLast > posLeft) {
    sumPosition = posCenter;
    //runMotor(MOTORREADY);
    //delay(1000);
    bleep(1, 2, 3500, -2000, 10, false);
    runMotor(MOTORSTART);
    posDelay = 15;
    reCenter=true;
  }
  else
  {
    posDelay = 10;
  }

  if (sumPosition < posLast)
  {
    for(pos = posLast; pos > sumPosition; pos -= 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      pwm.setPWM(1, 0, pos);
      //timerDelay(posDelay);                       // waits 15ms for the servo to reach the position 
      delay(posDelay);
    } 
  }
  else
  {
    for(pos = posLast; pos < sumPosition; pos += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      pwm.setPWM(1, 0, pos);
      //timerDelay(posDelay);                       // waits 15ms for the servo to reach the position 
      delay(posDelay);
    }
  }
  posLast = sumPosition;
  if (reCenter) {
    runMotor(MOTORSTART);
    runMotor(MOTORIDLE);
  }

}



void setServo(int sumPosition) {
  
  if (sumPosition >= posRight && sumPosition <= posLeft) {
    if (sumPosition < posLast)
    {
      posLast -= ((posLast - sumPosition)/2);
    }
    else
    {
      posLast += ((sumPosition - posLast)/2);
    }
  }
  pwm.setPWM(1, 0, posLast);
}


void setServo3(int sumPosition) {
  
  if (sumPosition >= posRight && sumPosition <= posLeft) {
    if (sumPosition < posLast)
    {
      posLast -= 1;
    }
    else
    {
      posLast += 1;
    }
  }
  pwm.setPWM(1, 0, posLast);
}






















//----------------------------------------------------------------
//
//
//   This section contains Timer subroutines
//
//
//----------------------------------------------------------------
void timerDelay(uint16_t delayms) {
  for (uint16_t delaytime = delayms; delaytime > 0; delaytime--) {
    tmr.update();
    delay(1);
  }
}




//----------------------------------------------------------------
//
//
//   This section contains the LED Display routines
//
//   - OK/Yes/Confirmation/Acknowledge    (Single Fast Chaser LED around perimeter of sphere)
//   - Uh-Oh/No/Deny/Decline/Negative    (.5sec timing of alternating LEDs on-off of entire LED belt)
//   - Error/Problem/Attention    (mirrored pair slowly traverse from front-to-back of each side of sphere)
//   - Transferring Data/Wireless Activity (continuous chaser LED, or X number of chaser sweeps)
//   - Power-Up/Activation   (fade in of entire belt)
//   - Siren/Alert/Warning   Negative    (360degree ping-pong sweep)
//   - Processing Data (Single Slow blinking/walking LED around perimeter of sphere)
//   - Blip/Heart-Beat   (single bright LED, center front, then three center front LEDs 1/3 as bright, then 2/3 as bright, then off)
//
//byte LEDSetup = 0;
//1   = On/Off
//2   = Pattern/All
//4   = Single/Alternates
//8   = PingPong/Forward
//16  = Mirror/Entire
//32  = Flip Direction
//64  = Blink/Normal
//128 = Fade/Normal
//----------------------------------------------------------------
//
//
//byte patterns[18] = {
//B10000000, 100,
//B01000000, 100,
//B00100000, 100,
//B00010000, 100,
//B00001000, 100,
//B00000100, 100,
//B00000010, 100,
//B00000001, 100,
//};
void CycleLEDs() {

  if (LEDSetup & B00000001 && ShowLEDCount > 0) {
    //Count the number of times we have dispayed the LED pattern
    ShowLEDCount-= 1;
    if (ShowLEDCount == 0) {
      LEDOutput = B00000000;  //Turn off the display!
      LEDSetup = B00000001;
    }
  }


  //capture the current setup setting
  if (StoredSetup != LEDSetup) {
    StoredSetup = LEDSetup;
    //Reset the pattern
    LEDindex = 0;
    LEDState = 0;
  }

  if (StoredSetup & B00000001) {   
    if (LEDOutput == 0) {
      //Initialize the pattern
      LEDOutput = B00000001;
    }
    if (StoredSetup & B01000000) {  
      if (LEDState == 1) {
        //Turn off the LED's
        LEDState = 0;
        //Exit without engaging the pattern logic
        return;
      }
      else
      {
        LEDState = 1;
      }
    }
    else
    {
      LEDState = 1;
    }
    //Cylce the LEDs!
    if (StoredSetup & B00000010) {
      //Run the pattern...  But which one?
      if (StoredSetup & B00000100) {
        //Run the Single Pattern

        //byte LEDSetup = 0;
        //1   = On/Off
        //2   = Pattern/All
        //4   = Single/Alternates
        //8   = PingPong/Forward
        //16  = Mirror/Entire
        //32  = Flip Direction
        //64  = Blink/Normal
        //128 = Invert/Normal
        if (StoredSetup & B00010000) {
          if (StoredSetup & B00001000) {   
            patternCount = 6;
            pingpoingPoint = 3;
          }
          else
          {
            patternCount = 4;
            pingpoingPoint = 16;
          }
        }
        else
        {
          if (StoredSetup & B00001000) {   
            patternCount = 14;
            pingpoingPoint = 7;
          }
          else
          {
            patternCount = 8;
          }
        }


        //Re-initialize the pattern
        LEDOutput = B00000001;
        if (LEDindex > 0) {
          //Do we need to ping-pong these?
          if (StoredSetup & B00001000) {   
            if (LEDindex > pingpoingPoint) {
              //Shift the bit/LED the other direction
              LEDOutput = B10000000;
              LEDOutput = LEDOutput >> (LEDindex - pingpoingPoint);
            }
            else
            {
              //Shift the bit/LED
              LEDOutput = LEDOutput << LEDindex;
            }
          }
          else
          {
            //Shift the bit/LED
            LEDOutput = LEDOutput << LEDindex;
          }
        }

        if (StoredSetup & B00010000) {
          //In order to accomplish the reverse effect,
          //we will load the byte in reverse in the shift-out process
          //Unfortunately, that has no effect on the mirror, so we need to deal with that here
          //Drop the upper four bits
          if (StoredSetup & B00100000) {
            LEDOutput = LEDOutput & B00001111;
            //Shift the lower four bits upstream
            LEDOutput = LEDOutput << 4; 
            if (LEDindex > pingpoingPoint) {
              LEDOutput = B10000000;
              LEDOutput = LEDOutput >> (LEDindex - pingpoingPoint);
            }
            LEDOutHold = LEDOutput;
            mirIndex = 0;
            for (LEDOutTemp = B00001000; LEDOutTemp>0; LEDOutTemp >>= 1) { //iterate through bit mask
              mirIndex +=1;
              //Move the LSB into position and Add it to the output
              LEDOutput = ((LEDOutHold >> (((mirIndex*2))-1)) & LEDOutTemp) | LEDOutput;          
            }
          }
          else
          {
            if (LEDindex > pingpoingPoint) {
              LEDOutput = B00001000;
              LEDOutput = LEDOutput >> (LEDindex - pingpoingPoint);
            }
            LEDOutput = LEDOutput & B00001111;
            mirIndex = 0;
            for (LEDOutTemp = B10000000; LEDOutTemp>8; LEDOutTemp >>= 1) { //iterate through bit mask
              mirIndex +=1;
              //Move the LSB into position and Add it to the output
              LEDOutput = ((LEDOutput << ((8-(mirIndex*2))+1)) & LEDOutTemp) | LEDOutput;          
            }
          }
        }
      }
      else
      {
        //Run the Alternates Pattern
        patternCount = 2;
        if (LEDindex > 0) {
          //XOR the Output so it flips
          LEDOutput = LEDOutput ^ B11111111;
        }
        else
        {
          LEDOutput = B01010101;
        }
      }
    }
    else
    {
      //Turn on all the LEDs!
      //LEDOutput = B11111111;  //All On!      
    }
    //Prepare to write the data!
    if (StoredSetup & B10000000) {
      //XOR and invert the output!
      LEDOutput = LEDOutput ^ B11111111;
    }
    if (StoredSetup & B00100000) {
      setLEDArray(MSBFIRST, LEDOutput);
    }
    else
    {
      setLEDArray(LSBFIRST, LEDOutput);
    }
    LEDindex++;
    if (LEDindex >= patternCount){
      LEDindex = 0;
    }
  }
  else
  {
    //Turn off all the LEDs!
    LEDOutput = B00000000;  //All Off!      
    setLEDArray(LSBFIRST, LEDOutput);
  }
}


void setLEDArray(byte bitOrder, byte LEDArray) {
  //LEDs start @ channel 4 and end at channel 11
  uint16_t i=12;
  if (bitOrder == MSBFIRST) {
    for (LEDOutTemp = B10000000; i>4; LEDOutTemp >>= 1) { //iterate through bit mask
      i += -1;
      setLEDstate(i, LEDArray, LEDOutTemp);
    }
  }
  else
  {
    for (LEDOutTemp = B00000001; i>4; LEDOutTemp <<= 1) { //iterate through bit mask
      i += -1;
      setLEDstate(i, LEDArray, LEDOutTemp);
    }
  }
  //Serial.println("");
}

void setLEDstate(uint16_t LEDChannel, byte LEDOut, byte LEDTestMask) {
  //Serial.print(LEDChannel);
  //Serial.print(":");
  if (servoTestState == false) {
    if (LEDOut & LEDTestMask) {
      pwm.setPWM(LEDChannel, 0, 1024);
      //Serial.print("1");
    } 
    else
    {
      pwm.setPWM(LEDChannel, 0, 0);
      //Serial.print("0");
    }
  } 
  else
  {
    pwm.setPWM(LEDChannel, 0, 0);
  }
  //Serial.print("-");
}



void setTargetHeadingLED(double errHeading) {
  //Turn on the LEDs
  LEDSetup = B00000001;

  //Pointing right at the target
  if (errHeading > -5 && errHeading < 5) {
    LEDOutput = B00011000;
  }

  //First quadrants left and right of the front center
  if (errHeading >= 5 && errHeading < 30) {
    LEDOutput = B00010000;
  }
  if (errHeading > -30 && errHeading <= -5) {
    LEDOutput = B00001000;
  }

  //Second quadrants left and right of the front center
  if (errHeading >= 30 && errHeading < 60) {
    LEDOutput = B00100000;
  }
  if (errHeading > -60 && errHeading <= -30) {
    LEDOutput = B00000100;
  }


  //Third quadrants left and right of the front center
  if (errHeading >= 60 && errHeading < 90) {
    LEDOutput = B01000000;
  }
  if (errHeading > -90 && errHeading <= -60) {
    LEDOutput = B00000010;
  }


  //Fourth quadrants left and right of the front center
  if (errHeading >= 90 && errHeading < 135) {
    LEDOutput = B10000000;
  }
  if (errHeading > -135 && errHeading <= -90) {
    LEDOutput = B00000001;
  }
  if (errHeading <= -135 || errHeading >= 135) {
    LEDOutput = B10000001;
  }
  
  
  //mydisp.drawStr(0, 0, "errHeadg:  "); //display string at: x=0, y=0
  //mydisp.print(errHeading); //display string at: x=0, y=0
  //mydisp.drawStr(0, 2, "LED:  "); //display string at: x=0, y=0
  //mydisp.print(LEDOutput); //display string at: x=0, y=0

  

  //setLEDArray(LSBFIRST, LEDOutput);
}




void setLEDPattern() {
  //byte LEDSetup = 0;
  //1   = On/Off
  //2   = Pattern/All
  //4   = Single/Alternates
  //8   = PingPong/Forward
  //16  = Mirror/Entire
  //32  = Flip Direction
  //64  = Blink/Normal
  //128 = Invert/Normal

  switch (demoLEDCount) {
  case 0:
    //This is everything off
    LEDSetup = B00000000;
    break;
  case 1:
    //This is the Alternate pattern
    LEDSetup = B00000011;
    break;
  case 2:
    //This is the loop pattern
    LEDSetup = B00000111;
    break;
  case 3:
    //This is the loop pattern running reverse
    LEDSetup = B00100111;
    break;
  case 4:
    //This is the loop pattern in ping/pong mode
    LEDSetup = B00001111;
    break;
  case 5:
    //This is the loop pattern running mirrored
    LEDSetup = B00010111;
    break;
  case 6:
    //This is the loop pattern running mirrored & reversed
    LEDSetup = B00110111;
    break;
  case 7:
    //This is the loop pattern running mirrored and ping-pong
    LEDSetup = B00011111;
    break;
  case 8:
    //This is the loop pattern inverted
    LEDSetup = B10000111;
    break;
  case 9:
    //This is the loop pattern running reverse & inverted
    LEDSetup = B10100111;
    break;
  case 10:
    //This is the loop pattern in ping/pong mode & inverted
    LEDSetup = B10001111;
    break;
  case 11:
    //This is the loop pattern running mirrored & inverted
    LEDSetup = B10010111;
    break;
  case 12:
    //This is the loop pattern running mirrored, reversed & inverted
    LEDSetup = B10110111;
    break;
  case 13:
    //This is the loop pattern running mirrored, ping-pong & inverted
    LEDSetup = B10011111;
    break;
    //--------------------------------
    //This begins the blink section
  case 14:
    //This is the Alternate pattern
    LEDSetup = B00000011;
    break;
  case 15:
    //This is the loop pattern
    LEDSetup = B00000111;
    break;
  case 16:
    //This is the loop pattern running reverse
    LEDSetup = B00100111;
    break;
  case 17:
    //This is the loop pattern in ping/pong mode
    LEDSetup = B00001111;
    break;
  case 18:
    //This is the loop pattern running mirrored
    LEDSetup = B00010111;
    break;
  case 19:
    //This is the loop pattern running mirrored & reversed
    LEDSetup = B00110111;
    break;
  case 20:
    //This is the loop pattern running mirrored and ping-pong
    LEDSetup = B00011111;
    break;
  case 21:
    //This is the loop pattern inverted
    LEDSetup = B10000111;
    break;
  case 22:
    //This is the loop pattern running reverse & inverted
    LEDSetup = B10100111;
    break;
  case 23:
    //This is the loop pattern in ping/pong mode & inverted
    LEDSetup = B10001111;
    break;
  case 24:
    //This is the loop pattern running mirrored & inverted
    LEDSetup = B10010111;
    break;
  case 25:
    //This is the loop pattern running mirrored, reversed & inverted
    LEDSetup = B10110111;
    break;
  case 26:
    //This is the loop pattern running mirrored, ping-pong & inverted
    LEDSetup = B10011111;
    break;
  default: 
    //All LEDs off
    LEDSetup = B00000000;
    break;
  }
  //byte LEDSetup = 0;
  //1   = On/Off
  //2   = Pattern/All
  //4   = Single/Alternates
  //8   = PingPong/Forward
  //16  = Mirror/Entire
  //32  = Flip Direction
  //64  = Blink/Normal
  //128 = Invert/Normal
  //LEDSetup = B00110111;
  //LEDSetup = B00010111;
  //LEDSetup = B00011111;
}







//----------------------------------------------------------------
//
//
//   This section contains audio feedback method (piezo element)
//
//
//----------------------------------------------------------------
void bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, int milDelay, boolean runTimer) {
  //This function is written so different styles of bleeps can be played, especially via a script, etc...
  for (int j = 0; j < numBleeps; j++)
  {
    for (int i = 0; i < numTones; i++)
    {
      tone(speakerPin, (bgnFreq + (stpFreq*i)));
      //if (runTimer==true) tmr.update();
      delay(milDelay);
    }
  }
  //Where are we sending the tone information?
  noTone(speakerPin);
}









//----------------------------------------------------------------
//
//
//   This section contains the Compass subroutines
//
//
//----------------------------------------------------------------
void ReadCompass2(){
  float tempBearing = 0;
  float tempRads;
  float tempSin = 0;
  float tempCos = 0;
  float minBearing = 0;
  float maxBearing = 0;
  compass_scalled_reading();

  //Serial.print("x = ");
  //Serial.println(compass_x_scalled);
  //Serial.print("y = ");
  //Serial.println(compass_y_scalled);
  //Serial.print("z = ");
  //Serial.println(compass_z_scalled);

  compass_heading();
  //Serial.print ("Heading angle = ");
  //Serial.print (bearing);
  tempBearing = bearing;
  minBearing = bearing;
  maxBearing = bearing;

  int numSamples = 20;    
  for(int i = 0; i < numSamples; i++) {
    //compass_scalled_reading();
    compass_heading();
    //tempBearing = tempBearing + bearing / numSamples;
    //tempBearing = tempBearing + bearing;
    //if (bearing > 359) bearing = 0;
    tempRads = bearing / 180.0 * PI;
    //tempSin = tempSin + sin(tempRads);
    //tempCos = tempCos + cos(tempRads);
    // running average
    tempCos = tempCos * .75 + cos(tempRads) * .25;
    tempSin = tempSin * .75 + sin(tempRads) * .25;
    //if (bearing > maxBearing)  maxBearing = bearing;
    //if (bearing < minBearing)  minBearing = bearing;
    delay(1);
  }
  //bearing = (tempBearing / numSamples);
  //if ((maxBearing - minBearing) > 270)
  //{
  //  tempBearing = ((360 - maxBearing) + minBearing) / 2;      
  //}
  //else
  //{
  //  tempBearing = (minBearing + maxBearing) / 2;  
  //}

  //tempSin = tempSin / numSamples;
  //tempCos = tempCos / numSamples;
  tempBearing = atan2(tempSin,tempCos) / PI * 180;
  if(tempBearing < 0) tempBearing += 360;

  
  //You need to use modulo 360 arithmetic.
  //Modulo 360 is simply the remainder after dividing the sum by 360. E.g.
  //(10 + 10 + 10 + 350) / 4 = 95 (incorrect)
  //((10 + 10 + 10 + 350) modulo 360) / 4 = 5 (correct)
  //tempBearing = (((int) tempBearing) % 360)/ numSamples;
  
  //Serial.print ("    Avg Heading angle = ");
  //Serial.println (bearing);
  bearing = tempBearing;
  //Serial.print ("AvgMinMaxBearing = ");
  //Serial.println (tempBearing);
}













//----------------------------------------------------------------
//
//
//   This section contains general EEProm subroutines
//
//
//----------------------------------------------------------------
// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
  int sizeEEPROM;
  bleep(1, 10, 3500, -300, 3, false);

  if (compass_x_offset != EEPROM_readFloat(CompassXOffsetAddress))
  {
    sizeEEPROM = EEPROM_writeFloat(CompassXOffsetAddress, compass_x_offset);
  }
  if (compass_y_offset != EEPROM_readFloat(CompassYOffsetAddress))
  {
    sizeEEPROM = EEPROM_writeFloat(CompassYOffsetAddress, compass_y_offset);
  }
  if (compass_z_offset != EEPROM_readFloat(CompassZOffsetAddress))
  {
    sizeEEPROM = EEPROM_writeFloat(CompassZOffsetAddress, compass_z_offset);
  }
}


// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  boolean runCal = false;

  Serial.println("Loading Compass Offsets");

  //Setup the Compass Variables
  compass_x_gainError = 1.12;
  compass_y_gainError = 1.13;
  compass_z_gainError = 1.03;  

  // Load from EEPROM
  compass_x_offset = EEPROM_readFloat(CompassXOffsetAddress);
  compass_y_offset = EEPROM_readFloat(CompassYOffsetAddress);
  compass_z_offset = EEPROM_readFloat(CompassZOffsetAddress);


  // Use defaults if EEPROM values are invalid
  if (isnan(compass_x_offset))
  {
    compass_x_offset = -413.98;
    runCal = true;
  }
  if (isnan(compass_y_offset))
  {
    compass_y_offset = 549.80;
    runCal = true;
  }
  if (isnan(compass_z_offset))
  {
    compass_z_offset = 83.99;
    runCal = true;
  }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
int EEPROM_writeFloat(int address, float value)
{
int i;

  byte* p = (byte*)(void*)&value;
  for (i = 0; i < sizeof(value); i++)
  {
    EEPROM.write(address++, *p++);
  }
  initstatusbar();
  //mydisp.print("");
  mydisp.print(address); //display string at: x=0, y=0
  mydisp.print("/"); //display string at: x=0, y=0
  mydisp.print(i); //display string at: x=0, y=0
  mydisp.print(":"); //display string at: x=0, y=0
  mydisp.print(value); //display string at: x=0, y=0
  delay(5000);
  return i;
}
// ************************************************
// Read floating point values from EEPROM
// ************************************************
float EEPROM_readFloat(int address)
{
  float value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    *p++ = EEPROM.read(address++);
  }

  initstatusbar();
  //mydisp.print("");
  mydisp.print(address); //display string at: x=0, y=0
  mydisp.print(":"); //display string at: x=0, y=0
  mydisp.print(value); //display string at: x=0, y=0
  delay(1000);
  
  return value;
}



