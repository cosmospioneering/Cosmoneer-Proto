//Loaded in order of importance
//
//
//This is required for the current sensor module
#include <Wire.h>
#include <Adafruit_INA219.h>

//CurrentSensor Setup
Adafruit_INA219 ina219;
float currentNow_mA = 0;


// So we can save and retrieve settings
#include <EEPROM.h>

#include "compass.h"
// EEPROM addresses for persisted data
const int CompassXOffsetAddress = 0;
const int CompassYOffsetAddress = 8;
const int CompassZOffsetAddress = 16;







#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
//Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x67);
// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
//
//These settings work for Servos...
#define SERVOMIN  120 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // this is the 'maximum' pulse length count (out of 4096)
double ServoOffset;  //Find the zero center of the servo offset.  We want the positional output to reflect turning...

//
//Trying these settings with the Motor Controller
const uint16_t MOTORZERO = 0; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORREADY = 200; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORSTART = 275; // was 275...  this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORIDLE = 310; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORMAX = 500; // 250this is the 'maximum' pulse length count (out of 4096)
// our servo # counter
uint8_t servochannelnum = 1;
uint16_t LastMOTOR;
double targHeading = 90;
int8_t valDirection = 0;
int8_t dwellCount = 0;
int8_t dwlTotalCount = 0;
int8_t dwellTest = 5; // this is minimum number of times the maneuver test must pass before a position is considered "hit"
uint16_t dwellDelay = 0;
float heading=0;
float turnRate=0;
boolean valDirection2 = false;
boolean dirPassN = false;
boolean dirPassS = false;
boolean dirPassE = false;
boolean dirPassW = false;










Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x78);
//Rev 0 Setting used
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x7C);
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x77);







#include <PID_v1.h>


//#include <PID_AutoTune_v0.h>

byte ATuneModeRemember=2;
double input=0, output=0, setpoint=0;
double kp=2,ki=0.5,kd=2;


/*
double kpmodel=1.5, taup=100, theta[50];
double outputStart=0;  //We will use some math to center the servo values around zero...
double aTuneStep=10, aTuneNoise=1, aTuneStartValue=0;
unsigned int aTuneLookBack=10;

boolean tuning = true;
unsigned long  modelTime, serialTime;
//PID_ATune aTune(&input, &output);

//set to false to connect to the real world
boolean useSimulation = false;
*/


PID_Basic myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);


#define propGAIN  .1 // .1 proportional gain (the bigger the number the harder the controller pushes)
#define intGAIN  .07 // .05 integral gain  (the SMALLER the number (except for 0, which turns it off,)  the more quickly the controller reacts to load changes, but the greater the risk of oscillations.)
float intgError;
float lastError;
float dervError;
#define dervGAIN  3 // .5 derivative gain  (the bigger the number  the more the controller dampens oscillations (to the point where performance can be hindered))


//Define Variables we'll be connecting to
//double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
//double aggKp=4, aggKi=0.2, aggKd=1;
//double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
//PID_Basic myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);












//
//
//This is what is required for the timer event support
#include "Timer.h"

Timer tmr;








 
//#define GUI_ADDRESS 0x8
#define CORE_ADDRESS 0x9







 


//This is the enable pin for the IrDa Serial Port
const int IrDaEnablePin = 5;
const int speakerPin = 4;


//What pin is the battery voltage divider mapped to?
const int vRegMonPin = A2;


boolean InitPower = false;    //This is the initial Power UP State...  This is zero if we have just powered up!  It will be one after we have powered completely up.
boolean SleepState = false;


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
int posLeft = 460;    // variable to store the servo position 
int posRight = 120;    // variable to store the servo position 
int posDelay = 15;    // variable to store the servo position 




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
  //Run the timer events
  tmr.update();
  delay(dwellDelay);

}



void setup() {
  //Before we do anything else, let the user know we are doing something!
  bleep(3, 25, 500, 80, 3);



  //Prepare the port so we can see debug messages!
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  

  //Initialize variables as needed
  setVariables();

  //Setup pins, as required
  setPins();


  //Start the I2C interface
  Wire.begin();
  
  //i2cScan();
  delay(3000);
  
  


  //Initialize the current monitor
  Serial.println("Init Curr. Mon.");
  ina219.begin();
  currentNow_mA = GetCurrentSensorData();






  Serial.println("Init Compass");
  LoadParameters();  //Load parameters from EEProm
  //compass_x_offset = -413.98;
  //compass_y_offset = 549.80;
  //compass_z_offset = 83.99;
  compass_init(2);
  //compass_debug = 0;
  //compass_offset_calibration(3);
  //SaveParameters();    //Save parameters after the calibration
  //Serial.println("Cal. Done");



  Serial.println("Init PWM");
  pwm.reset();
  pwm.begin();
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency for Servos, suggest using 50 instead
  //pwm.setPWMFreq(1000);  // This is the minimum PWM frequency for the fan controller
  //setLEDArray(LSBFIRST, LEDOutput);
  Serial.println("LED On");
  pwm.setPWM(5, 0, 1024);

  /*
  //Use this sequence to reset the Min/Max settings on the Mx-3A ESC!!
  pwm.setPWM(0, 0, MOTORZERO);
  delay(7000);
  pwm.setPWM(0, 0, MOTORMAX);
  delay(1000);
  pwm.setPWM(0, 0, MOTORSTART);
  delay(1000);
  pwm.setPWM(0, 0, MOTORREADY);
  delay(7000);
  */



  //delay(1000);
  //Serial.println("Init PID");
  //Setup the pid 
  //myPID.SetMode(MANUAL);
  //myPID.SetOutputLimits((SERVOMIN + ServoOffset) , (SERVOMAX + ServoOffset));

/*
  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
*/

  




  //Set the Compass timer event
  //tmr.every(500, ReadCompass2);
  //Set the LED timer event
//  tmr.every(100, CycleLEDs);
  //Set the Maneuver timer event
  //tmr.every(1500, Maneuver);
  //Set the Sensor timer event
//  tmr.every(1000, readSensors);

  
  //timerDelay(1000);
  delay(1000);

 
  //Set the "I am initialized" flag
  InitPower = true;    //We have completely powered up.  Now set the flag so we know we are already running
  //let the user know we are going into the Main Loop!
  //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay)
  //bleep(1, 20, 3500, -150, 10);
  bleep(1, 20, 3500, -150, 5);



  CompassCalibrationCheck();



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
  LEDOutput = B10000001;  //All Off!


}
 
void setPins() {
  analogReference(INTERNAL);
  
 //This pin is used for system activity feedback to the user
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  //Turn on the IrDa Transcoder
  pinMode(IrDaEnablePin, OUTPUT);
  digitalWrite(IrDaEnablePin, LOW);  //LOW is off, HIGH is on!

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
  runMotor(MOTORIDLE);

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
      
      if (calLastHeading < 90 && bearing > 90 && dirPassE == false) {
        dirPassE = true;
        bleep(1, 2, 3500, -500, 5);
      }
      if (calLastHeading < 180 && bearing > 180 && dirPassS == false) {
        dirPassS = true;
        bleep(1, 3, 3500, -500, 5);
      }
      if (calLastHeading < 270 && bearing > 270 && dirPassW == false) {
        dirPassW = true;
        bleep(1, 4, 3500, -500, 5);
      }
      if (calLastHeading < 360 && bearing > 0 && dirPassN == false) {
        dirPassN = true;
        bleep(1, 1, 3500, -500, 5);
      }
      if (dirPassE && dirPassS && dirPassW && dirPassN) {
        dirPassE = false;
        dirPassS = false;
        dirPassW = false;
        dirPassN = false;
        countRevolution +=1;
        //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay)
        bleep(4, 20, 1000, 50, 1);
      }
      calLastHeading = bearing;
    } 
    //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay)
    //bleep(1, 20, 3500, -150, 10);
    bleep(1, 10, 3500, -300, 3);
  } 
  digitalWrite(13, LOW);

  if (countRevolution > 0) {
    bleep(3, 25, 500, 80, 3);
    //Set the Compass timer event
    tmr.every(100, Maneuver);
  }
  else
  {
    //calibration is needed!!
    bleep(1, 2, 3500, -2000, 10);
    tmr.every(100, Maneuver);
    //compass_debug = 0;
    //compass_offset_calibration(3);
    //SaveParameters();    //Save parameters after the calibration
    //Serial.println("Cal. Done");
  }
  dirPassE = false;
  dirPassS = false;
  dirPassW = false;
  dirPassN = false;

  Serial.println("Compass Calibration Test Complete");

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
    setCMG2(targHeading, 100);
    cmgActive = false;
    
    double errHeading = abs(targHeading - bearing);
    dwellDelay = 0;
    
    dwlTotalCount +=1;
    Serial.print("errHeading:\t");
    Serial.println(errHeading);
    // are we within range?
    if (errHeading > 0 && errHeading < 15) {
      //increment the dwell counter
      dwellCount += 1;
      
      Serial.print("dwelltest:\t");
      Serial.println(dwellTest);
      Serial.print("dwellcount:\t");
      Serial.println(dwellCount);
      

      //have we spent enough time at this heading?  If not, then exit the subroutine
      if (dwellCount < dwellTest) {
        //bleep(1, 1, 2000, -10, 5);
        return;
      }
      
      //if (dwellCount >1) {
      //  bleep(1, 1, 3500, -10, 5);
      //}
      
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
      
      dwellTest = 5;
      dwellDelay = 2000;
      
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
        //bleep(4, 20, 1000, 50, 1);
        //bleep(1, 1, 1000, -10, 5);
      }
      else {
        //decrease the dwell counter...  We missed it this time.
        dwellCount -= 1;
        //check to make sure we haven't gone below the lower range of the dwell counter
        if (dwellCount < 0) {dwellCount = 0;}
      }
    }
    
    if (dwlTotalCount >10) {
      dwlTotalCount = 0;
      dwellTest -= 1;
      if (dwellTest < 1) {dwellTest = 1;}
      //bleep(1, 1, 500, -10, 5);
    }

  }
}



void setCMG2(double tgtYAW, uint16_t timeDomain) {
//This routine is based on similar logic used in quadricopters and other systems
//where a target is attempting to be reached...
//This is a Parallel PID

  float lastBearing = bearing;
  float adjTurnBrake;
  ReadCompass2();
  turnRate = lastBearing - bearing;
  

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




  if (tgtYAW < bearing) {
    //bleep(1, 1, 500, -10, 30);
    valDirection = -1;
  }
  else
  {
    //bleep(1, 1, 3500, -10, 30);
    valDirection = 1;
  }
  
  if (errHeading > 180) {
    //Motor must be spinning CCW
    //This result means the target position is closer if we spin in the opposite direction
    errHeading = 360 - errHeading;
    valDirection = valDirection * -1;
  }

  
  float u = (PID(errHeading) / 7);
  if (u < 1 && u > .6) {
    u = 1;
  }
  
  if ((turnRate/200) > 1) {
    adjTurnBrake = (u * 3);
  }
  else
  {
    adjTurnBrake = (u * 3) * (turnRate/200);
  }
  
  if (errHeading < 30) {
    adjTurnBrake = 2;
  }
  
  
  //was....  && errHeading < 7
  if (errHeading > 0 && errHeading < 15) {
    adjTurnBrake = 0;
    u = 0;
  }
  if (u > 10) {
    adjTurnBrake = 0;
    u = 5;
  }
  
  
  Serial.print("target:\t");
  Serial.println(tgtYAW);
  Serial.print("heading:\t");
  Serial.println(bearing);
  Serial.print("PID Results:\t");
  Serial.println(u);
  Serial.print("Turn Rate:\t");
  Serial.println(turnRate);
  Serial.print("adjTurnBrake:\t");
  Serial.println(adjTurnBrake);
  
  
  setServo(posLast + (u * valDirection) - adjTurnBrake);  
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
  //-------------------------------------
  //Serial.print("Kp:\t");
  //Serial.println(Kp);
  //Serial.print("Ki:\t");
  //Serial.println(Ki);
  //Serial.print("Kd:\t");
  //Serial.println(Kd);

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
    Serial.println("Resetting to Zero");  
    pwm.setPWM(servochannelnum, 0, MOTORZERO);
    timerDelay(3000);
    pwm.setPWM(servochannelnum, 0, MOTORREADY);
    timerDelay(2000);
    Serial.println("Starting Motor...");  
    pwm.setPWM(servochannelnum, 0, MOTORSTART);
    timerDelay(3000);
    LastMOTOR = MOTORSTART;
  }
  else
  {
    if (setMOTOR > LastMOTOR)
    {
      Serial.println("Speeding Up...");
      Serial.println(setMOTOR);  
      for (uint16_t pulselen = LastMOTOR; pulselen <= setMOTOR; pulselen++) {
        Serial.println(pulselen);  
        LastMOTOR = pulselen;
        pwm.setPWM(servochannelnum, 0, pulselen);
        timerDelay(25);
      }
    }
    else if (setMOTOR < LastMOTOR)
    {
      Serial.println("Slowing Down...");
      Serial.println(setMOTOR);  
      bleep(3, 2, 3500, -2000, 10);
      for (uint16_t pulselen = LastMOTOR ; pulselen >= setMOTOR; pulselen--) {
        Serial.println(pulselen);  
        LastMOTOR = pulselen;
        pwm.setPWM(servochannelnum, 0, pulselen);
        delay(5);
      }
      delay(3000);
    }
  }
}








//----------------------------------------------------------------
//
//
//   This section contains the Servo subroutines for performing turns
//
//
//----------------------------------------------------------------
void leftTurn (){
  Serial.print("LftStrt: ");
  for(pos = posLast; pos>=(posRight+1); pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    pwm.setPWM(1, 0, pos);
    timerDelay(posDelay);                       // waits 15ms for the servo to reach the position 
  } 
  Serial.print("LftDone: ");
  posLast = pos;
}

void rightTurn() {
  Serial.print("RgtStrt: ");
   for(pos = posLast; pos < posLeft; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    pwm.setPWM(1, 0, pos);
    timerDelay(posDelay);                       // waits 15ms for the servo to reach the position 
  } 
  Serial.print("RgtDone: ");
  posLast = pos;
}

void returnToCenter() {
    if (posLast > posRight)
    {
       for(pos = posLast; pos > posCenter; pos -= 1)  // goes from 0 degrees to 180 degrees 
      {                                  // in steps of 1 degree 
        pwm.setPWM(1, 0, pos);
        timerDelay(posDelay);                       // waits 15ms for the servo to reach the position 
      } 
    }
    else
    {
       for(pos = posLast; pos < posCenter; pos += 1)  // goes from 0 degrees to 180 degrees 
      {                                  // in steps of 1 degree 
        pwm.setPWM(1, 0, pos);
        timerDelay(posDelay);                       // waits 15ms for the servo to reach the position 
      }
    }
  posLast = pos;
}

void setServo(int sumPosition) {
boolean reCenter = false;
  Serial.print("Servo:\t");
  Serial.println(sumPosition);


  if (sumPosition < posRight || sumPosition > posLeft) {
    sumPosition = posCenter;
    runMotor(MOTORSTART);
    bleep(1, 2, 3500, -2000, 10);
    reCenter = true;
    /*
    if ((posLast + sumPosition) > posRight) {
      posLast = posRight;
    }
    else
    {
      posLast = posLeft;
    }
    */
  }

  if (sumPosition < posLast)
  {
     Serial.println("pos1");
     for(pos = posLast; pos > sumPosition; pos -= 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      pwm.setPWM(1, 0, pos);
      timerDelay(posDelay);                       // waits 15ms for the servo to reach the position 
    } 
  }
  else
  {
     Serial.println("pos2");
     for(pos = posLast; pos < sumPosition; pos += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      pwm.setPWM(1, 0, pos);
      timerDelay(posDelay);                       // waits 15ms for the servo to reach the position 
    }
  }
  posLast = sumPosition;
  Serial.print("EndServo:\t");
  Serial.println(posLast);
  if (reCenter) {
    runMotor(MOTORIDLE);
  }

}




















//----------------------------------------------------------------
//
//
//   This section contains the primary sensor task subroutine
//
//
//----------------------------------------------------------------
void readSensors() {

//byte LEDSetup = 0;
//1   = On/Off
//2   = Pattern/All
//4   = Single/Alternates
//8   = PingPong/Forward
//16  = Mirror/Entire
//32  = Flip Direction
//64  = Blink/Normal
//128 = Fade/Normal

  idxSensorToRead +=1;

  switch (idxSensorToRead) {
    case 1:    //Date
        LEDSetup = B00000000;
        break;
    case 2:    //Time
        break;
    case 3:    //Temp
        break;
    case 4:    //Pressure
        break;
    case 5:    //Altitude
        break;
    case 6:    //Local Voltage
        break;
    case 7:    //Supply Voltage
        break;
    case 8:    //Supply Current
        break;
    case 9:    //LED Data
        break;
    case 10:    //LED Data
        break;
    default:
        LEDSetup = B00000000;
        idxSensorToRead = 0;
        break;
  }
  demoLEDCount = idxSensorToRead;
  setLEDPattern();

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
  //Serial.print("CycleLEDs: ");
  //Serial.print(LEDOutput);
  //Serial.println("");

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
        //digitalWrite(latchPin, LOW);
        //shiftOut(dataPin, clockPin, LSBFIRST, B00000000);
        //digitalWrite(latchPin, HIGH);
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
      //Serial.println("LEDs On");
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
          //Serial.println("Mirror Enabled");
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
              //Serial.println("PingPong Active");
              LEDOutput = B00001000;
              LEDOutput = LEDOutput >> (LEDindex - pingpoingPoint);
            }
            //Serial.println("LSB First");
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
      LEDOutput = B11111111;  //All On!      
    }
    //Repare to write the data!
    //digitalWrite(latchPin, LOW);
    if (StoredSetup & B10000000) {
      //XOR and invert the output!
      LEDOutput = LEDOutput ^ B11111111;
    }
    if (StoredSetup & B00100000) {
      //shiftOut(dataPin, clockPin, MSBFIRST, LEDOutput);
      setLEDArray(MSBFIRST, LEDOutput);
    }
    else
    {
      //shiftOut(dataPin, clockPin, LSBFIRST, LEDOutput);
      setLEDArray(LSBFIRST, LEDOutput);
    }
    //digitalWrite(latchPin, HIGH);
    LEDindex++;
    if (LEDindex >= patternCount){
      LEDindex = 0;
    }
  }
  else
  {
    //Turn off all the LEDs!
    LEDOutput = B00000000;  //All Off!      
    //digitalWrite(latchPin, LOW);
    //shiftOut(dataPin, clockPin, LSBFIRST, LEDOutput);
    //digitalWrite(latchPin, HIGH);
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
void bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay) {
  //This function is written so different styles of bleeps can be played, especially via a script, etc...
  for (int j = 0; j < numBleeps; j++)
  {
    for (int i = 0; i < numTones; i++)
    {
      tone(speakerPin, (bgnFreq + (stpFreq*i)));
      tmr.update();
      delay(milDelay);
    }
  }
  //Where are we sending the tone information?
  noTone(speakerPin);
}







//----------------------------------------------------------------
//
//
//   This section contains voltage/current measurement code
//
//   An analog pin must be used, and may require using
//   the aurduino's internal voltage reference
//
//
//----------------------------------------------------------------
float GetCurrentSensorData()
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;


  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  int numSamples = 5;    
  for(int i = 0; i < numSamples; i++) {
    current_mA = current_mA + ina219.getCurrent_mA() / numSamples;
    delay(2);
  }
  //current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  //Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  //Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");
  
  return(current_mA);
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
float minBearing = 0;
float maxBearing = 0;
  compass_scalled_reading();
  
  Serial.print("x = ");
  Serial.println(compass_x_scalled);
  Serial.print("y = ");
  Serial.println(compass_y_scalled);
  Serial.print("z = ");
  Serial.println(compass_z_scalled);

  compass_heading();
  Serial.print ("Heading angle = ");
  Serial.print (bearing);
  tempBearing = bearing;
  minBearing = bearing;
  maxBearing = bearing;
  
  int numSamples = 20;    
  for(int i = 0; i < numSamples; i++) {
    compass_scalled_reading();
    compass_heading();
    //tempBearing = tempBearing + bearing / numSamples;
    tempBearing = tempBearing + bearing;
    
    if (bearing > maxBearing)  maxBearing = bearing;
    if (bearing < minBearing)  minBearing = bearing;
    delay(3);
  }
  //bearing = (tempBearing / numSamples);
  tempBearing = (minBearing + maxBearing) / 2;
  Serial.print ("    Avg Heading angle = ");
  Serial.println (bearing);
  bearing = tempBearing;
  Serial.print ("AvgMinMaxBearing = ");
  Serial.println (tempBearing);
}





//----------------------------------------------------------------
//
//
//   This section contains general I2C subroutines
//
//
//----------------------------------------------------------------
void i2cScan()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
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
   if (compass_x_offset != EEPROM_readFloat(CompassXOffsetAddress))
   {
      EEPROM_writeFloat(CompassXOffsetAddress, compass_x_offset);
   }
   if (compass_y_offset != EEPROM_readFloat(CompassYOffsetAddress))
   {
      EEPROM_writeFloat(CompassYOffsetAddress, compass_y_offset);
   }
   if (compass_z_offset != EEPROM_readFloat(CompassZOffsetAddress))
   {
      EEPROM_writeFloat(CompassZOffsetAddress, compass_z_offset);
   }
}


// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{

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
  }
  if (isnan(compass_y_offset))
  {
    compass_y_offset = 549.80;
  }
  if (isnan(compass_z_offset))
  {
    compass_z_offset = 83.99;
  }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeFloat(int address, float value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
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
   return value;
}


