//Loaded in order of importance
//
//This is the Shake2Wake module...
//#include <PinChangeInt.h>
//#include <PinChangeIntConfig.h>
//
//
//This is required for the encryption chip
//#include <sha204_library.h>
//
//This is required for the current sensor module
//#include <Adafruit_INA219.h>
#include <Wire.h>


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
#define SERVOMAX  530 // this is the 'maximum' pulse length count (out of 4096)
//
//Trying these settings with the Motor Controller
const uint16_t MOTORZERO = 0; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORREADY = 200; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORSTART = 275; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORIDLE = 310; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t MOTORMAX = 400; // this is the 'maximum' pulse length count (out of 4096)
// our servo # counter
uint8_t servochannelnum = 1;
uint16_t LastMOTOR;
uint16_t targHeading = 90;
int8_t valDirection = 0;




#define propGAIN  .4 // proportional gain
#define intGAIN  .05 // integral gain
float intgError;
float lastError;
float dervError;
#define dervGAIN  .5 // derivative gain



Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x78);
//Rev 0 Setting used
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x7C);
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x77);



// I2Cdev and HMC5883L must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "HMC5883L.h"
// class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;

int16_t mx, my, mz;





//This is for the IrDa Serial Interface
#include <SoftwareSerial.h>

#define RxIR A0
#define TxIR A1

SoftwareSerial infraredSerial(RxIR, TxIR);

int intPipeCnt = 0;
String aryInput[4];
String temp;

const unsigned int MAX_INPUT = 50;








//
//
//This is what is required for the RTC clock
//#include <RTClib.h>
//
//
//This is the required for the alitimeter/temp module
//#include <BMP180.h>
//
//
#include "Timer.h"






//#define _Digole_Serial_I2C_  //To tell compiler compile the special communication only, 
//all available are:_Digole_Serial_UART_, _Digole_Serial_I2C_ and _Digole_Serial_SPI_
//#include <DigoleSerial.h>
//--------I2C setup, if you don't use I2C, use // to comment following 2 lines
//#if defined(_Digole_Serial_I2C_)
//#include <Wire.h>
//DigoleSerialDisp mydisp(&Wire,'\x27');  //I2C:Arduino UNO: SDA (data line) is on analog input pin 4, and SCL (clock line) is on analog input pin 5 on UNO and Duemilanove
//#endif
//#define LCDCol 16
//#define LCDRow 2
//#define LCDW 240




 
//#define GUI_ADDRESS 0x8
#define CORE_ADDRESS 0x9






//(CHIP:   ATMEGA328P-AU)
//AREF â€“ (Unavailable via breakout pins.  Would need a hard-wired connection to be used).
//       We are using the internal reference, so this is not an issue!
//
// 
//RAW  â€“ (Incoming unregulated power.  Can support up to 12V)
//GND  â€“ 
//RST  - (Reset pin) Ties to IrDA, which resets when LOW
//VCC  â€“ (5V)
//  A5 - (I2C: SCL) Hardware I2C, used to communicate with the primary I2C Bus
//A3   â€“ 
//  A4 - (I2C: SDA) Hardware I2C, used to communicate with the primary I2C Bus
//A2   â€“  Raw Voltage Divider/Monitor
//A1   â€“  IrDA Tx (Out to IrDA)
//A0   â€“  IrDA Rx (In from IrDA)
//D13  â€“ (LED/SPI: SCK)  
//D12  â€“ (SPI: MISO)     
//D11  â€“ (PWM/SPI: MOSI) 
//D10  â€“ (PWM/SPI: SS)   

//  A6 â€“ 
//  A7 â€“ 

//TXO  â€“ BlueTooth Radio
//RXI  â€“ BlueTooth Radio (http://www.elecfreaks.com/wiki/index.php?title=Bluetooth_Bee)
//RST  - (Reset pin)
//GND  â€“ 
//D02  â€“ (INT0)
//D03  â€“ (INT1/PWM)
//D04  â€“ Piezo Speaker
//D05  â€“ IrDA  Enable
//D06  â€“ 
//D07  â€“ 
//D08  â€“ 
//D09  â€“ 

 


//This is the enable pin for the IrDa Serial Port
const int IrDaEnablePin = 5;




//
const int speakerPin = 4;


//What pin is the battery voltage divider mapped to?
const int vRegMonPin = A2;


boolean InitPower = false;    //This is the initial Power UP State...  This is zero if we have just powered up!  It will be one after we have powered completely up.
boolean SleepState = false;
boolean MotorRunning = false;
boolean cmgActive = false;
boolean CMGTestState = false;
boolean servoTestState = false;





//
//Servo myservo;  // create servo object to control a servo 
//Servo mymotor;  // create servo object to control a motor 
                // a maximum of eight servo objects can be created 
int motorspeed = 0;    // variable to store the servo position 
const int motorinit = 165;    // variable to store the servo position 
const int motorlow = 200;    // variable to store the servo position
int pos = 0;    // variable to store the servo position 
int posLast = 340;    // variable to store the servo position 
int posCenter = 340;    // variable to store the servo position 
int posLeft = 530;    // variable to store the servo position 
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


Timer tmr;


int idxSensorToRead = 0;


int value = 0; // set values you need to zero
 



void loop() {
  //Run the timer events
  tmr.update();

  if (infraredSerial.available () > 0){
    processIncomingByte (infraredSerial.read ());
    //Serial.print("char");
  }
}



void setup() {
  //Before we do anything else, let the user know we are doing something!
  bleep(3, 25, 500, 80, 3);



  //Prepare the port so we can see debug messages!
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  
  //Setup pins, as required
  setPins();





  //Start the I2C interface
  Wire.begin();
  
  i2cScan();
  
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  mag.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");


  Serial.println("Initializing PWM device...");
  pwm.reset();
  pwm.begin();
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency for Servos, suggest using 50 instead
  //pwm.setPWMFreq(1000);  // This is the minimum PWM frequency for the fan controller


/*
  pwm.setPWM(0, 0, MOTORZERO);
  delay(7000);
  pwm.setPWM(0, 0, MOTORREADY);
  delay(7000);
*/
  
  //Use this sequence to reset the Min/Max settings on the Mx-3A ESC!!
  
  pwm.setPWM(0, 0, MOTORZERO);
  delay(7000);
  pwm.setPWM(0, 0, MOTORMAX);
  delay(1000);
  pwm.setPWM(0, 0, MOTORSTART);
  delay(1000);
  pwm.setPWM(0, 0, MOTORREADY);
  delay(7000);
  


  digitalWrite(IrDaEnablePin, LOW);  //LOW is off, HIGH is on!
  //Just a small delay
  delay(100);
  //This can only be started when the 
  infraredSerial.begin(9600);

  
  //Initialize variables as needed
  setVariables();
  





  //Set the CMG timer event
  //tmr.every(50, setServo);
  //Set the LED timer event
  tmr.every(100, CycleLEDs);
  //Set the Compass timer event
  tmr.every(500, Maneuver);
  //Set the Sensor timer event
  tmr.every(1000, readSensors);
  //Set the Motor & Servo timer event
  //tmr.every(60000, ServoTest);
  //servoTestState = false;

  
  timerDelay(1000);

 
  //Set the "I am initialized" flag
  InitPower = true;    //We have completely powered up.  Now set the flag so we know we are already running
  //let the user know we are going into the Main Loop!
  //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay)
  bleep(1, 20, 3500, -150, 10);




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
    setCMG(targHeading, 500);
    cmgActive = false;
  }
}


void CMGTest() {
  if (CMGTestState == false) {
    CMGTestState = true;

    //Start the motor up!
    runMotor(MOTORMAX);

    leftTurn ();
    timerDelay(5000);
    returnToCenter();
    timerDelay(1000);
    rightTurn ();
    timerDelay(5000);
    returnToCenter();
    timerDelay(1000);

    //Start the motor up!
    runMotor(MOTORIDLE);

    CMGTestState = false;
  }
}


void setCMG(float tgtYAW, uint16_t timeDomain) {
//This routine is based on similar logic used in quadricopters and other systems
//where a target is attempting to be reached...
//This is a Parallel PID

  mag.getHeading(&mx, &my, &mz);
  // To calculate heading in degrees. 0 degree indicates North
  float heading = atan2(my, mx);
  if(heading < 0)
    heading += 2 * M_PI;




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




  float finalHeading = heading * 180/M_PI;
  float errHeading = abs(tgtYAW - finalHeading);




  if (tgtYAW < finalHeading) {
    valDirection = 1;
  }
  else
  {
    valDirection = -1;
  }
  
  if (errHeading > 180) {
    //Motor must be spinning CCW
    //This result means the target position is closer if we spin in the opposite direction
    errHeading = 360 - errHeading;
    valDirection = valDirection * -1;
  }

  
  float u = PID(errHeading);

  Serial.print("heading:\t");
  Serial.println(finalHeading);
  Serial.print("PID Results:\t");
  Serial.println(u);
  
  
  runMotor(MOTORMAX);
  Serial.print("Motor:\t");
  Serial.println(LastMOTOR);


  setServo(posLast + (u * valDirection));
  
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
      for (uint16_t pulselen = LastMOTOR ; pulselen >= setMOTOR; pulselen--) {
        Serial.println(pulselen);  
        LastMOTOR = pulselen;
        pwm.setPWM(servochannelnum, 0, pulselen);
        timerDelay(25);
      }
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
  Serial.print("Servo:\t");
  Serial.println(sumPosition);


  if (sumPosition < posRight || sumPosition > posLeft) {
    sumPosition = posCenter;
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
     for(pos = posLast; pos > sumPosition; pos -= 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      pwm.setPWM(1, 0, pos);
      timerDelay(posDelay);                       // waits 15ms for the servo to reach the position 
    } 
  }
  else
  {
     for(pos = posLast; pos < sumPosition; pos += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      pwm.setPWM(1, 0, pos);
      timerDelay(posDelay);                       // waits 15ms for the servo to reach the position 
    }
  }
  posLast = sumPosition;
  Serial.println(posLast);

}






//----------------------------------------------------------------
//
//
//   This section contains the primary Serial IrDA subroutines
//
//
//----------------------------------------------------------------
void processIncomingByte (const byte inByte)
  {
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;
      
  //Serial.print(char(inByte));

  switch (char(inByte))
  {
    case '?': 
    case '>':
    case '!':
      memset(aryInput,0,sizeof(aryInput));
      aryInput[0] += (char)inByte;
      Serial.println("Restart");
      intPipeCnt = 0;
      break;
      
    case '|':
      intPipeCnt += 1;
      //Serial.print('|');
      break;
      

    case '\n':   // end of text
      if (GetCheckSum){
        if (intPipeCnt == 4){
          for (int i = 0; i < 4;i++){
             temp = aryInput[i];
             Serial.print(temp.length());
            for (int j = 0; j < temp.length();j++){
                Serial.print("Byte" + temp[j]);
                }
                Serial.print('|');
             }
          Serial.println();
        }
        else
        {
          Serial.println(intPipeCnt);
        }
       Serial.println("OK");
      }
      else
      {
        Serial.println("NO");
      }
      break;

    case '\r':   // discard carriage return
      break;

    default:

        aryInput[intPipeCnt] += (char)inByte;
        //Serial.print(inByte);

      //if (input_pos < (MAX_INPUT - 1))
        //input_line [input_pos++] = inByte;
      break;

  }  // end of switch
   
} // end of processIncomingByte  


void process_data (const char * data)
  {
  // for now just display it
  // (but you could compare it to some value, convert to an integer, etc.)
  Serial.println (data);
}  // end of process_data


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
//   This section contains voltage measurement code
//
//   An analog pin must be used, and may require using
//   the aurduino's internal voltage reference
//
//
//----------------------------------------------------------------
float calcVolts(float adjMultiplier) {
  float volts = 0;
  //denominator = (float)resistor1 / (resistor1 + resistor2);
  //d = 3,300,000 / 1,000,000 + 3,300,000
  //d = .76744
  //voltage = (analogRead(anlgPin) / 1024) * 1.1;
  //v = pin * .00107
  //voltage = voltage / denominator;
  //v = pin * .0008
  
  int numSamples = 50;    //use 1000, but fewer may be ok
  for(int i = 0; i < numSamples; i++) {
    volts = volts + (analogRead(vRegMonPin) * adjMultiplier) / numSamples;  //.0045
    delay(1);
  }
  volts = volts * .98;
  //Serial.println("Bus Voltage:");    
  //Serial.println(volts);
  
  return volts;  
}









//----------------------------------------------------------------
//
//
//   This section contains the Compass subroutines
//
//
//----------------------------------------------------------------
void ReadCompass(){
  
    // read raw heading measurements from device
    mag.getHeading(&mx, &my, &mz);

    // display tab-separated gyro x/y/z values
    Serial.print("mag:\t");
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.print(mz); Serial.print("\t");
    
// To calculate heading in degrees. 0 degree indicates North
    float heading = atan2(my, mx);
    if(heading < 0)
      heading += 2 * M_PI;
    Serial.print("heading:\t");
    Serial.println(heading * 180/M_PI);
  
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




