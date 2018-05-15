
/*
>> Pulse Sensor Amped 1.1 <<
This code is for Pulse Sensor Amped by Joel Murphy and Yury Gitman
    www.pulsesensor.com 
    >>> Pulse Sensor purple wire goes to Analog Pin 0 <<<
Pulse Sensor sample aquisition and processing happens in the background via Timer 2 interrupt. 2mS sample rate.
PWM on pins 3 and 11 will not work when using this code, because we are using Timer 2!
The following variables are automatically updated:
Signal :    int that holds the analog signal data straight from the sensor. updated every 2mS.
IBI  :      int that holds the time interval between beats. 2mS resolution.
BPM  :      int that holds the heart rate value, derived every beat, from averaging previous 10 IBI values.
QS  :       boolean that is made true whenever Pulse is found and BPM is updated. User must reset.
Pulse :     boolean that is true when a heartbeat is sensed then false in time with pin13 LED going out.

This code is designed with output serial data to Processing sketch "PulseSensorAmped_Processing-xx"
The Processing sketch is a simple data visualizer. 
All the work to find the heartbeat and determine the heartrate happens in the code below.
Pin 13 LED will blink with heartbeat.
If you want to use pin 13 for something else, adjust the interrupt handler
It will also fade an LED on pin fadePin with every beat. Put an LED and series resistor from fadePin to GND.
Check here for detailed code walkthrough:
http://pulsesensor.myshopify.com/pages/pulse-sensor-amped-arduino-v1dot1

Code Version 02 by Joel Murphy & Yury Gitman  Fall 2012
This update changes the HRV variable name to IBI, which stands for Inter-Beat Interval, for clarity.
Switched the interrupt to Timer2.  500Hz sample rate, 2mS resolution IBI value.
Fade LED pin moved to pin 5 (use of Timer2 disables PWM on pins 3 & 11).
Tidied up inefficiencies since the last version. 
*/
#include <Wire.h>
//#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#include <SparkFun_MMA8452Q.h> // Includes the SFE_MMA8452Q library
#define TMP102_I2C_ADDRESS 72 /* This is the I2C address for our chip.
This value is correct if you tie the ADD0 pin to ground. See the datasheet for some other values. */
MMA8452Q accel;

//LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
LiquidCrystal_I2C lcd(0x3F,20,4); 

//  VARIABLES
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin


// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, the Inter-Beat Interval
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.


void setup(){

  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);
  lcd.init(); 
    lcd.backlight();
    lcd.setCursor(0, 0);
  
  // initialize the serial communications:
  //Serial.begin(9600);
  
 // pinMode(blinkPin,OUTPUT);         // pin that will blink to your heartbeat!
 // pinMode(fadePin,OUTPUT);          // pin that will fade to your heartbeat!
  Serial.begin(115200);             // we agree to talk fast!
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
   // UN-COMMENT THE NEXT LINE IF YOU ARE POWERING The Pulse Sensor AT LOW VOLTAGE, 
   // AND APPLY THAT VOLTAGE TO THE A-REF PIN
   //analogReference(EXTERNAL);   
   Wire.begin(); // start the I2C library

  accel.init(); 


}

void getTemp102(){
  byte firstbyte, secondbyte; //these are the bytes we read from the TMP102 temperature registers
  int val; /* an int is capable of storing two bytes, this is where we "chuck" the two bytes together. */ 
  float convertedtemp; /* We then need to multiply our two bytes by a scaling factor, mentioned in the datasheet. */ 
  float correctedtemp; 
  // The sensor overreads (?) 


  /* Reset the register pointer (by default it is ready to read temperatures)
You can alter it to a writeable register and alter some of the configuration - 
the sensor is capable of alerting you if the temperature is above or below a specified threshold. */

  Wire.beginTransmission(TMP102_I2C_ADDRESS); //Say hi to the sensor. 
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(TMP102_I2C_ADDRESS, 2);
  Wire.endTransmission();


  firstbyte      = (Wire.read());    /*read the TMP102 datasheet - here we read one byte from each of the temperature registers on the TMP102*/
  secondbyte     = (Wire.read());    /*The first byte contains the most significant bits, and the second the less significant */
   
    val = ((firstbyte) << 4);  /* MSB */
    val |= (secondbyte >> 4);  /* LSB is ORed into the second 4 bits of our byte. Bitwise maths is a bit funky, but there's a good tutorial on the playground*/
    convertedtemp = val*0.0625;
    correctedtemp = convertedtemp - 5;  /* See the above note on overreading */

  Serial.print("Converted temp is ");
  Serial.print("\t");
  Serial.println(convertedtemp);
  Serial.print("Corrected temp is ");
  Serial.print("\t");
  Serial.println(correctedtemp);
  lcd.setCursor(0,0);
  lcd.print("Temp: ");
  lcd.print(correctedtemp);
  lcd.print(" graden");
  Serial.println();
  delay(400);
  
  
}
void sendDataToProcessing(char symbol, int data ){
    Serial.print(symbol);                // symbol prefix tells Processing what type of data is coming
    Serial.println(data);                // the data to send culminating in a carriage return
    lcd.setCursor(0,1);
    lcd.print("Heartbeat: ");
    lcd.print(data);
    lcd.print("BPM");
    delay(1000);
    //lcd.clear();
  }

 


void loop(){
 // sendDataToProcessing('S', Signal);     // send Processing the raw Pulse Sensor data
  if (QS == true){                       // Quantified Self flag is true when arduino finds a heartbeat
        fadeRate = 255;                  // Set 'fadeRate' Variable to 255 to fade LED with pulse
        sendDataToProcessing('B',BPM);   // send heart rate with a 'B' prefix
       // sendDataToProcessing('Q',IBI);   // send time between beats with a 'Q' prefix
        QS = false;                      // reset the Quantified Self flag for next time    
     }
  delay(20);                             //  take a break


getTemp102();
  
//sendDataToProcessing();
delay(1000); //wait 5 seconds before printing our next set of readings. 

  if (accel.available())
  {
                                             // First, use accel.read() to read the new variables:
    accel.read();
    
                                            // accel.read() will update two sets of variables. 
                                            // * int's x, y, and z will store the signed 12-bit values 
                                            //   read out of the accelerometer.
                                            // * floats cx, cy, and cz will store the calculated 
                                            //   acceleration from those 12-bit values. These variables 
                                            //   are in units of g's.
                                            // Check the two function declarations below for an example
                                            // of how to use these variables.
    printCalculatedAccels();
                                            //printAccels(); // Uncomment to print digital readings
    
                                            // The library also supports the portrait/landscape detection
                                            //  of the MMA8452Q. Check out this function declaration for
                                            //  an example of how to use that.
    printOrientation();
    
    Serial.println(); // Print new line every time.
  }

} //end loop

void printAccels()
{
  Serial.print(accel.x, 3);
    
  Serial.print("\t");
  Serial.print(accel.y, 3);
    
  Serial.print("\t");
  Serial.print(accel.z, 3);
    
  Serial.print("\t");
}

void printCalculatedAccels()
{ 
  Serial.print("X = ");
  Serial.print(accel.cx, 3);
    lcd.setCursor(0,2);
    lcd.print("x:");
    lcd.print(accel.cx, 2);
  Serial.print("\t");
  Serial.print("Y = ");
  Serial.print(accel.cy, 3);
    lcd.print("y:");
    lcd.print(accel.cy, 2);
  Serial.print("\t");
  Serial.print("Z = ");
  Serial.print(accel.cz, 3);
    lcd.print("z:");
    lcd.print(accel.cz, 2);
  Serial.print("\t");
  
}

void printOrientation()
{
  // accel.readPL() will return a byte containing information
  // about the orientation of the sensor. It will be either
  // PORTRAIT_U, PORTRAIT_D, LANDSCAPE_R, LANDSCAPE_L, or
  // LOCKOUT.
  byte pl = accel.readPL();
  switch (pl)
  {
    
  case PORTRAIT_U:
    Serial.print("Portrait Up");
    lcd.setCursor(0,3);
    lcd.print("                ");
    lcd.setCursor(0,3);
    lcd.print("Portrait Up");
    break;
  case PORTRAIT_D:
    Serial.print("Portrait Down");
    lcd.setCursor(0,3);
    lcd.print("                ");
    lcd.setCursor(0,3);
    lcd.print("Portrait Down");
    break;
  case LANDSCAPE_R:
    Serial.print("Landscape Right");
    lcd.setCursor(0,3);
    lcd.print("                ");
    lcd.setCursor(0,3);
    lcd.print("Landscape Right");
    break;
  case LANDSCAPE_L:
    Serial.print("Landscape Left");
    lcd.setCursor(0,3);
    lcd.print("                ");
    lcd.setCursor(0,3);
    lcd.print("Landscape Left");
    break;
  case LOCKOUT:
    Serial.print("Flat");
    lcd.setCursor(0,3);
    lcd.print("                ");
    lcd.setCursor(0,3);
    lcd.print("Flat");
    break;
  }
  delay(400);
}







