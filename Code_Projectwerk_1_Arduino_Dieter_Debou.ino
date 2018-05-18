/*
 * Code by Dieter Debou
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SparkFun_MMA8452Q.h> // Includes the SFE_MMA8452Q library
#define TMP102_I2C_ADDRESS 72  // This is the I2C address for our chip.
MMA8452Q accel;
LiquidCrystal_I2C lcd(0x3F,20,4); 

//  VARIABLES
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin


// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                         // used to hold the pulse rate
volatile int Signal;                      // holds the incoming raw data
volatile int IBI = 600;                   // holds the time between beats, the Inter-Beat Interval
volatile boolean Pulse = false;           // true when pulse wave is high, false when it's low
volatile boolean QS = false;              // becomes true when Arduoino finds a beat.
volatile int rate[10];                    // used to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find the inter beat interval
volatile int P =512;                      // used to find peak in pulse wave
volatile int T = 512;                     // used to find trough in pulse wave
volatile int thresh = 512;                // used to find instant moment of heart beat
volatile int amp = 100;                   // used to hold amplitude of pulse waveform
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = true;       // used to seed rate array so we startup with reasonable BPM


void interruptSetup(){     
  // Initializes Timer2 to throw an interrupt every 2mS.
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER 
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED      
} 

// THIS IS THE TIMER 2 INTERRUPT SERVICE ROUTINE. 
// Timer 2 makes sure that we take a reading every 2 miliseconds
ISR(TIMER2_COMPA_vect){                         // triggered when Timer2 counts to 124
    cli();                                      // disable interrupts while we do this
    Signal = analogRead(pulsePin);              // read the Pulse Sensor 
    sampleCounter += 2;                         // keep track of the time in mS with this variable
    int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

//  find the peak and trough of the pulse wave
    if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
        if (Signal < T){                        // T is the trough
            T = Signal;                         // keep track of lowest point in pulse wave 
         }
       }
      
    if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
        P = Signal;                             // P is the peak
       }                                        // keep track of highest point in pulse wave
    
  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
if (N > 250){                                   // avoid high frequency noise
  if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){        
    Pulse = true;                               // set the Pulse flag when we think there is a pulse
    digitalWrite(blinkPin,HIGH);                // turn on pin 13 LED
    IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
    lastBeatTime = sampleCounter;               // keep track of time for next pulse
         
         if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
             firstBeat = false;                 // clear firstBeat flag
             return;                            // IBI value is unreliable so discard it
            }   
         if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
            secondBeat = false;                 // clear secondBeat flag
               for(int i=0; i<=9; i++){         // seed the running total to get a realisitic BPM at startup
                    rate[i] = IBI;                      
                    }
            }
          
    // keep a running total of the last 10 IBI values
    word runningTotal = 0;                   // clear the runningTotal variable    

    for(int i=0; i<=8; i++){                // shift data in the rate array
          rate[i] = rate[i+1];              // and drop the oldest IBI value 
          runningTotal += rate[i];          // add up the 9 oldest IBI values
        }
        
    rate[9] = IBI;                          // add the latest IBI to the rate array
    runningTotal += rate[9];                // add the latest IBI to runningTotal
    runningTotal /= 10;                     // average the last 10 IBI values 
    BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
    QS = true;                              // set Quantified Self flag 
    // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
}

  if (Signal < thresh && Pulse == true){     // when the values are going down, the beat is over
      digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
      Pulse = false;                         // reset the Pulse flag so we can do it again
      amp = P - T;                           // get amplitude of the pulse wave
      thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
      P = thresh;                            // reset these for next time
      T = thresh;
     }
  
  if (N > 2500){                             // if 2.5 seconds go by without a beat
      thresh = 512;                          // set thresh default
      P = 512;                               // set P default
      T = 512;                               // set T default
      lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
      firstBeat = true;                      // set these to avoid noise
      secondBeat = true;                     // when we get the heartbeat back
     }

  sei();                                     // enable interrupts when youre done!
  
}// end isr

void setup(){
  
  lcd.begin(20, 4);   
  lcd.init(); 
  lcd.backlight();
  lcd.setCursor(0, 0);  // set up the LCD's number of columns and rows
  
  Serial.begin(115200);             // set up the communication fast
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
  Wire.begin(); // start the I2C library
  accel.init(); 

}

void getTemp102(){
  byte firstbyte, secondbyte; //these are the bytes we read from the TMP102 temperature registers
  int val; /* an int is capable of storing two bytes, this is where we "chuck" the two bytes together. */ 
  float convertedtemp; /* We then need to multiply our two bytes by a scaling factor, mentioned in the datasheet. */ 
  float correctedtemp; 
  // The sensor overreads (?) 

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
    correctedtemp = convertedtemp/*- 5*/;  /* See the above note on overreading */

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







