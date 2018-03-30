//#include <LiquidCrystal.h>



void setup() {
Serial.begin(115200);
/*lcd.begin(16, 2); 
lcd.print("hello");
delay(2000);*/
  
}

void loop() {
 double waterlevel = 12.33;
 int pubg = 1234;

 /*String output = "";
 output += "[" + x + "|" + y + "]"; 

Serial.println(output);*/

 Serial.print("[ " );
 Serial.print(waterlevel);
 Serial.print(" | ");
 Serial.print(pubg);
 Serial.println(" ]");
 delay(000);

}
