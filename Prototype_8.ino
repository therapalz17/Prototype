#include <Wire.h>
#include <SPI.h>
#include <Adafruit_CAP1188.h>
#include <avr/wdt.h>
#include "PCM.h"
#include "meow.h"

// Reset Pin is used for I2C or SPI
#define CAP1188_RESET  9

// CS pin is used for software or hardware SPI
#define CAP1188_CS  10

// These are defined for software SPI, for hardware SPI, check your 
// board's SPI pins in the Arduino documentation
#define CAP1188_MOSI  11
#define CAP1188_MISO  12
#define CAP1188_CLK  13

// For I2C, connect SDA to your Arduino's SDA pin, SCL to SCL pin
// On UNO/Duemilanove/etc, SDA == Analog 4, SCL == Analog 5
// On Leonardo/Micro, SDA == Digital 2, SCL == Digital 3
// On Mega/ADK/Due, SDA == Digital 20, SCL == Digital 21

// Use I2C, no reset pin!
//Adafruit_CAP1188 cap = Adafruit_CAP1188();

// Or...Use I2C, with reset pin
Adafruit_CAP1188 cap = Adafruit_CAP1188(CAP1188_RESET);

// Or... Hardware SPI, CS pin & reset pin 
// Adafruit_CAP1188 cap = Adafruit_CAP1188(CAP1188_CS, CAP1188_RESET);

// Or.. Software SPI: clock, miso, mosi, cs, reset
//Adafruit_CAP1188 cap = Adafruit_CAP1188(CAP1188_CLK, CAP1188_MISO, CAP1188_MOSI, CAP1188_CS, CAP1188_RESET);


unsigned long time0 = 0;
unsigned long time1 = 0;
unsigned long runTime;
long curInactive;
long prevInactive = 0;

const int heartPin = 6;                             //physical Arduino digital output pin
const int vibrate_out = 10;                         //analog output for 'purring'
const int resetPin = 12;
const int heartIntensity = 100;                     //intensity of heartbeat (analog output)
int pin1 = 0;
int pin2 = 0;
int pin3 = 0;
int pin4 = 0;
//int pin5 = 0;
//int pin6 = 0;
//int pin7 = 0;

int snoozeWaiting = 0;


int touched;
uint8_t all_touched;

class Heartbeat{
  long onTime;
  long offTime;

  int heartState;
  unsigned long previousMillis;

public:
  Heartbeat(long on, long off){
      onTime = on;
      offTime = off;

      heartState = 0;
      previousMillis = 0;
  }


  void update(){
    unsigned long currentMillis = millis();
     
    if((heartState == 0) && (currentMillis - previousMillis >= offTime))
    {
      heartState = 1;  // Turn it off
      previousMillis = currentMillis;  // Remember the time
      analogWrite(heartPin, heartIntensity);  // Update the actual LED
    }
    else if ((heartState == 1) && (currentMillis - previousMillis >= onTime))
    {
      heartState = 0;  // turn it on
      previousMillis = currentMillis;   // Remember the time
      analogWrite(heartPin, 0);   // Update the actual LED
    }
  }
};


class Reset{
  long onTime;
  long offTime;

  int resetState;
  unsigned long previousMillis;


public:
  Reset(long on, long off){
      onTime = on;
      offTime = off;

      resetState = 0;
      previousMillis = 0;
  }

  void update(){
    unsigned long currentMillis = millis();

    if((resetState == 0) && (currentMillis - previousMillis >= offTime))
    {
      resetState = 1;  // Turn it off
      previousMillis = currentMillis;  // Remember the time
      analogWrite(resetPin, 1);  // Update the actual LED
    }
    else if ((resetState == 1) && (currentMillis - previousMillis >= onTime))
    {
      resetState = 0;  // turn it on
      previousMillis = currentMillis;   // Remember the time
      analogWrite(resetPin, 0);   // Update the actual LED
    }
  }
};


class Purr{
  int timeElapsed;
  //unsigned long previousMillis;
public:
  Purr(int elapsed){
    timeElapsed = elapsed;
  }

  void update(){
    //unsigned long currentMillis = millis();
    Serial.println("touched: ");
    Serial.println(touched);
    Serial.println("\t");
    Serial.println("time: ");
    Serial.println(timeElapsed); 
    if((touched == 1) && (timeElapsed == 0))
    {
      doMeow();
      if(pin1 == 1) vibrate1();
      else if(pin2 == 1) vibrate1();
      else if(pin3 == 1) vibrate1();
      else if(pin4 == 1) vibrate1();
      doMeow();
      timeElapsed = 1;
    }
    if((touched == 0))
    {
      timeElapsed = 0;
    }
  }
};

Heartbeat heart(300, 800);
Reset reset1(1, 20000);
Purr purring(0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("CAP1188 test!");

  // Initialize the sensor, if using i2c you can pass in the i2c address
   if (!cap.begin(0x2B)) {
    Serial.println("CAP1188 not found");
    while (1);
   }
  Serial.println("CAP1188 found!");
  

  pinMode(heartPin, OUTPUT);
  pinMode(vibrate_out, OUTPUT);

}

void snooze(){
  analogWrite(vibrate_out, 0);
  analogWrite(heartPin, 0);
  while(!cap.touched());
}

void doMeow(){
  startPlayback(meow, sizeof(meow));
}

void vibrate1() 
{
  Serial.println("CHECKvibrate");
  
  for (int i = 0; i < random(1,3); i++) 
  {
      int range = random(1,3);
      for(int i = 0; i < range; i++)
      {
        pulse();
        heart.update();
        reset1.update();
        delay(100);
      }
  }
}

void vibrate2()
{
  Serial.println("CHECKvibrate");
  
  for (int i = 0; i < random(8, 18); i++) 
  {
      int range = random(1,10);
      for(int i = 0; i < range; i++)
      {
        pulse();
        heart.update();
        reset1.update();
        delay(50);
      }
  }
}

void vibrate3()
{
  Serial.println("CHECKvibrate");
  
  for (int i = 0; i < random(2, 6); i++) 
  {
      int range = random(1,5);
      for(int i = 0; i < range; i++)
      {
        pulse();
        heart.update();
        reset1.update();
        delay(50);
      }
  }
}

void vibrate4()
{
  Serial.println("CHECKvibrate");
  
  for (int i = 0; i < random(4, 9); i++) 
  {
      int range = random(1,12);
      for(int i = 0; i < range; i++)
      {
        pulse();
        heart.update();
        reset1.update();
        delay(50);
      }
  }
}
void pulse()
{
    //Serial.println("Check");
    int random_num = random(10, 15);
    for (int fadeVal = 130; fadeVal >= 0; fadeVal -= random_num)
    {
      Serial.println(fadeVal);
      analogWrite(vibrate_out, fadeVal);
      heart.update();
      reset1.update();
      delay(500);
    }  
}

void loop() {
  int prevTime;
  int curTime;
  int curInactive, preInactive, snoozeTime;
  
    
  heart.update();
  reset1.update();
  purring.update();
    
  all_touched = cap.touched();
  Serial.println(all_touched);
  Serial.println("\t");
  
  if (all_touched <= 0) {
    touched = 0;
    return;
  }
  
  for (uint8_t i=0; i<8; i++) {
    if (all_touched & (1 << i)) {
      if(i == 1) {          //just test pins; can change to any value of 'i'
        pin1 = 1;
        touched = 1;
        //vibrate();
      }
      
      if(i == 2) {          //just test pins; can change to any value of 'i'
        pin2 = 1;
        touched = 1;
        //vibrate();
      }
      if(i == 3) {          //just test pins; can change to any value of 'i'
        pin3 = 1;
        touched = 1;
        //vibrate();
      }
      
      if(i == 4) {
        pin4 = 1;
        touched = 1;
        //vibrate1();
      }
    }
  }

  if(touched == 0) {
    if(snoozeWaiting){
      curInactive = millis();
      if(curInactive - prevInactive >= snoozeTime){
          snooze();
      }
    }
    else{
      snoozeWaiting = 1;
      prevInactive = millis();
    }
  }
  else{
    snoozeWaiting = 0;
  }

  purring.update();
  reset1.update();
  heart.update();
  delay(50);
}  
