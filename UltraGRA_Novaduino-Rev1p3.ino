//*******10********20********30********40********50********60********70*******80********90*******100

#define LABEL "UltraGRA_Novaduino-Rev1p3" // File Name
// IDE:Arduino 1.8.19
// Date: 08/22/2025
// Author: Nova Radio Labs LLC
// for Novaduino(R) with MaxBotix MB1010 Ultrasonic Range Sensor
//
/*
 * *  Copyright (c) 2024-25 Nova Radio Labs LLC

 *Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 *and associated documentation files (the “Software”), to deal in the Software without restriction, 
 *including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 *sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
 *furnished to do so, subject to the following conditions:

 *The above copyright notice and this permission notice shall be included in all copies or 
 *substantial portions of the Software.

 *THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 *BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 *NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 *DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *  
 *  This software is released under the MIT License(http://opensource.org/licenses/MIT).
 *  
 *  Graphics and display and other driver software is from Adafruit Industries. Thank you Adafruit.
 *  Other software contained herein is from other sources as noted.
 *  * Feather form factor is from Adafruit.
 *  
 */

//#define DEBUG   //if you want to send debug data to the Arduino IDE serial monitor, uncomment
                  // this statement
                  
#define ILI9341 // enter ILI9341 or ST7789. must correspond to the display you have

//*******10********20********30********40********50********60********70*******80********90*******100
//***************Include LCD Drivers****************************************************************

#include <SPI.h>               // by Arduino.cc
#include <Adafruit_GFX.h>      // graphics library by Adafruit https://www.adafruit.com/product/3787
//#include <Fonts/FreeSansBoldOblique9pt7b.h>  // for future release

#if defined(ST7789)
  #include <Adafruit_ST7789.h> //  library for ST7789 by Adafruit
#endif

#if defined(ILI9341)
  #include <Adafruit_ILI9341.h>  // library for ILI9341 display by Adafruit
#endif


  // From Adafruit: SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
  // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
  // may end up with a black screen some times, or all the time.
#define SPISPEED 16000000   // in Hz

#define TFT_CS        5     // CS is on D5
#define TFT_RST       A5    // Disp_RST(Display Reset) is on A5
#define TFT_DC        6     // DC(Data/Command) is on D6

#if defined(ST7789)
  Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); //instantiate tft.
#endif

#if defined(ILI9341)
  Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST ); //instantiate tft.
#endif


// Color Map that works with both display drivers

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GREY    0xCE79
#define LIGHTGREY 0xDEDB

//Item Colors
#define TOPBOX RED
#define TOPTEXT WHITE
#define BACKGROUND BLUE     // when the car is out of range, the background is this color
#define RANGETEXT WHITE
#define INRANGE GREEN       // when the car reaches max range, the background changes to this color
#define MINBACKGROUND RED   // when the car reaches min range, the background changes to this color

int BLpin = A2;         // LCD backlight buffer is connected to A2 on the Novaduino Display
int deltaIntens = 0;    // used for setting intensity ranges [-10 to 0]
int intensity = 255;    // set the initial intensity to full on 

//**************WS2812 RGB LED is on Pin D13********************************************************

#include <Adafruit_NeoPixel.h> // from Adafruit
#define NEOPIN    13  // Novaduino NEOPIX is on pin D13
#define NUMPIXELS 1   // only 1 NEOPIX on the Novaduino 2.4 Display
uint8_t red = 0;      // initial NEOPIX colors
uint8_t blue = 0;
uint8_t green = 0;
uint8_t greenTemp = 0;
uint8_t blueTemp = 0;
uint8_t redTemp = 0;
int RGBcount = 0;

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);     //instantiate pixels.


//*******Initialize TWI(I2C) driver and ATtiny1628 Keyscan/R.Encoder read Interrupt variables *****

#include <Wire.h>  // by Arduino. Used to read keyboard & Rot Encoder

const uint8_t twiInterruptPin = 12; //the I2C interrupt signal from the kbd proc is on D12

// variables that are used in the ISR should be declared volatile:

volatile bool twiIntFlag = false;   // if the interrupt occurs, this flag is set true
volatile int8_t button[4];          // receives the data from the keyboard processor over I2C
volatile int8_t buttonTot =0;       // accumulates the rotary encoder for display
volatile int8_t prevButtonTot = 0;  // used to detect changes to the rot encoder
volatile int8_t buttonNum = 0;      // collects the final button number

char Bbuffer[2];      // Button buffer used for formatted display
char REbuffer[3];     // Rotary Encoder buffer used for formatted display

//*******10********20********30********40********50********60********70*******80********90*******100
// Touchscreen Driver from Adafruit. 
// NOTE: this is not used for the UltrasonicCPA but included if
// you would like to modify the program to use touchscreen inputs.

//This is calibration data for mapping the raw touch data to the screen coordinates

//#include "Adafruit_TSC2007.h"
//
//#define TS_MINX 300
//#define TS_MINY 300
//#define TS_MAXX 3800
//#define TS_MAXY 3850
//#define TS_MIN_PRESSURE 200
//#define TSC_IRQ 9           //the Novaduino TSC2007 interrupt pin is connected to D9
//
//Adafruit_TSC2007 ts = Adafruit_TSC2007();   //instantiate touch screen as ts.
//#define PENRADIUS 3                         // used for drawing dots on the screen


//*******10********20********30********40********50********60********70*******80********90*******100
//*********************Initialize The Ultrasonic Sensor**************************************************

#define sensorPin A1       // use input A1 to measure pulse width on the PW pin of the EZ1 sensor
int pwmRange, rangeInch;   // display output in inches
int Rmax = 40;             // initialize the max target range, inches
int Rmin = 35;             // initialize the min target range, inches
int SenCount = 0;          // delay reading reading the sensor
static char RangeBuf[3];   // a string buffer for formatting the ultrasonic range

//variables for reading the Ultrasonic Sensor occasionally. Defined by an "interval"
//the following is from "BlinkWithoutDelay" by David A Mellis et al, 2005

unsigned long previousMillis = 0;
const long INTERVAL = 100;         // in milliseconds. This sets the rate for reading the sensor
                                   // if the rate is too slow, button response becomes sluggish



////////////////////////////////////////////////////////////////////////////////////////////////////
//*******10********20********30********40********50********60********70*******80********90*******100
//*************************SETUP********************************************************************

void setup(void) {

//********setup SERIAL for DEBUG only***************************************************************
  #ifdef DEBUG 
    Serial.begin(115200);
    //while (!Serial) delay(10);  // you must open the serial monitor or program wont run
  #endif

//SETUP*********initialize the Wire interface as Host***********************************************

  Wire.begin();

//SETUP********** setup the LCD Backlight **********************************************************


  pinMode(BLpin, OUTPUT);   
  digitalWrite(BLpin, HIGH);   // initially set the LCD brightness to full on
  //see PWM control by Rotary Encoder in Main


//SETUP********** setup LCD TFT Display ************************************************************

  #if defined(ST7789)
    tft.init(240, 320);    // only needed for ST7789
    tft.setRotation(1);    // rotate screen to landscape mode
  #endif
   
  #if defined(ILI9341)
    tft.begin();
    tft.invertDisplay(1);  // only needed for ILI9341
    tft.setRotation(3);    // rotate screen to landscape mode
  #endif

//SETUP*********** setup SPI interface *************************************************************
  
  tft.setSPISpeed(SPISPEED);  //setting SPI
  #ifdef DEBUG
    Serial.println("TFT Initialized");
  #endif
 
  
//SETUP********* measure time to boot and show on flash screen ************************************

  uint16_t boottime = millis();  // time of boot
  tft.fillScreen(BLACK);
  boottime = millis() - boottime;  // time since boot

//SETUP**********print the flash screen with file information**************************************

  tft.fillScreen(WHITE);
  tft.setTextSize(3);
  tft.setTextColor(BLUE);
  tft.println(LABEL);  //LABEL is set to the file name at the top of this program
  tft.println(boottime, DEC);
  delay(3000);          // a little bit of time to read the flash screen

  
//*******10********20********30********40********50********60********70*******80********90*******100
//SETUP*******************Regular information Display Starts Here***********************************

//******************Set the overall Screen color****************************************************

  tft.fillScreen(BACKGROUND); 

//SETUP****************print the top label box *****************************************************

  tft.fillRoundRect(3, 5, 313, 25, 4, TOPBOX);  // adds the filled and rounded rectangle
  tft.setTextWrap(false);
  tft.setCursor(10, 11);   //start text here
  tft.setTextColor(TOPTEXT);
  tft.setTextSize(2);
  tft.println("NOVADUINO Ultrasonic GRA"); // Customize by putting your name here
   

//initialize digital pin for RGB LED as an output
  pinMode(13, OUTPUT); // on Novaduino this controls the RGB LED

//SETUP********set up buffers for displaying the Rotary Enc and Key numbers*************************

  sprintf(Bbuffer, "%02d", 0);  // 2 places for button string buffer
  sprintf(REbuffer, "%03d", 0);  // 3 places for rotary encoder string buffer

//SETUP***********set up I2C/TWI Interrupt for reading key and R.E. processor **********************

  pinMode(twiInterruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(twiInterruptPin), readTWI, FALLING);

//SETUP****************initialize the NEOPIX *******************************************************

  pixels.begin();

//SETUP************Using PWM signal from MaxBotix EZx Ultrasonic Sensor***************************

  pinMode(sensorPin, INPUT);

 }   
//*************************END Of SETUP***********************************************************





////////////////////////////////////////////////////////////////////////////////////////////////////
//**********************MAIN LOOP*******************************************************************
//*******10********20********30********40********50********60********70*******80********90*******100

void loop(){

//set up display for the range reading

  tft.setTextWrap(false);
  tft.setCursor(15, 90);                    //start range text here
  tft.setTextSize(10);
  //tft.setFont(&FreeSansBoldOblique9pt7b); // still working this. For future release
  tft.setTextColor(RANGETEXT,BACKGROUND);   //1st color is text, 2nd color is background

//Scan the Ultrasonic Sensor: MaxBotix MB1010 every INTERVAL milliseconds

  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= INTERVAL)   //non-blocking time delay
  {
  // Save the last time the ultrasonic sensor was read
    previousMillis = currentMillis;
    
  // Scan the sensor

    SenCount += 1;
    if(SenCount >= 4)   // delay reading the sensor
    {    
      pwmRange = pulseIn(sensorPin, HIGH);  // uses the Arduino pulseIn command to measure Pulse width
      rangeInch = pwmRange/147;             // 147 uSec per inch according to MB1010 data sheet
        
     sprintf(RangeBuf, "%03d" "in", rangeInch);  // buffer the range value to 3 digits

     // this section checks the range against Rmax and Rmin and changes the range colors
      if(rangeInch > Rmax)
          {
          tft.println(RangeBuf);
          red = 0; blue = 128; green = 0;   // change the RGB LED to Blue
      } else {
              if(rangeInch <= Rmax && rangeInch >= Rmin){
                tft.setTextColor(INRANGE,BACKGROUND);                // if within range change text color
                tft.println(RangeBuf);
                red = 0; blue = 0; green = 128;                      // change the RGB LED to Green.
                } else {
                        if(rangeInch < Rmin)
                          {
                          tft.setTextColor(RANGETEXT,MINBACKGROUND);  // if too close, use red background
                          tft.println(RangeBuf);  
                          red = 128; blue = 0; green = 0;             // change the RGB LED to Red.
                          }
                   }
              }
      SenCount = 0;
    }

    // Change the RGB LED every 5*INTERVAL seconds. the LED stops blinking when adjusting controls
    RGBcount += 1;    
    switch(RGBcount)
    {
      case 0 ... 4: 
          greenTemp = green;
          blueTemp = blue;
          redTemp = red; 
          break;
      case 5 ... 8:
          greenTemp = 0;
          blueTemp = 0;
          redTemp = 0; 
          break; 
      case 9:
          RGBcount = 0;
          greenTemp = 0;
          blueTemp = 0;
          redTemp = 0; 
          break;   
      default:
        RGBcount = 0;
        break;
    }

  int pixnum = NUMPIXELS -1;
      pixels.clear();
      pixels.setPixelColor(pixnum, pixels.Color(redTemp, greenTemp, blueTemp)); // 0 to 255, (Red, Green, Blue)
      pixels.show();   
    
      
  }
  



// Set the LCD Backlight intensity according to the rotary endcoder
//
//  buttonNum (number of button pressed) and buttonTot (encoder reading) come from 
//  the Interrupt Service Routine triggered when the Keyscan processor detects
//  a change.

int tempButtonNo = buttonNum;  // this traps the button number since the Keyscan processor
                               // sets it to 0 when it detects and sends a rotary encoder change

while(tempButtonNo == 3)   // stay here if button 3 is pressed. press button 11 to escape
    {
      deltaIntens = deltaEncoder(deltaIntens, -10, 0);
      intensity = (255 + (deltaIntens*25));           // deltaIntense ranges[-10 to 0]
                                                      // so intensity ranges 5 to 255
      analogWrite(BLpin, intensity); 
  
    // display deltaIntens and intensity numbers in DEBUG mode only
    #ifdef DEBUG
      //tft.setFont();
      tft.setCursor(15,50);
      tft.setTextSize(2);
      sprintf(REbuffer, "%03d", deltaIntens);
      tft.print(REbuffer);
      tft.print("  intens:");
      sprintf(REbuffer, "%03d", intensity);
      tft.setTextColor(RANGETEXT,BACKGROUND);
      tft.print(REbuffer);
    #endif

     if(buttonNum == 11) tempButtonNo = 0;  //exit the while() loop if button 11 is pressed
    }

//**********set Target Range min and max *************

// set max/far Range to Target w/ Encoder
while(tempButtonNo == 4)  // stay here if button 4 is pressed. press button 11 to escape
  {
    Rmax = deltaEncoder( Rmax, 6, 150);
    
    //Display Rmax value
      //tft.setFont();
      tft.setCursor(30, 190);          //display text here
      tft.setTextSize(2);
      tft.print(" Max Range,inches: ");   
      //print the Rmax, formatted to 3 digits
      sprintf(Bbuffer, "%03d", Rmax);
      tft.setTextColor(RANGETEXT,BACKGROUND);
      tft.println(Bbuffer);

    if(buttonNum == 11) tempButtonNo = 0;  //exit the while() loop if button 11 is pressed
  }


// set min/close Range to Target w/ Encoder
while(tempButtonNo == 5)  // stay here if button 5 is pressed. press button 11 to escape
  {

    Rmin = deltaEncoder(Rmin, 12, 140); // change Rmin w/ Encoder, can be 12<Rmin<140
    
    //Display Rmin value
      //tft.setFont();
      tft.setCursor(30, 210);          //display text here
      tft.setTextSize(2);
      tft.print(" Min Range,inches: ");   
      //print the Rmin, formatted to 3 digits
      sprintf(Bbuffer, "%03d", Rmin);
      tft.setTextColor(RANGETEXT,BACKGROUND);
      tft.println(Bbuffer);

    if(buttonNum == 11) tempButtonNo = 0;  //exit the while() loop if button 11 is pressed
  }

  
}   
//*******10********20********30********40********50********60********70*******80********90*******100
//***************************************End of Main************************************************
///////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************fUNCTIONS AND STRUCTURES********************************/
//
//************************I2C read Interrupt Service Routine for Keyscan & R.E.*********************
//
void readTWI(){

    Wire.requestFrom(0x40, 4, true);    // Request 4 bytes from slave device number 0x40

    // Slave may send less than requested
      //while(Wire.available())
        for(int i=0; i<4; i++)
        {
            button[i] = Wire.read();    // Receive a byte as int
        }
        buttonTot += button[2];
        buttonNum = button[0];
        
        twiIntFlag = false;
} 
//////////////////////////////////////////////////////////////////////////////////////////////////////
//*************************deltaEncoder Function******************************************************
//  Accepts a starting value of Val and increases or decreases it using the rotary encoder and 
//  returns the newVal. Limits the newVal according to minVal<newVal<maxVal
//
//****************************************************************************************************

int deltaEncoder(int Val, int minVal, int maxVal)
{
int newVal = Val;
int deltaEnc = buttonTot - prevButtonTot; // calculate change in rotary encoder

  if(deltaEnc != 0)      // check for a change in the rotary encoder
    {
    if(deltaEnc > 0)     //check for cw rotation or +1
        {
        if(newVal < maxVal)   // only increase newVal if not at upper limit
          {
            newVal += 1;
          }
         deltaEnc = 0;
        }
    if(deltaEnc < 0)        // check for ccw rotation or -1
        {
       if(newVal > minVal)  // only decrease newVal if not at lower limit
          {
            newVal -= 1;
          }
        deltaEnc = 0;
        }
      deltaEnc = 0;
      prevButtonTot = buttonTot;
    }
    return newVal;
}
