/****************************************************************************************************************************\
 * Arduino project "ESP Easy" � Copyright www.esp8266.nu
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 * You received a copy of the GNU General Public License along with this program in file 'License.txt'.
 *
 * IDE download    : https://www.arduino.cc/en/Main/Software
 * ESP8266 Package : https://github.com/esp8266/Arduino
 *
 * Source Code     : https://sourceforge.net/projects/espeasy/
 * Support         : http://www.esp8266.nu
 * Discussion      : http://www.esp8266.nu/forum/
 *
 * Additional information about licensing can be found at : http://www.gnu.org/licenses
\*************************************************************************************************************************/

// This file is to be loaded onto an Arduino Pro Mini so it will act as a simple IO extender to the ESP module.
// Communication between ESP and Arduino is using the I2C bus, so only two wires needed.
// It best to run the Pro Mini on 3V3, although the 16MHz versions do not officially support this voltage level on this frequency.
// That way, you can skip levelconverters on I2C.
// Arduino Mini Pro hardware pins A4 and A5 for I2C bus. ESP I2C can be configured but they are on GPIO-4 and GPIO-5 by default.

// Energy Monitor code ported over by Chester Lowrey hilo90mhz.com 5-1-2016
// Ucglib screen code added by Chester Lowrey hilo90mhz.com 9-1-2016 Reference for ucglib https://github.com/olikraus/ucglib/wiki/reference
// 

#include <Wire.h>
#include "RunningAverage.h"
#include "EmonLib.h" 


#include <SPI.h>
#include "Ucglib.h"
Ucglib_ST7735_18x128x160_HWSPI ucg(/*cd=*/ 9 , /*cs=*/ 6, /*reset=*/ 5);

#define emonTxV3
EnergyMonitor ct1, ct2, ct3, ct4, ct5;
unsigned long lastpost = 0;

RunningAverage v1Avg(10);  // 20 is about max now with screen code added - - with all averages set to 30 it seems to run out of memory
RunningAverage ct1Avg(10); // loop time is limited to 6 seconds with screen update code so averaging 10 values should give us a running 1 minute average
RunningAverage ct2Avg(10); // may be adventagious to run at 5V and 16Mhz to double speed... but then level shifinting is neccessary to ESP8266 and possibly TFT screen
RunningAverage ct3Avg(10); // and AREF would need to be used to keep analog inputs at 3.3V full scale
RunningAverage ct4Avg(10);
RunningAverage ct5Avg(10);



#define I2C_MSG_IN_SIZE    4
#define I2C_MSG_OUT_SIZE   4

#define CMD_DIGITAL_WRITE  1
#define CMD_DIGITAL_READ   2
#define CMD_ANALOG_WRITE   3
#define CMD_ANALOG_READ    4

volatile uint8_t sendBuffer[I2C_MSG_OUT_SIZE];

//double ct6Irms;

int valueRead;
char dispCounter = 1;
#define BOX_OFFSET  28
#define FIRST_BOX_OFFSET  20

void dispCT(char CT = 1) // valid CT values are 1-5 0 will cause overflow
{
  float A;
  float VA;
  float W;
  switch(CT)
  {
    case 1: // send AC current sensor 1
      A = ct1.Irms;
      VA = ct1.apparentPower;
      W = ct1.realPower;
      break;
    case 2: // send AC current sensor 2
      A = ct2.Irms;
      VA = ct2.apparentPower;
      W = ct2.realPower;
      break;
    case 3: // send AC current sensor 3
      A = ct3.Irms;
      VA = ct3.apparentPower;
      W = ct3.realPower;
      break;
    case 4: // send AC current sensor 4
      A = ct4.Irms;
      VA = ct4.apparentPower;
      W = ct4.realPower;
      break;
    case 5: // send AC current sensor 5
      A = ct5.Irms;
      VA = ct5.apparentPower;
      W = ct5.realPower;
      break;
  }
//First Line Of Box
  ucg.setPrintPos(58,3+FIRST_BOX_OFFSET+(BOX_OFFSET*(CT-1))); // clear end of line incase # of characters has decreased
  ucg.print(" "); 

  ucg.setPrintPos(25,3+FIRST_BOX_OFFSET+(BOX_OFFSET*(CT-1)));
  ucg.print(A,1); 
  ucg.print("A"); 

  ucg.setPrintPos(102,3+FIRST_BOX_OFFSET+(BOX_OFFSET*(CT-1))); // clear end of line incase # of characters has decreased
  ucg.print("   "); 

  ucg.setPrintPos(69,3+FIRST_BOX_OFFSET+(BOX_OFFSET*(CT-1)));
  ucg.print(VA, 0);
  ucg.print("VA"); 

//Second Line of Box
  ucg.setPrintPos(82,12+FIRST_BOX_OFFSET+(BOX_OFFSET*(CT-1))); // clear end of line incase # of characters has decreased
  ucg.print("     "); 
  
  ucg.setPrintPos(55,12+FIRST_BOX_OFFSET+(BOX_OFFSET*(CT-1)));
  ucg.print(W, 0);
  ucg.print("W  "); 
}

void setup()
{
  Serial.begin(57600);
  Wire.begin(0x7f);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

 
 // First value is pin number 0 = A0 which is used for AC voltage sensing
 // voltage calibration value
 // phase shift calibration
 
 // 2ma:2ma voltage sense transformer 68K series input resistor 360ohm burden output resistor: 121.9V input = .639V RMS, 1.82Vpk-pk output voltage - waveform is perfect match to input phase
  //ct1.voltage(0, 197.2, 1.7);    //Values for 68K + 360 Ohm Resistors               
  //ct1.voltage(0, 205.0, 1.7);      //Value for 68K + 330 Ohm Resistors
  //ct1.voltage(0, 210.2, 1.7);      //Value for 68K + 330 Ohm Resistors
  //ct1.voltage(0, 225, 1.7);      //Value for 68K + 330 in paralllel with 1k in series with 5K POT Ohm Resistors - Must calibrate on first use with voltmeter

  float voltageCal = 225;
  float phaseShift = 1.0;  // was 1.03
  
  ct1.voltage(0, voltageCal, phaseShift);
  ct2.voltage(0, voltageCal, phaseShift);
  ct3.voltage(0, voltageCal, phaseShift);
  ct4.voltage(0, voltageCal, phaseShift);
  ct5.voltage(0, voltageCal, phaseShift);
  
  // CT Current calibration 
  // Based on 100A : 50MA clamp on current transformers
  // (2000 turns / 22 Ohm burden resistor = 90.909) = 100A Max Current = 12KW at 120V
  // (2000 turns / 47 Ohm burden resistor = 42) = 50A Max Current = 6KW at 120V
  // (2000 turns / 120 Ohm burden resistor = 16.26) = ~20A Max Current = 2.4KW at 120V high accuracy @ low power
  // (2000 turns / 130 Ohm burden resistor = 15.38) =  ~20A Max Current - high accuracy @ low power
  // 8.2 = 120 Ohm 1:1 Current transformer from AliExpress 0-50A = 0-50Ma output
  // First value is pin number 1 = A1
  //Analog pins 4, 5 are used by I2C hardware module - not available

  float currentCal = 16.26;
  
  ct1.current(1, currentCal); 
  ct2.current(2, currentCal);
  ct3.current(3, currentCal);
  ct4.current(6, 16.26);
  ct5.current(7, 16.26);
  
  lastpost = 0;

  ucg.begin(UCG_FONT_MODE_SOLID);
  ucg.setRotate180();
  ucg.setFontPosTop();
  ucg.setColor(255, 255, 255);
  ucg.setColor(1, 0, 0, 0);    // background color in solid mode
  
  //ucg.setFont(ucg_font_helvB08_hr); // common height, variable width
  ucg.setFont(ucg_font_amstrad_cpc_8r);
  //ucg.setFont(ucg_font_ncenR14_hr);
  ucg.clearScreen();

  

  ucg.setColor(255, 0, 255); // Magenta color
  ucg.drawString(3, 4, 0, "V1");
  ucg.drawRFrame(0,0,128,17,7);  // Draw outline with rounded edges

  ucg.setColor(255, 0, 0); // Red color
  ucg.drawString(3,8+FIRST_BOX_OFFSET,0, "C1"); 
  ucg.drawRFrame(0,FIRST_BOX_OFFSET,128,25,8);  // Draw outline with rounded edges

  ucg.setColor(255, 255, 0); // Yellow color
  ucg.drawString(3,8+FIRST_BOX_OFFSET+BOX_OFFSET,0, "C2");
  ucg.drawRFrame(0,FIRST_BOX_OFFSET+BOX_OFFSET,128,25,8);  // Draw outline with rounded edges 

  ucg.setColor(0, 255, 0); // Green color
  ucg.drawString(3,8+FIRST_BOX_OFFSET+BOX_OFFSET+BOX_OFFSET,0, "C3");
  ucg.drawRFrame(0,FIRST_BOX_OFFSET+BOX_OFFSET+BOX_OFFSET,128,25,8);  // Draw outline with rounded edges 

  //ucg.setColor(0, 0, 255); // Blue color
  ucg.setColor(0, 255, 255); // Teal color
  ucg.drawString(3,8+FIRST_BOX_OFFSET+BOX_OFFSET+BOX_OFFSET+BOX_OFFSET,0, "C4");
  ucg.drawRFrame(0,FIRST_BOX_OFFSET+BOX_OFFSET+BOX_OFFSET+BOX_OFFSET,128,25,8);  // Draw outline with rounded edges 

  ucg.setColor(0, 0, 255); // Blue color
  ucg.drawString(3,8+FIRST_BOX_OFFSET+BOX_OFFSET+BOX_OFFSET+BOX_OFFSET+BOX_OFFSET,0, "C5");
  ucg.drawRFrame(0,FIRST_BOX_OFFSET+BOX_OFFSET+BOX_OFFSET+BOX_OFFSET+BOX_OFFSET,128,25,8);  // Draw outline with rounded edges 

  
  ucg.setColor(255, 255, 255);
  delay(2000);
  
}

void loop() {

// A simple timer to calculation rate to once every 1 seconds
  if ((millis()-lastpost)>=6000)
  {
    
    Serial.print("LOOP T  ");
    Serial.println(((float)millis()-lastpost)/1000); // must cast millis() as float or result will not contain milliseconds
    
    Serial.println(' '); // blank lines
    Serial.println(' '); // blank lines    
    //float loopTime = millis(); // alternative way of measuring loop time 
    //loopTime = (loopTime-lastpost)/1000;
    //Serial.println(loopTime);
    
    lastpost = millis();
    
    // .calcVI: Calculate all. No.of half wavelengths (crossings), time-out 
    
    ct1.calcVI(48,4000); 
    ct2.calcVI(48,4000); 
    ct3.calcVI(48,4000);  
    ct4.calcVI(48,4000); 
    ct5.calcVI(48,4000); 


// using 48 half wavelenghts each calc takes 1/5th of 2 seconds
// Or 2 seconds for all 5 calcs
// later on we average together 30 datapoints in our running average
// this means we should get 1 minute of averaged data :)
// our master host sends the data once per minute so this should give accurate power readings over the time period

    v1Avg.addValue(ct1.Vrms);       // add current data to running average
    ct1Avg.addValue(ct1.realPower); 
    ct2Avg.addValue(ct2.realPower);
    ct3Avg.addValue(ct3.realPower);
    ct4Avg.addValue(ct4.realPower);
    ct5Avg.addValue(ct5.realPower);


//Top of screen
    ucg.setPrintPos(45,4);
    ucg.print(ct1.Vrms,1);
    ucg.print("V  ");

    
    //dispCT(dispCounter);
    //dispCounter ++;
    //if (dispCounter > 5) dispCounter = 1;

    for (int i=1; i < 6; i++){
      dispCT(i);
    } 
//-------------------------------------------------------------

//----------------- Debugging Serial Prints -------------------
/*
    Serial.print("V1      ");
    Serial.println(ct1.Vrms);  
    Serial.print("V1 avg  ");
    Serial.println(v1Avg.getAverage());  
    Serial.println(' '); // blank lines

    Serial.print("CT1 A   "); 
    Serial.println(ct1.Irms); 
    Serial.print("CT1 VA  "); 
    Serial.println(ct1.apparentPower); 
    Serial.print("CT1 W   ");
    Serial.print(ct1.realPower);  
    Serial.print(" avg ");
    Serial.println(ct1Avg.getAverage());
    Serial.println(' '); // blank lines

    Serial.print("CT2 A   "); 
    Serial.println(ct2.Irms); 
    Serial.print("CT2 VA  "); 
    Serial.println(ct2.apparentPower); 
    Serial.print("CT2 W   ");
    Serial.print(ct2.realPower);  
    Serial.print(" avg ");
    Serial.println(ct2Avg.getAverage());
    Serial.println(' '); // blank lines

    Serial.print("CT3 A   "); 
    Serial.println(ct3.Irms); 
    Serial.print("CT3 VA  "); 
    Serial.println(ct3.apparentPower); 
    Serial.print("CT3 W   ");
    Serial.print(ct3.realPower);  
    Serial.print(" avg ");
    Serial.println(ct3Avg.getAverage());
    Serial.println(' '); // blank lines

    Serial.print("CT4 A   "); 
    Serial.println(ct4.Irms); 
    Serial.print("CT4 VA  "); 
    Serial.println(ct4.apparentPower); 
    Serial.print("CT4 W   ");
    Serial.print(ct4.realPower);  
    Serial.print(" avg ");
    Serial.println(ct4Avg.getAverage());
    Serial.println(' '); // blank lines

    Serial.print("CT5 A   "); 
    Serial.println(ct5.Irms); 
    Serial.print("CT5 VA  "); 
    Serial.println(ct5.apparentPower); 
    Serial.print("CT5 W   ");
    Serial.print(ct5.realPower);  
    Serial.print(" avg ");
    Serial.println(ct5Avg.getAverage());

        
    Serial.println(' '); // blank lines
    Serial.println(' '); 
*/

    
  }
}

void receiveEvent(int count)
{
  if (count == I2C_MSG_IN_SIZE)
  {
    byte cmd = Wire.read();
    byte port = Wire.read();
    int value = Wire.read();
    value += Wire.read()*256;
    switch(cmd)
      {
        case CMD_DIGITAL_WRITE:
          pinMode(port,OUTPUT);
          digitalWrite(port,value);
          break;
        case CMD_DIGITAL_READ:
          pinMode(port,INPUT_PULLUP);
          clearSendBuffer();
          sendBuffer[0] = digitalRead(port);
          break;
        case CMD_ANALOG_WRITE:
          analogWrite(port,value);
          break;
        case CMD_ANALOG_READ:
          clearSendBuffer();
          
          //int valueRead = map((ct6.Irms*10),0,1000,0,1000); // send in whole numbers with 10ths first digit
          // use %value%/10 to get back the decimal with 10ths in the "Formula" window on ESP Easy
          //int valueRead = map((ct6.Irms*10),0,1000,0,1000); // display amps
          //int valueRead = map((ct6.realPower),0,1000,0,1000); // display watts - no need for decimal

          switch(port)
            {
              case 0: // send AC voltage
                valueRead = (v1Avg.getAverage()*10)+32767;
                break;
              case 1: // send AC current sensor 1
                valueRead = ct1Avg.getAverage()+32767;
                break;
              case 2: // send AC current sensor 2
                valueRead = ct2Avg.getAverage()+32767;
                break;
              case 3: // send AC current sensor 3
                valueRead = ct3Avg.getAverage()+32767;
                break;
              case 4: // send AC current sensor 4
                valueRead = ct4Avg.getAverage()+32767;
                break;
              case 5: // send AC current sensor 5
                valueRead = ct5Avg.getAverage()+32767;
                break;
              case 11: // send AC watt sensor 1
                valueRead = ct1.realPower+32767;
                break;
              case 12: // send AC watt sensor 2
                valueRead = ct2.realPower+32767;
                break;
              case 13: // send AC watt sensor 3
                valueRead = ct3.realPower+32767;
                break;
              case 14: // send AC watt sensor 4
                valueRead = ct4.realPower+32767;
                break;
              case 15: // send AC watt sensor 5
                valueRead = ct5.realPower+32767;
                break;
            }

          if ((valueRead >= 32766)&&(valueRead <= 32768)) valueRead = 32767; // if watts = 1 or -1 reduce to 0 for clean numbers
          
          //if (valueRead < 0) valueRead = 0; // value will sometimes go to negative 1.5W if no load present which causes rollover - need way to handle negative values for instances where power flow reverses.  
          //Serial.println(valueRead); // for testing send to local serial port
          
          sendBuffer[0] = valueRead & 0xff; // data that is sent to I2C port
          sendBuffer[1] = valueRead >> 8; 

          // Add D13 LED blink on data send
          
          // use %value%-32767 on ESPEasy to decode to watts with no decimals


          
          break;
      }
  }
}

void clearSendBuffer()
{
  for(byte x=0; x < sizeof(sendBuffer); x++)
    sendBuffer[x]=0;
}

void requestEvent()
{
  Wire.write((const uint8_t*)sendBuffer,sizeof(sendBuffer));
}
