#include "SparkFunLSM6DS3.h"
#include <SPI.h>
#include <Wire.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>


/*
 * Libraries required:
 * https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library
 * https://www.arduino.cc/en/Reference/WiFi101
 * https://github.com/CNMAT/OSC
 */


void SLIPSerialWrite(int value);
void sendSpongeOSC();
void initAccelGyro();

//SLIP for wired serial backup.
const byte END=192;
const byte ESC=219;
const byte ESC_END=220;
const byte ESC_ESC=221;

const char* ssid     = "tstick_network";
const char* password = "mappings";

//const char* ssid     = "spot";
//const char* password = "superspot";


WiFiUDP udp; // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(192,168,1,102); // remote IP of your computer
// const IPAddress outIp(192,168,137,1); // remote IP for multicast
// const IPAddress outIp(224,0,0,1); // remote IP for multicast
// const IPAddress outIp(10,42,0,1); // remote IP of your computer
const unsigned int outPort = 8000; // remote port to receive OSC
const unsigned int localPort = 8001; // local port to listen for OSC packets


// const int butPins[10] = { 0,1,20,21,5,6,9,10,11,12 };
// GPIO 9 can be used as A7 to monitor battery voltage.
// Use GPIO 16 (A2) instead.
const int butPins[10] = { 0,1,20,21,5,6,16,10,11,12 };
// const int butPins[10] = { 0,1,20,13,5,6,16,10,11,12 }; // For sponge Ana, use pin 13 instead of 21!!!!!!!!!
const int ledPin = 17; // Controler pin for LED strip.

OSCErrorCode error;

LSM6DS3 SensorOne( SPI_MODE, 18 );  // pin A4 on feather
LSM6DS3 SensorTwo( SPI_MODE, 19 );  // pin A5 on feather

void setup(){
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);
  Serial.begin(115200);
  analogReadResolution(12);
  // Make butPins INPUT pins with pull up resistor.
  for (int i=0; i < 10; i++){
    pinMode(butPins[i], INPUT_PULLUP);
  }

  // pinMode(12, INPUT);
  /* Wire.begin(); */

  delay(100);
  // We start by connecting to a WiFi network
 

   Serial.println();
   Serial.print("Connecting to ");
   Serial.println(ssid);

  // WiFi.begin(ssid);
  WiFi.begin(ssid,password);
  // WiFi.beginProvision();
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.write(END);
  }
 
   Serial.println("");
   Serial.println("WiFi connected");  
   Serial.print("IP address: ");
   Serial.println(WiFi.localIP());

   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());

  // Serial.println("Starting UDP");
  // udp.begin(localPort);
  udp.beginMulti(outIp, localPort);
  // Serial.print("Local port: ");
  // Serial.println(localPort);


  delay(1000);
  initAccelGyro();

  // SensorOne.begin();
  // SensorTwo.begin();
  if( SensorOne.begin() != 0 ) {
    Serial.write(END);
    Serial.println("Problem starting the sensor with CS @ Pin 18 (A4).");
    Serial.write(END);
  } else {
    Serial.write(END);
    Serial.println("Sensor with CS @ Pin 18 (A4) started.");
    Serial.write(END);
  }
  if( SensorTwo.begin() != 0 ) {
    Serial.write(END);
    Serial.println("Problem starting the sensor with CS @ Pin 19 (A5).");
    Serial.write(END);
  } else {
    Serial.write(END);
    Serial.println("Sensor with CS @ Pin 19 (A5) started.");
    Serial.write(END);
  }
}

void loop(){
  OSCMessage message;
  int size = udp.parsePacket();

  if (size) {
    while (size--) {
      message.fill(udp.read());
    }
    if (!message.hasError()) {
      /* message.dispatch("/set_register", osc_set_register); */
    } else {
      error = message.getError();
      Serial.print("error: ");
      Serial.println(error);
    }
  }

  // Serial.println( (SensorOne.readRawAccelX() >> 6) + 512);
  // Serial.print(SensorOne.readRawAccelY());
  // Serial.print(SensorOne.readRawAccelZ());
  // Serial.print(SensorOne.readRawGyroX());
  // Serial.print(SensorOne.readRawGyroY());
  // Serial.print(SensorOne.readRawGyroZ());

  sendSpongeOSC();
  
  // int anal0 = analogRead(1);
  // OSCBundle bndl;

  // wait a bit.
  // delay(5);
  // Serial.println(WiFi.SSID());
  delay(20); // 15 seems to be too short.
  
  // delay(1);
  // delayMicroseconds(10);

  // if (WiFi.status() != WL_CONNECTED) {
  //   // WiFi.end();
  //   while (WiFi.begin(ssid) != WL_CONNECTED) {
  //     delay(500);
  //     Serial.print(".");
  //   }
  //   Serial.print("Reconnected to ");
  //   Serial.print(ssid);
  // }
}

void initAccelGyro(){
  //Over-ride default settings if desired
  SensorOne.settings.gyroEnabled = 1;  //Can be 0 or 1
  SensorOne.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  SensorOne.settings.gyroSampleRate = 1666;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  SensorOne.settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
  SensorOne.settings.accelEnabled = 1;
  SensorOne.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
  SensorOne.settings.accelSampleRate = 13330;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  SensorOne.settings.accelBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
  SensorOne.settings.fifoModeWord = 0;  //FIFO mode.

    //Over-ride default settings if desired
  SensorTwo.settings.gyroEnabled = 1;  //Can be 0 or 1
  SensorTwo.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  SensorTwo.settings.gyroSampleRate = 1666;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  SensorTwo.settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
  SensorTwo.settings.accelEnabled = 1;
  SensorTwo.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
  SensorTwo.settings.accelSampleRate = 13330;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  SensorTwo.settings.accelBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
  SensorTwo.settings.fifoModeWord = 0;  //FIFO mode.

  //FIFO mode.  Can be:
  //  0 (Bypass mode, FIFO off)
  //  1 (Stop when full)
  //  3 (Continuous during trigger)
  //  4 (Bypass until trigger)
  //  6 (Continous mode)
}


void sendSpongeOSC() {
  int compButVal=0; // composed into a single 32 bit integer (only 10 are used).
  OSCMessage acc1x("/sponge/acc1x");
  OSCMessage acc1y("/sponge/acc1y");
  OSCMessage acc1z("/sponge/acc1z");
  OSCMessage acc2x("/sponge/acc2x");
  OSCMessage acc2y("/sponge/acc2y");
  OSCMessage acc2z("/sponge/acc2z");
  OSCMessage fsr1("/sponge/fsr1");
  OSCMessage fsr2("/sponge/fsr2");
  OSCMessage buttons("/sponge/buttons");
  acc1x.add(int(SensorOne.readRawAccelX()));
  acc1y.add(int(SensorOne.readRawAccelY()));
  acc1z.add(int(SensorOne.readRawAccelZ()));
  acc2x.add(int(SensorTwo.readRawAccelX()));
  acc2y.add(int(SensorTwo.readRawAccelY())); // int 4
  acc2z.add(int(SensorTwo.readRawAccelZ())); // int 5
  fsr1.add(int(analogRead(0)));
  fsr2.add(int(analogRead(1)));
  for (int i=0; i < 10; i ++){
    compButVal |= ( digitalRead(butPins[i]) << (9-i));
    /* La ligne prÃ©cÃ©dente place les valeurs binaires de chacun des boutons
       dans un seul entier Ã  16 bit.  Si on reprÃ©sente l'entier en binaire:
       (b01001011001 = d601),  on a une reprÃ©sentation des boutons activÃ©s.
       Dans ce cas, les boutons 0,3,4,6 et 9 sont enfoncÃ©s.
    */
  };
  buttons.add(compButVal ^ 1023); // 4 bytes  (^ is a bitwise XOR)
  sendOSC(acc1x);
  sendOSC(acc1y);
  sendOSC(acc1z);
  sendOSC(acc2x);
  sendOSC(acc2y);
  sendOSC(acc2z);
  sendOSC(fsr1);
  sendOSC(fsr2);
  sendOSC(buttons);
}

void sendOSC(OSCMessage msg) {
  udp.beginPacket(outIp, outPort);
  msg.send(udp);
  udp.endPacket();
  msg.empty();
}


void SLIPSerialWrite(int value){
  if(value == END) {
    Serial.write(ESC);
    Serial.write(ESC_END);
    return;
  } else if (value == ESC) {
    Serial.write(ESC);
    Serial.write(ESC_ESC);
    return;
  } else {
    Serial.write(value);
    return;
  }
}
