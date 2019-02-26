// VARBIS main program sketch for the Arduino MKR1000 and MPU-6050 written by Anders Grasdal
// Adapted from the MPU6050_DMP6 demo sketch and RUBS_8plus_Summer sketches.
// using the I2C device class (12Cdev), MPU6050 class, and DMP (MotionApps v2.0)
// 2018-11-01

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Modifications 2018 Anders Grasdal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
=============================================== */


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <SPI.h>
#include <WiFi101_OSC.h>
#include <WiFiUdp.h>
#include <Ethernet.h>
#include "OSCMessage.h"



// ================================================================
// ===                       DEFINITIONS                        ===
// ================================================================

#define INTERRUPT_PIN 0  // use pin 2 on Arduino Uno & most boards //use pin 0 on MKR family boards

#define LED_PIN 6 // (MKR1000 is 6)
bool blinkState = false;

//int pin_Out_BattSwitch = 0;             // Pin to control transistor
//int pin_In_Battery = A5;

int pinReading = 0;
int Vin= 3.3;
const int baud_rate = 9600;


// redefined macro, (not included in all arduino builds.
#ifndef _BV
  #define _BV(bit)  (1 << (bit))
#endif


//WIFI INIT and DEFINITIONS
//************************************************************

//** CHECK & CHANGE THESE!!!!!
char ssid[] = "**********";                   // Computer with MAX/MSP must be in the same Network
char pass[] = "**********";
int pc_port = 8003;                             // Port opened on Computer ---- Ex. on MAX -> "udpreceive 8003" 8003 for Ziyian, 8004 for Emma
IPAddress ip(192, 168, 0, 121);                 // IP Address of MKR1000   ---- Ex. on MAX -> "udpsend 192.168.0.121 3001"  121 for Ziyian, 122 for Emma        

//** DO NOT Change these!!!!!
unsigned int localPort = 3001;                  // Port opened on MKR1000  ---- Ex. on MAX -> "udpsend 192.168.0.121 3001"

int status = WL_IDLE_STATUS;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
char ReplyBuffer[] = "acknowledge";

OSCMessage msg;                         // Create new osc message
OSCMessage resp;
WiFiUDP Udp;

//MPU6050 INIT and DEFINITIONS
//************************************************************

MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino MKR1000 this is digital I/O pin 0.
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

//****** NOT SUPPORTED OVER WIFI *******
// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

//possibly unneeded.
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

//Function pointer to the 0th address, we invoke this to cause a programatic reset.
void(* resetFunc) (void) = 0;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(baud_rate);
    
    //VARBIS: We remove this line, so that it will start up on battery
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    //VARBIS: wait for ready removed to enable wireless startup.
    /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    //TODO Calibrate.
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
  pinMode(LED_PIN,OUTPUT);
  //pinMode(pin_Out_BattSwitch, OUTPUT);
  

  if (WiFi.status() == WL_NO_SHIELD)                // Check if the WiFi shield is available
  {
    Serial.println("WIFI SHIELD NOT AVAILABLE");
    return; 
  }
  connectToWifi();
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming MPU failed, don't try to do anything
    if (!dmpReady) return;
    
    if ( WiFi.status() != WL_CONNECTED){
      Serial.println("Connection to SSID lost");
      Udp.stop();
      connectToWifi(); //just in case it didn't connect
    }

    // wait for MPU interrupt or extra packet(s) available
    // Handle UDP interrups in this phase.
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  

        //Parse incomming UDP packets once caught up on FIFO reads.
        int packetSize = Udp.parsePacket(); // Triggered when receiving a UDP packet from the computer
        if (packetSize){
            PacketHandler();  
        }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();


    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        //OUTPUT 
        if (Udp.remoteIP())                          // Start sending the sensor values when we know the IP Address of the computer.
        {
          readGyroData(); //prepare and append gyro data in the required format to the UDP message
             
          sendUDP();
          //TODO, fine-tune delay (if nessesary) to prevent network congestion while guaranteeing timely reads from the buffer.
          //delay(50); causes issues reading from the FIFO buffer (6 reads before overflow)
        }     
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}


// ================================================================
// ===                    WIRELESS HELPERS                      ===
// ================================================================

void PacketHandler() {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    
    IPAddress remoteIp = Udp.remoteIP();                  // Record IP of MAX computer
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);       // Records the message contained in the packet received
    String contents(packetBuffer);
    int len = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    if (len > 0) packetBuffer[len] = 0;
    Serial.println("Contents:");
    Serial.println(contents);

    if(contents == "battery"){
      //checkBattery();
    }
    if(contents == "signal") {
      checkSignalStrength();
    }
    if(contents == "connect") {
      sendConnectedMSG();
    } 
    if(contents == "reset"){
      resetFunc(); //Function call to the 0th address, causes a system reset.
    }
}

//************************************************************
void sendUDP(){
    Udp.beginPacket(Udp.remoteIP(), pc_port);
    Udp.oscWrite(&msg);
    Udp.endPacket();
    msg.flush();
}


//************************************************************
void printWifiStatus() {
  
  Serial.print("SSID: ");                     // Print the SSID of the connected network
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();              // Print MKR1000 IP Address
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();                    // Print the received signal strength
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void blinkLED() {
  for(int i = 0; i < 2; i++){
    digitalWrite(LED_PIN, HIGH);  
    delay(500);             
    digitalWrite(LED_PIN, LOW);   
    delay(500);              
  }
}

//************************************************************
void connectToWifi() {
  digitalWrite(LED_PIN, LOW);
    
  while ( WiFi.status() != WL_CONNECTED)      // Attempt to connect to WiFi network 
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    
    WiFi.begin(ssid, pass);                   // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    //WiFi.config(ip); //TODO debug this! Static IP address causes issues handshaking?                       
    blinkLED();
  }
  
  digitalWrite(LED_PIN, HIGH);   
  
  Serial.println("Connected to wifi");
  printWifiStatus();
  Serial.println("\nStarting connection to server...");
  Serial.println((IPAddress) WiFi.localIP()); //cast to desplay in dotted decimal.
  
  Udp.begin(localPort);
}

//************************************************************
/*void checkBattery(){
  digitalWrite(pin_Out_BattSwitch, HIGH);
  
  float batteryVoltage = (analogRead(pin_In_Battery)*(3.3*2/1023.0)) + 1.12;
  float batteryLevel = 100 *(batteryVoltage)/4;

  Serial.print("Battery voltage: ");
  Serial.println(batteryVoltage);
  Serial.print("Battery level: ");
  Serial.println(batteryLevel);
    
  OSCMessage batLevelMSG;
  batLevelMSG.beginMessage("battery");
 //batLevelMSG.addArgFloat(batteryLevel); //changed to batteryVoltage
   batLevelMSG.addArgFloat(batteryVoltage);
  Udp.beginPacket(Udp.remoteIP(), pc_port);
  Udp.oscWrite(&batLevelMSG);
  Udp.endPacket();
  batLevelMSG.flush();
  
  digitalWrite(pin_Out_BattSwitch, LOW);
}*/

//************************************************************
void checkSignalStrength() {
  Serial.print("Signal Strength: ");
  Serial.println(WiFi.RSSI());

  OSCMessage signalMSG;
  signalMSG.beginMessage("signal");
  signalMSG.addArgInt32(WiFi.RSSI());

  Udp.beginPacket(Udp.remoteIP(), pc_port);
  Udp.oscWrite(&signalMSG);
  Udp.endPacket();
  signalMSG.flush();
  
}


//************************************************************
void sendConnectedMSG() {
  OSCMessage connectedMSG;
  connectedMSG.beginMessage("wmmsgreceived");
  connectedMSG.addArgInt32(1);

  Udp.beginPacket(Udp.remoteIP(), pc_port);
  Udp.oscWrite(&connectedMSG);
  Udp.endPacket();
  connectedMSG.flush();
  
}

//************************************************************
void readGyroData(){
  Serial.print("Sending over WIFI:\t");
  //TODO, refactor MPU reads into helper?
  #ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      
      msg.beginMessage("quat");
      Serial.print("quat\t");
      msg.addArgFloat(q.w);
      Serial.print(q.w);
      Serial.print("\t");
      msg.addArgFloat(q.x);
      Serial.print(q.x);
      Serial.print("\t");
      msg.addArgFloat(q.y);
      Serial.print(q.y);
      Serial.print("\t");
      msg.addArgFloat(q.z);
      Serial.println(q.z);
  #endif

  #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      msg.beginMessage("ypr");
      Serial.print("ypr\t");
      msg.addArgFloat(ypr[0] * 180/M_PI);
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      msg.addArgFloat(ypr[1] * 180/M_PI);
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      msg.addArgFloat(ypr[2] * 180/M_PI);
      Serial.println(ypr[2] * 180/M_PI);
  #endif

  #ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      Serial.print("euler\t");
      Serial.print(euler[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(euler[2] * 180/M_PI);

      msg.beginMessage("euler");
      msg.addArgFloat(euler[0] * 180/M_PI);
      msg.addArgFloat(euler[1] * 180/M_PI);
      msg.addArgFloat(euler[2] * 180/M_PI);
  #endif

  #ifdef OUTPUT_READABLE_REALACCEL
      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);

      msg.beginMessage("areal");
      msg.addArgFloat(aaReal.x);
      msg.addArgFloat(aaReal.y);
      msg.addArgFloat(aaReal.z);
  #endif

  #ifdef OUTPUT_READABLE_WORLDACCEL
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      Serial.print("aworld\t");
      Serial.print(aaWorld.x);
      Serial.print("\t");
      Serial.print(aaWorld.y);
      Serial.print("\t");
      Serial.println(aaWorld.z);

      msg.beginMessage("aworld");
      msg.addArgFloat(aaWorld.x);
      msg.addArgFloat(aaWorld.y);
      msg.addArgFloat(aaWorld.z);
  #endif

  #ifdef OUTPUT_TEAPOT
      // display quaternion values in InvenSense Teapot demo format:
      teapotPacket[2] = fifoBuffer[0];
      teapotPacket[3] = fifoBuffer[1];
      teapotPacket[4] = fifoBuffer[4];
      teapotPacket[5] = fifoBuffer[5];
      teapotPacket[6] = fifoBuffer[8];
      teapotPacket[7] = fifoBuffer[9];
      teapotPacket[8] = fifoBuffer[12];
      teapotPacket[9] = fifoBuffer[13];
      Serial.write(teapotPacket, 14);
      teapotPacket[11]++; // packetCount, loops at 0xFF on purpose

      //NOT supported over WIFI
  #endif
  
}


