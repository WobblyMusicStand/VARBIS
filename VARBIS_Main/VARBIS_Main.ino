// VARBIS main program sketch for the Arduino MKR1000 and MPU-6050
// Adapted from the MPU6050_DMP6 demo sketch and RUBS_8plus_Summer sketches.
// using the I2C device class (12Cdev), MPU6050 class, and DMP (MotionApps v2.0)
// 2018-11-01


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <SPI.h>
#include <WiFi101_OSC.h>
#include <WiFiUdp.h>
#include <Ethernet.h>
#include "OSCMessage.h"



// ******************* DEFINITIONS **************************

#define INTERRUPT_PIN 0  // use pin 2 on Arduino Uno & most boards //use pin 0 on MKR family boards

#define LED_PIN 6 // (MKR1000 is 6)
bool blinkState = false;

//int pin_Out_BattSwitch = 0;             // Pin to control transistor
//int pin_In_Battery = A5;

int pinReading = 0;
int Vin= 3.3;
const int baud_rate = 9600;



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

#define OUTPUT_READABLE_QUATERNION

// redefined macro, (not included in all arduino builds.
#ifndef _BV
  #define _BV(bit)  (1 << (bit))
#endif

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

/* //possibly unneeded.
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
*/

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// SETUP
//************************************************************
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
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

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

    // wait for ready removed to speedup startup.
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


//************************************************************
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
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif


        if (Udp.remoteIP())                          // Start sending the sensor values when we know the IP Address of the computer.
        {    
          msg.beginMessage("gyro");
          msg.addArgInt32(q.w);
          msg.addArgInt32(q.x);    
          msg.addArgInt32(q.y);    
          msg.addArgInt32(q.z);
               
          sendUDP();
          delay(50);
        }
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}


//************************************************************
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
    WiFi.config(ip);                          
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
void read_gyro(){
  
  }


