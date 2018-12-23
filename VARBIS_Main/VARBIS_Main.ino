// VARBIS main program sketch for the Arduino MKR1000 and MPU-6050
// Adapted from the MPU6050_DMP6 and RUBS_8plus_Summer sketches.
// using the I2C device class (12Cdev), MPU6050 class, and DMP (MotionApps v2.0)
// 2018-11-01


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"



#include <SPI.h>
#include <WiFi101_OSC.h>
#include <WiFiUdp.h>
#include <Ethernet.h>
#include "OSCMessage.h"

//************************************************************
//** CHECK & CHANGE THESE!!!!!

char ssid[] = "**********";                   // Computer with MAX/MSP must be in the same Network
char pass[] = "**********";
int pc_port = 8003;                             // Port opened on Computer ---- Ex. on MAX -> "udpreceive 8003" 8003 for Ziyian, 8004 for Emma
IPAddress ip(192, 168, 0, 121);                 // IP Address of MKR1000   ---- Ex. on MAX -> "udpsend 192.168.0.121 3001"  121 for Ziyian, 122 for Emma        

//************************************************************
//** DO NOT Change these!!!!!
unsigned int localPort = 3001;                  // Port opened on MKR1000  ---- Ex. on MAX -> "udpsend 192.168.0.121 3001"

int status = WL_IDLE_STATUS;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
char ReplyBuffer[] = "acknowledge";


//int pin_Out_BattSwitch = 0;             // Pin to control transistor
//int pin_In_Battery = A5;

int pin_LED = 6;

int pinReading = 0;
int Vin= 3.3;
const int baud_rate = 9600;

OSCMessage msg;                         // Create new osc message
OSCMessage resp;
WiFiUDP Udp;



//************************************************************
void setup() 
{
  pinMode(pin_LED,OUTPUT);
  //pinMode(pin_Out_BattSwitch, OUTPUT);
  
  Serial.begin(baud_rate);

  if (WiFi.status() == WL_NO_SHIELD)                // Check if the WiFi shield is available
  {
    Serial.println("WIFI SHIELD NOT AVAILABLE");
    return; 
  }
  connectToWifi();
}


//************************************************************
void loop() {
  if ( WiFi.status() != WL_CONNECTED){
    Serial.println("Connection to SSID lost");
    Udp.stop();
    connectToWifi(); //just in case it didn't connect
  }
  
  int packetSize = Udp.parsePacket();
  
  if (packetSize)                                         // Triggered when receiving a UDP packet from the computer
  {
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

  if (Udp.remoteIP())                          // Start sending the sensor values when we know the IP Address of the computer.
  {
    msg.beginMessage("sensors");
      
    //pinReading = 
    //Serial.print(pinReading);
    Serial.print(", ");
    
   
    Serial.println("");
    
    sendUDP();
    delay(50);
  }
}

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
    digitalWrite(pin_LED, HIGH);  
    delay(500);             
    digitalWrite(pin_LED, LOW);   
    delay(500);              
  }
}

//************************************************************
void connectToWifi() {
  digitalWrite(pin_LED, LOW);
    
  while ( WiFi.status() != WL_CONNECTED)      // Attempt to connect to WiFi network 
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    
    WiFi.begin(ssid, pass);                   // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    WiFi.config(ip);                          
    blinkLED();
  }
  
  digitalWrite(pin_LED, HIGH);   
  
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



//************************************************************

