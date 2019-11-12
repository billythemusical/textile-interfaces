#include <SPI.h>
//#include <OSCMessage.h>
#include <OSCBundle.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

// These constants won't change. They're used to give names to the pins used:
const int analogIn0 = A0;  // Analog input pin that the potentiometer is attached to
const int analogIn1 = A1;  // Analog input pin that the potentiometer is attached to
const int analogIn2 = A2;  // Analog input pin that the potentiometer is attached to
const int analogIn3 = A3;  // Analog input pin that the potentiometer is attached to

int buttonVal0 = 0;        // value read from the buttons
int buttonVal1 = 0;        // value read from the buttons
int buttonVal2 = 0;        // value read from the buttons
int buttonVal3 = 0;        // value read from the buttons

// constants won't change. They're used here to set pin numbers:
int status = WL_IDLE_STATUS;
#include "arduino_secrets.h"
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

//destination IP
IPAddress outIp(172, 20, 10, 2);

WiFiUDP Udp;

void setup() {
  // initialize the pushbutton pin as an input:
  Serial.begin(9600);  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  connectToWifi();

  Udp.begin(8888);
}



void loop() {
  
  sendSensorsOverOSC();
}

void sendSensorsOverOSC() {
   
   // read the analog in value:
  buttonVal0 = analogRead(analogIn0);
//  delay(1);
  buttonVal1 = analogRead(analogIn1);
//  delay(1);
  buttonVal2 = analogRead(analogIn2);
//  delay(1);
  buttonVal3 = analogRead(analogIn3);
//  delay(2);

  // print the results to the Serial Monitor:
//  Serial.print("button0 = ");
//  Serial.print(buttonVal0);  
//  Serial.print("\t button1 = ");
//  Serial.print(buttonVal1);  
//  Serial.print("\t button2 = ");
//  Serial.print(buttonVal2);  
//  Serial.print("\t button3 = ");
//  Serial.println(buttonVal3);  
   
   // make a new OSC message
   OSCBundle buttons;
   
   buttons.add("/button0").add((int32_t)buttonVal0);
   buttons.add("/button1").add((int32_t)buttonVal1);
   buttons.add("/button2").add((int32_t)buttonVal2);
   buttons.add("/button3").add((int32_t)buttonVal3);
   
   //send the bundle together  
   Udp.beginPacket(outIp, 5005);
      buttons.send(Udp);
   Udp.endPacket();
   buttons.empty();
   
   delay(2);
}


void connectToWifi() {
  Serial.println("Connecting to wifi...");
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println("Connected to wifi");
  printWiFiStatus();
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
