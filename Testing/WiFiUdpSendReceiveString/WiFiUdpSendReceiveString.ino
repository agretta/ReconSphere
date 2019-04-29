/*************************************************************************
 * TinyDuino WiFi Tutorial:
 * Just a basic tutorial showing you how to connect to WiFi with the Wifi
 * TinyShield
 * 
 * Hardware by: TinyCircuits
 * Written by: Laverena Wienclaw
 * 
 * NOTE: There are a couple things you'll need to change for this to work!
 *
 * Initiated: Mon. 5/29/2018 
 * Updated: Tue. 06/15/2018
 ************************************************************************/

// This library is for the wifi connection
#include <WiFi101.h>
#include <WiFiUdp.h>

char ssid[] = "TinyZeroTest";  //  your network SSID (name)
char wifiPassword[] = "gt123456";  // your network password

unsigned int localport = 1000;

char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

WiFiUDP UDP;

void setup() {
  SerialUSB.begin(9600);
  WiFi.setPins(8, 2, A3, -1); // VERY IMPORTANT FOR TINYDUINO
  while(!SerialUSB);

  // Attempt to connect to Wifi network:
  SerialUSB.print("Connecting Wifi: ");
  SerialUSB.println(ssid);

  // Connect to WiFi, and loop until connection is secured
  WiFi.begin(ssid, wifiPassword);
  while (WiFi.status() != WL_CONNECTED)
    SerialUSB.print("Can't Connect");
    delay(500);

  // Print out the local IP address
  SerialUSB.println(" ");
  SerialUSB.println("WiFi connected");
  SerialUSB.println("IP address: ");
  IPAddress ip = WiFi.localIP();
  SerialUSB.println(ip);

  //Try and connect to Python Server to transmit data
  SerialUSB.println("\nStarting connection to server...");
  UDP.begin(localport);
  SerialUSB.println(" ");
}

void loop()
{
  int packetSize = UDP.parsePacket();
  
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = UDP.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(UDP.remotePort());

    // read the packet into packetBufffer
    int len = UDP.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    // send a reply, to the IP address and port that sent us the packet we received
    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    UDP.write(ReplyBuffer);
    UDP.endPacket();
  }
  //SerialUSB.println("Main loop entered. Now that we're connected, let's do something cool.");
  //delay(60000); // Wait a minute before going back through main loop
}
