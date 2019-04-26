// ReconSphere TinyZero Code
#include <WiFi101.h>      // Wifi Shield library
#include <WiFiUdp.h>
#include <Wire.h>         // For I2C communication with sensor
#include "BMA250.h"       // For interfacing with the accel. sensor
#include <SharpDistSensor.h>

// Accelerometer sensor variables for the sensor and its values
BMA250 accel_sensor;
int x, y, z;
int prev_x, prev_y, prev_z;
double x_rot, y_rot;
//256 is roughly gravity
double gravity_mag = 0.0;

unsigned int localPort = 8888;

char ssid[] = "TinyZeroTest";  //  your network SSID (name)
char pass[] = "gt123456";  // your network password
int status = WL_IDLE_STATUS;     // the WiFi radio's status
//IPAddress server(129,6,15,28);
IPAddress server(143,215,115,187);
int server_port = 8888;

WiFiClient client;
WiFiUDP Udp;

int pin1 = A2;
SharpDistSensor sensor(pin1, 5);

void setup() {
  SerialUSB.begin(9600);
  Wire.begin();

  SerialUSB.print("Initializing BMA...");
  // Set up the BMA250 acccelerometer sensor
  accel_sensor.begin(BMA250_range_2g, BMA250_update_time_64ms); 

  connectToWiFi();
  Udp.begin(localPort);
 
  
  gravity_mag = callibrateToGravity();  
  pinMode(pin1, INPUT);
}

void loop() {
  // listNetworks();
  String s = String(String(x) + "|" + String(y) + "|" + String(z));
  char c[] = "Hello World";
  //SerialUSB.write(s);
  //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.beginPacket(server, 8888);
  Udp.write(c);
  Udp.endPacket();
  
  delay(10000); // Wait a minute before going back through main loop

  accel_sensor.read();
  x = accel_sensor.X;
  y = accel_sensor.Y;
  z = accel_sensor.Z;

  // error checking
  if (x == -1 && y == -1 && z == -1) {
    SerialUSB.print("ERROR! NO BMA250 DETECTED!");
  } else {
    double x_Buff = float(x);
    double y_Buff = float(y);
    double z_Buff = float(z);
    y_rot = atan2(y_Buff , z_Buff) * 57.3;
    x_rot = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;
    SerialUSB.print("Roll:");
    SerialUSB.println(y_rot);
    SerialUSB.print("Pitch:");
    SerialUSB.println(x_rot);
    //SerialUSB.print("Grav Mag:");
    //SerialUSB.println(gravity_mag);
    showSerial();
  }
  //SerialUSB.println("SENSOR READING:");
  //unsigned int distance = sensor.getDist();
  //SerialUSB.println(distance);

  // The BMA250 can only poll new sensor values every 64ms
  delay(250);
}

double callibrateToGravity() {
  delay(500);
  double gravity_mag = 0.0;
  
  accel_sensor.read();
  int x_t = accel_sensor.X;
  int y_t = accel_sensor.Y;
  int z_t = accel_sensor.Z;
  
  if (x_t == -1 && y_t == -1 && z_t == -1) {
    SerialUSB.print("ERROR! NO BMA250 DETECTED!");
    while(true);
  } else {
    gravity_mag = sqrt(x_t*x_t + y_t*y_t + z_t*z_t);  
  }   
  delay(250);
  
  return gravity_mag;
}


void connectToWiFi() {
  WiFi.setPins(8, 2, A3, -1); // VERY IMPORTANT FOR TINYDUINO
  while(!SerialUSB);
  if (WiFi.status() == WL_NO_SHIELD) {
    SerialUSB.println("WiFi shield not present");
    while (true);
  }

  while ( status != WL_CONNECTED) {
    SerialUSB.print("Attempting to connect to SSID: ");
    SerialUSB.println(ssid);
    status = WiFi.begin(ssid, pass);
    listNetworks();
    delay(10000);
  }
  
  if ( status != WL_CONNECTED) {
    SerialUSB.println("Couldn't get a wifi connection");
    while(true);
  }
  // you're connected now, so print out the data:
  SerialUSB.println("You're connected to the network");
  //printCurrentNet();
  //printWiFiData();
  //printMacAddress();
  
}

void printWiFiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  SerialUSB.print("IP Address: ");
  SerialUSB.println(ip);
  SerialUSB.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  SerialUSB.print("MAC address: ");
  printMacAddress(mac);

  // print your subnet mask:
  IPAddress subnet = WiFi.subnetMask();
  SerialUSB.print("NetMask: ");
  SerialUSB.println(subnet);

  // print your gateway address:
  IPAddress gateway = WiFi.gatewayIP();
  SerialUSB.print("Gateway: ");
  SerialUSB.println(gateway);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  SerialUSB.print("SSID: ");
  SerialUSB.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  SerialUSB.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  SerialUSB.print("signal strength (RSSI):");
  SerialUSB.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  SerialUSB.print("Encryption Type:");
  SerialUSB.println(encryption, HEX);
}

void listNetworks() {
  // scan for nearby networks:
  SerialUSB.println("** Scan Networks **");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1)
  {
    SerialUSB.println("Couldn't get a wifi connection");
    while (true);
  }

  // print the list of networks seen:
  SerialUSB.print("number of available networks:");
  SerialUSB.println(numSsid);

  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    SerialUSB.print(thisNet);
    SerialUSB.print(") ");
    SerialUSB.print(WiFi.SSID(thisNet));
    SerialUSB.print("\tSignal: ");
    SerialUSB.print(WiFi.RSSI(thisNet));
    SerialUSB.print(" dBm");
    SerialUSB.print("\tEncryption: ");
    printEncryptionType(WiFi.encryptionType(thisNet));
    SerialUSB.flush();
  }
}

void printEncryptionType(int thisType) {
  // read the encryption type and print out the name:
  switch (thisType) {
    case ENC_TYPE_WEP:
      SerialUSB.println("WEP");
      break;
    case ENC_TYPE_TKIP:
      SerialUSB.println("WPA");
      break;
    case ENC_TYPE_CCMP:
      SerialUSB.println("WPA2");
      break;
    case ENC_TYPE_NONE:
      SerialUSB.println("None");
      break;
    case ENC_TYPE_AUTO:
      SerialUSB.println("Auto");
      break;
  }
}

void printMacAddress() {
  // the MAC address of your WiFi shield
  byte mac[6];

  // print your MAC address:
  WiFi.macAddress(mac);
  SerialUSB.print("MAC: ");
  printMacAddress(mac);
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      SerialUSB.print("0");
    }
    SerialUSB.print(mac[i], HEX);
    if (i > 0) {
      SerialUSB.print(":");
    }
  }
  SerialUSB.println();
}

// Prints the sensor values to the Serial Monitor, or Serial Plotter (found under 'Tools')
void showSerial() {
  SerialUSB.print("X = ");
  SerialUSB.print(x);
  
  SerialUSB.print("  Y = ");
  SerialUSB.print(y);
  
  SerialUSB.print("  Z = ");
  SerialUSB.println(z);
}
