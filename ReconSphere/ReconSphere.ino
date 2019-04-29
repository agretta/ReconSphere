// ReconSphere TinyZero Code
#include <WiFi101.h>      // Wifi Shield library
#include <WiFiUdp.h>
#include <Wire.h>         // For I2C communication with sensor
#include "BMA250.h"       // For interfacing with the accel. sensor
#include <SharpDistSensor.h>

// Accelerometer globals
BMA250 accel_sensor;
int x, y, z;
int init_x, init_y, init_z;
double x_rot, y_rot;
// Note: 256 is roughly gravity
double gravity_mag = 0.0;


// IR Reader globals
const byte nbSensors = 4;
const byte medianFilterWindowSize = 3;
SharpDistSensor sensorArray[] = {
  SharpDistSensor(A1, medianFilterWindowSize),
  SharpDistSensor(A2, medianFilterWindowSize),
  SharpDistSensor(A4, medianFilterWindowSize),
  SharpDistSensor(A5, medianFilterWindowSize),
};
uint16_t distArray[nbSensors];
int directionArray[nbSensors][3] = {{0,0,1},
                                    {0,0,-1},
                                    {0,1,0},
                                    {0,-1,0}};
                                     

//WiFi globals
char ssid[] = "TinyZeroTest";  // network SSID
char pass[] = "gt123456";      // network password
int status = WL_IDLE_STATUS;   // WiFi radio status
//IPAddress server(192,168,43,233); // Alec's Laptop
IPAddress server(192,168,43,207); // Amrut's Laptop
int server_port = 8347;
WiFiUDP Udp;
unsigned int localPort = 2390;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[255];
const char* delim = "|";



void setup() {
  SerialUSB.begin(9600);

  // Set up the BMA250 acccelerometer sensor
  SerialUSB.print("Initializing BMA...");
  Wire.begin();
  accel_sensor.begin(BMA250_range_2g, BMA250_update_time_64ms);   
 
  initIRSensors();

  //connectToWiFi();
  Udp.begin(localPort);
  
  delay(500);  
}


void loop() {
  bool run_scan = false;

  // Wait for a start packet to begin scanning
  run_scan = checkContinueScan();

  // Send the initial gravity reading to the server
  if (run_scan) {
    char c[11];
    itoa(init_x,c,10);
    itoa(init_y,&(c[4]),10);
    itoa(init_z,&(c[8]),10);
    c[3] = *delim;
    c[7] = *delim;
    SerialUSB.println(c);
    sendPacket(c, 11);
  }
  
  while(run_scan) {
    
    //temp delay   
    delay(10000);
    
    // Read from the accelorometer to determine current posistion 
    bool accel_error = false;
    accel_sensor.read();
    x = accel_sensor.X;
    y = accel_sensor.Y;
    z = accel_sensor.Z;

    // Error checking
    if (x == -1 && y == -1 && z == -1) {
      SerialUSB.print("ERROR! NO BMA250 DETECTED!");
      accel_error = true;
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
      showSerial();
    }

    // scan the world with the IR sensors
    if (!accel_error) {
      SerialUSB.println("SENSOR READING:");
      for (byte i = 0; i < nbSensors; i++) {
        unsigned int distance = sensorArray[i].getDist();
        SerialUSB.println(distance);
        SerialUSB.println(analogRead(A1));
        distArray[i] = distance;
      }
    }

    // Construct and send the scanned data packet to the server
    if (!accel_error) {
      char c[31];
      // gravity reading
      itoa(x,c,10);
      itoa(y,&(c[4]),10);
      itoa(z,&(c[7]),10);
      
      // sensor readings
      itoa(distArray[0],&(c[12]),10);
      itoa(distArray[1],&(c[17]),10);
      itoa(distArray[2],&(c[22]),10);
      itoa(distArray[3],&(c[27]),10);
      
      c[3] = *delim;
      c[7] = *delim;
      c[11] = *delim;
      c[16] = *delim;
      c[21] = *delim;
      c[26] = *delim;
      SerialUSB.println(c);
      sendPacket(c, 31);
    }
    
    // The BMA250 can only poll new sensor values every 64ms
    delay(70);
    
    // if the sphere recived a end packet, stop scanning and reset
    run_scan = checkContinueScan();    
  }

}

/*
 * Checks for an incoming packet indicating whether or not the ReconSphere
 * should begin or end the scanning process, also inits/resets the gravity measurement
 * Returns a boolean indicating whether or not the ReconSphere should be scanning
 */
 
bool checkContinueScan() {
  bool continue_scan = false;

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    SerialUSB.print("Received packet of size ");
    SerialUSB.println(packetSize);
    SerialUSB.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    SerialUSB.print(remoteIp);
    SerialUSB.print(", port ");
    SerialUSB.println(Udp.remotePort());

    int len = Udp.read(packetBuffer, packetSize);
    if (len > 0) packetBuffer[len] = 0;
    SerialUSB.println("Contents:");
    SerialUSB.println((char *)packetBuffer);
    if ((char *)packetBuffer == "START") {
      continue_scan = true;
      gravity_mag = callibrateToGravity();  
    }
    if ((char *)packetBuffer == "END") {
      continue_scan = false;
      gravity_mag = 0;
      init_x, init_y, init_z = 0;
    }
  }
  return continue_scan;
}

/*
 * Sends a UDP packet to the server specified in the globals
 */
void sendPacket(char packetData[], int packet_size) {
  Udp.beginPacket(server, server_port);
  Udp.write(packetData, packet_size);
  Udp.endPacket();
  SerialUSB.println("Sending packet"); 
  SerialUSB.println(packetData);
}

/*
 * Sets up the custom 5 variable polynomial function for decoding analog voltage to a distance in mm
 * also initilzes posistions of sensors with relation to sphere
 */
void initIRSensors() {
  SerialUSB.print("Initializing IR Sensors...");
  // Custom callibration of 3.3V ir sensor based on calibration data
  const float polyCoefficients[] = {1015.787378, -56.37879315, 1.475534708, -1.85676847E-2,  1.103053715E-4, -2.479657003E-7};
  const byte nbCoefficients = 6;
  const unsigned int minVal = 80; // ~1500 mm
  const unsigned int maxVal = 875; // ~10mm

  for (byte i = 0; i < nbSensors; i++) {
    sensorArray[i].setPolyFitCoeffs(nbCoefficients, polyCoefficients, minVal, maxVal);
  }
}

/*
 * Callibrates the accelorometer to gravity and records the maginitude
 */
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
    init_x = x_t;
    init_y = y_t;
    init_z = z_t;
  }   
  delay(250);
  
  return gravity_mag;
}

/*
 * Sets up the WiFi Shield and connects to the network
 */
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
  
  SerialUSB.println("Connected to the network");
  printCurrentNet();
  printWiFiData();
  
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
