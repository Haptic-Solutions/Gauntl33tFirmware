/*
* Gauntl33t ESP32Bridge is designed to allow an ESP-32 board or micromodule to perform the task of passing messages back and forth from other processors and a host over Serial, Bluetooth or WiFi. 
*
* TODO
* -Add ways to set and store WiFi Credentials dynamically over bluetooth in a secure way.
* -See if there is a better way of achieving soft serial for high speed communication with multiple processors when USB Is in Use and both other physical ports are not accessible.
*
*  Referenced 
*  - https://github.com/espressif/arduino-esp32/blob/master/libraries/AsyncUDP/examples/AsyncUDPServer/AsyncUDPServer.ino
*  - https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiUDPClient/WiFiUDPClient.ino
*  - https://github.com/espressif/arduino-esp32/blob/a81e2d48f57c0a4c0b90c4a06596904d0905805a/libraries/WiFi/examples/WiFiClientStaticIP/WiFiClientStaticIP.ino
*  - https://github.com/espressif/arduino-esp32/blob/a81e2d48f57c0a4c0b90c4a06596904d0905805a/libraries/BluetoothSerial/examples/SerialToSerialBT/SerialToSerialBT.ino
*/

/*
* Wifi Includes and Settings
*/
#include "WiFi.h"
#include "WiFiUdp.h"
#include "AsyncUDP.h"

//Set Network Name for WiFi
const char * networkName = "########";
//Set password for WiFi
const char * password = "#######";

/*Async UDP Object for receiving messages*/
AsyncUDP asyncUDP;
/*WiFi UDP Object for sending messages*/
WiFiUDP wifiUDP;

/*Variable for setting if WiFi initialization was successful*/
bool WiFiSuccess = true;

/*Set IP Addresses to communicate with*/
const std::vector<char*> remoteEndPoints = {"192.168.0.10"};
/*Set Ports to communicate with*/
const std::vector<uint16_t> remotePorts = {2023};
/*Set local port*/
const uint16_t localPort = 2024;

//Set IP Information for glove.
const IPAddress staticIP(192, 168, 0, 100);
const IPAddress defaultGateway(192, 168, 0, 1);
const IPAddress subnetMask(255, 255, 255, 0);
const IPAddress dnsPrimary(8, 8, 8, 8);
const IPAddress dnsSecondary(8, 8, 4, 4);

/* Bluetooth Includes/Settings */
#include "BluetoothSerial.h"
#include "HardwareSerial.h"

BluetoothSerial bluetoothSerial;

/*
* SoftwareSerial Includes/Settings
* 
*/
#include <SoftwareSerial.h>

//Uncomment if using soft serial ports to communicate with MCUs. Make sure to also define what pins will be used for communication.
//#define USE_SOFTSERIAL1
//#define USE_SOFTSERAIAL2

#ifdef USE_SOFTSERIAL1
EspSoftwareSerial::UART SoftSerial1;
int SS1_RX = 36;
int SS1_TX = 39;
#endif
#ifdef USE_SOFTSERAIAL2
EspSoftwareSerial::UART SoftSerial2;
int SS2_RX = 34;
int SS2_TX = 35;
#endif

//This baud rate may actually be hard to achieve potentially. Decrease as needed. Will keep looking into ways to make this better and open to suggestions.
const uint32_t softBaudRate = 921600;

/*
* General Includes/Settings
*/
#include <vector>

//Uncomment if using USB to communicate to a Computer.
//#define USE_USB
//Uncomment ports you will use to communicate to microcontrollers (Usually Teensies). If using soft serial ports please update ConfigureSoftSerial.
//Note: USE_Serial0 cannot be used when USE_USB is defined.
#define USE_SERIAL0
//#define USE_SERIAL1
#define USE_SERIAL2

const uint32_t baudRate = 921600;

/**
 * Sets up Wifi for UDP communication.
 */
void WIFISetup()
{
  //Set WiFi Mode and connect to proveded network name with provided password
  WiFi.mode(WIFI_STA);
  WiFi.begin(networkName, password);

  //If the WiFi fails to connect print an error and store WiFiSuccess as false;
  if (WiFi.waitForConnectResult() != WL_CONNECTED) 
  {
    Serial.println("WiFi failed to connect!");
    WiFiSuccess = false;
    return;
  }
  //Print out that the WiFi succeeded to connect
  Serial.println("WiFi succeeded in connecting!");

  //Try setting static IP, if it fails print out an error, set WiFiSuccess to false, and return.
  if (!WiFi.config(staticIP, defaultGateway, subnetMask, dnsPrimary, dnsSecondary)) 
  {
    WiFiSuccess = false;
    Serial.println("Setting Static IP Address failed");
    return;
  }
  Serial.println("Setting Static IP Address succeeded");

  //listen for incoming asynchronous UDP packets.
  if(asyncUDP.listen(localPort)) {
        //when UDP packets are received send the packets to the serial terminal along with the serial port connected to other microcontroller(s)
        asyncUDP.onPacket([](AsyncUDPPacket asyncUDPPacket) {
          String receivedString((char*)asyncUDPPacket.data(),asyncUDPPacket.length());
          PrintMCUs(receivedString);
        });
    }
}

/**
* Configures Soft Serial Ports to the configured RX and TX Pins if in use.
*/
void ConfigureSoftSerial()
{
#ifdef USE_SOFTSERIAL1
  SoftSerial1.begin(softBaudRate, SWSERIAL_8N1, SS1_RX, SS1_TX, false);
#endif
#ifdef USE_SOFTSERIAL2
  SoftSerial2.begin(softBaudRate, SWSERIAL_8N1, SS2_RX, SS2_TX, false);
#endif
}

/**
 * setup function.
 * Starts up Serial ports.
 * Starts up Bluetooth Serial port.
 * Runs WIFISetup.
 */
void setup()
{
  Serial.begin(baudRate);
  Serial1.begin(baudRate);
  Serial2.begin(baudRate);
  ConfigureSoftSerial();
  bluetoothSerial.begin("Gauntl33t");
  WIFISetup();
}

/**
* PrintWifi sends an input message to a specified list of ports at a specified list of endpoints.
* @param aMessage string to send
* @param aEndPoints IP addresses to send the message to.
* @param aPorts ports to send the message to at the IP addresses.
*/
void PrintWiFi(const String& aMessage, const std::vector<char*>& aEndPoints, const std::vector<uint16_t>& aPorts)
{
  //iterate through endpoints and ports to send the message.
  for(auto endPoint : aEndPoints)
  {
    for(auto port : aPorts)
    {
      //set the combination of endPoint and port to send the packet to.
      wifiUDP.beginPacket(endPoint,port);
      //write message characters
      wifiUDP.write((uint8_t*)aMessage.c_str(),aMessage.length());
      //end and send packet
      wifiUDP.endPacket();
    }
  }
}

/**
* BroadCast_WF_BT_USB sends an input message over bluetooth, WiFi, and USB
* @param aMessage string to send
*/
void BroadCast_WF_BT_USB(const String& aMessage)
{
  //If WiFi is connected send the message over WiFi.
  if(WiFiSuccess == true)
    {
      PrintWiFi(aMessage,remoteEndPoints,remotePorts);
    }
  //Send message over bluetooth
  bluetoothSerial.println(aMessage);
  //also send to serial in case being controlled by usb
  #ifdef USE_USB
  Serial.println(aMessage);
  #endif
}

/**
* PrintMCUs sends an input message to enabled serial ports to talk with MCUs
* @param aMessage string to send
*/
void PrintMCUs(const String& aMessage)
{
  #ifndef USE_USB
  #ifdef USE_SERIAL0
  Serial.println(aMessage);
  #endif
  #endif
  #ifdef USE_SERIAL1
  Serial1.println(aMessage);
  #endif
  #ifdef USE_SERIAL2
  Serial2.println(aMessage);
  #endif
  #ifdef USE_SOFTSERIAL1
  SoftSerial1.println(aMessage);
  #endif
  #ifdef USE_SOFTSERIAL2
  SoftSerial2.println(aMessage);
  #endif
}

/**
* ReadMCUs reads messages from enabled serial ports (MCUs) and retransmits the messages to the Wifi, Bluetooth, and USB
*/
void ReadMCUs()
{
  #ifndef USE_USB
  #ifdef USE_SERIAL0
  if(Serial.available())
  {
    BroadCast_WF_BT_USB(Serial.readStringUntil('\n'));
  }
  #endif
  #endif
  #ifdef USE_SERIAL1
  if (Serial1.available())
  {
    BroadCast_WF_BT_USB(Serial1.readStringUntil('\n'));
  }
  #endif
  #ifdef USE_SERIAL2
  if(Serial2.available())
  {
    BroadCast_WF_BT_USB(Serial2.readStringUntil('\n'));
  }
  #endif
  #ifdef USE_SOFTSERIAL1
  if(SoftSerial1.available())
  {
    BroadCast_WF_BT_USB(SoftSerial1.readStringUntil('\n'));
  }
  #endif
  #ifdef USE_SOFTSERIAL2
  if(SoftSerial2.available())
  {
    BroadCast_WF_BT_USB(SoftSerial2.readStringUntil('\n'));
  }
  #endif
}

/**
 * main loop function. Handles passing messages between interfaces.
 */
void loop() 
{
  //If there are messages coming from the Microcontrollers serial ports read them and broadcast to Wifi, Bluetooth, and USB.
  ReadMCUs();
  
  //If data is coming in from a PC over USB Serial, read it and send to the microcontroller over serial.
  #ifdef USE_USB
  if (Serial.available())
  {
    auto message = Serial.readStringUntil('\n');
    PrintMCUs(message);
  }
  #endif

  //If data comes in over bluetooth read it and send it to the microcontrollers.
  if (bluetoothSerial.available())
  {
    auto message = bluetoothSerial.readStringUntil('\n');
    PrintMCUs(message);
  }
}
