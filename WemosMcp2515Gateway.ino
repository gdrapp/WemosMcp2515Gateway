/* Gateway between CANbus and http over WiFi.
 *   
 * The code is slightly biassed toward the Renault brand in the sense
 * that it uses standard (11 bits) addressing, assumes bare frames
 * for addresses under 0x700, ISOTP formatted diagnostic commands for
 * addresses of 0x700 and above, and does NOT adhere to the convention
 * that an ECU anwers on an address which is 8 above the receiving
 * address.
 * 
 * All these constraints / assumptions can be airly easily changed.
 * 
 * WiFi configuration is implemented using Dean Gording code.
 * 
 * http is unencrypted and unsecured.
 * 
 * The http server implements the following primitives:
 * /Init                Initialises the MCP2515 CANbus controller
 * /FreeFrame?f=x.y     Request the value of frame ID x in hex. y is
 *                      a decimal timeout in ms, but it is ignored
 * /IsoTpFrame?f=x.y.z  Send hex command string z as ISOTP to ID y 
 *                      and expect answer on ID x. Note that if the
 *                      first nibble of the command string >= 4, 4
 *                      is substracted. Answer is checked to adhere
 *                      to ISOTP, but not check the response for an
 *                      UDS header. Max command length is 7 bytes.
 * /Raw?f=x,y           Send hex command y to ID x. Max command
 *                      length is 8 bytes.
 * /Config              this is defuct and should not be used
 * 
 * All answers are JSON strings with the elements, "C", "F" and "R".
 * C represents the comand, F the frame and R the result. The result
 * is one of
 * - a hex string (for FreeFrame and IsoTpFrame
 * - the literal OK
 * - a human readable error that always starts with a dash
 * 
 * This interface is compatible with the CanZE app as an alternative
 * to the ELM327 driver. There is also a WiFi to Bluetooth (ELM327)
 * sketch available using the same interface, as well as as a
 * simple PHP script emulating the car.
 * 
 * First roughly 400 lines are an adaption of Dean Cording's
 * excellent code for OTA and Wifi configuration and have nothing
 * to do with the factionality of the gateway.
 */

/**
   ESP8266 project template with optional:
    - WiFi config portal - auto or manual trigger
    - OTA update - Arduino or web server
    - Deep sleep
    - Process timeout watchdog

   Copyright (c) 2016 Dean Cording  <dean@cording.id.au>

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
*/

/**
   ESP8266 Pin Connections

   GPIO0/SPI-CS2/_FLASH_ -                           Pull low for Flash download by UART
   GPIO1/TXD0/SPI-CS1 -                              _LED_
   GPIO2/TXD1/I2C-SDA/I2SO-WS -
   GPIO3/RXD0/I2SO-DATA -
   GPIO4 -
   GPIO5/IR-Rx -
   GPIO6/SPI-CLK -                                  Flash CLK
   GPIO7/SPI-MISO -                                 Flash DI
   GPIO8/SPI-MOSI/RXD1 -                            Flash DO
   GPIO9/SPI-HD -                                   Flash _HD_
   GPIO10/SPI-WP -                                  Flash _WP_
   GPIO11/SPI-CS0 -                                 Flash _CS_
   GPIO12/MTDI/HSPI-MISO/I2SI-DATA/IR-Tx -
   GPIO13/MTCK/CTS0/RXD2/HSPI-MOSI/I2S-BCK -
   GPIO14/MTMS/HSPI-CLK/I2C-SCL/I2SI_WS -
   GPIO15/MTDO/RTS0/TXD2/HSPI-CS/SD-BOOT/I2SO-BCK - Pull low for Flash boot
   GPIO16/WAKE -
   ADC -
   EN -
   RST -
   GND -
   VCC -
*/

#include <Arduino.h>

/* ********** WIFI AND UPDATE CONFIGURAION STARTS HERE ************************
*/

/* If DEBUG is not defined, nothin will be send to Serial.
 *  
 * It is good practice to extend this behavior in your user code to easily
 *  enable other usage of the Serial interface
 */

#define DEBUG
#define SERIAL_BPS 115200

// Optional functionality. Comment out defines to disable feature
#define WIFI_PORTAL                     // Enable WiFi config portal
//#define ARDUINO_OTA                   // Enable Arduino IDE OTA updates
//#define HTTP_OTA                      // Enable OTA updates from http server
#define LED_STATUS_FLASH                // Enable flashing LED status
//#define DEEP_SLEEP_SECONDS   5        // Define for sleep period between process repeats. No sleep if not defined

#define STATUS_LED  LED_BUILTIN         // Built-in blue LED

#include <ESP8266WiFi.h>

#ifdef WIFI_PORTAL
#include <DNSServer.h>                  // Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>           // Local WebServer used to serve the configuration portal
#include <WiFiManager.h>                // https://github.com/tzapu/WiFiManager WiFi Configuration Magic

WiFiManager wifiManager;

//#define WIFI_PORTAL_TRIGGER_PIN  D2   // A low input on this pin will trigger the Wifi Manager Console at boot. Comment out to disable.

#else
#define WIFI_SSID "SSID"                // this is only relevant when the WIFI_PORTAL is NOT used
#define WIFI_PASSWORD "password"
#endif

 /* Over The Air updates directly from Arduino IDE
  */
#ifdef ARDUINO_OTA                      // do NOT use this without a complex password, as your device maybe reprogrammed
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define ARDUINO_OTA_PORT      8266      // leave untouched
#define ARDUINO_OTA_HOSTNAME  "esp8266" // leave untouched, used by mDNS
#define ARDUINO_OTA_PASSWD    "123"
#endif

/* Over The Air automatic firmware update from a web server.  ESP8266 will contact the
 * server on every boot and check for a firmware update.  If available, the update will
 * be downloaded and installed.  Server can determine the appropriate firmware for this
 * device from any combination of HTTP_OTA_VERSION, MAC address, and firmware MD5 checksums.
 */
#ifdef HTTP_OTA
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

#define HTTP_OTA_ADDRESS      F("84.104.236.252")      // Address of OTA update server
#define HTTP_OTA_PATH         F("/esp8266-ota/update/update.php") // Path to update firmware
#define HTTP_OTA_PORT         8123                     // Port of update server
// Name of firmware
#define HTTP_OTA_VERSION       String(__FILE__).substring(String(__FILE__).lastIndexOf('/')+1) + ".001"

#endif

const char* SSID = "portal";

/* ********** NO CONFIGURABLE DATA AFTER THIS POINT ***************************

   All code until "END OF WIFI INITIALISATION CODE" should be left untouched
   Write your use code as mySetup() and myLoop()
*/

/* Watchdog to guard against the ESP8266 wasting battery power looking for
    non-responsive wifi networks and servers. Expiry of the watchdog will trigger
    either a deep sleep cycle or a delayed reboot. The ESP8266 OS has another built-in
    watchdog to protect against infinite loops and hangups in user code.
*/
#include <Ticker.h>
Ticker watchdog;
#define WATCHDOG_SETUP_SECONDS  30     // Setup should complete well within this time limit
#define WATCHDOG_LOOP_SECONDS   20    // Loop should complete well within this time limit

void timeout_cb() {
  // This sleep happened because of timeout. Do a restart after a sleep
  debugMsgLn(F("Watchdog timeout..."));

#ifdef DEEP_SLEEP_SECONDS
  // Enter DeepSleep so that we don't exhaust our batteries by countinuously trying to
  // connect to a network that isn't there.
  ESP.deepSleep(DEEP_SLEEP_SECONDS * 1000, WAKE_RF_DEFAULT);
  // Do nothing while we wait for sleep to overcome us
  while (true) {};

#else
  delay(1000);
  ESP.restart();
#endif
}

#ifdef LED_STATUS_FLASH
Ticker flasher;

void flash() {
  digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
}
#endif

#ifdef WIFI_PORTAL
// Callback for entering config mode
void configModeCallback (WiFiManager *myWiFiManager) {
  // Config mode has its own timeout
  watchdog.detach();

#ifdef LED_STATUS_FLASH
  flasher.attach(0.2, flash);
#endif
}
#endif

// Put any project specific initialisation here

template <typename Generic> void debugMsg (Generic g) {
#ifdef DEBUG
  Serial.print(g);
#endif
}
template <typename Generic> void debugMsgLn (Generic g) {
#ifdef DEBUG
  Serial.println(g);
#endif
}
template <typename Generic> void debugMsgf (char *f, Generic g) {
#ifdef DEBUG
  Serial.printf(f, g);
#endif
}

void setup() {
  Serial.begin(SERIAL_BPS);
  debugMsgLn(F("Booting"));

#ifdef LED_STATUS_FLASH
  pinMode(STATUS_LED, OUTPUT);
  flasher.attach(0.6, flash);
#endif

  // Watchdog timer - resets if setup takes longer than allocated time
  watchdog.once(WATCHDOG_SETUP_SECONDS, &timeout_cb);

  // Set up WiFi connection
  // Previous connection details stored in eeprom
#ifdef WIFI_PORTAL
#ifndef DEBUG
  wifiManager.setDebugOutput(false);
#endif

#ifdef WIFI_PORTAL_TRIGGER_PIN
  pinMode(WIFI_PORTAL_TRIGGER_PIN, INPUT_PULLUP);
  delay(100);
  if ( digitalRead(WIFI_PORTAL_TRIGGER_PIN) == LOW ) {
    watchdog.detach();
    if (!wifiManager.startConfigPortal(SSID, NULL)) {
      debugMsgLn(F("Config Portal Failed!"));
      timeout_cb();
    }
  } else {
#endif

    wifiManager.setConfigPortalTimeout(180);
    wifiManager.setAPCallback(configModeCallback);
    if (!wifiManager.autoConnect()) {
      debugMsgLn(F("Connection Failed!"));
      timeout_cb();
    }

#ifdef WIFI_PORTAL_TRIGGER_PIN
  }
#endif

#else
  // Save boot up time by not configuring them if they haven't changed
  if (WiFi.SSID() != WIFI_SSID) {
    debugMsgLn(F("Initialising Wifi..."));
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.persistent(true);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
  }
#endif

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    debugMsgLn(F("Connection Failed!"));
    timeout_cb();
  }

  debugMsg(F("IP address: "));
  debugMsgLn(WiFi.localIP());

#ifdef LED_STATUS_FLASH
  flasher.detach();
  digitalWrite(STATUS_LED, HIGH);
#endif

#ifdef HTTP_OTA
  // Check server for firmware updates
  debugMsg("Checking for firmware updates from server http://");
  debugMsg(HTTP_OTA_ADDRESS);
  debugMsg(":");
  debugMsg(HTTP_OTA_PORT);
  debugMsgLn(HTTP_OTA_PATH);
  debugMsgLn(HTTP_OTA_VERSION);
  switch (ESPhttpUpdate.update(HTTP_OTA_ADDRESS, HTTP_OTA_PORT, HTTP_OTA_PATH, HTTP_OTA_VERSION)) {
    case HTTP_UPDATE_FAILED:
      // debugMsgf("HTTP update failed: Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      debugMsgLn("HTTP update failed: Error " + ESPhttpUpdate.getLastErrorString() + "\n");
      break;

    case HTTP_UPDATE_NO_UPDATES:
      debugMsgLn(F("No updates"));
      break;

    case HTTP_UPDATE_OK:
      debugMsgLn(F("Update OK"));
      break;
  }
#endif

#ifdef ARDUINO_OTA
  // Arduino OTA Initalisation
  ArduinoOTA.setPort(ARDUINO_OTA_PORT);
  ArduinoOTA.setHostname(SSID);
  ArduinoOTA.setPassword(ARDUINO_OTA_PASSWD);
  ArduinoOTA.onStart([]() {
    watchdog.detach();
    debugMsgLn("Start");
  });
  ArduinoOTA.onEnd([]() {
    debugMsgLn("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    debugMsgf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    debugMsgf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) debugMsgLn("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) debugMsgLn("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) debugMsgLn("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) debugMsgLn("Receive Failed");
    else if (error == OTA_END_ERROR) debugMsgLn("End Failed");
  });
  ArduinoOTA.begin();
#endif
  debugMsgLn(F("Ready"));
  watchdog.detach();

  mySetup ();
}

void loop() {
  // Watchdog timer - resets if setup takes longer than allocated time
  watchdog.once(WATCHDOG_LOOP_SECONDS, &timeout_cb);

  myLoop ();

  watchdog.detach();

#ifdef ARDUINO_OTA
  // Handle any OTA upgrade
  ArduinoOTA.handle();
#endif

#ifdef DEEP_SLEEP_SECONDS
  // Enter DeepSleep
  debugMsgLn(F("Sleeping..."));
  ESP.deepSleep(DEEP_SLEEP_SECONDS * 1000000, WAKE_RF_DEFAULT);
  // Do nothing while we wait for sleep to overcome us
  while (true) {};
#endif

}


/* ********** END OF WIFI INITIALISATION CODE ********************************* */

/* Wiring for SPI (GPIO number = pin integer in shield, D number is user friendly define and number on the PCB)
 * See http://www.pighixxx.com/test/portfolio-items/esp8266/?portfolioID=360 for ESP-12 pin layout
 * 
 * SPI pin        GPIOx   Dx    ESP-12  Function
 * =============  ======  ====  ======  ========
 * CLK   SCK      GPIO14  (D5)  5       HSPI-clk
 * D_OUT MISO     GPIO12  (D6)  6       HSPI-q
 * D_IN  MOSI     GPIO13  (D7)  7       HSPI-d
 * CS    SS       GPIO15  (D8)  16      HSPI-cs
 * 
 * We use D2 as the INTPIN. It is more usual to use D4 for that, bus as the WeMos has
 * the LED wired there, we want to keep that one unused
 * 
 * As we use a cheap controller card using the TJA1050 CANbus tranceiver, we need
 * to make a small modification to allow the MCP2515 to run on 3.3 volt and
 * the TJA1050 on 5 volt, see https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=141052
 * 
 */

ESP8266WebServer server(80);            

// https://github.com/coryjfowler/MCP_CAN_lib
#include <mcp_can.h>
#include <SPI.h>
MCP_CAN CAN0(D8);                       // Set CS to D8 on Wemos

#define LEDPIN          LED_BUILTIN     // LED pin
#define INTPIN          D2              // INT pin of MCP2515

// communication globals
String  lastInitProblem;
byte    idIndex [0x700];                // index in Qbuf, for ID's 0x000 - 0x6ff
byte    Qbuf [900];                     // room for 100 different free frames, 8 bytes data plus length
int     q = 1;                          // first position to occupy (all times)

byte    isoTpBuf [1024];
int     isoTpBufPtr;
int     isoTpState;
int     isoTpInProgress;
int     isoTpCnt;
int     isoTpLen;
INT32U  isoTpToId;


/******************************************
    HTTP
 ******************************************/

void initHttp () {

  server.on("/", handleRoot);

  server.on("/Init", handleInitDevice);
  server.on("/IsoTp", handleIsoTpFrame);
  server.on("/Free", handleFreeFrame);
  server.on("/Raw", handleRawFrame);
  server.on("/Config", handleConfig);
  server.onNotFound(handleRoot);

  server.begin();
  debugMsgLn("HTTP server started");
}

void handleRoot() {
  server.send(200, "application/json", "{\"C\":\"\",\"R\":\"-E-Wrong command\"}");
}


/******************************************
    Functional handlers
 ******************************************/

void handleInitDevice () {
  if (initDevice (0)) {
    server.send(200, "application/json", "{\"C\":\"InitDevice\",\"R\":\"OK\"}");
  } else {
    server.send(200, "application/json", "{\"C\":\"InitDevice\",\"R\":\"-E-" + lastInitProblem + "\"}");
  }
}

void handleFreeFrame () {
  String f = server.arg("f");
  String result = requestFreeFrame (f);
  server.send(200, "application/json", "{\"C\":\"FreeFrame\",\"R\":\"" + result + "\"}");
}

void handleIsoTpFrame () {
  String f = server.arg("f");
  String result = requestIsoTpFrame (f);
  server.send(200, "application/json", "{\"C\":\"IsoTpFrame\",\"R\":\"" + result + "\"}");
}

void handleRawFrame () {
  String f = server.arg("f");
  String result = requestRawFrame (f);
  server.send(200, "application/json", "{\"C\":\"RawFrame\",\"R\":\"" + result + "\"}");
}

void handleConfig() {
}


/******************************************
    MCP2515
 ******************************************/

boolean initDevice(int toughness) {
  pinMode(INTPIN, INPUT);

  if (CAN0.begin(MCP_STD, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    lastInitProblem = "Error Initializing MCP2515";
    return false;
  }
  if (CAN0.setMode(MCP_NORMAL) != CAN_OK) {
    lastInitProblem = "Error Setting Mode MCP2515";
    return false;
  }
  return true;
}

boolean sendFlowControlAll (INT32U id) {
                                          // some ECU's require fillers, so we send a full frame
  byte sendBuffer [] = {0x30, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (CAN0.sendMsgBuf(id, 8, sendBuffer) != CAN_OK) {
    debugMsgLn("Error Sending Flow Control MCP2515");
    return false;
  }
  return true;
}

boolean sendIsoTpCommand (INT32U id, String command) {

  byte sendBuffer [8];
  byte len = command.length() / 2;
  byte result;
  int i;

  if (len > 7) return false;              // For now only support command to 7 bytes (FIRST)
  sendBuffer [0] = len;                   // first niblle = 0 (FIRST), second niblle = length
  for (i = 1; i <= len; i++) {            // fill the buffer
    // debugMsgLn (command.substring(i * 2 - 2, i * 2));
    sendBuffer [i] = (byte)strtol(command.substring(i * 2 - 2, i * 2).c_str(), NULL, 16);
  }                                       // some ECU's require fillers, so we send a full frame
  for (; i <= 7; i++) {
    sendBuffer [i] = 0;
  }
                                          // send the buffer
  result = CAN0.sendMsgBuf(id, 8, sendBuffer);
  if (result != CAN_OK) {
    debugMsgLn("Error Sending IsoTp MCP2515");
    debugMsgLn(result);
    debugMsgLn(CAN0.getError());
    debugMsgLn(CAN0.errorCountTX());
    return false;
  }
  return true;
}

boolean sendRawCommand (INT32U id, String command) {

  byte sendBuffer [8];
  byte len = command.length() / 2;
  byte result;
  int i;
  
  if (len > 8) return false;              // raw is of course single frame
  for (len = 0; i < len; i++) {           // fill the buffer
    sendBuffer [i] = (byte)strtol(command.substring(i * 2, i * 2 + 2).c_str(), NULL, 16);
  }
                                          // send the buffer
  result = CAN0.sendMsgBuf(id, len, sendBuffer);
  if (result != CAN_OK) {
    debugMsgLn("Error Sending Raw MCP2515");
    debugMsgLn(result);
    debugMsgLn(CAN0.getError());
    debugMsgLn(CAN0.errorCountTX());
    return false;
  }
  return true;
}

void handleIncomingCanFrame () {

  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];
  unsigned int p;                         // p is the postition of the current frame in the Qbuf. It is obtained by a lookup in idIndex

  if (digitalRead(INTPIN)) return;        // stop if there is no data available

  CAN0.readMsgBuf(&rxId, &len, rxBuf);    // get the frame
  if (rxId < 0x700) {                     // if if is a free frame, store it
    p = idIndex [rxId];                   // find it's buffer
    if (p == 0) {                         // if none assigned
      p = idIndex [rxId] = q++;           // assign it as q and increment
      if (q >= 100) {                     // check for overflow
        debugMsgLn ("Q buffer overflow!");
        return;
      }
    }
    memcpy(Qbuf + (p * 9), rxBuf, 8);     // copy the data in the buffer
    Qbuf [(p * 9) + 8] = len;             // and the frame length
    return;
  }

                                          // it is a diagnostic frame
  if (isoTpInProgress == rxId) {          // is it the current, requested frame?
    switch (isoTpState) {                 // state 0: FIRST or SINGLE
      case 0:
        if ((rxBuf[0] & 0xf0) == 0x00) {  // SINGLE frame. Handle it and state is unchanged
          memcpy(isoTpBuf, rxBuf + 1, rxBuf[0]);
          isoTpLen = rxBuf[0];            // first nibble = 0, second nibble is length
          isoTpInProgress = 0;            // indicate we're succesfully done
          return;
        } else if ((rxBuf[0] & 0xf0) == 0x10) { // FIRST frame. Handle it and state <- 1
          isoTpLen = (rxBuf[0] & 0x0f) * 256 + rxBuf[1];
          if (isoTpLen > 1024) {          // length is second - fourth nibble
            isoTpState = 0;               // if > 1K (in theory, 4K is allowed)
            isoTpInProgress = -3;         // reset state and indicate done with error
            return;
          }
          memcpy(isoTpBuf, rxBuf + 2, 6); // copy first 6 bytes
          isoTpBufPtr = 6;                //pointer in isoTpBuf moves up 6 positions
          isoTpState = 1;                 // we're moving to state 1 (NEXT expected)
          isoTpCnt = 1;                   // NEXT frame count to expext
          sendFlowControlAll (isoTpToId); // we're ready for the remainder
          return;
        }
        break;
      case 1:                             // state 1: NEXT
        if ((rxBuf[0] & 0xf0) == 0x20) {  // check if it is indeed a NEXT frame
          if ((rxBuf[0] & 0x0f) == isoTpCnt) {
            isoTpCnt = (isoTpCnt + 1) & 0x0f; // increment and roll over
            int l = isoTpLen - isoTpBufPtr;   // calculate how much is left
            if (l > 7) {
              l = 7;
            } else {                      // if nothing 
              isoTpState = 0;             // we're succesfully done
              isoTpInProgress = 0;
            }                             // copy the data
            memcpy(isoTpBuf + isoTpBufPtr, rxBuf + 1, l);
            isoTpBufPtr += l;             // increment the pointer
            return;
          } else {                        // out of sequence
            isoTpState = 0;               // reset state and indicate done with error
            isoTpInProgress = -1;
            return;
          }
        } else {                          // not a NEXT frame
          isoTpState = 0;                 // reset state and indicate done with error
          isoTpInProgress = -2;
          return;
        }
        break;                            // ignore all others i.e. remaining frames after an error situation
    }
  }                                       // ignore diagnostic from unrequested ECUs
}

String byteArrayToHexString (byte *b, int len) {
  String hexData = "";
  for (int i = 0; i < len; i++) {
    byte v = *(b + i);
    if (v > 15) {
      hexData += String(v, HEX);
    } else {
      hexData += "0" + String(v, HEX);
    }
  }
  return hexData;
}

/* Note that requestFreeFrame is a non blocking function. It assumes the
 * requested frame has floated around on the bus and the last seen
 * value is returned. It will NOT wait for the current value and age
 * is not indicated.
 */
String requestFreeFrame(String frame) {

  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];
  int p;

  String frameId = frame.substring (0, frame.indexOf('.'));

                                          // find the buffer
  rxId = strtol (frameId.c_str(), NULL, 16);
  p = idIndex [rxId];
  if (p == 0) return "";                  // if not assigned (never received), return empty
  p = p * 9;                              // calculate pointer
                                          // and return as hex string
  return byteArrayToHexString (Qbuf + p, (int)Qbuf [p + 8]);
}

String requestIsoTpFrame(String frame) {

  String hexData = "";
  int len = 0;
  char buf [512];
  char *token;
  int framesToReceive;
  unsigned long endTime;

  // example 764.744.6121
  frame.toCharArray(buf, 512);
  token = strtok (buf, ".");
  unsigned int frameId = strtol (token, NULL, 16); // 764 this is the IS the CLIM replies from
  token = strtok (NULL, ".");
  isoTpToId = strtol (token, NULL, 16);   // 744 this is the ID to use to send to the CLIM
  token = strtok (NULL, ".");
  String responseId = String (token);     // command response 6121
  String requestId = String (token);
  requestId.setCharAt (0, requestId.charAt(0) - 4); // command 2121

  // Avoid overloading the processor or the MCP or the SPI bus by filtering
  // just diagnostics. this could in fact be reduced to only the requested
  // answer frame ID. Remember there are about 1600 fps on the bus
  
  CAN0.init_Mask(0,0,0x07000000);         // set filter/mask 0 to pick up diagnostic only
  CAN0.init_Filt(0,0,0x07000000);
  CAN0.init_Mask(1,0,0x07ff0000);         // set filter/mask 1 to never trigger
  CAN0.init_Filt(2,0,0x00000000);
  handleIncomingCanFrame ();              // this is just to flush

                                          // send the command to the ECU
  if (!sendIsoTpCommand (isoTpToId, requestId)) {
    return "-E-Bus error";
  }

  isoTpState = 0;                         // state is response can be SINGLE or FIRST
  isoTpInProgress = frameId;              // in progress

  // handle incoming frames
  endTime = millis() + 1000;              // wait until done or 1 second has passed
  while (isoTpInProgress > 0 && millis () < endTime) {
    yield();                              // give the SDK a few cycles
    handleIncomingCanFrame ();            // assemble diagnostic frame result
  }

  CAN0.init_Mask(0,0,0x00000000);         // remove filter/mask 0
  CAN0.init_Filt(0,0,0x00000000);

  if (isoTpInProgress == 0) {             // normal result bytes in isoTpBuf
    return byteArrayToHexString (isoTpBuf, isoTpLen);
  } else if (isoTpInProgress == -1) {
    return "-E-Out of sequence";
  } else if (isoTpInProgress == -2) {
    return "-E-Expected type NEXT";
  } else if (isoTpInProgress == -3) {
    return "-E-Frame too long";
  } else if (isoTpInProgress == frameId) {
    return "-E-Timeout";
  }
  return "-E-Unknown";
}

String requestRawFrame (String frame) {
  String hexData = "";
  int len = 0;
  char buf [512];
  char *token;

  // example 130.AA203040
  frame.toCharArray(buf, 512);
  token = strtok (buf, ".");
  unsigned int frameId = strtol (token, NULL, 16); // 130
  token = strtok (NULL, ".");
  String content = String (token); // AA203040

                                          // send the command to the ECU
  if (!sendRawCommand (frameId, content)) {
    return "-E-Bus error";
  }
  return "OK";
}


/******************************************
    Setup
 ******************************************/

void mySetup() {
  debugMsg("Start");

  // start the MCP2515
  initDevice (1);

  // Start the webserver
  initHttp();
}


/******************************************
    Loop
 ******************************************/
void myLoop() {
  // Run timed functions
  // timer.run();

  // listen for incoming clients for the webserver
  server.handleClient();

  // absorb data from the CANbus
  handleIncomingCanFrame ();
}
