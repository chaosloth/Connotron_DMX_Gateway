/*
  ============================================================
  Copyright (c) 2022 Christopher Connolly
  Project:     ArtNet to DMX Gateway
  Author:      Christopher Connolly
  Last update: 22.02.2022
  Description: ESP32 using MAX485 for RS485 communication
  License:     GNU GPL 3, see libraries for respective licenes too

  Dependencies & Kudos:
  https://github.com/rstephan/ArtnetWifi from Stephan Ruloff 2016-2019
  https://github.com/someweisguy/esp_dmx from By Mitch Weisbrod 2021
  https://github.com/khoih-prog/ESPAsync_WiFiManager from Khoi Hoangx
  ============================================================

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 3.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#define DEVICE_NAME               "ConnoDMX"
#define OTA_PASSWORD              "infinite"
#define WIFI_PASSWORD             "infinite"
#define USE_SPIFFS                true
#define ESP_DRD_USE_EEPROM        true
#define USE_ESP_WIFIMANAGER_NTP   false
#define DRD_TIMEOUT               10
#define DRD_ADDRESS               0
#define LED_BUILTIN               2
#define DMX_RX_TIMEOUT            20   // Instead of DMX_RX_PACKET_TOUT_TICK
#define ART_NET_TIMEOUT_MS        5000


#include "ArtnetWifi.h"
#include <esp_dmx.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPAsync_WiFiManager.h>
#include <ESP_DoubleResetDetector.h>

// Wifi configuration portal settings
String ssid = "ConnoDMX_" + String(ESP_getChipId(), HEX);   // Suffix AP name with Chip ID
String password = "";   // No password

DoubleResetDetector* drd;
const int PIN_LED       = 2;
bool      initialConfig = false;
AsyncWebServer webServer(80);
#if !( USING_ESP32_S2 || USING_ESP32_C3 )
DNSServer dnsServer;
#endif

// Status LED, blinks when all is groovy
boolean ledState = false;

WiFiUDP UdpSend;
ArtnetWifi artnet;

/* First, lets define the hardware pins that we are using with our ESP32. We
  need to define which pin is transmitting data and which pin is receiving data.
  DMX circuits also often need to be told when we are transmitting and when we are
  receiving data. We can do this by defining an enable pin. */
int dmxOutTransmitPin = 17;
int dmxOutReceivePin = 16;
int dmxOutEnablePin = 4;

int dmxInTransmitPin = 19;
int dmxInReceivePin = 18; 
int dmxInEnablePin = 21;

/* Next, lets decide which DMX ports to use. The ESP32 has either 2 or 3 UART ports.
  Port 0 is typically used to transmit serial data back to your Serial Monitor,
  so we shouldn't use that port. Lets use port 2! */
dmx_port_t dmxInPort = 1;   // UART 1 - Pins not exposed on board, must remap GPIOs
dmx_port_t dmxOutPort = 2;  // UART 2 - Default pins used

/* Now we want somewhere to store our DMX data. Since a single packet of DMX
  data can be up to 513 bytes long, we want our array to be at least that long.
  This library knows that the max DMX packet size is 513, so we can fill in the
  array size with `DMX_MAX_PACKET_SIZE`. */
byte dataOut[DMX_MAX_PACKET_SIZE];
byte dataIn[DMX_MAX_PACKET_SIZE];

/* The last variable that we need to read DMX is a queue handle. It's not
  important to know all the details about queues right now. All that you need to
  know is that when we receive a packet of DMX, our queue will be populated with
  an event that informs us that we've received new data. This allows us to wait
  until a new packet is received. Then we can then process the incoming data! */
QueueHandle_t queue;

/* The last few main variables that we need allow us to log data to the Serial
  Monitor. In this sketch, we want to log some information about the DMX we've
  received once every second. We'll declare a timer variable that will keep track
  of the amount of time that has elapsed since we last logged a serial message.
  We'll also want a flag that tracks if we are connected to the DMX controller so
  that we can log when we connect and when we disconnect. */
unsigned int timer = 0;
bool dmxIsConnected = false;

/* These variables track if we are receiving ArtNet packets, if so ignore DMX In
   otherwise forward DMX packets from input to DMX Out port. */
bool artNetIsConnected = false;
unsigned int lastArtNetPacket = 0;


#define UniverseLabel         "ArtNet Universe"
int myUniverse = 1;
ESPAsync_WMParameter p_UniverseID(UniverseLabel, "ArtNet Universe", "1", 1);


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  // set-up serial for debug output
  Serial.begin(115200); while (!Serial); delay(200);
  Serial.print("\nStarting ConnoDMX Gateway on " + String(ARDUINO_BOARD) + " WIFI Manager: ");
  Serial.println(ESP_ASYNC_WIFIMANAGER_VERSION);

  initWifi();     // Setup WIFI
  initOTA();      // Setup OTA
  initDMXIn();    // Setup DMX port to use UART1
  initDMXOut();   // Setup DMX port to use UART2
  initArtNet();   // Setup ART NET
}

void initOTA() {
  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(ssid.c_str());

  // Setup authentication for OTA
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("OTA Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nOTA Update Complete");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready");
}


// connect to wifi – returns true if successful or false if not
void initWifi(void)
{
  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
  if (drd->detectDoubleReset()) {
    Serial.println(F("Double Reset Detected"));
    initialConfig = true;
  }
#if ( USING_ESP32_S2 || USING_ESP32_C3 )
  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, NULL, DEVICE_NAME);
#else
  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer, DEVICE_NAME);
#endif
  if (ESPAsync_wifiManager.WiFi_SSID() == "") {
    Serial.println(F("No AP credentials"));
    initialConfig = true;
  }

  // Add custom configurable ArtNet Universe Number
  ESPAsync_wifiManager.addParameter(&p_UniverseID);

  if (initialConfig) {
    Serial.println(F("Starting Config Portal")); digitalWrite(PIN_LED, HIGH);
    if (!ESPAsync_wifiManager.startConfigPortal(ssid.c_str(), WIFI_PASSWORD)) {
      Serial.println(F("Not connected to WiFi"));
    }
    else {
      Serial.println(F("connected"));
    }
  }
  else {
    WiFi.mode(WIFI_STA);
    WiFi.begin();
  }
  digitalWrite(PIN_LED, LOW);

  unsigned long startedAt = millis();

  Serial.print(F("After waiting "));

  int connRes = WiFi.waitForConnectResult();
  float waited = (millis() - startedAt);

  Serial.print(waited / 1000); Serial.print(F(" secs , Connection result is ")); Serial.println(connRes);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Failed to connect"));
  }
  else {
    Serial.print(F("Local IP: "));
    Serial.println(WiFi.localIP());
  }

  Serial.printf("Configured Universe ID: %d \n", atoi(p_UniverseID.getValue()));
  myUniverse = atoi(p_UniverseID.getValue());
}

void initDMXIn() {
  /* Configure the DMX hardware to the default DMX settings and tell the DMX
    driver which hardware pins we are using. */
  dmx_config_t dmxInConfig = DMX_DEFAULT_CONFIG;
  dmx_param_config(dmxInPort, &dmxInConfig);
  dmx_set_pin(dmxInPort, dmxInTransmitPin, dmxInReceivePin, dmxInEnablePin);

  /* Now we can install the DMX driver! We'll tell it which DMX port to use and
    how big our DMX packet is expected to be. Typically, we'd pass it a handle
    to a queue, but since we are only transmitting DMX, we don't need a queue.
    We can write `NULL` where we'd normally put our queue handle. We'll also
    pass some interrupt priority information. The interrupt priority can be set
    to 1. */
  int queueSize = 1;
  int interruptPriority = 1;
  dmx_driver_install(dmxInPort, DMX_MAX_PACKET_SIZE, queueSize, &queue, interruptPriority);
}

void initDMXOut() {
  /* Configure the DMX hardware to the default DMX settings and tell the DMX
    driver which hardware pins we are using. */
  dmx_config_t dmxOutConfig = DMX_DEFAULT_CONFIG;
  dmx_param_config(dmxOutPort, &dmxOutConfig);
  dmx_set_pin(dmxOutPort, dmxOutTransmitPin, dmxOutReceivePin, dmxOutEnablePin);

  /* Now we can install the DMX driver! We'll tell it which DMX port to use and
    how big our DMX packet is expected to be. Typically, we'd pass it a handle
    to a queue, but since we are only transmitting DMX, we don't need a queue.
    We can write `NULL` where we'd normally put our queue handle. We'll also
    pass some interrupt priority information. The interrupt priority can be set
    to 1. */
  int queueSize = 0;
  int interruptPriority = 1;
  dmx_driver_install(dmxOutPort, DMX_MAX_PACKET_SIZE, queueSize, NULL, interruptPriority);

  /* Finally, since we are transmitting DMX, we should tell the DMX driver that
    we are transmitting, not receiving. We should also set our DMX start code
    to 0.*/
  dmx_set_mode(dmxOutPort, DMX_MODE_TX);
  dataOut[0] = 0;
}

void initArtNet() {
  // This will be called for each packet received
  artnet.setArtDmxCallback(onArtNetFrame);
  artnet.begin();
}


void onArtNetFrame(uint16_t universe, uint16_t numberOfChannels, uint8_t sequence, uint8_t* dmxData)
{
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState);

  for (int i = 1; i < numberOfChannels && i < DMX_MAX_PACKET_SIZE; ++i) {
    dataOut[i] = dmxData[i - 1];
  }

  Serial.printf("<< ArtNet Uni %u Seq %03u Chans %u => DMX: %03d - %03d %03d %03d %03d - %03d %03d %03d %03d \n", universe, sequence, numberOfChannels, dataOut[0], dataOut[1], dataOut[2], dataOut[3], dataOut[4], dataOut[5], dataOut[6], dataOut[7], dataOut[8]);

  // Only transmit our configured Universe
  if (myUniverse != universe) return;

  /* If this is the first ArtNet data we've received, lets log it! */
  lastArtNetPacket = millis();

  if (!artNetIsConnected) {
    Serial.println("ArtNet Connected and getting packets for this Universe!");
    artNetIsConnected = true;
  }

  dmx_write_packet(dmxOutPort, dataOut, DMX_MAX_PACKET_SIZE);
  dmx_tx_packet(dmxOutPort);
  dmx_wait_tx_done(dmxOutPort, DMX_TX_PACKET_TOUT_TICK);
}

void dmxInLoop() {
  /* We need a place to store information about the DMX packet we receive. We
    will use a dmx_event_t to store that packet information.  */
  dmx_event_t packet;

  /* And now we wait! The DMX standard defines the amount of time until DMX
    officially times out. That amount of time is converted into ESP32 clock ticks
    using the constant `DMX_RX_PACKET_TOUT_TICK`. If it takes longer than that
    amount of time to receive data, this if statement will evaluate to false. */
  if (xQueueReceive(queue, &packet, DMX_RX_TIMEOUT)) {

    /* If this code gets called, it means we've recieved DMX data! */

    /* We should check to make sure that there weren't any DMX errors. */
    if (packet.status == DMX_OK) {

      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);

      /* If this is the first DMX data we've received, lets log it! */
      if (!dmxIsConnected) {
        Serial.println("DMX In Connected!");
        dmxIsConnected = true;
      }

      /* We can read the DMX data into our buffer and increment our timer. */
      dmx_read_packet(dmxInPort, dataIn, packet.size);


      // Forward DMX IN to DMX OUT when no ArtNet packets
      if (!artNetIsConnected) {
        dmx_write_packet(dmxOutPort, dataIn, DMX_MAX_PACKET_SIZE);
        dmx_tx_packet(dmxOutPort);
        dmx_wait_tx_done(dmxOutPort, DMX_TX_PACKET_TOUT_TICK);
        Serial.printf(">> DMX In => Forward: %03d - %03d %03d %03d %03d - %03d %03d %03d %03d \n", dataIn[0], dataIn[1], dataIn[2], dataIn[3], dataIn[4], dataIn[5], dataIn[6], dataIn[7], dataIn[8]);
      } else {
        Serial.printf(">> DMX In => Ignored: %03d - %03d %03d %03d %03d - %03d %03d %03d %03d \n", dataIn[0], dataIn[1], dataIn[2], dataIn[3], dataIn[4], dataIn[5], dataIn[6], dataIn[7], dataIn[8]);
      }

    } else {
      /* Uh-oh! Something went wrong receiving DMX. */
      Serial.println("DMX In Error!");
    }
  }
}

void artNetCheck() {
  if (artNetIsConnected) {
    if (lastArtNetPacket + ART_NET_TIMEOUT_MS <= millis()) {
      artNetIsConnected = false;
      Serial.println("No ArtNet packets within timeout, setting to disconnected!");
    }
  }
}

void loop()
{
  artnet.read();
  artNetCheck();
  dmxInLoop();
}
