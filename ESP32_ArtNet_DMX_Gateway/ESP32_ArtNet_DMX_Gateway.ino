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
#define USE_SPIFFS                true
#define ESP_DRD_USE_EEPROM        true
#define USE_ESP_WIFIMANAGER_NTP   false
#define DRD_TIMEOUT               10
#define DRD_ADDRESS               0
#define LED_BUILTIN               2

#include "ArtnetWifi.h"
#include <esp_dmx.h>
#include <Arduino.h>
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
int transmitPin = 17;
int receivePin = 16;
int enablePin = 21;

/* Next, lets decide which DMX port to use. The ESP32 has either 2 or 3 ports.
  Port 0 is typically used to transmit serial data back to your Serial Monitor,
  so we shouldn't use that port. Lets use port 2! */
dmx_port_t dmxPort = 2;

/* Now we want somewhere to store our DMX data. Since a single packet of DMX
  data can be up to 513 bytes long, we want our array to be at least that long.
  This library knows that the max DMX packet size is 513, so we can fill in the
  array size with `DMX_MAX_PACKET_SIZE`. */
byte data[DMX_MAX_PACKET_SIZE];

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

  initWifi();   // Setup WIFI
  initDMX();    // Setup DMX port to use UART2
  initArtNet(); // Setup ART NET
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
    if (!ESPAsync_wifiManager.startConfigPortal(ssid.c_str(), password.c_str())) {
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

void initDMX() {
  /* Configure the DMX hardware to the default DMX settings and tell the DMX
    driver which hardware pins we are using. */
  dmx_config_t dmxConfig = DMX_DEFAULT_CONFIG;
  dmx_param_config(dmxPort, &dmxConfig);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  /* Now we can install the DMX driver! We'll tell it which DMX port to use and
    how big our DMX packet is expected to be. Typically, we'd pass it a handle
    to a queue, but since we are only transmitting DMX, we don't need a queue.
    We can write `NULL` where we'd normally put our queue handle. We'll also
    pass some interrupt priority information. The interrupt priority can be set
    to 1. */
  int queueSize = 0;
  int interruptPriority = 1;
  dmx_driver_install(dmxPort, DMX_MAX_PACKET_SIZE, queueSize, NULL, interruptPriority);

  /* Finally, since we are transmitting DMX, we should tell the DMX driver that
    we are transmitting, not receiving. We should also set our DMX start code
    to 0.*/
  dmx_set_mode(dmxPort, DMX_MODE_TX);
  data[0] = 0;
}

void initArtNet() {
  // This will be called for each packet received
  artnet.setArtDmxCallback(onDmxFrame);
  artnet.begin();
}


void onDmxFrame(uint16_t universe, uint16_t numberOfChannels, uint8_t sequence, uint8_t* dmxData)
{
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState);

  for (int i = 1; i < numberOfChannels && i < DMX_MAX_PACKET_SIZE; ++i) {
    data[i] = dmxData[i - 1];
  }

  Serial.printf("ArtNet Uni %u Seq %03u Chans %u => DMX: %03d - %03d %03d %03d %03d - %03d %03d %03d %03d \n", universe, sequence, numberOfChannels, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);

  // Only transmit our configured Universe
  if (myUniverse != universe) return;
  
  dmx_write_packet(dmxPort, data, DMX_MAX_PACKET_SIZE);
  dmx_tx_packet(dmxPort);
  dmx_wait_tx_done(dmxPort, DMX_TX_PACKET_TOUT_TICK);
}

void loop()
{
  artnet.read();
}
