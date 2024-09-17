/* Copyright (c) 2021 Neil McKechnie
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/*
 * HttpManager class definition - methods and static data.
 * 
 * Encapsulates the WiFI and Web Server functionality of the DCC inspector program.
 * 
 */

#include <Arduino.h>
extern byte strictMode;
extern bool filterInput;

#include "Config.h"

#ifdef USE_TELNETSERVER

#include "TelnetManager.h"

#include "OledDisplay.h"
#include "DCCStatistics.h"

#if defined(ESP32) 
  #define PLATFORM "ESP32"
#elif defined(ESP8266)
  #define PLATFORM "ESP8266"
#endif

// Buffer pointer reference.  This is the buffer of dynamic text sent to the web server client.
char *TelnetManagerClass::bufferPointer = 0;

// Web server object.
WiFiServer TelnetManagerClass::server(23);  //default port
// Web server object.
WiFiClient TelnetManagerClass::serverClient; 

static IPAddress telNetIp(192, 168, 2, 25);   // "D"
static IPAddress telNetIpMask(255, 255, 255, 0);
static IPAddress telNetIpGateWay(192, 168, 2, 0);
static IPAddress telNetIpDNS1(192, 168, 2, 0);
static IPAddress telNetIpDNS2(192, 168, 2, 0);

// Function to initialise the object.  It connects to the WiFi access point, 
//  registers an mDNS name (e.g. ESP.local) and sets up the web server.
//  By preference, it will connect using the same SSID and password as last time.
//  If this fails, it tries WPS.  If that fails, it tries the configured SSID/Password.
//  If nothing works, the WiFi/HTTP capabilities are disabled.
bool TelnetManagerClass::begin(const char *ssid, const char *password, const char *dnsName) {
  // Configuration for WiFi wps mode.
  #if defined(ESP32)
  esp_wps_config_t config;
  memset(&config, 0, sizeof(config));
  config.crypto_funcs = &g_wifi_default_wps_crypto_funcs;
  config.wps_type = WPS_TYPE_PBC;

  WiFi.onEvent (WiFiEvent);
  #endif
  Serial.print("Connecting as wifi client to SSID: ");
  Serial.println(ssid);

  // use in case of mode problem
  WiFi.disconnect();
  // switch to Station mode
  if (WiFi.getMode() != WIFI_STA) {
    WiFi.mode(WIFI_STA);
  }

  WiFi.config(telNetIp, telNetIpGateWay, telNetIpMask);
  WiFi.begin ();

  // ... Give ESP 10 seconds to connect to station.
  unsigned long startTime = millis();
  boolean blinkOn = true;
  #if defined(LEDPIN_DECODING)
    digitalWrite(LEDPIN_DECODING,HIGH);
  #endif

  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    delay(500);
    blinkOn= !blinkOn;
    #if defined(LEDPIN_DECODING)
    if (blinkOn ) {
      digitalWrite(LEDPIN_DECODING, HIGH);
    } else {
      digitalWrite(LEDPIN_DECODING, LOW);
    }
    #endif
    Serial.print(".");
  }
  Serial.println("");
  // Check connection
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected; IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.print("WiFi connect failed to ssid: ");
    Serial.println(ssid);
    Serial.print("WiFi password <");
    Serial.print(password);
    Serial.println(">");
    Serial.println("Check for wrong typing!");
  }
  #if defined(ESP32)
    esp_wifi_wps_disable();
  #endif

  connected = true;
  
  Serial.println();
  Serial.print(F("Connected to \""));
  Serial.print(WiFi.SSID());
  Serial.print(F("\", IP address: "));
  Serial.println(WiFi.localIP());
  #if defined(USE_OLED)
  OledDisplay.append("Connected!");
  #endif
  #if defined(ESP32)
    Serial.println("ESP32"); 
  #endif
  #if defined(ESP8266)
    Serial.println("ESP8266"); 
  #endif
  Serial.println(INPUTPIN);
  if (MDNS.begin(dnsName)) {
    Serial.println(F("MDNS responder started"));
  }
  
  server.begin(19021);
  Serial.println(F("TelnetServer server started"));

  return true;
}

// Function to set the pointer to dynamic data to be sent to the
//  web client on request.
void TelnetManagerClass::setBuffer(char *bp) {
    bufferPointer = bp;
    Serial.println("Called");
    Serial.println(bufferPointer);
    Serial.println("Sent");
    if (serverClient && serverClient.connected()) {
        serverClient.println(bufferPointer);
    }
}

// Function to be called from the loop() function to do all the
//  regular processing related to the class.
void TelnetManagerClass::process() {
    if (connected && server.hasClient()) {
      if (!serverClient || !serverClient.connected()) {
        if (serverClient) {
          serverClient.stop();
          Serial.println("Telnet Client Stop");
        }
        serverClient = server.available();
        Serial.println("New Telnet client");
        serverClient.flush();  // clear input buffer, else you get strange characters 
      }
    }
}

//=======================================================================
// Process commands sent over the telnet connection.
//  Return false if nothing done.

int TelnetManagerClass::getCommands() {
  if (connected && serverClient.available()) {
    return serverClient.read();
  } else
    return -1;
}

// Function to display the IP Address of the server if connected
void TelnetManagerClass::getIP() {
  if (connected){
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("Not Connected!"); 
  }
}

// Declare singleton class instance.
TelnetManagerClass TelnetManager;

#endif
