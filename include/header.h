//Bibliotecas Arduino
#include <Arduino.h>

//Bibliotecas Wifi
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>
#include <WebServer.h>
#else
#error Invalid platform
#endif
#include <DNSServer.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

//Bibliotecas Time
#include <Time.h>
#include <TimeLib.h>

//Bibliotecas MQTT
#include <PubSubClient.h>

//Bibliotecas GERAIS
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <Wire.h>
#include "Adafruit_HTU21DF.h"
