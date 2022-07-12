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

//PINOS
#define gpioBtnWm 0
#define gpioOut1 4
#define gpioOut2 5
#define gpioDht1 5
#define gpioDht2 5

//DEVICE SETUP
char deviceName[] = "ModuloCozinha";
char deviceId[] = "ModuloCozinha";
unsigned long currentMillis;

//DEBUG
bool debugSerial = true;

//SENSOR TEMPERATURA UMIDADE
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
float sensorTemperature;
float sensorHumidity;
unsigned long sensorHTU21DOk = 0;
unsigned long sensorHTU21DPreviousMillis = 0;
int sensorHTU21DInterval = 20000;

//SAIDAS
bool out1 = false;
bool out2 = false;

//MQTT
char mqttServer[] = "192.168.0.200";
char mqttPort[] = "1883";
char mqttUser[] = "admin";
char mqttPassword[] = "vine3561";
char mqttStatTopicSfx[] = "/stat";
char mqttCmdTopicSfx[] = "/cmnd";
char mqttLwtTopicSfx[] = "/tele";
char mqttStatTopic[] = "device_name/stat_sfx";
char mqttCmdTopic[] = "device_name/cmnd_sfx";
char mqttLwtTopic[] = "device_name/lwt_sfx";
int mqttReconnectInterval = 300000;
unsigned long mqttReconnectPreviousMillis = 0;
int mqttStatInterval = 3000;
unsigned long mqttStatPreviousMillis = 0;
WiFiClient espClient;
PubSubClient mqtt(espClient);

//NTP SERVER
char ntpServer[] = "a.st1.ntp.br";
int ntpTimeZone = -3;     // UTC
unsigned int localPort = 8888;
WiFiUDP UDP;

//WIFIMANAGER
WiFiManager wm;
bool deviceConnectedFistScan = true;
WiFiManagerParameter custom_deviceName("deviceName", "Device Name", deviceName, 40);
WiFiManagerParameter custom_mqttServer("mqttServer", "Mqtt server", mqttServer, 40);
WiFiManagerParameter custom_mqttPort("mqttPort", "Mqtt port", mqttPort, 6);
WiFiManagerParameter custom_mqttUser("mqttUser", "Mqtt User", mqttUser, 20);
WiFiManagerParameter custom_mqttPassword("mqttPassword", "Mqtt Password", mqttPassword, 20);
WiFiManagerParameter p_hint("<p>*Dica: Para continuar usando o WIFI configurado, deixe SSID e senha em branco.</p>");
WiFiManagerParameter p_deviceName("<h2>Nome do Dispositivo</h2>");
WiFiManagerParameter p_mqtt("<h2>Mqtt</h2>");

//---DEBUG---
void debug(bool linha, String mensagem) {    //Função para imprimir na porta serial
  if (debugSerial) {
    if (linha) {
      Serial.println(mensagem);
    } else {
      Serial.print(mensagem);
    }
  }
}

//--- NTP---
const int ntpPacketSize = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[ntpPacketSize]; //buffer to hold incoming & outgoing packets
// send an NTP request to the time server at the given address
void ntpSendPacket(IPAddress &address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, ntpPacketSize);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  UDP.beginPacket(address, 123); //NTP requests are to port 123
  UDP.write(packetBuffer, ntpPacketSize);
  UDP.endPacket();
}

//--- NTP - GET TIME---
time_t ntpGetTime() {
  IPAddress timeServerIP; // time.nist.gov NTP server address

  while (UDP.parsePacket() > 0) ; // discard any previously received packets
  debug(false, "*NTP -  ");
  //get a random server from the pool
  WiFi.hostByName(ntpServer, timeServerIP);
  //imprimirSerial(true, String(timeServerIP));

  ntpSendPacket(timeServerIP);
  uint32_t beginWait = millis();
  while ((millis() - beginWait) < 1500) {
    int size = UDP.parsePacket();
    if (size >= ntpPacketSize) {
      debug(true, "OK");
      UDP.read(packetBuffer, ntpPacketSize);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + (ntpTimeZone * SECS_PER_HOUR);
    }
  }
  debug(true, "ERRO");
  return 0; // return 0 if unable to get the time
}

//--- INIT NTP---
void ntpInit() {
  UDP.begin(localPort);
  setSyncProvider(ntpGetTime);
  setSyncInterval(600);
}

//---SENSOR HTU21D (Temp. e Umid)---
void sensorHTU21D() {
  if (currentMillis - sensorHTU21DPreviousMillis > sensorHTU21DInterval) {
    float temp = htu.readTemperature();
    float hum = htu.readHumidity();
    
    if (not(isnan(temp) || isnan(hum))) {
      sensorTemperature = temp;
      sensorHumidity = hum;
      debug(false, "*HTU21D - Temperatura: " + String(sensorTemperature) + " ºC - ");
      debug(true, "Umidade: " + String(sensorHumidity) + " %");    
    } else {
      debug(true, "*HTU21D - Falha ao ler HTU21D");
    }
    

    sensorHTU21DPreviousMillis = currentMillis;
  }
}

//---COMANDOS---
void cmdOut(int out, bool value) {
  switch (out) {
    case 1:
      if (value) {
        digitalWrite(gpioOut1, HIGH);
        debug(true, "*OUT 1 - ON");
      } else {
        digitalWrite(gpioOut1, LOW);
        debug(true, "*OUT 1 - OFF");
      }
      break;
    case 2:
      if (value) {
        digitalWrite(gpioOut2, HIGH);
        debug(true, "*OUT 2 - ON");
      } else {
        digitalWrite(gpioOut2, LOW);
        debug(true, "*OUT 2 - OFF");
      }
      break;
   }  
}

//---mqtt---
void mqttReconnect() {
  String clientId = String(deviceName) + String(random(0xffff), HEX);
  if (mqtt.connect(clientId.c_str(), mqttUser, mqttPassword)) {   
    //availability_topic
    sprintf(mqttLwtTopic, "%s%s",deviceName, mqttLwtTopicSfx);
    debug(true, "*MQTT - LWT TOPIC: " + String(mqttLwtTopic));
    //stat_topic  
    sprintf(mqttStatTopic, "%s%s",deviceName, mqttStatTopicSfx);
    debug(true, "*MQTT - STAT TOPIC: " + String(mqttStatTopic));
    //command_topic
    sprintf(mqttCmdTopic, "%s%s/#", deviceName, mqttCmdTopicSfx);
    debug(true, "*MQTT - CMD TOPIC: " + String(mqttCmdTopic));

    mqtt.publish(mqttLwtTopic, "Online", true);

    mqtt.subscribe(mqttCmdTopic);
    debug(true, "*MQTT - OK");
  } else {
    debug(false, "*MQTT - ERRO -> ");
    debug(true, String(mqtt.state()));
  }
  mqttReconnectPreviousMillis = currentMillis;
}

void mqttPublishStat() {
  // create a JSON object
  DynamicJsonDocument  doc(1024);
  
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  doc["DeviceName"] = String(deviceName);
  doc["DateTime"] = String(now());
  doc["Temperatua"] = String(sensorTemperature);
  doc["Umidade"] = String(sensorHumidity);
  doc["Saida1"] = out1;
  doc["Saida2"] = out2;

  char msg[1024];
  serializeJson(doc, msg);

  debug(false, "*MQTT - STAT TOPIC: " + String(mqttStatTopic));
  debug(false, " - MSG: " + String(msg));
    
  if (mqtt.publish(mqttStatTopic, msg)) {
    debug(true, " - OK");
  } else {
    debug(true, " - ERRRO");
  }

}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  payload[length] = 0;
  String inData = String((char *)payload);
  String inTopic = String(topic);
  
  debug(false, "*MQTT - Mensagem recebida");
  debug(false, " - CMD TOPIC: " + inTopic);
  debug(true, " - MSG: " + inData);

  if (inTopic == String(mqttCmdTopic) + "/out1/set") {
    if ((inData == "on") || (inData == "1") || (inData == "True")) {
      cmdOut(1, 2);
    } else if ((inData == "off") || (inData == "0") || (inData == "False")) {
      cmdOut(1, 0);
    }
  }

  mqttPublishStat();
}



//---WIFI MANAGER - PARAMETERS---
void wifiManageSaveParams() {
  strcpy(deviceName, custom_deviceName.getValue());
  strcpy(mqttServer, custom_mqttServer.getValue());
  strcpy(mqttPort, custom_mqttPort.getValue());
  strcpy(mqttUser, custom_mqttUser.getValue());
  strcpy(mqttPassword, custom_mqttPassword.getValue());
  
  debug(true, "*mqtt - Parametros salvos");
  debug(true, "*mqtt - Nome do Dispositivo: " + String(deviceName));
  debug(true, "*mqtt - Servidor: " + String(mqttServer));
  debug(true, "*mqtt - Porta: " + String(mqttPort));
  debug(true, "*mqtt - Usuário: " + String(mqttUser));

}

//---INIT WIFI MANAGER---
void wifiManageIinit() {
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

  //reset settings - wipe credentials for testing
  //wm.resetSettings();
  
  wm.addParameter(&custom_mqttServer);
  wm.addParameter(&p_hint);
  wm.addParameter(&p_deviceName);
  wm.addParameter(&custom_deviceName);
  wm.addParameter(&p_mqtt);
  wm.addParameter(&custom_mqttServer);
  wm.addParameter(&custom_mqttPort);
  wm.addParameter(&custom_mqttUser);
  wm.addParameter(&custom_mqttPassword);

  //wm.setConfigPortalTimeout(180);
  wm.setConfigPortalBlocking(false);
  wm.setSaveConfigCallback(wifiManageSaveParams);

  //automatically connect using saved credentials if they exist
  //If connection fails it starts an access point with the specified name
  if (wm.autoConnect(deviceName, "12345678")) {
    debug(true, "*WIFI - Conectado! :)");
  }
  else {
    debug(true, "*WIFI - Portal de configuração disponível.");
  }
}

//---CONECTADO---
void onConnected() {
  //FIRST SCAN
  if (deviceConnectedFistScan) {
    //NTP
    ntpInit();
    
    //MQTT
    mqtt.setServer(mqttServer, atoi(mqttPort));   //informa qual broker e porta deve ser conectado
    mqtt.setCallback(mqttCallback);
    mqttReconnect();
    deviceConnectedFistScan = false;
    debug(true, "*WIFI - FirtScan - OK");
  }


  //MQTT
  if (mqtt.connected()) {
    mqtt.loop();
    if (currentMillis - mqttStatPreviousMillis > mqttStatInterval) {
      mqttPublishStat();
      mqttStatPreviousMillis = currentMillis;
    }
  } else {
    if (currentMillis - mqttReconnectPreviousMillis > mqttReconnectInterval) {
      mqttReconnect();
    }
  }
  
}

//--- INIT GPIO---
void gpioInit() {
  pinMode(gpioBtnWm, INPUT);
  pinMode(gpioOut1, OUTPUT);
  pinMode(gpioOut2, OUTPUT);
  debug(true, "*GPIO - OK");
}

void setup() {
  if (debugSerial) {
    Serial.begin(115200);
  }

  debug(true, "***");
  debug(true, "***");
  debug(true, "***");

  gpioInit();

  //Inicia sensor HTU21D
  if (htu.begin()) {
    debug(true, "*HTU21D - OK");
    sensorHTU21DOk = 1;
  } else {
    debug(true, "*HTU21D - ERRO");
    sensorHTU21DOk = 0;
  }

  //Inicia WifiManager
  wifiManageIinit();

  debug(true, "*SETUP - OK");
}



void loop() {
  //Processo WifiManager
  wm.process();

  //Sensor HTU21D
  if (sensorHTU21DOk) {
    sensorHTU21D();
  }

  //Dispositivo conectado
  if (WiFi.status() == WL_CONNECTED){
    onConnected();
  }
  
  yield();
  currentMillis = millis();  
}











