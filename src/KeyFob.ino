#include <ESP8266WiFi.h>
#include <QuickEspNow.h>
#include <elapsedMillis.h>

#define ESPNOW_CHANNEL 13
#define ESPNOW_PREFIX "D00"

// Commands: ? will answer via ESPNOW-Broadcast

#define DEST_ADDR ESPNOW_BROADCAST_ADDRESS 

// Debug prints on Serial activ if defined
#define ESPNOW_TESTMODE 

// Send something every 5s
//#define ESPNOW_AUTOSEND

const char espNowPrefix[] = ESPNOW_PREFIX;
int espNowPrefixLen;
volatile bool doAnswer = false;

void dataReceived (uint8_t* address, uint8_t* data, uint8_t len, signed int rssi, bool broadcast) {
  #ifdef ESPNOW_TESTMODE
    Serial.print ("Received: ");
    Serial.printf ("%.*s\n", len, data);
    Serial.printf ("RSSI: %d dBm\n", rssi);
    Serial.printf ("From: " MACSTR "\n", MAC2STR (address));
    Serial.printf ("%s\n", broadcast ? "Broadcast" : "Unicast");
    Serial.printf ("Prefixlen %d\n", espNowPrefixLen);
  #endif
  if(len > espNowPrefixLen) {
    if(strncmp((const char *)data, espNowPrefix, espNowPrefixLen) == 0) {
      data[len] = 0; // termination of string
      Serial1.print((const char *)&data[espNowPrefixLen]);
      Serial.printf("Data:%s\n", (const char *)&data[espNowPrefixLen]);
      if(data[espNowPrefixLen] == '?') doAnswer = true;
    }
  }
}

void setup() {
  espNowPrefixLen = strlen(espNowPrefix);
  Serial.begin(74880);   //  GPIO1 (TX) and GPIO3 (RX)
  Serial.setDebugOutput(true);
  pinMode(2, OUTPUT);
  //pinMode(3, INPUT_PULLUP);
  Serial1.begin(115200); //  GPIO2 TX1 (output only, connected to LED)
  //WiFi.disconnect();
	//ESP.eraseConfig();
	//delay(3000);
  WiFi.mode(WIFI_STA);
  //wifi_promiscuous_enable(true); // this kills broadcast receive capability!
  //wifi_set_channel(ESPNOW_CHANNEL);
  //wifi_promiscuous_enable(false); 
  WiFi.disconnect (false);
  
  quickEspNow.begin(ESPNOW_CHANNEL); // 
  #ifdef ESPNOW_TESTMODE
    Serial.printf ("Connected to %s in channel %d\n", WiFi.SSID ().c_str (), WiFi.channel ());
    Serial.printf ("IP address: %s\n", WiFi.localIP ().toString ().c_str ());
    Serial.printf ("MAC address: %s\n", WiFi.macAddress ().c_str ());
    Serial.println(system_get_sdk_version());
  #endif
  quickEspNow.onDataRcvd(dataReceived);
}

char msg[256];
void loop() {
  static elapsedMillis bcTx = 0;
  static int cnt = 0;

  // Testbroadcast every 5s
  if(bcTx > 5000) {
    bcTx = 0;
    sprintf(msg,"D00g%d", cnt++);

    #ifdef ESPNOW_AUTOSEND
    if (!quickEspNow.send (DEST_ADDR, (uint8_t*)msg, strlen(msg))) {
        Serial.println (">>>>>>>>>> Message sent");
    } else {
        Serial.println (">>>>>>>>>> Message not sent");
    }
    #endif
  }

  if(doAnswer) {
    doAnswer = false;
    sprintf(msg,"KeyFob V1.0, Uptime: %lu", millis());

    if (!quickEspNow.send (DEST_ADDR, (uint8_t*)msg, strlen(msg))) {
        Serial.println (">>>>>>>>>> Answer Message sent");
    } else {
        Serial.println (">>>>>>>>>> Answer Message not sent");
    }
  }

  // put your main code here, to run repeatedly:
  if(Serial.available()) {
    char b = Serial.read();

    sprintf(msg,"D00g%c", b);

    if (!quickEspNow.send (DEST_ADDR, (uint8_t*)msg, strlen(msg))) {
        Serial.println (">>>>>>>>>> Message sent");
    } else {
        Serial.println (">>>>>>>>>> Message not sent");
    }
  }  
  delay(1);
}

