#include <ESP8266WiFi.h>
#include <QuickEspNow.h>
#include <espnow.h>
#include <elapsedMillis.h>
#include <Crypto.h>
#include <Ed25519.h>
#include <MyCreds.h> // Contains DOOR_PRIKEY, DOOR_PUBKEY, DOOR_NAME (not in GIT...)
#include <EEPROM.h>

#include "user_interface.h"
#include "include/WiFiState.h"

#ifndef DOOR_PRIKEY
#define DOOR_PRIKEY "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff"
#define DOOR_PUBKEY "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff"
#define DOOR_NAME   "Name for Key UTF8"
#endif

#define ESPNOW_CHANNEL 13
#define DEST_ADDR ESPNOW_BROADCAST_ADDRESS 
#define ESPNOW_TX_POWER 10.0 /* dBm */

// Debug prints on Serial activ if defined
//#define ESPNOW_TESTMODE 

// Send something every 5s
//#define ESPNOW_AUTOSEND

volatile int mainState = 0;

uint8_t door_prikey[32];
uint8_t door_pubkey[32];
uint8_t door_name[32];
uint8_t lastChallenge[8];

uint8_t rand_data[32];

ADC_MODE(ADC_VCC); 
uint16_t vcc = 0;

void blinkLED(int cnt, int ms) {
  for(int i=0; i<cnt; i++) {
    digitalWrite(2, LOW); // LED on
    delay(ms);
    digitalWrite(2, HIGH);
    delay(ms);
  }
} 

uint8_t hex1(const char c) {
  uint8_t val = 255;
  if((c >= 'A') && (c <= 'F')) val = c - 'A' + 10;
  if((c >= 'a') && (c <= 'f')) val = c - 'a' + 10;
  if((c >= '0') && (c <= '9')) val = c - '0';
  return val;
}

size_t hex2bytes(const char *hexString, uint8_t *byteArray, size_t byteArrayLen) {
  size_t byteCount = 0;

  while(byteCount < byteArrayLen) {
    uint8_t h = 0;
    uint8_t l = 0;
    char c;
    while((h = hex1(c = *hexString++)) == 255) {
      if(c == 0) break;
    }
    if(c == 0) break;
    while((l = hex1(c = *hexString++)) == 255) {
      if(c == 0) break;
    }
    if(c == 0) break;    
    *byteArray++ = h*16+l;
    byteCount++;
  }

  return byteCount;
}

bool loadKey(const char *priKey, const char *pubKey, const char *name) {
  size_t len = 0;
  len = hex2bytes(priKey, door_prikey, 32);
  if(len != 32) return false;
  len = hex2bytes(pubKey, door_pubkey, 32);
  if(len != 32) return false;  
  memset(door_name, 0, 32);
  strlcpy((char *)door_name, name, 31);
  uint8_t temp_pubkey[32];
  //Ed25519::derivePublicKey(temp_pubkey, door_prikey);
  //if(memcmp(temp_pubkey, door_pubkey, 32) != 0) return false;
  return true;
}  

/**
 * Protocol Tx:
 * Off Len
 *   0   4  Preemble "D00r"
 *   4  64  Signature
 *  68  32  Public Key
 * 100  32  Name (Cleartext UTF8, 0 filled)
 * 132   8  Challenge respone
 * 140   1  Command (might be longer than 1 if needed)   
 * 
 */
bool txCommand(const char *cmd) {
  uint8_t msg[4+64+32+32+8+8];
  int cmdlen = strlen(cmd);
  if(cmdlen > 8) cmdlen = 8;
  msg[0] = 'D'; msg[1] = '0'; msg[2] = '0'; msg[3] = 'r';
  memcpy(&msg[68], door_pubkey, 32);
  memcpy(&msg[100], door_name, 32);
  memcpy(&msg[132], lastChallenge, 8);
  memcpy(&msg[140], cmd, cmdlen+1);
  Ed25519::sign(&msg[4], door_prikey, door_pubkey, &msg[68], 32+32+8+cmdlen+1);
//  WiFi.forceSleepWake();
  return(quickEspNow.send(DEST_ADDR, msg, 4+64+32+32+8+cmdlen+1) == 0);
}

/**
 * Protocol Rx:
 * Off Len
 *   0   4  Preemble "D00a" (for answer)
 *   4  32  Public Key from Rx
 *  36   8  New challenge
 *  44   1  Answer (might be longer than 1 if needed)
 * 
 */ 
void dataReceived (uint8_t* address, uint8_t* data, uint8_t len, signed int rssi, bool broadcast) {
  #ifdef ESPNOW_TESTMODE
    Serial.print ("Received: ");
    Serial.printf ("%.*s\n", len, data);
    Serial.printf ("RSSI: %d dBm\n", rssi);
    Serial.printf ("From: " MACSTR "\n", MAC2STR (address));
    Serial.printf ("%s\n", broadcast ? "Broadcast" : "Unicast");
  #endif
  // Check if it is valid Door protocol
  if(len >= (4+32+8+1)) {
    if(((data[0] == 'D') && (data[1] == '0') && (data[2] == '0') && (data[3] == 'a'))) {
      if(memcmp(&data[4], door_pubkey, 32) == 0) {
        char cmd = data[44];
        memcpy(lastChallenge, &data[36], 8);

        switch (mainState)
        {
          case 1: // challenge requested, expect now 'c'
            if(cmd == 'c') mainState++; else mainState = 100;
            break;

          case 3: // trigger requested, expect now 't'
            if(cmd == 't') mainState++; else mainState = 100;
            break;
          
          default:
            break;
        }
        Serial.printf("RSSI: %d dBm, RxCmd: %.*s\n", rssi, len-44, &data[44]);
      }  
    }
  }

}

uint32_t timestamp = 0;
elapsedMillis activeTime;

void setup() {
  pinMode(2, OUTPUT);
  pinMode(0, INPUT_PULLUP);  
  digitalWrite(2, HIGH);
  Serial.begin(74880);   //  GPIO1 (TX) and GPIO3 (RX)
  #ifdef ESPNOW_TESTMODE
  Serial.setDebugOutput(true);
  #endif

  EEPROM.begin(512);
  //WiFi.forceSleepBegin(1000000UL);
  vcc = system_get_vdd33();

  //Serial.printf("SDK version: %s \n", system_get_sdk_version());
  Serial.printf("VCC %d\r\n", vcc);

  int ret = os_get_random((unsigned char *)rand_data, 32);
  Serial.printf("Rand ret %d, %02x %02x %02x %02x\r\n", ret, 
    (int)rand_data[0], (int)rand_data[1], (int)rand_data[30], (int)rand_data[31]);

  //Serial1.begin(115200); //  GPIO2 TX1 (output only, connected to LED)
  //WiFi.disconnect();
	//ESP.eraseConfig();
	//delay(3000);
  Serial.printf("MS: %d\n", (int)millis());
  if(loadKey(DOOR_PRIKEY, DOOR_PUBKEY, DOOR_NAME) == false) {
    Serial.println("loadKey failed!");
    blinkLED(5, 200);
  } else {
    Serial.println("\n\nloadKey ok.");
  }  

  WiFi.mode(WIFI_STA);
  //wifi_promiscuous_enable(true); // this kills broadcast receive capability!
  //wifi_set_channel(ESPNOW_CHANNEL);
  //wifi_promiscuous_enable(false); 
  WiFi.disconnect (false);

  quickEspNow.onDataRcvd(dataReceived);
  // wifi_set_phy_mode();
  // esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_11M_L /* WIFI_PHY_RATE_LORA_250K */ );

  quickEspNow.begin(ESPNOW_CHANNEL); // 
  #ifdef ESPNOW_TESTMODE
    Serial.printf ("Connected to %s in channel %d\n", WiFi.SSID ().c_str (), WiFi.channel ());
    Serial.printf ("IP address: %s\n", WiFi.localIP ().toString ().c_str ());
    Serial.printf ("MAC address: %s\n", WiFi.macAddress ().c_str ());
    Serial.println(system_get_sdk_version());
  #endif
  
  system_phy_set_max_tpw(40); // 10mW/10dBm (0.25dBm x X, 0..82)
  wifi_set_phy_mode(PHY_MODE_11N);
  
  //WiFi.setOutputPower(ESPNOW_TX_POWER);
  wifi_set_user_fixed_rate(FIXED_RATE_MASK_ALL, PHY_RATE_54);

  activeTime = 0;

  //WiFi.forceSleepBegin(1000000UL);
  Serial.printf("MS0: %d (Trig via reset) %d\n", (int)(millis() - timestamp), wifi_get_phy_mode());
  if(txCommand("c")) mainState = 1;
  Serial.printf("MS1: %d\n", (int)(millis() - timestamp));
  wifi_set_sleep_type(MODEM_SLEEP_T);
  os_delay_us(100);
  wifi_set_sleep_type(NONE_SLEEP_T);
}

char msg[256];
bool butTrig = false;

void loop() {

  if(mainState == 2) {
    Serial.printf("MS2: %d\n", (int)(millis() - timestamp));
    if(txCommand("t")) mainState = 3;
    Serial.printf("MS3: %d\n", (int)(millis() - timestamp));
  }

  if(mainState == 4) {
    Serial.printf("MS4: %d\n", (int)(millis() - timestamp));
    Serial.println("Trigger done!\n");
    blinkLED(1, 200);
    mainState = 10;
  }  

  if((digitalRead(0) == LOW) || (butTrig)) {
    butTrig = false;
    timestamp = millis(); activeTime = 0;
    if(txCommand("c")) mainState = 1;
    Serial.printf("MS1: %d (Trig via but)\n", (int)(millis() - timestamp));
    delay(10);
    while(digitalRead(0) == LOW) {}
    delay(10);
  }  

  // put your main code here, to run repeatedly:
  if(Serial.available()) {
    char b = Serial.read();
    if(b == 't') {
      butTrig = true;
    } else {
      sprintf(msg,"%c", b);
      if (txCommand(msg)) {
          Serial.println (">>>>>>>>>> Message sent");
      } else {
          Serial.println (">>>>>>>>>> Message not sent");
      }
    }
  }  
  
  if(activeTime > 3000) {
    activeTime = 0;
    if((mainState != 10) && (mainState != 101)) {
      Serial.println("Timeout!");
      blinkLED(3, 100);
      mainState = 101;
    } else {
      ESP.deepSleep(0);
    }
    //WiFi.forceSleepBegin(1000000UL);
  }

  delay(1);
}

