#include <ESP8266WiFi.h>
#include <QuickEspNow.h>
#include <elapsedMillis.h>
#include <Crypto.h>
#include <Ed25519.h>
#include <MyCreds.h> // Contains DOOR_PRIKEY, DOOR_PUBKEY, DOOR_NAME (not in GIT...)

#ifndef DOOR_PRIKEY
#define DOOR_PRIKEY "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff"
#define DOOR_PUBKEY "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff"
#define DOOR_NAME   "Name for Key UTF8"
#endif

#define ESPNOW_CHANNEL 13

#define DEST_ADDR ESPNOW_BROADCAST_ADDRESS 

// Debug prints on Serial activ if defined
//#define ESPNOW_TESTMODE 

// Send something every 5s
//#define ESPNOW_AUTOSEND

volatile int mainState = 0;

uint8_t door_prikey[32];
uint8_t door_pubkey[32];
uint8_t door_name[32];
uint8_t lastChallenge[8];

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
  Ed25519::derivePublicKey(temp_pubkey, door_prikey);
  if(memcmp(temp_pubkey, door_pubkey, 32) != 0) return false;
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

void setup() {
  Serial.begin(74880);   //  GPIO1 (TX) and GPIO3 (RX)
  #ifdef ESPNOW_TESTMODE
  Serial.setDebugOutput(true);
  #endif
  pinMode(2, OUTPUT);
  pinMode(0, INPUT_PULLUP);
  //Serial1.begin(115200); //  GPIO2 TX1 (output only, connected to LED)
  //WiFi.disconnect();
	//ESP.eraseConfig();
	//delay(3000);
  Serial.printf("MS: %d\n", (int)millis());
  if(loadKey(DOOR_PRIKEY, DOOR_PUBKEY, DOOR_NAME) == false) {
    Serial.println("loadKey failed!");
  } else {
    Serial.println("\n\nloadKey ok.");
  }  

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

  Serial.printf("MS0: %d (Trig via reset)\n", (int)(millis() - timestamp));
  if(txCommand("c")) mainState = 1;
  Serial.printf("MS1: %d\n", (int)(millis() - timestamp));
}

char msg[256];
bool butTrig = false;
void loop() {
  static elapsedMillis bcTx = 0;
  static int cnt = 0;

  if(mainState == 2) {
    Serial.printf("MS2: %d\n", (int)(millis() - timestamp));
    if(txCommand("t")) mainState = 3;
    Serial.printf("MS3: %d\n", (int)(millis() - timestamp));
  }

  if(mainState == 4) {
    Serial.printf("MS4: %d\n", (int)(millis() - timestamp));
    Serial.println("Trigger done!\n");
    mainState++;
  }  

  if((digitalRead(0) == LOW) || (butTrig)) {
    butTrig = false;
    timestamp = millis();
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

  delay(1);
}

