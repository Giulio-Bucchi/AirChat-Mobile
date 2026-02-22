// Arduino: framed chat + ACK compatible with STM32 (DEBUG RX)
#include <SPI.h>
#include <LoRa.h>

const int csPin = 10;          
const int resetPin = 9;        
const int irqPin = 2;

#define ADDR_ARDUINO 0x02
#define ADDR_STM32   0x01
#define PKT_TYPE_DATA 0x01
#define PKT_TYPE_ACK  0x02

String inputArduino = "";
uint8_t seq_ctr = 0;

void setup() {
  Serial.begin(9600);
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(433E6)) {
    Serial.println("Errore LoRa Arduino!");
    while (1);
  }
  LoRa.setSyncWord(0xF3);
}

void loop() {
  // TEST BLE AT-09: invio comandi AT da Serial Monitor
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.length() > 0) {
      Serial.println(cmd); // invia comando AT al BLE
      delay(200);
      while (Serial.available()) {
        char c = Serial.read();
        Serial.print(c); // stampa risposta BLE
      }
    }
  }
  // ...existing code...
  // 1. ASCOLTO DALLA RADIO (framed packets)
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    uint8_t buf[256];
    int idx = 0;
    while (LoRa.available() && idx < sizeof(buf)) {
      buf[idx++] = LoRa.read();
    }
    if (idx >= 5 && (buf[0] == ADDR_ARDUINO || buf[0] == ADDR_STM32)) {
      // framed packet
      uint8_t dst = buf[0];
      uint8_t src = buf[1];
      uint8_t type = buf[2];
      uint8_t seq = buf[3];
      uint8_t len = buf[4];
      if (dst == ADDR_ARDUINO) {
        if (type == PKT_TYPE_DATA) {
          // print payload
          int payloadLen = min((int)len, idx - 5);
          char msg[251];
          memcpy(msg, &buf[5], payloadLen);
          msg[payloadLen] = '\0';
          Serial.print("Utente 1: ");
          Serial.println(msg);
          // send ACK back
          uint8_t ack[5];
          ack[0] = src;         // dst = original sender
          ack[1] = ADDR_ARDUINO;
          ack[2] = PKT_TYPE_ACK;
          ack[3] = seq;         // same seq
          ack[4] = 0;           // len = 0
          LoRa.beginPacket();
          LoRa.write(ack, 5);
          LoRa.endPacket();
        } else if (type == PKT_TYPE_ACK) {
          // ACK received silently (no print)
        }
      }
    }
    // Ignore non-framed packets (no RAW RX fallback)
  }

  // 2. INVIO DA SERIALE (framed packets)
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputArduino.length() > 0) {
        // build framed packet
        uint8_t payload[250];
        int len = min((int)inputArduino.length(), 250);
        memcpy(payload, inputArduino.c_str(), len);
        uint8_t pkt[5 + len];
        pkt[0] = ADDR_STM32;     // destination
        pkt[1] = ADDR_ARDUINO;   // source
        pkt[2] = PKT_TYPE_DATA;  // type
        pkt[3] = seq_ctr++;      // sequence
        pkt[4] = len;            // payload length
        memcpy(&pkt[5], payload, len);
        LoRa.beginPacket();
        LoRa.write(pkt, 5 + len);
        LoRa.endPacket();
        Serial.print("Utente 2: ");
        Serial.println(inputArduino);
        inputArduino = "";
      }
    } else {
      inputArduino += inChar;
    }
  }
}
