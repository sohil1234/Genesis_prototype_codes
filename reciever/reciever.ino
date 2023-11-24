#include <SPI.h>
#include <LoRa.h>

#define ss 10
#define rst 9
#define dio0 2

String incomming =  "";
String data = "";

void setup() {
  Serial.begin(9600);
  while (!Serial);
  LoRa.setPins(ss, rst, dio0);
  Serial.println("LoRa Receiver");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // read packet
    while (LoRa.available()) {
      int inChar = LoRa.read();
      incomming += (char) inChar;
      data = incomming;
    }
    incomming = "";
    LoRa.packetRssi();
  }
  Serial.println(data);
  delay(500);
}
