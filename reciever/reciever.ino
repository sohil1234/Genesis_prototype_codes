#include <SPI.h>
#include <LoRa.h>
#include <ros.h>
#include <std_msgs/String.h>

#define ss 10
#define rst 9
#define dio0 2

String incomming = "";
String data = "";

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher arduino_data_pub("/arduino_data", &str_msg);

void setup() {
  LoRa.setPins(ss, rst, dio0);
  Serial.println("LoRa Receiver");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  nh.initNode();
  nh.advertise(arduino_data_pub);
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

    str_msg.data = data.c_str();
    arduino_data_pub.publish(&str_msg);
    nh.spinOnce(); // Handle ROS communication
  }
  delay(500);
}
