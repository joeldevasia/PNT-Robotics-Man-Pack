#include "WiFi.h"
#include <ros.h>
#include "arduino_bma456.h"
#include <std_msgs/Int32.h>
#include <Wire.h>

// IPAddress ip(192, 168, 178, 47);
// IPAddress server(192, 168, 67, 11);
IPAddress ip(192, 168, 67, 47);
IPAddress server(192, 168, 67, 11);

uint16_t serverPort = 11411;
// const char *ssid = "TP-Link_Guest_466B";
// const char *password = "Pnt@107#";

const char *ssid = "Test";
const char *password = "12345678";

ros::NodeHandle nh;
std_msgs::Int32 steps;
ros::Publisher steps_pub("sensors/step_counter", &steps);

void setupWiFi();

void setup() {
  Serial.begin(115200);
  Serial.println("Step Counter ESP");
  setupWiFi();

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("ROS IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  nh.advertise(steps_pub);

  bma456.initialize(RANGE_4G, ODR_1600_HZ, NORMAL_AVG4, CONTINUOUS);
  bma456.stepCounterEnable();
}

void loop() {
  if (nh.connected()) {
    steps_pub.publish(&steps);
    Serial.println("Connected");
  } else {
    Serial.println("Not Connected");
  }
  readSensorData();
  nh.spinOnce();
  delay(100);
}

void readSensorData() {
  getStepCount();
}

void getStepCount() {
  steps.data = bma456.getStepCounterOutput();
  Serial.println(steps.data);
}


void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:   ");
  Serial.println(WiFi.localIP());
}