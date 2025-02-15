#include "WiFi.h"
#include <ros.h>
#include "arduino_bma456.h"
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <FastLED.h>

#define NUM_LEDS 1
#define DATA_PIN 27
#define CLOCK_PIN 13

CRGB leds[NUM_LEDS];

// IPAddress ip(192, 168, 178, 47);
// IPAddress server(192, 168, 67, 11);
IPAddress ip(192, 168, 67, 47);
IPAddress server(192, 168, 0, 6);

uint16_t serverPort = 11411;
// const char *ssid = "TP-Link_Guest_466B";
// const char *password = "Pnt@107#";

// const char *ssid = "Test";
const char *ssid = "ManPack";
const char *password = "12345678";

int last_heart_beat_received_time = millis();
bool blink_status_led = true;

int heart_beat_counter = 100;

ros::NodeHandle nh;
std_msgs::Int32 steps;
ros::Publisher steps_pub("sensors/step_counter", &steps);

std_msgs::Int32 heart_beat_msg;
ros::Publisher heart_beat_pub("heart_beat", &heart_beat_msg);

void heart_beat_callback(const std_msgs::Int32& msg){
  last_heart_beat_received_time = millis();
  heart_beat_msg.data = heart_beat_counter;
  heart_beat_pub.publish(&heart_beat_msg);
  heart_beat_counter += 1;
}

ros::Subscriber<std_msgs::Int32> heart_beat_sub("/heart_beat", &heart_beat_callback);

void gps_coordinates_callback(const std_msgs::String& msg){
  Serial2.println(msg.data);
  Serial.print("Transmitted Coordinates: ");
  Serial.println(msg.data);
}

ros::Subscriber<std_msgs::String> gps_coordinates_sub("/gps_coordinates", &gps_coordinates_callback);

void setupWiFi();

void Set_LED_Red(){
  // Turn the LED on, then pause
  leds[0] = CRGB::Green;
  FastLED.show();
}

void Set_LED_Green(){
  // Turn the LED on, then pause
  leds[0] = CRGB::Red;
  FastLED.show();
}

void Set_LED_Blue(){
  // Turn the LED on, then pause
  leds[0] = CRGB::Blue;
  FastLED.show();
}

void Set_LED_Off(){
  // Turn the LED on, then pause
  leds[0] = CRGB::Black;
  FastLED.show();
}

void setup() {
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); 
  Set_LED_Red();
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 18, 19);
  Serial.println("Step Counter ESP");
  setupWiFi();

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("ROS IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  nh.advertise(steps_pub);
  nh.advertise(heart_beat_pub);
  nh.subscribe(heart_beat_sub);
  nh.subscribe(gps_coordinates_sub);
  bma456.initialize(RANGE_4G, ODR_1600_HZ, NORMAL_AVG4, CONTINUOUS);
  bma456.stepCounterEnable();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED){
    ESP.restart();
  }
  if (nh.connected()) {
    steps_pub.publish(&steps);
    Serial.println("Connected");
    blink_status_led = false;
  } else {
    Serial.println("Not Connected");
    blink_status_led = true;
  }
  if(millis()-last_heart_beat_received_time >=2000){
    blink_status_led = true;
  }
  else{
    blink_status_led = false;
  }
  readSensorData();
  if(blink_status_led == false){
    delay(200);
  }
  else{
    Set_LED_Off();
    delay(200);
  }
  Set_LED_Green();
  nh.spinOnce();
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
    Set_LED_Blue();
    delay(250);
    Set_LED_Off();
    delay(250);
    Serial.print(".");
  }
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:   ");
  Serial.println(WiFi.localIP());
  Set_LED_Blue();
}