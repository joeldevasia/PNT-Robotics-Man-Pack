#include "WiFi.h"
#include <ros.h>
#include "Adafruit_MLX90393.h"
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

int heart_beat_counter = 200;

ros::NodeHandle nh;
std_msgs::Int32 magnetic_heading;
ros::Publisher magnetometer_pub("sensors/magnetometer", &magnetic_heading);

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

// Hard-iron calibration settings
// Set to {0, 0, 0} for no hard-iron offset
const float hard_iron[3] = {
  8.97, -32.22, 17.35
};

// Soft-iron calibration settings
// Set to identity matrix for no soft-iron scaling
const float soft_iron[3][3] = {
  { 0.979, -0.024, 0.006 },
  { -0.024, 1.039, 0.007 },
  { 0.006, 0.007, 0.983 }
};

// Magnetic declination from magnetic-declination.com
// East is positive (+), west is negative (-)
// mag_decl = (+/-)(deg + min/60 + sec/3600)
// Set to 0 to get magnetic heading instead of geographic heading
const float mag_decl = -0.0333;

Adafruit_MLX90393 mlx = Adafruit_MLX90393();

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
  Serial2.begin(9600);
  Serial.println("Magnetometer ESP");
  setupWiFi();
  if (!mlx.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
    while (1) { delay(10); }
  }

  mlx.setGain(MLX90393_GAIN_1X);

  // Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
  mlx.setResolution(MLX90393_X, MLX90393_RES_17);
  mlx.setResolution(MLX90393_Y, MLX90393_RES_17);
  mlx.setResolution(MLX90393_Z, MLX90393_RES_16);

  // Set oversampling
  mlx.setOversampling(MLX90393_OSR_3);

  // Set digital filtering
  mlx.setFilter(MLX90393_FILTER_5);

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("ROS IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  nh.advertise(magnetometer_pub);
  nh.advertise(heart_beat_pub);
  nh.subscribe(heart_beat_sub);
  nh.subscribe(gps_coordinates_sub);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED){
    ESP.restart();
  }
  if (nh.connected()) {
    magnetometer_pub.publish(&magnetic_heading);
    Serial.println("Connected");
    blink_status_led = false;
  } else {
    Serial.println("Not Connected");
    blink_status_led = true;
  }
  if(millis()-last_heart_beat_received_time >=2000){
    blink_status_led = true;
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
  getMagnetometerData();
}

void getMagnetometerData() {
  // Raw magnetometer data stored as {x, y, z}
  static float mag_data[3] = { 0.0, 0.0, 0.0 };

  static float hi_cal[3];
  static float heading;

  // Sample and compensate with hard/soft-iron calibration data
  if (mlx.readData(&mag_data[0], &mag_data[1], &mag_data[2])) {

    // Apply hard-iron offsets
    for (uint8_t i = 0; i < 3; i++) {
      hi_cal[i] = mag_data[i] - hard_iron[i];
    }

    // Apply soft-iron scaling
    for (uint8_t i = 0; i < 3; i++) {
      mag_data[i] = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
    }

    // Calculate angle for heading, assuming board is parallel to
    // the ground and +X points toward heading.
    // WARNING: X and Y silkscreen marketings are backward on v1 of board
    heading = (atan2(mag_data[1], mag_data[0]) * 180) / M_PI;

    // Apply magnetic declination to convert magnetic heading
    // to geographic heading
    heading += mag_decl;

    // Normalize to 0..360
    if (heading < 0) {
      heading += 360;
    }

    magnetic_heading.data = heading;
  }
  Serial.println(magnetic_heading.data);
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