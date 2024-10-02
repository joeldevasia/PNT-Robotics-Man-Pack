#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "Arduino.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <DFRobot_WT61PC.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <tf/tf.h>
#include <sensor_msgs/MagneticField.h>

ros::NodeHandle nh;

std_msgs::Float32 magnetic_dir;
std_msgs::Float32 magnetic_dir_raw;
std_msgs::Float32 magnetic_dir_degrees;
std_msgs::Int32 left_encoder_val;
std_msgs::Int32 right_encoder_val;

sensor_msgs::Imu imu_data;
sensor_msgs::Imu magnetometer_data;
sensor_msgs::MagneticField magnetic_field_data;

sensor_msgs::NavSatFix gps_data;

ros::Publisher imu_pub("raw_imu", &imu_data);
ros::Publisher magnetic_dir_pub("magnetic_dir", &magnetic_dir);
ros::Publisher magnetic_dir_raw_pub("magnetic_dir_raw", &magnetic_dir_raw);
ros::Publisher magnetic_dir_degrees_pub("magnetometer_degrees", &magnetic_dir_degrees);
ros::Publisher magnetic_field_pub("magnetic_field", &magnetic_field_data);
ros::Publisher magnetometer_pub("magnetometer", &magnetometer_data);
ros::Publisher gps_pub("raw_gps", &gps_data);
ros::Publisher left_encoder_val_pub("left_encoder_tick", &left_encoder_val);
ros::Publisher right_encoder_val_pub("right_encoder_tick", &right_encoder_val);

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Initialize the HMC5883L
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Magnetic declination for your location
float declinationAngle = -0.0333;  // (0 degrees 2' west = -0.0333 degrees)

// Calibration offsets (example values, you may need to adjust these)
// float xOffset = 3;
// float yOffset = 59;
// float zOffset = 25;
float xOffset = 0;
float yOffset = 0;
float zOffset = 0;

// DFRobot_WT61PC object
DFRobot_WT61PC sensor(&Serial2);

double latitude = 0.0;
double longitude = 0.0;
double altitude = 0.0;

double lin_acc_x = 0.0;
double lin_acc_y = 0.0;
double lin_acc_z = 0.0;

double ang_vel_x = 0.0;
double ang_vel_y = 0.0;
double ang_vel_z = 0.0;

double orient_x = 0.0;
double orient_y = 0.0;
double orient_z = 0.0;

double compass_angle = 0.0;

int ENCODER_L_A = 2;
int ENCODER_L_B = 4;
int ENCODER_R_A = 3;
int ENCODER_R_B = 5;
long encoderValueSet_R = 0;
long encoderValueSet_L = 0;

void getEncoderCounter_Right() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_R_A);
  int B = digitalRead(ENCODER_R_B);

  if ((A == HIGH) != (B == LOW)) {
    encoderValueSet_R--;
  } else {
    encoderValueSet_R++;
  }
  right_encoder_val.data = encoderValueSet_R;
}
void getEncoderCounter_Left() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_L_A);
  int B = digitalRead(ENCODER_L_B);

  if ((A == HIGH) != (B == LOW)) {
    encoderValueSet_L--;
  } else {
    encoderValueSet_L++;
  }
  left_encoder_val.data = encoderValueSet_L;
}

void setup() {
  Serial.begin(57600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  sensor.modifyFrequency(FREQUENCY_1HZ);
  mag.begin();

  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(magnetic_dir_pub);
  // nh.advertise(magnetic_dir_raw_pub);
  nh.advertise(magnetic_dir_degrees_pub);
  nh.advertise(magnetometer_pub);
  nh.advertise(magnetic_field_pub);
  nh.advertise(gps_pub);
  nh.advertise(left_encoder_val_pub);
  nh.advertise(right_encoder_val_pub);

  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), getEncoderCounter_Right, RISING);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), getEncoderCounter_Left, RISING);
}

void loop() {
  readSensorData();
  imu_pub.publish(&imu_data);
  magnetic_dir_pub.publish(&magnetic_dir);
  // magnetic_dir_raw_pub.publish(&magnetic_dir_raw);
  magnetometer_pub.publish(&magnetometer_data);
  magnetic_dir_degrees_pub.publish(&magnetic_dir_degrees);
  magnetic_field_pub.publish(&magnetic_field_data);
  
  // gps_pub.publish(&gps_data);
  // left_encoder_val_pub.publish(&left_encoder_val);
  // right_encoder_val_pub.publish(&right_encoder_val);

  nh.spinOnce();

  delay(10);
}

void readSensorData() {
  // gpsRead();
  gyroRead();
  compassRead();
}

void gpsRead() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
  if (gps.location.isValid()) {

    gps_data.header.frame_id = "gps_frame";
    gps_data.latitude = latitude;
    gps_data.longitude = longitude;
    gps_data.altitude = altitude;

    latitude = gps.location.lat();
    longitude = gps.location.lng();
    altitude = gps.altitude.meters();
    int time = gps.time.value();
    gps_pub.publish(&gps_data);
  }
}

void gyroRead() {
  if (sensor.available()) {
    lin_acc_x = sensor.Acc.X;
    lin_acc_y = sensor.Acc.Y;
    lin_acc_z = sensor.Acc.Z;

    ang_vel_x = sensor.Gyro.X;
    ang_vel_y = sensor.Gyro.Y;
    ang_vel_z = sensor.Gyro.Z;

    orient_x = sensor.Angle.X;
    orient_y = sensor.Angle.Y;
    orient_z = sensor.Angle.Z;

    imu_data.header.frame_id = "imu_frame";

    imu_data.orientation.x = orient_x;
    imu_data.orientation.y = orient_y;
    imu_data.orientation.z = orient_z;

    imu_data.angular_velocity.x = ang_vel_x;
    imu_data.angular_velocity.y = ang_vel_y;
    imu_data.angular_velocity.z = ang_vel_z;

    imu_data.linear_acceleration.x = lin_acc_x;
    imu_data.linear_acceleration.y = lin_acc_y;
    imu_data.linear_acceleration.z = lin_acc_z;
  }
}

void compassRead() {
  sensors_event_t event;
  mag.getEvent(&event);

  float xCalibrated = event.magnetic.x + xOffset;
  float yCalibrated = event.magnetic.y + yOffset;
  float zCalibrated = event.magnetic.z + zOffset;
  magnetic_field_data.magnetic_field.x = event.magnetic.x;
  magnetic_field_data.magnetic_field.y = event.magnetic.y;
  magnetic_field_data.magnetic_field.z = event.magnetic.z;

  float heading = atan2(yCalibrated, xCalibrated);
  magnetic_dir_degrees.data = heading*(180/PI);
  heading  = 2*PI - heading;
  heading += declinationAngle;
  // heading *= -1;


  heading += PI;
  
  if (heading < 0) {
    heading += 2 * PI;
  }

  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }

  // float headingDegrees = heading * 180 / PI;

  // if (headingDegrees < 0) {
  //   headingDegrees += 360;
  // }

  // compass_angle = headingDegrees;
  
  magnetic_dir.data = heading;
  magnetic_dir_degrees.data = heading*(180/PI);

  // double compass_angle_euler = (heading*PI)/180.0;

  geometry_msgs::Quaternion compass_angle_quat = tf::createQuaternionFromYaw(heading);

  magnetometer_data.orientation = compass_angle_quat;
}
