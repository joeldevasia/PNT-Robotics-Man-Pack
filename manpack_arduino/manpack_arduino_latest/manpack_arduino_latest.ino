#include <ros.h>
#include <std_msgs/Int32.h>
#include "Arduino.h"
#include <Wire.h>
#include <tf/tf.h>
#include "Adafruit_MLX90393.h"
#include "arduino_bma456.h"
ros::NodeHandle nh;

std_msgs::Int32 magnetic_heading;
ros::Publisher magnetometer_pub("sensors/magnetometer", &magnetic_heading);

std_msgs::Int32 steps;
ros::Publisher steps_pub("sensors/step_counter", &steps);

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

void setup() {
  Serial.begin(57600);
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

  bma456.initialize(RANGE_4G, ODR_1600_HZ, NORMAL_AVG4, CONTINUOUS);
  bma456.stepCounterEnable();

  nh.initNode();
  nh.advertise(magnetometer_pub);
  nh.advertise(steps_pub);
}

void loop() {
  readSensorData();
  magnetometer_pub.publish(&magnetic_heading);
  steps_pub.publish(&steps);
  nh.spinOnce();

  delay(10);
}

void readSensorData() {
  getStepCount();
  getMagnetometerData();
}

void getStepCount() {
  steps.data = bma456.getStepCounterOutput();
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
}
