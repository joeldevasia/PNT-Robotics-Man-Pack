/*
 * rosserial Publisher Example
 * Prints "hello world!"
 * This intend to connect to an Arduino Ethernet Shield
 * and a rosserial socket server.
 * You can launch the rosserial socket server with
 * roslaunch rosserial_server socket.launch
 * The default port is 11411
 *
 */
#include "WiFi.h"
#include <ros.h>
#include <std_msgs/String.h>

// To use the TCP version of rosserial_arduino
// IPAddress ip(192, 168, 0, 47);
// IPAddress server(192, 168, 0, 171);

IPAddress ip(192, 168, 67, 47);
IPAddress server(192, 168, 67, 11);

// Set the rosserial socket server port
const uint16_t serverPort = 11411;

// const char *ssid = "TP-Link_Guest_466B";
// const char *password = "Pnt@107#";

const char *ssid = "Test";
const char *password = "12345678";

ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

// Be polite and say hello
char hello[13] = "hello world!";
uint16_t period = 1000;
uint32_t last_time = 0;

void setup()
{
  // Use serial to monitor the process
  Serial.begin(115200);

  // Let some time for the Ethernet Shield to be initialized
  delay(1000);

  setupWiFi();

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  // Start to be polite
  nh.advertise(chatter);
}

void loop()
{
  if(millis() - last_time >= period)
  {
    last_time = millis();
    if (nh.connected())
    {
      Serial.println("Connected");
      // Say hello
      str_msg.data = hello;
      chatter.publish( &str_msg );
    } else {
      Serial.println("Not Connected");
    }
  }
  nh.spinOnce();
  delay(1);
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
