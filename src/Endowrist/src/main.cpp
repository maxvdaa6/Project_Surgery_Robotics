#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU9250.h"
#include <Wire.h>
#include <ArduinoJson.h>  // No cal .hpp, la versió 7 manté compatibilitat amb .h

// Device ID
const char *deviceId = "G2_Endo";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// Botons
const int PIN_S3 = 14;
const int PIN_S4 = 27;
int s3Status = HIGH;
int s4Status = HIGH;

// UDP settings
IPAddress receiverComputerIP(192, 168, 1, 25);
const int udpPort = 12345;
WiFiUDP udp;

// MPU-9250 object
MPU9250 mpu;

// Orientation data
float Endo_roll = 0.0, Endo_pitch = 0.0, Endo_yaw = 0.0;

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println("IP Address: " + WiFi.localIP().toString());
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void updateOrientation() {
  if (mpu.update()) {
    Endo_yaw = -mpu.getYaw();
    Endo_pitch = -mpu.getPitch();
    Endo_roll = mpu.getRoll();
    s3Status = digitalRead(PIN_S3);
    s4Status = digitalRead(PIN_S4);
  }
}

void sendOrientationUDP() {
  JsonDocument doc;
  doc["device"] = deviceId;
  doc["roll"] = Endo_roll;
  doc["pitch"] = Endo_pitch;
  doc["yaw"] = Endo_yaw;
  doc["s3"] = s3Status;
  doc["s4"] = s4Status;
  //Serial.print("Roll: ");
  //Serial.println(Endo_roll);
  //Serial.print("Pitch: ");
  //Serial.println(Endo_pitch);
  //Serial.print("Yaw: ");
  //Serial.println(Endo_yaw);

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {
    while (1) {
      Serial.println("MPU connection failed.");
      delay(5000);
    }
  }
  Serial.println("MPU connected");
  delay(2000);

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized.");
}

void loop() {
  updateOrientation();
  sendOrientationUDP();
  delay(10);
}
