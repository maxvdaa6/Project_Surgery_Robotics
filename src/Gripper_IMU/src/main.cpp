#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU9250.h"
#include <Wire.h> // Needed for I2C to read IMU
#include <ArduinoJson.h> // Compatible amb versió 7.4.2
#include <IMU_RoboticsUB.h>   // Nom de la llibreria custom

// Device ID
const char *deviceId = "G2_Gri";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// Vibration motor settings
const int vibrationPin = 23; // Pin for the vibration motor

// Botons
const int PIN_S1 = 14;
const int PIN_S2 = 27;
int s1Status = HIGH;
int s2Status = HIGH;

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 23); // IP of receiver ESP32
IPAddress receiverComputerIP(192, 168, 1, 25); // IP of PC
const int udpPort = 12345;
WiFiUDP udp;

// IMU object
IMU imu;

// Orientation data
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;
float Torque_roll1 = 0.0, Torque_roll2 = 0.0, Torque_pitch = 0.0, Torque_yaw = 0.0;

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
  // Llegeix FIFO del DMP i actualitza càlculs interns
  imu.ReadSensor();
  // Obté els angles (roll, pitch, yaw) via GetRPW()
  float* rpw = imu.GetRPW();
  Gri_roll  = rpw[0];
  Gri_pitch = rpw[1];
  Gri_yaw   = rpw[2];
  s1Status = digitalRead(PIN_S1);
  s2Status = digitalRead(PIN_S2);

}

void sendOrientationUDP() {
  JsonDocument doc;
  doc["device"] = deviceId;
  doc["roll"] = Gri_roll;
  doc["pitch"] = Gri_pitch;
  doc["yaw"] = Gri_yaw;
  doc["s1"] = s1Status;
  doc["s2"] = s2Status;

  // Serial.print("Roll: ");
  //Serial.print(Gri_roll);
  //Serial.print(", Pitch: ");
  //Serial.print(Gri_pitch);
  //Serial.print(", Yaw: ");
  //Serial.println(Gri_yaw);

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);

  // Send to ESP32 Servos
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();

  // Send to Computer
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();
}

void receiveTorquesUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    byte packetBuffer[512];
    int len = udp.read(packetBuffer, 512);
    if (len > 0) {
      packetBuffer[len] = '\0';

      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, packetBuffer);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      const char *device = doc["device"];
      if (strcmp(device, "G2_Servos") == 0) {
        Torque_roll1 = doc["Torque_Roll_1"];
        Torque_roll2 = doc["Torque_Roll_2"];
        Torque_pitch = doc["Torque_Pitch"];
        Torque_yaw   = doc["Torque_Yaw"];

        // Print the received torques
        //Serial.println("Received Torques from Servos:");
        //Serial.print("Torque Roll 1: "); Serial.println(Torque_roll1, 3);
        //Serial.print("Torque Roll 2: "); Serial.println(Torque_roll2, 3);
        //Serial.print("Torque Pitch : "); Serial.println(Torque_pitch, 3);
        //Serial.print("Torque Yaw   : "); Serial.println(Torque_yaw, 3);
        //Serial.println("----------------------------------");

        // 🔸 Vibration motor control
        float totalTorque = Torque_roll1 + Torque_roll2 + Torque_pitch + Torque_yaw;
        int vibrationValue = constrain(totalTorque * 2.5, 0, 255);
        ledcWrite(0, vibrationValue); // PWM control for vibration
        Serial.print("Vibration motor value: ");
        Serial.println(vibrationValue);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);
  
  // Inicialitza IMU (amb DMP)
  imu.Install();

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);

  // Configure PWM for the vibration motor (channel 0)
  ledcSetup(0, 5000, 8); // Channel 0, frequency 5kHz, resolution 8 bits
  ledcAttachPin(vibrationPin, 0); // Attach the vibration motor to channel 0
}

void loop() {
  updateOrientation();
  sendOrientationUDP();
  receiveTorquesUDP();
  delay(10);
}
