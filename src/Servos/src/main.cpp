#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h> // Compatible amb versió 7.4.2
#include <ESP32Servo.h>

// ─────────────────────────────────────────────
// Device & Wi-Fi setup
// ─────────────────────────────────────────────
const char *deviceId = "G2_Servos";
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// UDP setup
IPAddress receiverESP32IP(192, 168, 1, 21);
IPAddress receiverComputerIP(192, 168, 1, 25);
const int udpPort = 12345;
WiFiUDP udp;

// ─────────────────────────────────────────────
// Servo setup
// ─────────────────────────────────────────────
Servo servo_yaw;
Servo servo_pitch;
Servo servo_roll1;
Servo servo_roll2;

const int PIN_ANALOG_YAW = 36;
const int PIN_SIGNAL_YAW = 32;
const int PIN_ANALOG_PITCH = 39;
const int PIN_SIGNAL_PITCH = 33;
const int PIN_ANALOG_ROLL1 = 34;
const int PIN_SIGNAL_ROLL1 = 25;
const int PIN_ANALOG_ROLL2 = 35;
const int PIN_SIGNAL_ROLL2 = 27;

const float Rshunt = 1.6;

// ─────────────────────────────────────────────
// Global variables
// ─────────────────────────────────────────────
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;
float Torque_roll1 = 0.0, Torque_roll2 = 0.0, Torque_pitch = 0.0, Torque_yaw = 0.0;
float prevRoll1 = 0, prevRoll2 = 0, prevPitch = 0, prevYaw = 0;
float sumRoll1 = 0, sumRoll2 = 0, sumPitch = 0, sumYaw = 0;

float yawOffset = 0.0;
bool yawOffsetSet = false;

int s1 = 1, s2 = 1;

// ─────────────────────────────────────────────
// Wi-Fi connection
// ─────────────────────────────────────────────
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

// ─────────────────────────────────────────────
// Receive orientation + button data from Gripper
// ─────────────────────────────────────────────
void receiveOrientationUDP() {
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

      const char* device = doc["device"];
      if (strcmp(device, "G5_Gri") == 0) {
        Gri_roll = round(doc["roll"].as<float>());
        Gri_pitch = round(doc["pitch"].as<float>());
        Gri_yaw = round(doc["yaw"].as<float>());
        s1 = doc["s1"];
        s2 = doc["s2"];

        // Initialize yaw offset only once
        if (!yawOffsetSet) {
          yawOffset = Gri_yaw;
          yawOffsetSet = true;
          Serial.print("Yaw offset initialized to: ");
          Serial.println(yawOffset);
        }

        // Optional: recalibrate yaw offset with button S2
        if (s2 == 0) {
          yawOffset = Gri_yaw;
          Serial.println("Yaw offset recalibrated!");
        }

        Serial.print("Gri_Roll: "); Serial.print(Gri_roll);
        Serial.print(" Gri_Pitch: "); Serial.print(Gri_pitch);
        Serial.print(" Gri_Yaw: "); Serial.println(Gri_yaw);
        Serial.print("S1: "); Serial.print(s1);
        Serial.print(" S2: "); Serial.println(s2);
      }
    }
  }
}

// ─────────────────────────────────────────────
// Send torque values to PC and Gripper
// ─────────────────────────────────────────────
float getCurrent(uint32_t integrationTimeMs, int pin) {
  uint32_t startTime = millis();
  float integratedCurrent = 0;
  while (millis() < startTime + integrationTimeMs) {
    uint16_t adcValue = analogRead(pin);
    integratedCurrent += ((float)adcValue / 4095.0 * 3.3) / Rshunt;
  }
  return integratedCurrent;
}

float getTorque(float& sum, int analogPin, float& previous) {
  float current = getCurrent(20, analogPin);
  sum += current;
  float diff = abs(sum - previous);
  previous = sum;
  return diff;
}

void sendTorqueUDP() {
  JsonDocument doc;

  Torque_roll1 = getTorque(sumRoll1, PIN_ANALOG_ROLL1, prevRoll1);
  Torque_roll2 = getTorque(sumRoll2, PIN_ANALOG_ROLL2, prevRoll2);
  Torque_pitch = getTorque(sumPitch, PIN_ANALOG_PITCH, prevPitch);
  Torque_yaw = getTorque(sumYaw, PIN_ANALOG_YAW, prevYaw);

  doc["device"] = deviceId;
  doc["Torque_Roll_1"] = Torque_roll1;
  doc["Torque_Roll_2"] = Torque_roll2;
  doc["Torque_Pitch"] = Torque_pitch;
  doc["Torque_Yaw"] = Torque_yaw;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);

  // Send to Gripper ESP32
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();

  // Send to PC
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();
}

// ─────────────────────────────────────────────
// Move servos according to IMU data
// ─────────────────────────────────────────────
void moveServos() {
  float roll_cmd = Gri_roll;
  float pitch_cmd = Gri_pitch;
  float yaw_cmd = Gri_yaw - yawOffset; // independent from magnetic north

  servo_roll1.write(90 + roll_cmd);
  servo_roll2.write(90 - roll_cmd);
  servo_pitch.write(90 + pitch_cmd);
  servo_yaw.write(90 + yaw_cmd);

  if (s1 == 0) {
    Serial.println("S1 pressed → opening gripper");
  }
}

// ─────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  // Servo setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo_yaw.setPeriodHertz(50);
  servo_pitch.setPeriodHertz(50);
  servo_roll1.setPeriodHertz(50);
  servo_roll2.setPeriodHertz(50);

  servo_yaw.attach(PIN_SIGNAL_YAW);
  servo_pitch.attach(PIN_SIGNAL_PITCH);
  servo_roll1.attach(PIN_SIGNAL_ROLL1);
  servo_roll2.attach(PIN_SIGNAL_ROLL2);

  pinMode(PIN_ANALOG_YAW, INPUT);
  pinMode(PIN_ANALOG_PITCH, INPUT);
  pinMode(PIN_ANALOG_ROLL1, INPUT);
  pinMode(PIN_ANALOG_ROLL2, INPUT);

  // Initial neutral position
  servo_yaw.write(90);
  servo_pitch.write(90);
  servo_roll1.write(90);
  servo_roll2.write(90);
}

// ─────────────────────────────────────────────
// Main loop
// ─────────────────────────────────────────────
void loop() {
  receiveOrientationUDP();
  sendTorqueUDP();
  moveServos();
  delay(10);
}
