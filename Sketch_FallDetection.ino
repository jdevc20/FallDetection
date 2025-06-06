// ✅ Setup (sensors, WiFi, Blynk, GPS, SIM800)
// 🔄 Loop: Read GPS ➔ Detect Fall ➔ Check Button ➔ Read Vitals.
// 🚨 On Fall: Alarm ➔ SMS ➔ Blynk Alert.
// 🛑 User can stop alarm by pressing button.
// ⏳ After timeout, if no response → "Unconscious" status.
// 📡 Continuous vitals & GPS updates to Blynk.

//ESP32
// Max30102
//MPU6050
//Gps Neo6m
// SIM800L

// ===================== Library Includes =====================
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <MPU6050.h>
#include "MAX30105.h"
#include "heartRate.h"
// ===================== BLYNK Configuration =====================
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL63NYe0GUE"
#define BLYNK_TEMPLATE_NAME "Fall Detection"
#define BLYNK_AUTH_TOKEN "BYnM7a9A-cgi05LWI27PorOGCD_cAIyM"
// ===================== WiFi Credentials =====================
char ssid[] = "EGHERT";
char pass[] = "Keena1023";
// ===================== Pin Definitions =====================
#define GPS_TX 34
#define GPS_RX 35
#define buzzerPin 27
#define buttonPin 32
// ===================== 🗜️ Hardware Serial =====================
HardwareSerial SerialGPS(2);
HardwareSerial sim800(1);
// ===================== 🕵️ Sensor Instances =====================
MAX30105 particleSensor;
TinyGPSPlus gps;
MPU6050 mpu;
// ===================== 🌍 Global Variables =====================
BlynkTimer timer;
bool fallDetected = false;
bool buttonPressed = false;
unsigned long fallTime = 0;
unsigned long fallTimeout = 20000;  // 20 seconds timeout
int bpm = 0;
int spo2 = 0;
unsigned long lastButtonPressTime = 0;
unsigned long debounceDelay = 500;  // Debounce delay

// ===================== Setup Function =====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    // Pin modes
    pinMode(buzzerPin, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);

    // I2C for sensors
    Wire.begin(21, 22);

    // Serial for GPS and SIM800L
    SerialGPS.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
    sim800.begin(9600, SERIAL_8N1, 26, 27);

    // Connect to Blynk
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

    // Initialize MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
    }

    // Initialize MAX30102
    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("MAX30102 not found");
    } else {
        particleSensor.setup();
        particleSensor.setPulseAmplitudeRed(0x0A);
        particleSensor.setPulseAmplitudeIR(0x0A);
    }

    // Timer intervals
    timer.setInterval(500L, checkButton);   // Check button status
    timer.setInterval(2000L, updateGPS);    // Update GPS location
    timer.setInterval(2500L, readVitals);   // Read vitals
}

// ===================== Main Loop =====================
void loop() {
    while (SerialGPS.available()) {
        gps.encode(SerialGPS.read());
    }

    checkFall();       // Detect fall event
    Blynk.run();       // Blynk communication
    timer.run();       // Handle timers
}

void triggerSendSMS(String latStr, String lngStr) {
  if (!smsInProgress) {
    smsInProgress = true;
    smsStep = 0;
    smsLat = latStr;
    smsLng = lngStr;
    smsStartTime = millis();
    Serial.println("Starting Non-Blocking SMS...");
  }
}

void handleSendSMS() {
  if (!smsInProgress) return;

  unsigned long currentMillis = millis();

  switch (smsStep) {
    case 0:
      sim800.println("AT+CMGF=1");
      smsStartTime = currentMillis;
      smsStep++;
      break;

    case 1:
      if (currentMillis - smsStartTime >= 100) {
        sim800.println("AT+CMGS=\"+639943106884\"");
        smsStartTime = currentMillis;
        smsStep++;
      }
      break;

    case 2:
      if (currentMillis - smsStartTime >= 300) {
        sim800.print("\xF0\x9F\x9A\xA8 WARNING: A Fall Has Been Detected. Mr. Eghert Mijares may need urgent assistance!\n");
        sim800.print("\xF0\x9F\x93\x8D Location:\n");
        sim800.print("Latitude: " + smsLat + "\n");
        sim800.print("Longitude: " + smsLng + "\n");
        sim800.print("\xF0\x9F\x93\x8C Google Maps: https://maps.google.com/?q=" + smsLat + "," + smsLng);
        smsStartTime = currentMillis;
        smsStep++;
      }
      break;

    case 3:
      if (currentMillis - smsStartTime >= 100) {
        sim800.write(26);
        smsStartTime = currentMillis;
        smsStep++;
      }
      break;

    case 4:
      if (currentMillis - smsStartTime >= 5000) {
        Serial.println("SMS sent (Non-Blocking).");
        smsInProgress = false;
      }
      break;
  }
}
// --------------------------------


// ===================== Fall Detection Logic =====================
void checkFall() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw to g (assuming ±2g range, sensitivity = 16384 LSB/g)
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  // Compute acceleration magnitude (G-force)
  float accMagnitude = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  // Compute tilt angles
  float pitch = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;
  float roll = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / PI;

  // Debug prints
  Serial.print("G-Force: "); Serial.print(accMagnitude, 2);
  Serial.print(" | Pitch: "); Serial.print(pitch, 1);
  Serial.print(" | Roll: "); Serial.println(roll, 1);

  // Adaptive thresholds
  const float impactThreshold = 2.5;  // 2.5g impact
  const float tiltChangeThreshold = 45.0; // 45 degrees tilt
  static unsigned long impactStartTime = 0;
  static bool impactDetected = false;

  if (!fallDetected) {
    if (accMagnitude > impactThreshold) {
      impactDetected = true;
      impactStartTime = millis();
    }

    // Check if tilt confirms fall (after impact)
    if (impactDetected && (millis() - impactStartTime < 1000)) {
      if (abs(pitch) > tiltChangeThreshold || abs(roll) > tiltChangeThreshold) {
        fallDetected = true;
        fallTime = millis();
        buttonPressed = false;

        digitalWrite(buzzerPin, HIGH);

        String latStr = gps.location.isValid() ? String(gps.location.lat(), 6) : "N/A";
        String lngStr = gps.location.isValid() ? String(gps.location.lng(), 6) : "N/A";

        if (gps.location.isValid()) {
          Blynk.virtualWrite(V8, gps.location.lat());
          Blynk.virtualWrite(V9, gps.location.lng());
        }

        triggerSendSMS(latStr, lngStr);
        Blynk.logEvent("fall_alert", "Fall detected!");

        Serial.println("Fall Detected (Adaptive). Alarm On. SMS Triggered.");
        impactDetected = false;
      }
    }

    // Reset if no tilt change within timeout
    if (impactDetected && (millis() - impactStartTime >= 1000)) {
      impactDetected = false;
    }
  }

  // Unconscious state check
  if (fallDetected && (millis() - fallTime >= fallTimeout)) {
    if (!buttonPressed) {
      Blynk.virtualWrite(V10, "Unconscious");
      Serial.println("Elderly is unconscious");
    }
  }
}

// ===================== Button Check =====================
void checkButton() {
    if (millis() - lastButtonPressTime > debounceDelay) {
        lastButtonPressTime = millis();

        if (fallDetected && digitalRead(buttonPin) == LOW) {
            Serial.println("Button pressed. Stopping buzzer...");
            buttonPressed = true;

            digitalWrite(buzzerPin, LOW);  // Turn off buzzer
            Blynk.virtualWrite(V10, "Conscious");

            Serial.println("Button pressed: Conscious");

            // Reset state
            fallDetected = false;
        }
    }
}

// ===================== GPS Update to Blynk =====================
void updateGPS() {
    if (gps.location.isValid()) {
        Blynk.virtualWrite(V8, gps.location.lat());
        Blynk.virtualWrite(V9, gps.location.lng());
    }
}

// ===================== Vitals Reading (BPM & SpO2) =====================
void readVitals() {
    long irValue = particleSensor.getIR();

    if (irValue > 50000) {
        if (particleSensor.available()) {
            int beat = checkForBeat(irValue);

            if (beat) {
                static byte rateSpot = 0;
                static float rates[4];
                static float beatsPerMinute;
                static long lastBeat = millis();

                long delta = millis() - lastBeat;
                lastBeat = millis();

                beatsPerMinute = 60 / (delta / 1000.0);

                if (beatsPerMinute < 255 && beatsPerMinute > 20) {
                    rates[rateSpot++] = beatsPerMinute;
                    rateSpot %= 4;

                    float beatAvg = 0;
                    for (byte x = 0; x < 4; x++)
                        beatAvg += rates[x];

                    beatAvg /= 4;
                    bpm = (int)beatAvg;
                }
            }

            spo2 = random(95, 100); // Simulated SpO2 (Replace with real calculation)

            Serial.print("BPM: ");
            Serial.print(bpm);
            Serial.print(" | SpO2 (simulated): ");
            Serial.println(spo2);

            Blynk.virtualWrite(V6, bpm);
            Blynk.virtualWrite(V7, spo2);

            particleSensor.nextSample();
        }
    } else {
        Blynk.virtualWrite(V6, 0);
        Blynk.virtualWrite(V7, 0);
        Serial.println("No finger detected");
    }
}
