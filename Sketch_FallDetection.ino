// ===================== BLYNK Configuration =====================
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL63NYe0GUE"
#define BLYNK_TEMPLATE_NAME "Fall Detection"
#define BLYNK_AUTH_TOKEN "BYnM7a9A-cgi05LWI27PorOGCD_cAIyM"

// ===================== WiFi Credentials =====================
char ssid[] = "EGHERT";
char pass[] = "Keena1023";

// ===================== Library Includes =====================
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <MPU6050.h>
#include "MAX30105.h"
#include "heartRate.h"

// ===================== Pin Definitions =====================
#define GPS_TX 34
#define GPS_RX 35
#define buzzerPin 27
#define buttonPin 32

// ===================== Hardware Serial =====================
HardwareSerial SerialGPS(2);
HardwareSerial sim800(1);

// ===================== Sensor Instances =====================
MAX30105 particleSensor;
TinyGPSPlus gps;
MPU6050 mpu;

// ===================== Global Variables =====================
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

// ===================== Send SMS Alert =====================
void sendSMS(String latStr, String lngStr) {
    Serial.println("Sending SMS...");
    sim800.println("AT+CMGF=1");    // Set SMS to Text Mode
    delay(100);
    sim800.println("AT+CMGS=\"+639943106884\""); // Caregiver number
    delay(300);

    // Compose SMS message
    sim800.print("🚨 WARNING: A Fall Has Been Detected. Mr. Eghert Mijares may need urgent assistance!\n");
    sim800.print("📍 Location:\n");
    sim800.print("Latitude: " + latStr + "\n");
    sim800.print("Longitude: " + lngStr + "\n");
    sim800.print("📌 Google Maps: https://maps.google.com/?q=" + latStr + "," + lngStr);

    delay(100);
    sim800.write(26); // CTRL+Z to send
    delay(5000);

    Serial.println("SMS sent.");
}

// ===================== Fall Detection Logic =====================
void checkFall() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Thresholds for detecting a fall
    if (!fallDetected && (abs(ax) > 20000 || abs(ay) > 25000 || abs(az) < 5000)) {
        fallDetected = true;
        fallTime = millis();
        buttonPressed = false;

        digitalWrite(buzzerPin, HIGH);  // Activate buzzer

        // Get GPS location
        String latStr = gps.location.isValid() ? String(gps.location.lat(), 6) : "N/A";
        String lngStr = gps.location.isValid() ? String(gps.location.lng(), 6) : "N/A";

        if (gps.location.isValid()) {
            Blynk.virtualWrite(V8, gps.location.lat());
            Blynk.virtualWrite(V9, gps.location.lng());
        }

        // Send SMS and Blynk event
        sendSMS(latStr, lngStr);
        Blynk.logEvent("fall_alert", "Fall detected!");
        Serial.println("Fall Detected. Alarm On. SMS Sent.");
    }

    // Handle unconscious case after timeout
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
