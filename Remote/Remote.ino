#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

#define CE_PIN 5
#define CSN_PIN 6

RF24 radio(CE_PIN, CSN_PIN);

const uint64_t pipe = 0xE8E8F0F0E1LL;

// Joystick input pins
#define X_PIN A1
#define Y_PIN A2
#define ROTATION_PIN A3
#define POWER_PIN A4

// Struct to send data
struct DroneData {
  int x;
  int y;
  int rotation;
  int power;
};

DroneData data;

// Calibration values
int rollOffset = 0;
int pitchOffset = 0;
int yawOffset = 0;

// Safety check
bool safetyCheckPassed = false;

void setup(void){
  
 Serial.begin(57600);

  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(pipe);
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();

  calibrateJoystick();

  int mappedPower = map(analogRead(POWER_PIN), 0, 1023, 1000, 2000);

  //Safety: If power is too high at startup, require user action
  if (mappedPower > 1200) {
    Serial.println("Warning: Power too high at startup! Move throttle to zero.");
    Serial.print(mappedPower);
    while (true) {  
      int currentPower = analogRead(POWER_PIN);
      int mappedCurrentPower = map(currentPower, 0, 1023, 1000, 2000);
      if (mappedCurrentPower < 1050) {  // Allow start when power is low
        safetyCheckPassed = true;
        Serial.println("Safety check passed. Ready to transmit.");
        delay(1000);
        break;
      }
      delay(100);
    }
  } else {
    safetyCheckPassed = true;
  }
}

void loop(void)
{
  //if (!safetyCheckPassed) return;  // Block transmission if safety check failed

  // Read analog values
  data.x = constrain(map(analogRead(X_PIN), 0, 1023, 1000, 2000) - rollOffset, 1000, 2000);
  data.y = constrain(map(analogRead(Y_PIN), 0, 1023, 1000, 2000) - pitchOffset, 1000, 2000);
  data.rotation = constrain(map(analogRead(ROTATION_PIN), 0, 1023, 1000, 2000) - yawOffset, 1000, 2000);
  data.power = map(analogRead(POWER_PIN), 0, 1023, 1000, 2000);

  // Send data
  radio.write(&data, sizeof(data));

  Serial.print("X: ");
  Serial.print(data.x);
  Serial.print(", Y: ");
  Serial.print(data.y);
  Serial.print(" | Power: ");
  Serial.print(data.power);
  Serial.print(", Rotation: ");
  Serial.println(data.rotation);

  delay(50);  // Adjust delay based on communication requirements
}

void calibrateJoystick() {
  long rollSum = 0, pitchSum = 0, yawSum = 0;
  const int numSamples = 1000;
  
  Serial.println("Calibrating joystick... Hold still!");
  for (int i = 0; i < numSamples; i++) {
    rollSum += analogRead(X_PIN);
    pitchSum += analogRead(Y_PIN);
    yawSum += analogRead(ROTATION_PIN);
    delay(2);
  }
  
  rollOffset = map(rollSum / numSamples, 0, 1023, 1000, 2000) - 1500;
  pitchOffset = map(pitchSum / numSamples, 0, 1023, 1000, 2000) - 1500;
  yawOffset = map(yawSum / numSamples, 0, 1023, 1000, 2000) - 1500;
}