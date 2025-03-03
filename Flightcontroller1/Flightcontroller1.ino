#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>

// NRF24L01 CE and CSN pins
#define CE_PIN 8
#define CSN_PIN 10

// Initialize the RF24 object
RF24 radio(CE_PIN, CSN_PIN);

// Address of the communication pipe
const uint64_t pipeAddress = 0xE8E8F0F0E1LL;

// Define motor control pins (D0 to D3)
#define MOTOR1_PIN 3 //FL
#define MOTOR2_PIN 5 //FR
#define MOTOR3_PIN 6 //BR
#define MOTOR4_PIN 9 //BL

Servo motor1, motor2, motor3, motor4;

int x_value = 1500;
int y_value = 1500;
int rotation_value = 1500;
int power_value = 1000;  // Default power set to minimum

struct DroneData 
{
  int x;
  int y;
  int rotation;
  int power;
};

DroneData data;

//mpu6050 offset
float rollOffset = 0;
float pitchOffset = 0;

//remote offset
int remoteXOffset = 0;
int remoteYOffset = 0;
int remoteRotationOffset = 0;
const int remoteCalibSamples = 100; // Number of samples for calibration

// MPU6050 Variables
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float RateRoll, RatePitch, RateYaw;
float AngleRoll, AnglePitch;
float RollError, PitchError, YawError;
float RollPID, PitchPID, YawPID;

float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
const int CalibrationSamples = 2000;

// PID Variables
float Kp = 4.75, Ki = 0.03, Kd = 3; // Tune these values
float prevRollError, prevPitchError, prevYawError;
float rollIntegral, pitchIntegral, yawIntegral;
float dt = 0.01; // 10ms loop time

unsigned long lastReceivedTime = 0;

void setup() {
  // Start Serial communication for debugging
  Serial.begin(57600);
  Serial.println("Begin code");
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);  
  radio.openReadingPipe(0, pipeAddress);
  radio.setChannel(108); // Ensure the same channel as the transmitter
  radio.setDataRate(RF24_250KBPS); // Set data rate
  // Start the radio in receiving mode
  radio.startListening();

  Serial.println("Receiver initialized. Waiting for data...");

  // Attach ESCs to PWM pins
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  motor3.attach(MOTOR3_PIN);
  motor4.attach(MOTOR4_PIN);

  calibrateESCs(); 
  calibrateMPU6050();
  calibrateSensorOffsets();
  calibrateRemote();
}

void loop() 
{
  if (radio.available()) {
    lastReceivedTime = millis();
    radio.read(&data, sizeof(data));

  // Subtract remote calibration offsets before applying limits
  x_value = constrain(data.x - remoteXOffset, 1000, 2000);
  y_value = constrain(data.y - remoteYOffset, 1000, 2000);
  rotation_value = constrain(data.rotation - remoteRotationOffset, 1000, 2000);
  power_value = constrain(data.power, 1000, 2000); // Avoid full motor stop

  }

  readMPU6050();
  computePID();
  adjustMotors();
  delay(10); // Maintain 100Hz update rate

  // Debugging
  Serial.print("Throttle: "); Serial.print(power_value);
  Serial.print(" | Roll: "); Serial.print(x_value);
  Serial.print(" | Pitch: "); Serial.print(y_value);
  Serial.print(" | Yaw: "); Serial.print(rotation_value);

  // Failsafe: Stop motors if no signal received
  if (millis() - lastReceivedTime > 1000) {
    Serial.println("Signal lost! Stopping motors.");
    motor1.writeMicroseconds(1150);
    motor2.writeMicroseconds(1150);
    motor3.writeMicroseconds(1150);
    motor4.writeMicroseconds(1150);
  } 
}

void calibrateESCs() 
{
  Serial.println("Calibrating ESCs...");

  motor1.writeMicroseconds(2000);
  motor2.writeMicroseconds(2000);
  motor3.writeMicroseconds(2000);
  motor4.writeMicroseconds(2000);
  delay(3000); // Give ESCs time to register full throttle

  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(3000); // Give ESCs time to register minimum throttle

  Serial.println("ESC Calibration Done.");
}

void calibrateMPU6050() {
  Serial.println("Calibrating MPU6050...");

  for (int i = 0; i < CalibrationSamples; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);

    int16_t rawGyroX = Wire.read() << 8 | Wire.read();
    int16_t rawGyroY = Wire.read() << 8 | Wire.read();
    int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

    RateCalibrationRoll += (float)rawGyroX / 65.5;
    RateCalibrationPitch += (float)rawGyroY / 65.5;
    RateCalibrationYaw += (float)rawGyroZ / 65.5;
  }
  RateCalibrationRoll /= CalibrationSamples;
  RateCalibrationPitch /= CalibrationSamples;
  RateCalibrationYaw /= CalibrationSamples;

  Serial.println("MPU6050 Calibration Done.");
}

void readMPU6050() 
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Accelerometer data register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  
  AccX = Wire.read() << 8 | Wire.read();
  AccY = Wire.read() << 8 | Wire.read();
  AccZ = Wire.read() << 8 | Wire.read();
  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();
  
  RateRoll = (GyroX / 65.5) - RateCalibrationRoll;
  RatePitch = (GyroY / 65.5) - RateCalibrationPitch;
  RateYaw = (GyroZ / 65.5) - RateCalibrationYaw;

  // Convert accelerometer data to angles
  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI;
  AnglePitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;

  // Subtract calibration offsets to get corrected angles
  AngleRoll = AngleRoll - rollOffset;
  AnglePitch = AnglePitch - pitchOffset;
}
void computePID() 
{
  RollError = AngleRoll;
  PitchError = AnglePitch;
  YawError = RateYaw; // Use gyro for yaw correction

  // PID calculations
  rollIntegral += RollError * dt;
  pitchIntegral += PitchError * dt;
  yawIntegral += YawError * dt;

  RollPID = (Kp * RollError) + (Ki * rollIntegral) + (Kd * (RollError - prevRollError) / dt);
  PitchPID = (Kp * PitchError) + (Ki * pitchIntegral) + (Kd * (PitchError - prevPitchError) / dt);
  YawPID = (Kp * YawError) + (Ki * yawIntegral) + (Kd * (YawError - prevYawError) / dt);

  prevRollError = RollError;
  prevPitchError = PitchError;
  prevYawError = YawError;

  Serial.print("Roll: "); Serial.print(AngleRoll);
  Serial.print(" Pitch: "); Serial.print(AnglePitch);
  Serial.print(" Yaw: "); Serial.println(RateYaw);
}

void adjustMotors() 
{
  int m1_speed = power_value - (y_value - 1500) + (x_value - 1500) + (rotation_value - 1500);
  int m2_speed = power_value - (y_value - 1500) - (x_value - 1500) - (rotation_value - 1500);
  int m3_speed = power_value + (y_value - 1500) - (x_value - 1500) + (rotation_value - 1500);
  int m4_speed = power_value + (y_value - 1500) + (x_value - 1500) - (rotation_value - 1500);

  // Apply PID corrections
  m1_speed += PitchPID - RollPID - YawPID;
  m2_speed += PitchPID + RollPID + YawPID;
  m3_speed -= PitchPID - RollPID + YawPID;
  m4_speed -= PitchPID + RollPID - YawPID;

  // Constrain motor speeds
  m1_speed = constrain(m1_speed, 1100, 2000);
  m2_speed = constrain(m2_speed, 1100, 2000);
  m3_speed = constrain(m3_speed, 1100, 2000);
  m4_speed = constrain(m4_speed, 1100, 2000);

  Serial.print(" | M1: "); Serial.print(m1_speed);
  Serial.print(" | M2: "); Serial.print(m2_speed);
  Serial.print(" | M3: "); Serial.print(m3_speed);
  Serial.print(" | M4: "); Serial.println(m4_speed);

  // Send PWM signals to motors
  motor1.writeMicroseconds(m1_speed);
  motor2.writeMicroseconds(m2_speed);
  motor3.writeMicroseconds(m3_speed);
  motor4.writeMicroseconds(m4_speed);
}

void calibrateSensorOffsets() {
  Serial.println("Calibrating sensor offsets... Ensure the quadcopter is level.");
  long rollSum = 0;
  long pitchSum = 0;
  const int samples = 1000;  // Number of samples to average

  for (int i = 0; i < samples; i++) {
    readMPU6050();  // This function calculates AngleRoll and AnglePitch
    rollSum += AngleRoll;
    pitchSum += AnglePitch;
    delay(5);
  }
  
  rollOffset = rollSum / samples;
  pitchOffset = pitchSum / samples;
  
  Serial.print("Roll Offset: ");
  Serial.println(rollOffset);
  Serial.print("Pitch Offset: ");
  Serial.println(pitchOffset);
  Serial.println("Calibration complete.");
}

void calibrateRemote() {
  long sumX = 0, sumY = 0, sumRotation = 0;
  Serial.println("Calibrating remote inputs. Please leave sticks at neutral.");
  
  for (int i = 0; i < remoteCalibSamples; i++) {
    // Wait for available data before reading
    while(!radio.available()) {
      // Optionally add a small delay
      delay(5);
    }
    radio.read(&data, sizeof(data));
    sumX += data.x;
    sumY += data.y;
    sumRotation += data.rotation;
    delay(10);
  }
  
  int avgX = sumX / remoteCalibSamples;
  int avgY = sumY / remoteCalibSamples;
  int avgRotation = sumRotation / remoteCalibSamples;
  
  // Calculate offset from 1500 (neutral)
  remoteXOffset = avgX - 1500;
  remoteYOffset = avgY - 1500;
  remoteRotationOffset = avgRotation - 1500;
  
  Serial.print("Remote calibration offsets - X: ");
  Serial.print(remoteXOffset);
  Serial.print(", Y: ");
  Serial.print(remoteYOffset);
  Serial.print(", Rotation: ");
  Serial.println(remoteRotationOffset);
}
