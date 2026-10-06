/*
 * OTOS + HMC6343 serial publisher for Arduino Nano ESP32
 *
 * Control input format:
 * START
 * STOP
 * RESET
 * PING
 *
 * Serial output format:
 * OTOS,<millis>,<x_m>,<y_m>,<heading_rad>
 * HMC,<millis>,<roll_rad>,<pitch_rad>,<heading_rad>,<ax_mps2>,<ay_mps2>,<az_mps2>
 */

#include <Wire.h>

#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <SFE_HMC6343.h>

namespace
{
const unsigned long kOtosPublishIntervalMs = 20;      // 50 Hz
const unsigned long kCompassPublishIntervalMs = 200;  // 5 Hz
const unsigned long kStatusIntervalMs = 1000;
const unsigned long kBridgeTimeoutMs = 1000;
const float kDegToRad = 0.01745329252f;
const float kGToMps2 = 9.80665f;

QwiicOTOS otos;
SFE_HMC6343 compass;

bool otosReady = false;
bool compassReady = false;
bool streamingEnabled = false;
unsigned long lastOtosPublishMs = 0;
unsigned long lastCompassPublishMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastBridgeSeenMs = 0;
String commandBuffer;
}

void resetSensors()
{
  if (!otosReady)
  {
    otosReady = otos.begin();
    if (otosReady)
    {
      otos.setLinearUnit(kSfeOtosLinearUnitMeters);
      otos.setAngularUnit(kSfeOtosAngularUnitRadians);
    }
  }

  compassReady = compass.init();

  if (otosReady)
  {
    otos.calibrateImu();
    otos.resetTracking();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500); // Give USB serial and HMC6343 time to settle

  Wire.begin();

  otosReady = otos.begin();
  if (otosReady)
  {
    otos.setLinearUnit(kSfeOtosLinearUnitMeters);
    otos.setAngularUnit(kSfeOtosAngularUnitRadians);
    Serial.println("STATUS,OTOS_READY");
  }
  else
  {
    Serial.println("STATUS,OTOS_INIT_FAILED");
  }

  compassReady = compass.init();
  if (compassReady)
  {
    Serial.println("STATUS,HMC_READY");
  }
  else
  {
    Serial.println("STATUS,HMC_INIT_FAILED");
  }

  resetSensors();
  lastBridgeSeenMs = millis();
}

void handleCommand(const String &command)
{
  if (command == "START")
  {
    streamingEnabled = true;
    lastBridgeSeenMs = millis();
    Serial.println("STATUS,STREAM_ON");
    return;
  }

  if (command == "STOP")
  {
    streamingEnabled = false;
    Serial.println("STATUS,STREAM_OFF");
    return;
  }

  if (command == "PING")
  {
    lastBridgeSeenMs = millis();
    return;
  }

  if (command == "RESET")
  {
    resetSensors();
    lastOtosPublishMs = 0;
    lastCompassPublishMs = 0;
    lastBridgeSeenMs = millis();
    Serial.println("STATUS,RESET_DONE");
    return;
  }

  Serial.print("STATUS,UNKNOWN_COMMAND,");
  Serial.println(command);
}

void pollCommands()
{
  while (Serial.available() > 0)
  {
    const char incoming = (char)Serial.read();
    if (incoming == '\r')
      continue;

    if (incoming == '\n')
    {
      if (commandBuffer.length() > 0)
      {
        handleCommand(commandBuffer);
        commandBuffer = "";
      }
      continue;
    }

    commandBuffer += incoming;
  }
}

void publishOtos(unsigned long nowMs)
{
  sfe_otos_pose2d_t pose;
  otos.getPosition(pose);

  Serial.print("OTOS,");
  Serial.print(nowMs);
  Serial.print(",");
  Serial.print(pose.x, 6);
  Serial.print(",");
  Serial.print(pose.y, 6);
  Serial.print(",");
  Serial.println(pose.h, 6);
}

void publishCompass(unsigned long nowMs)
{
  compass.readHeading();
  compass.readAccel();

  const float rollRad = ((float)compass.roll / 10.0f) * kDegToRad;
  const float pitchRad = ((float)compass.pitch / 10.0f) * kDegToRad;
  const float headingRad = ((float)compass.heading / 10.0f) * kDegToRad;
  const float accelXMps2 = ((float)compass.accelX / 1024.0f) * kGToMps2;
  const float accelYMps2 = ((float)compass.accelY / 1024.0f) * kGToMps2;
  const float accelZMps2 = ((float)compass.accelZ / 1024.0f) * kGToMps2;

  Serial.print("HMC,");
  Serial.print(nowMs);
  Serial.print(",");
  Serial.print(rollRad, 6);
  Serial.print(",");
  Serial.print(pitchRad, 6);
  Serial.print(",");
  Serial.print(headingRad, 6);
  Serial.print(",");
  Serial.print(accelXMps2, 6);
  Serial.print(",");
  Serial.print(accelYMps2, 6);
  Serial.print(",");
  Serial.println(accelZMps2, 6);
}

void loop()
{
  pollCommands();

  const unsigned long nowMs = millis();
  bool published = false;

  if (streamingEnabled && (nowMs - lastBridgeSeenMs >= kBridgeTimeoutMs))
  {
    streamingEnabled = false;
    Serial.println("STATUS,BRIDGE_TIMEOUT");
  }

  if (streamingEnabled && otosReady && (nowMs - lastOtosPublishMs >= kOtosPublishIntervalMs))
  {
    lastOtosPublishMs = nowMs;
    publishOtos(nowMs);
    published = true;
  }

  if (streamingEnabled && compassReady && (nowMs - lastCompassPublishMs >= kCompassPublishIntervalMs))
  {
    lastCompassPublishMs = nowMs;
    publishCompass(nowMs);
    published = true;
  }

  if (!otosReady && !compassReady && (nowMs - lastStatusMs >= kStatusIntervalMs))
  {
    lastStatusMs = nowMs;
    Serial.println("STATUS,NO_SENSORS_READY");
  }

  if (!published)
    delay(2);
}
