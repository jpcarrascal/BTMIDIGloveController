/*
  DESCRIPTION:
  Sketch for a basic Bluetooth-LE MIDI glove controller.

  HARDWARE REQUIREMENTS:
  - Adafruit Bluefruit LE Feather
  - MPU6050 6-DOF inertial sensor

  LIBRARIES:
  - Adafruit BluefruitLE nRF51 library: https://github.com/adafruit/Adafruit_BluefruitLE_nRF51
  - Adafruit MPU6050
  - FortySevenEffects MIDI Arduino Library: https://github.com/FortySevenEffects/arduino_midi_library


  Author: JP Carrascal
  Based on origianl code by Adafruit Industries (Phil Burgess, Todd Treece)

*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
//#include <SPI.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BLEMIDI.h"
#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEMIDI midi(ble);
Adafruit_MPU6050 mpu;

#define CHANNEL 0  // MIDI channel number
bool isConnected = false;
bool sending = false;
bool zPeak = false, xPeak = false;
float aPrev[3] = {0, 0, 0};
float gPrev[3] = {0, 0, 0};
float gDelta[3] = {0, 0, 0};
float gDeltaPrev[3] = {0, 0, 0};
float gThreshold = 12;
int led = 13;

void setup() {
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  Serial.print(F("Bluefruit Feather: "));

  if ( !ble.begin(VERBOSE_MODE) ) {
    error(F("Couldn't find Bluefruit, check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE ) {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  ble.println("AT+GAPDEVNAME=MIDIglove");
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Set BLE callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);
  Serial.println(F("Enable MIDI: "));

  if ( ! midi.begin(true) ) {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(false);
  Serial.println(F("Waiting for a connection..."));

  // Initialize MPU
  // Borrowed from Adafruit MPU6050 examples:
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //  midi.setRxCallback(MIDI_in_callback);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if (detectPeak(g.gyro.y, gPrev[1], gDelta[1], gDeltaPrev[1], gThreshold) ) {
    sending = !sending;
    digitalWrite(13, sending);
    //if (sending) midi.send(0xB90 | CHANNEL, 63, 127);
    //else midi.send(0x90 | CHANNEL, 63, 0);
  }

  if (detectPeak(g.gyro.x, gPrev[0], gDelta[0], gDeltaPrev[0], gThreshold) ) {
    xPeak = !xPeak;
    //if (zPeak) midi.send(0xB90 | CHANNEL+1, 64, 127);
    //else midi.send(0x90 | CHANNEL+1, 64, 0);
  }

  if (detectPeak(g.gyro.z, gPrev[2], gDelta[2], gDeltaPrev[2], gThreshold) ) {
    zPeak = !zPeak;
    //if (zPeak) midi.send(0xB90 | CHANNEL+1, 65, 127);
    //else midi.send(0x90 | CHANNEL+1, 65, 0);
  }
  
  if (isConnected && sending) {
    send_controller(110, a.acceleration.x);
    send_controller(111, a.acceleration.y);
    send_controller(112, a.acceleration.z);
  }
  ble.update(1);
  //Serial.print("Is connected?");
}

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void connected(void) {
  isConnected = true;
  Serial.println(F(" CONNECTED!"));
}

void disconnected(void) {
  Serial.println("disconnected");
  isConnected = false;
}

void send_controller(int cc, int val)
{
  int index = cc - 110;
  int sendVal = map(val, -10, 10, 0, 127);
  if (aPrev[index] != sendVal) {
    aPrev[index] = sendVal;
    midi.send(0xB0 | CHANNEL, cc, sendVal);
  }
}

static inline int8_t sign(float val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

bool detectPeak(float val, float& valPrev, float& delta, float& deltaPrev, float th) {
  bool result = false;
  delta = val - valPrev;
  if (abs(val) > th && sign(delta) != sign(deltaPrev)) {
    result = true;
  }
  valPrev = val;
  deltaPrev = delta;
  return (result);
}
