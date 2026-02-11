#ifndef MAX31855_DRIVER_H
#define MAX31855_DRIVER_H

#include <Arduino.h>
#include <SPI.h>
#include "hvac_types.h"
#include "hvac_config.h"

// ===================== MAX31855 THERMOCOUPLE DRIVER =====================

// SPI configuration for MAX31855
static const SPISettings MAX31855_SPI_SETTINGS(
  MAX31855_SPI_CLOCK_HZ,
  MSBFIRST,
  SPI_MODE0
);

// ===================== INITIALIZATION =====================
inline void initMax31855Bus() {
  SPI.begin(SPI_CLOCK_PIN, SPI_MISO_DO_PIN, /* MOSI unused */ -1);

  pinMode(MAX31855_CS_HIGH_PRESSURE, OUTPUT);
  pinMode(MAX31855_CS_LOW_PRESSURE,  OUTPUT);
  pinMode(MAX31855_CS_SUPPLY_AIR,    OUTPUT);
  pinMode(MAX31855_CS_RETURN_AIR,    OUTPUT);

  digitalWrite(MAX31855_CS_HIGH_PRESSURE, HIGH);
  digitalWrite(MAX31855_CS_LOW_PRESSURE,  HIGH);
  digitalWrite(MAX31855_CS_SUPPLY_AIR,    HIGH);
  digitalWrite(MAX31855_CS_RETURN_AIR,    HIGH);

  Serial.println("MAX31855 SPI bus initialized");
}

// ===================== RAW FRAME READING =====================
inline uint32_t readMax31855RawFrame(uint8_t chipSelectPin) {
  SPI.beginTransaction(MAX31855_SPI_SETTINGS);
  
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(20);

  uint32_t rawFrame = SPI.transfer32(0x00000000);

  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();

  return rawFrame;
}

// ===================== DECODING =====================
inline float decodeColdJunctionTempC(uint32_t rawFrame) {
  // CJ: bits [15:4], signed 12-bit, 0.0625°C/LSB
  int16_t coldJunctionCounts = (rawFrame >> 4) & 0x0FFF;
  if (coldJunctionCounts & 0x0800) {
    coldJunctionCounts |= 0xF000; // sign extend 12->16
  }
  return (float)coldJunctionCounts * 0.0625f;
}

inline float decodeThermocoupleTempC(uint32_t rawFrame, bool hasFault) {
  // TC: bits [31:18], signed 14-bit, 0.25°C/LSB
  if (hasFault) return NAN;

  int16_t thermocoupleCounts = (rawFrame >> 18) & 0x3FFF;
  if (thermocoupleCounts & 0x2000) {
    thermocoupleCounts |= 0xC000; // sign extend 14->16
  }
  return (float)thermocoupleCounts * 0.25f;
}

inline Max31855Reading decodeMax31855Frame(uint32_t rawFrame) {
  Max31855Reading reading{};
  reading.rawFrame32 = rawFrame;

  reading.hasFault          = (rawFrame & (1UL << 16)) != 0;
  reading.faultOpenCircuit  = (rawFrame & (1UL << 0))  != 0;
  reading.faultShortToGND   = (rawFrame & (1UL << 1))  != 0;
  reading.faultShortToVCC   = (rawFrame & (1UL << 2))  != 0;

  reading.coldJunctionTempC = decodeColdJunctionTempC(rawFrame);
  reading.thermocoupleTempC = decodeThermocoupleTempC(rawFrame, reading.hasFault);

  return reading;
}

// ===================== FILTERED READING =====================
// Multi-sample read with majority vote fault filtering
// Reduces "random fault flickers" due to electrical noise
inline Max31855Reading readMax31855Filtered(uint8_t chipSelectPin, 
                                             int sampleCount = MAX31855_FILTER_SAMPLE_COUNT) {
  int faultSampleCount = 0;
  uint32_t lastRawFrame = 0;
  uint32_t lastGoodFrame = 0;
  bool hasValidSample = false;

  for (int i = 0; i < sampleCount; i++) {
    uint32_t rawFrame = readMax31855RawFrame(chipSelectPin);
    lastRawFrame = rawFrame;

    bool frameHasFault = (rawFrame & (1UL << 16)) != 0;
    if (frameHasFault) {
      faultSampleCount++;
    } else {
      lastGoodFrame = rawFrame;
      hasValidSample = true;
    }

    delayMicroseconds(500);
  }

  // If majority are faults or no valid samples, return the fault
  if (!hasValidSample || faultSampleCount > (sampleCount / 2)) {
    return decodeMax31855Frame(lastRawFrame);
  }
  
  return decodeMax31855Frame(lastGoodFrame);
}

// ===================== PRINTING =====================
inline float celsiusToFahrenheit(float tempC) {
  if (isnan(tempC)) return NAN;
  return (tempC * 9.0f / 5.0f) + 32.0f;
}

inline float deltaCelsiusToDeltaFahrenheit(float deltaC) {
  if (isnan(deltaC)) return NAN;
  return deltaC * 9.0f / 5.0f;
}

inline void printTemperatureOrFaultF(const char* label, float tempC) {
  if (isnan(tempC)) {
    Serial.printf("%s: FAULT\n", label);
  } else {
    float tempF = celsiusToFahrenheit(tempC);
    Serial.printf("%s: %.2f F\n", label, tempF);
  }
}

inline void printMax31855Reading(const char* label, const Max31855Reading& reading) {
  float cjTempF = celsiusToFahrenheit(reading.coldJunctionTempC);

  Serial.printf("%s RAW=0x%08lX  CJ=%.2f F  ",
    label,
    (unsigned long)reading.rawFrame32,
    cjTempF);

  if (reading.hasFault) {
    Serial.print("TC=FAULT ");
    if (reading.faultOpenCircuit) Serial.print("[OC] ");
    if (reading.faultShortToGND)  Serial.print("[SCG] ");
    if (reading.faultShortToVCC)  Serial.print("[SCV] ");
  } else {
    float tcTempF = celsiusToFahrenheit(reading.thermocoupleTempC);
    Serial.printf("TC=%.2f F", tcTempF);
  }
  Serial.println();
}

#endif // MAX31855_DRIVER_H
