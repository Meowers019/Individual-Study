#include <Arduino.h> //pinMode, digitalWrite, delay, Serial, millis
#include <Wire.h>    //I2C
#include <SPI.h>     //SPI
#include <math.h>    //isnan, fabsf, NAN


// ===================== PINOUT =====================
// If wiring changes, update these pin definitions accordingly.
// ADS1115 (I2C)
static constexpr uint8_t I2C_SDA_PIN = 21;
static constexpr uint8_t I2C_SCL_PIN = 22;
static constexpr uint8_t ADS1115_I2C_ADDRESS = 0x48;  // ADDR = GND

// MAX31855 (SPI, read-only)
static constexpr uint8_t SPI_CLOCK_PIN    = 18;
static constexpr uint8_t SPI_MISO_DO_PIN  = 19;   // DO/MISO (shared)

// MAX31855 Chip Select pins
static constexpr uint8_t MAX31855_CS_HIGH_PRESSURE = 16;  // Good cable
static constexpr uint8_t MAX31855_CS_LOW_PRESSURE  = 17;  // Good cable
static constexpr uint8_t MAX31855_CS_SUPPLY_AIR    = 26;  // Shit cable
static constexpr uint8_t MAX31855_CS_RETURN_AIR    = 27;  // Shit cable

// SPI configuration for MAX31855
static const SPISettings MAX31855_SPI_SETTINGS(
  1000000,      // 1 MHz
  MSBFIRST,
  SPI_MODE0
);
//----------------------------------------------------------------------//

// ===================== HVAC MODE / DIAGNOSTICS =====================
// "High level" HVAC operating modes inferred from temperature readings (for now)
enum class HvacMode : uint8_t {        // what is the unit doing
  Off = 0,
  Cooling,
  Heating
};

enum class DiagnosticState : uint8_t { // is it behaving correctly?
  Normal = 0,
  SensorFault,
  WeakPerformance,
  Unknown
};

enum class HvacSystemState : uint8_t {
  Off = 0,
  Running
};

// Placeholder for future fault substates (you’ll expand later)
enum class HvacSubstate : uint8_t {
  None = 0,
  Unknown
};

//----------------------------------------------------------------------//


// ===================== STRING HELPERS =====================
// String helpers for printing enums (clean debug output)
static const char* hvacModeToString(HvacMode mode) {
  switch (mode) {
    case HvacMode::Off:     return "OFF";
    case HvacMode::Cooling: return "COOLING";
    case HvacMode::Heating: return "HEATING";
    default:                return "UNKNOWN";
  }
}

static const char* diagToString(DiagnosticState diag) {
  switch (diag) {
    case DiagnosticState::Normal:          return "NORMAL";
    case DiagnosticState::SensorFault:     return "SENSOR_FAULT";
    case DiagnosticState::WeakPerformance: return "WEAK_PERFORMANCE";
    case DiagnosticState::Unknown:         return "UNKNOWN";
    default:                               return "UNKNOWN";
  }
}
//----------------------------------------------------------------------//


// ===================== HVAC STATE =====================
// Will expand later with more diagnostic info, timers, etc.
struct HvacState {
  HvacSystemState systemState; // OFF/RUNNING (based on deltaT OR deltaPsi)
  HvacMode mode;               // Off/Cooling/Heating (what kind of running)
  HvacSubstate substate;       // future: fault subcases
  DiagnosticState diagnostic;
  uint32_t lastModeChangeMs;
};

// Start conservative
static HvacState hvacState = {
  HvacSystemState::Off,
  HvacMode::Off,
  HvacSubstate::None,
  DiagnosticState::Unknown,
  0
};
//----------------------------------------------------------------------//


// ===================== HVAC TEMPERATURE STATE =====================
// Temps are stored in Celsius internally, printed in Fahrenheit.
// DeltaTempC = Return - Supply (summer)
// heatingDeltaTempC = Supply - Return (winter) or abs(DeltaTempC)
struct HvacTemperatures {
  float highPressureLineTempC; // TC temp (°C) when valid
  float lowPressureLineTempC;  // TC temp (°C) when valid
  float supplyAirTempC;        // TC temp (°C) when valid
  float returnAirTempC;        // TC temp (°C) when valid
  float deltaTempC;            // Return - Supply (°C)
  uint32_t updatedAtMs;
};

// ===================== TEMP CALIBRATION =====================
// +XX°F offset converted to Celsius: 20 * 5 / 9 ≈ 11.11°C
// Shitty cables that need replacement
static constexpr float SUPPLY_TEMP_OFFSET_C = 1.0f * 5.0f / 9.0f;
static constexpr float RETURN_TEMP_OFFSET_C = 1.0f * 5.0f / 9.0f;



// Portable, explicit initialization
static HvacTemperatures hvacTemps = {
  NAN,  // highPressureLineTempC
  NAN,  // lowPressureLineTempC
  NAN,  // supplyAirTempC
  NAN,  // returnAirTempC
  NAN,  // deltaTempC
  0     // updatedAtMs
};
//----------------------------------------------------------------------//


// ===================== I2C =====================
static bool isI2CDevicePresent(TwoWire &bus, uint8_t address) {
  bus.beginTransmission(address);
  return (bus.endTransmission() == 0);
}

static void initADS1115() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  if (isI2CDevicePresent(Wire, ADS1115_I2C_ADDRESS)) {
    Serial.println("ADS1115 detected");
  } else {
    Serial.println("WARNING: ADS1115 not responding");
  }
}
//----------------------------------------------------------------------//


// ===================== ADS1115 (ADC) HELPERS =====================
// NOTE: ADS1115 has a full-scale range that depends on PGA gain.
// We'll use ±4.096V range (gain=1), which is safe for a 3.3V-powered ADS1115.
static constexpr uint16_t ADS1115_REG_CONVERSION = 0x00;
static constexpr uint16_t ADS1115_REG_CONFIG     = 0x01;

// PGA = ±4.096V (GAIN=1)
static constexpr uint16_t ADS1115_PGA_4_096V = 0x0200;

// Single-shot mode
static constexpr uint16_t ADS1115_MODE_SINGLESHOT = 0x0100;

// Data rate = 128 SPS (stable). You can bump to 250/475/860 later.
static constexpr uint16_t ADS1115_DR_128SPS = 0x0080;

// Disable comparator (we're just reading values)
static constexpr uint16_t ADS1115_COMP_DISABLE = 0x0003;

// Voltage divider constants (your 10k over 20k divider)
static constexpr float VOLTAGE_DIVIDER_GAIN_TO_SENSOR = 3.0f / 2.0f; // Vsensor = Vadc * 1.5

// Convert ADS1115 counts to volts for the selected PGA range
static float ads1115CountsToVolts(int16_t counts, float fullScaleVolts) {
  // ADS1115 is 16-bit signed. Full scale is +/- fullScaleVolts.
  // LSB size = fullScaleVolts / 32768.
  return (float)counts * (fullScaleVolts / 32768.0f);
}

// Write 16-bit value to ADS1115 register
static bool ads1115WriteRegister16(uint8_t i2cAddress, uint16_t reg, uint16_t value) {
  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)(value >> 8));   // MSB
  Wire.write((uint8_t)(value & 0xFF)); // LSB
  return (Wire.endTransmission() == 0);
}

// Read 16-bit value from ADS1115 register
static bool ads1115ReadRegister16(uint8_t i2cAddress, uint16_t reg, uint16_t &outValue) {
  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)reg);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)i2cAddress, 2) != 2) return false;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  outValue = ((uint16_t)msb << 8) | lsb;
  return true;
}

// Read single-ended channel (0-3) in single-shot mode. Returns true if successful.
static bool readAds1115SingleEndedCounts(uint8_t channel, int16_t &outCounts) {
  if (channel > 3) return false;

  // MUX bits for single-ended:
  // AIN0-GND = 100, AIN1-GND = 101, AIN2-GND = 110, AIN3-GND = 111
  uint16_t muxBits = 0x4000 + (channel * 0x1000); // 0x4000,0x5000,0x6000,0x7000

  // OS=1 (start single conversion)
  uint16_t config =
    0x8000 |                 // OS = 1 (start conversion)
    muxBits |                // MUX = single-ended channel
    ADS1115_PGA_4_096V |      // PGA = ±4.096V
    ADS1115_MODE_SINGLESHOT | // single-shot
    ADS1115_DR_128SPS |       // data rate
    ADS1115_COMP_DISABLE;     // comparator disabled

  if (!ads1115WriteRegister16(ADS1115_I2C_ADDRESS, ADS1115_REG_CONFIG, config)) {
    return false;
  }

  // Wait long enough for one conversion at 128 SPS: ~7.8ms
  delay(10);

  uint16_t rawConversion = 0;
  if (!ads1115ReadRegister16(ADS1115_I2C_ADDRESS, ADS1115_REG_CONVERSION, rawConversion)) {
    return false;
  }

  outCounts = (int16_t)rawConversion;
  return true;
}

// Read channel voltage at ADS1115 pin (after divider)
static bool readAds1115ChannelVoltageAtAdcPin(uint8_t channel, float &outVoltageAtAdcPin) {
  int16_t counts = 0;
  if (!readAds1115SingleEndedCounts(channel, counts)) {
    outVoltageAtAdcPin = NAN;
    return false;
  }

  // For PGA ±4.096V
  outVoltageAtAdcPin = ads1115CountsToVolts(counts, 4.096f);
  return true;
}

// Convert ADC-pin voltage back to the *sensor output* voltage (undo divider)
static float convertAdcPinVoltageToSensorVoltage(float adcPinVoltage) {
  if (isnan(adcPinVoltage)) return NAN;
  return adcPinVoltage * VOLTAGE_DIVIDER_GAIN_TO_SENSOR;
}
//----------------------------------------------------------------------//


// ===================== MAX31855 (TC + CJ) =====================
// Full decoded reading (per sensor)
struct Max31855Reading {
  float thermocoupleTempC;     // NAN if fault
  float coldJunctionTempC;     // usually valid even if TC fault
  bool hasFault;
  bool faultOpenCircuit;
  bool faultShortToGND;
  bool faultShortToVCC;
  uint32_t rawFrame32;
};

static uint32_t readMax31855RawFrame(uint8_t chipSelectPin) {
  SPI.beginTransaction(MAX31855_SPI_SETTINGS);
  // Cycles through each CS pins and read data on MAX31855
  digitalWrite(chipSelectPin, LOW); 
  delayMicroseconds(20);

  uint32_t rawFrame = SPI.transfer32(0x00000000);

  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();

  return rawFrame;
}

static float celsiusToFahrenheit(float tempC) {
  if (isnan(tempC)) return NAN;
  return (tempC * 9.0f / 5.0f) + 32.0f;
}

static float deltaCelsiusToDeltaFahrenheit(float deltaC) {
  if (isnan(deltaC)) return NAN;
  return deltaC * 9.0f / 5.0f;
}


static float decodeColdJunctionTempC(uint32_t rawFrame) {
  // CJ: bits [15:4], signed 12-bit, 0.0625C/LSB
  int16_t coldJunctionCounts = (rawFrame >> 4) & 0x0FFF;
  if (coldJunctionCounts & 0x0800) {
    coldJunctionCounts |= 0xF000; // sign extend 12->16
  }
  return (float)coldJunctionCounts * 0.0625f;
}

static float decodeThermocoupleTempC(uint32_t rawFrame, bool hasFault) {
  // TC: bits [31:18], signed 14-bit, 0.25C/LSB
  if (hasFault) return NAN;

  int16_t thermocoupleCounts = (rawFrame >> 18) & 0x3FFF;
  if (thermocoupleCounts & 0x2000) {
    thermocoupleCounts |= 0xC000; // sign extend 14->16
  }
  return (float)thermocoupleCounts * 0.25f;
}

static Max31855Reading decodeMax31855Frame(uint32_t rawFrame) {
  // Reading fault bits and calls decode helpers
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

// Multi-sample read: majority vote fault filtering, uses last good frame when possible.
// This reduces "random fault flickers" due to electrical noise.
static Max31855Reading readMax31855Filtered(uint8_t chipSelectPin, int sampleCount = 5) {
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

  if (!hasValidSample || faultSampleCount > (sampleCount / 2)) {
    return decodeMax31855Frame(lastRawFrame);
  }
  return decodeMax31855Frame(lastGoodFrame);
}
//----------------------------------------------------------------------//


// ===================== INIT =====================
// Configures SPI bus and CS pins as outputs, sets them HIGH (inactive)
static void initMax31855Bus() {
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
//----------------------------------------------------------------------//


// ===================== OUTPUT =====================
// Print temperature or FAULT if NAN
static void printTemperatureOrFaultF(const char* label, float tempC) {
  if (isnan(tempC)) {
    Serial.printf("%s: FAULT\n", label);
  } else {
    float tempF = celsiusToFahrenheit(tempC);
    Serial.printf("%s: %.2f F\n", label, tempF);
  }
}

// Prints a debug line for a MAX31855 reading
static void printMax31855Reading(const char* label, const Max31855Reading& reading) {
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
//----------------------------------------------------------------------//


// ===================== MODE DETECTION (TEMP-ONLY) =====================
// These thresholds are STARTING POINTS. Tune with real duct measurements.
// Will revisit later with more sophisticated logic. 
// coolingDeltaTempC = Return - Supply (summer)
// heatingDeltaTempC = Supply - Return (winter) or abs(coolingDeltaTempC)

static constexpr float DELTA_COOL_ON_C      = +3.0f; // return - supply >= +3C
static constexpr float DELTA_HEAT_ON_C      = -3.0f; // return - supply <= -3C
static constexpr float DELTA_OFF_BAND_C     =  1.0f; // |delta| <= 1C means likely off/fan-only

static constexpr float SUPPLY_COOL_MAX_C    = 18.0f; // supply <= 18C supports cooling
static constexpr float SUPPLY_HEAT_MIN_C    = 30.0f; // supply >= 30C supports heating

static constexpr int   MODE_CONFIRM_COUNT   = 3;    // require N consecutive confirmations

// ===================== RUNNING DETECTION =====================
// Placeholders you will tune later:
static constexpr float RUNNING_DELTA_T_C_THRESHOLD  = 0.6f;   // ~1.1F
static constexpr float RUNNING_DELTA_PSI_THRESHOLD  = 25.0f;

static float absf_safe(float v) { return (v < 0.0f) ? -v : v; }
static bool okf(float v) { return !isnan(v); }


static bool isValidTemp(float t) { return !isnan(t); }

static void updateHvacModeFromTemperatures(const HvacTemperatures& temps, HvacState& state) {
  static int coolingConfirmCount = 0;
  static int heatingConfirmCount = 0;
  static int offConfirmCount     = 0;

  // NEW: if system isn't running, don't try to decide cooling/heating
  if (state.systemState != HvacSystemState::Running) {
    state.mode = HvacMode::Off;
    return;
  }

  // you also need this validity guard back (or you’ll compare NANs)
  if (!isValidTemp(temps.supplyAirTempC) || !isValidTemp(temps.returnAirTempC) || isnan(temps.deltaTempC)) {
    state.diagnostic = DiagnosticState::SensorFault;
    return;
  }

  const bool looksOffByDelta     = (fabsf(temps.deltaTempC) <= DELTA_OFF_BAND_C);

  const bool looksCoolingByDelta = (temps.deltaTempC >= DELTA_COOL_ON_C);
  const bool looksCoolingByDuct  = (temps.supplyAirTempC <= SUPPLY_COOL_MAX_C);

  const bool looksHeatingByDelta = (temps.deltaTempC <= DELTA_HEAT_ON_C);
  const bool looksHeatingByDuct  = (temps.supplyAirTempC >= SUPPLY_HEAT_MIN_C);

  const bool coolingEvidence = looksCoolingByDelta && looksCoolingByDuct;
  const bool heatingEvidence = looksHeatingByDelta && looksHeatingByDuct;
  const bool offEvidence     = looksOffByDelta;

  // Update counters
  coolingConfirmCount = coolingEvidence ? (coolingConfirmCount + 1) : 0;
  heatingConfirmCount = heatingEvidence ? (heatingConfirmCount + 1) : 0;
  offConfirmCount     = offEvidence     ? (offConfirmCount + 1)     : 0;

  // Decide mode with persistence (prevents flip-flop)
  HvacMode previousMode = state.mode;
  HvacMode newMode = previousMode;

  if (coolingConfirmCount >= MODE_CONFIRM_COUNT) newMode = HvacMode::Cooling;
  else if (heatingConfirmCount >= MODE_CONFIRM_COUNT) newMode = HvacMode::Heating;
  else if (offConfirmCount     >= MODE_CONFIRM_COUNT) newMode = HvacMode::Off;

  if (newMode != previousMode) {
    state.mode = newMode;
    state.lastModeChangeMs = millis();
  }

  // Basic diagnostic placeholder (you’ll expand this later)
  state.diagnostic = DiagnosticState::Normal;
}

// ===================== PSI READINGS =====================
// Pressures in PSI (computed from ADS1115 voltages + divider undo).
struct HvacPressures {
  float lowSidePressurePsi;    // TDH33 on ADS1115 A0
  float highSidePressurePsi;   // TD1000 on ADS1115 A1
  uint32_t updatedAtMs;        // millis() when these pressures were updated
};

// Portable, explicit initialization
static HvacPressures hvacPressures = {
  NAN, // lowSidePressurePsi
  NAN, // highSidePressurePsi
  0    // updatedAtMs
};

static void updateHvacRunStateFromDeltaTOrDeltaPsi(
  const HvacTemperatures& t,
  const HvacPressures& p,
  HvacState& s
) {
  // deltaT_C is Return - Supply (already computed in your loop)
  const bool deltaTOk = okf(t.deltaTempC);

  // deltaPsi = High - Low
  const float deltaPsi =
    (okf(p.highSidePressurePsi) && okf(p.lowSidePressurePsi))
      ? (p.highSidePressurePsi - p.lowSidePressurePsi)
      : NAN;

  const bool deltaPsiOk = okf(deltaPsi);

  // If neither metric exists, don't guess
  if (!deltaTOk && !deltaPsiOk) {
    s.diagnostic = DiagnosticState::SensorFault;
    return;
  }

  const bool runningByDeltaT   = deltaTOk   && (absf_safe(t.deltaTempC) >= RUNNING_DELTA_T_C_THRESHOLD);
  const bool runningByDeltaPsi = deltaPsiOk && (absf_safe(deltaPsi)     >= RUNNING_DELTA_PSI_THRESHOLD);

  const HvacSystemState newSys = (runningByDeltaT || runningByDeltaPsi)
    ? HvacSystemState::Running
    : HvacSystemState::Off;

  // Transition handling
  if (newSys != s.systemState) {
    s.systemState = newSys;
    s.lastModeChangeMs = millis();
  }

  // If system is OFF, force mode OFF for now (you said we'll revisit OFF logic later)
  if (s.systemState == HvacSystemState::Off) {
    s.mode = HvacMode::Off;
    s.substate = HvacSubstate::None;
  }

  // Don’t overwrite diagnostic here if it was already set to SensorFault by other code
  if (s.diagnostic == DiagnosticState::Unknown) {
    s.diagnostic = DiagnosticState::Normal;
  }
}

//----------------------------------------------------------------------//


// ===================== ARDUINO =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== HVAC ESP32 Bring-up (MAX31855 TC + CJ + Mode) ===");

  initADS1115();
  initMax31855Bus();

  Serial.println("Init complete.\n");
}
//----------------------------------------------------------------------//


// ===================== PRESSURE SENSOR CONSTANTS =====================

// Voltage divider: sensor -> 10k -> ADC -> 20k -> GND
// Vadc = Vsensor * (20k / (10k+20k)) = Vsensor * (2/3)
// Vsensor = Vadc * (3/2)
// A0 - TDH33 (0-1000 PSI - 0-5V), A1 - TD1000 (0-600 PSI - 1-5V)

// A0 sensor: 0-5V => 0-1000 PSI
static constexpr float PRESSURE_SENSOR_A0_FULL_SCALE_PSI = 1000.0f;
static constexpr float PRESSURE_SENSOR_A0_VOLTAGE_AT_ZERO_PSI = 0.0f;
static constexpr float PRESSURE_SENSOR_A0_VOLTAGE_AT_FULL_PSI = 5.0f;

// A1 sensor: 1-5V => 0-600 PSI
static constexpr float PRESSURE_SENSOR_A1_FULL_SCALE_PSI = 600.0f;
static constexpr float PRESSURE_SENSOR_A1_VOLTAGE_AT_ZERO_PSI = 1.0f;
static constexpr float PRESSURE_SENSOR_A1_VOLTAGE_AT_FULL_PSI = 5.0f;

// Calibrate these by reading voltage when pressure is known = 0 psi
static constexpr float A0_ZERO_CAL_V = 0.151f;  // <-- update from your live reading
static constexpr float A1_ZERO_CAL_V = 1.025f;  // <-- update from your live reading
//----------------------------------------------------------------------//

static float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

static float convertSensorVoltageToPsiCalibrated(
  float sensorVoltageV,
  float voltageAtFullScalePsi,
  float fullScalePsi,
  float zeroCalVoltageV
) {
  if (isnan(sensorVoltageV)) return NAN;

  const float spanV = voltageAtFullScalePsi - zeroCalVoltageV;
  if (spanV <= 0.0f || fullScalePsi <= 0.0f) return NAN;

  // Clamp to [zeroCal, fullScaleV]
  float v = clampFloat(sensorVoltageV, zeroCalVoltageV, voltageAtFullScalePsi);

  float fraction = (v - zeroCalVoltageV) / spanV;
  float psi = fraction * fullScalePsi;

  // Just to be safe against tiny negatives from noise
  if (psi < 0.0f) psi = 0.0f;
  return psi;
}

static float applyPsiDeadband(float psi, float deadbandPsi = 0.5f) {
  if (isnan(psi)) return NAN;
  return (psi < deadbandPsi) ? 0.0f : psi;
}

static bool readPressureSensorA0Psi(float &outPressurePsi) {
  float adcPinVoltageV = NAN;

  if (!readAds1115ChannelVoltageAtAdcPin(0, adcPinVoltageV)) {
    outPressurePsi = NAN;
    return false;
  }

  float sensorVoltageV = convertAdcPinVoltageToSensorVoltage(adcPinVoltageV);

  // A0: 0-5V -> 0-1000psi, calibrated zero
  outPressurePsi = convertSensorVoltageToPsiCalibrated(
    sensorVoltageV,
    PRESSURE_SENSOR_A0_VOLTAGE_AT_FULL_PSI,   // 5.0
    PRESSURE_SENSOR_A0_FULL_SCALE_PSI,        // 1000
    A0_ZERO_CAL_V
  );

  outPressurePsi = applyPsiDeadband(outPressurePsi, 0.5f);
  return true;
}

static bool readPressureSensorA1Psi(float &outPressurePsi) {
  float adcPinVoltageV = NAN;

  if (!readAds1115ChannelVoltageAtAdcPin(1, adcPinVoltageV)) {
    outPressurePsi = NAN;
    return false;
  }

  float sensorVoltageV = convertAdcPinVoltageToSensorVoltage(adcPinVoltageV);

  // A1: 1-5V -> 0-600psi, calibrated zero
  outPressurePsi = convertSensorVoltageToPsiCalibrated(
    sensorVoltageV,
    PRESSURE_SENSOR_A1_VOLTAGE_AT_FULL_PSI,   // 5.0
    PRESSURE_SENSOR_A1_FULL_SCALE_PSI,        // 600
    A1_ZERO_CAL_V
  );

  outPressurePsi = applyPsiDeadband(outPressurePsi, 0.5f);
  return true;
}


static void updateHvacPressures() {
  hvacPressures.lowSidePressurePsi  = NAN;
  hvacPressures.highSidePressurePsi = NAN;

  bool a0Ok = readPressureSensorA0Psi(hvacPressures.lowSidePressurePsi);
  bool a1Ok = readPressureSensorA1Psi(hvacPressures.highSidePressurePsi);

  if (a0Ok || a1Ok) {
    hvacPressures.updatedAtMs = millis();
  }
}

static void printPressureOrFault(const char* label, float pressurePsi) {
  if (isnan(pressurePsi)) {
    Serial.printf("%s: FAULT\n", label);
  } else {
    Serial.printf("%s: %.1f PSI\n", label, pressurePsi);
  }
}
//----------------------------------------------------------------------//

// ===================== SATURATION TEMPERATURE (R-22) =====================
// Uses low/high side pressures to estimate saturation temps from an R-22 P/T chart.
// Table format: [psi_low_inclusive, psi_high_exclusive, sat_temp_F]
struct PressureToTempRangeF {
  float psiLow;
  float psiHigh;
  float satTempF;
};

// R-22 saturation table. Update for different refrigerants as needed.
// Source: standard R-22 P/T chart
static constexpr PressureToTempRangeF R22_PT_TABLE_F[] = {
  {0, 50, 0}, {50, 52, 1}, {52, 54, 3}, {54, 56, 4}, {56, 58, 6},
  {58, 60, 7}, {62, 64, 10}, {64, 66, 11}, {66, 68, 12}, {68, 70, 14},
  {70, 72, 15}, {72, 74, 16}, {74, 76, 17}, {76, 78, 18}, {78, 80, 20},
  {80, 85, 21}, {85, 90, 24}, {90, 95, 26}, {95, 100, 29}, {100, 105, 31},
  {105, 110, 34}, {110, 115, 36}, {115, 120, 38}, {120, 125, 41},
  {125, 130, 43}, {130, 135, 45}, {135, 140, 47}, {140, 145, 49},
  {145, 150, 51}, {150, 155, 53}, {155, 160, 54}, {160, 165, 56},
  {165, 170, 58}, {170, 175, 60}, {175, 180, 61}, {180, 185, 63},
  {185, 190, 65}, {190, 195, 66}, {195, 200, 68}, {200, 205, 69},
  {205, 210, 71}, {210, 220, 72}, {220, 230, 75}, {230, 240, 78},
  {240, 250, 81}, {250, 260, 84}, {260, 275, 86}, {275, 290, 90},
  {290, 305, 94}, {305, 320, 97}, {320, 335, 100}, {335, 350, 104},
  {350, 365, 107}, {365, 380, 110}, {380, 400, 113}, {400, 420, 116},
  {420, 440, 120}, {440, 460, 124}, {460, 480, 127}, {480, 500, 130},
  {500, 525, 134},
  // last "catch-all" from your python list
  {525, 10000, 1000}
};

static constexpr size_t R22_PT_TABLE_COUNT = sizeof(R22_PT_TABLE_F) / sizeof(R22_PT_TABLE_F[0]);

// Saturation temps derived from pressures
struct HvacSaturationTemps {
  float lowSideSatTempF;   // from low side pressure
  float highSideSatTempF;  // from high side pressure
  uint32_t updatedAtMs;
};

// Init
static HvacSaturationTemps hvacSatTemps = {
  NAN, // lowSideSatTempF
  NAN, // highSideSatTempF
  0    // updatedAtMs
};

// Lookup helper: PSI -> Saturation Temp (F)
static float r22PsiToSaturationTempF(float psi) {
  if (isnan(psi) || psi < 0.0f) return NAN;

  // Scan the table and return the sat temp for the first matching range
  for (size_t i = 0; i < R22_PT_TABLE_COUNT; i++) {
    const auto &refrig = R22_PT_TABLE_F[i];
    if (psi >= refrig.psiLow && psi < refrig.psiHigh) {
      return refrig.satTempF;
    }
  }
  return NAN; // should not happen due to catch-all, but safe
}

// Update saturation temps from current hvacPressures
static void updateHvacSaturationTempsFromPressures() {
  hvacSatTemps.lowSideSatTempF  = r22PsiToSaturationTempF(hvacPressures.lowSidePressurePsi);
  hvacSatTemps.highSideSatTempF = r22PsiToSaturationTempF(hvacPressures.highSidePressurePsi);
  hvacSatTemps.updatedAtMs = millis();
}

// Print helper
static void printSatTempOrFault(const char* label, float tempF) {
  if (isnan(tempF)) {
    Serial.printf("%s: FAULT\n", label);
  } else {
    Serial.printf("%s: %.1f F\n", label, tempF);
  }
}
//----------------------------------------------------------------------//


// ===================== SUPERHEAT / SUBCOOLING =====================
// Assumes:
//  - lowPressureLineTempC  = suction line temp (evaporator outlet)
//  - highPressureLineTempC = liquid line temp (condenser outlet)
//  - hvacSatTemps.lowSideSatTempF  from low side pressure
//  - hvacSatTemps.highSideSatTempF from high side pressure
struct HvacShSc {
  float superheatF;     // suction temp - low side sat temp
  float subcoolingF;    // high side sat temp - liquid temp
  uint32_t updatedAtMs;
};

static HvacShSc hvacShSc = {
  NAN, // superheatF
  NAN, // subcoolingF
  0    // updatedAtMs
};


static float computeSuperheatF(float suctionLineTempF, float lowSideSatTempF) {
  if (isnan(suctionLineTempF) || isnan(lowSideSatTempF)) return NAN;
  return suctionLineTempF - lowSideSatTempF;
}

static float computeSubcoolingF(float highSideSatTempF, float liquidLineTempF) {
  if (isnan(highSideSatTempF) || isnan(liquidLineTempF)) return NAN;
  return highSideSatTempF - liquidLineTempF;
}

// Update SH/SC from your existing hvacTemps + hvacSatTemps
static void updateHvacSuperheatSubcool() {
  // Convert line temps to F
  const float suctionLineTempF = celsiusToFahrenheit(hvacTemps.lowPressureLineTempC);
  const float liquidLineTempF  = celsiusToFahrenheit(hvacTemps.highPressureLineTempC);

  hvacShSc.superheatF  = computeSuperheatF(suctionLineTempF, hvacSatTemps.lowSideSatTempF);
  hvacShSc.subcoolingF = computeSubcoolingF(hvacSatTemps.highSideSatTempF, liquidLineTempF);

  hvacShSc.updatedAtMs = millis();
}

static void printShScOrFault(const char* label, float valueF) {
  if (isnan(valueF)) Serial.printf("%s: FAULT\n", label);
  else               Serial.printf("%s: %.1f F\n", label, valueF);
}
//----------------------------------------------------------------------//


// ===================== FaultCode(s) ====================
enum class FaultCode : uint16_t {
  None = 0,

  // Sensor / compute validity
  Fault_MissingTemp,
  Fault_MissingPressure,
  Fault_MissingSatTemp,
  Fault_MissingShSc,

  // Pressure-based
  Fault_StaticNotEqualizing,
  Fault_HighPsiTooHigh,
  Fault_LowPsiTooLow,

  // Line-temp based
  Fault_SuctionLineFreezing,
  Fault_LiquidLineTooHot,

  // Delta-T based
  Fault_DeltaTTooLow,
  Fault_DeltaTTooHigh,
  Fault_DeltaTZero,

  // SH/SC based
  Fault_SubcoolLow,
  Fault_SubcoolHigh,
  Fault_SuperheatLow,
  Fault_SuperheatHigh,

  // Cross-check combos (optional but useful)
  Fault_LowChargePattern,      // low subcool + high superheat
  Fault_RestrictionPattern     // high subcool + high superheat (often)
};

// ===================== MULTI-FAULT REPORT =====================
static constexpr uint8_t MAX_ACTIVE_FAULTS = 16;

struct FaultReport {
  FaultCode codes[MAX_ACTIVE_FAULTS];
  uint8_t count;
  DiagnosticState diag;
};

static void initFaultReport(FaultReport &r) {
  r.count = 0;
  r.diag = DiagnosticState::Normal;
}

static void bumpDiag(FaultReport &r, DiagnosticState d) {
  // Worst wins: SensorFault > WeakPerformance > Normal
  if (d == DiagnosticState::SensorFault) {
    r.diag = DiagnosticState::SensorFault;
  } else if (d == DiagnosticState::WeakPerformance) {
    if (r.diag != DiagnosticState::SensorFault) r.diag = DiagnosticState::WeakPerformance;
  }
}

static bool reportHasFault(const FaultReport &r, FaultCode code) {
  for (uint8_t i = 0; i < r.count; i++) {
    if (r.codes[i] == code) return true;
  }
  return false;
}

static void addFault(FaultReport &r, FaultCode code, DiagnosticState severity) {
  if (code == FaultCode::None) return;

  // Avoid duplicates
  if (reportHasFault(r, code)) {
    bumpDiag(r, severity);
    return;
  }

  // Append if room
  if (r.count < MAX_ACTIVE_FAULTS) {
    r.codes[r.count++] = code;
  }
  // Even if full, still bump diag
  bumpDiag(r, severity);
}

static const char* faultCodeToString(FaultCode f) {
  switch (f) {
    case FaultCode::None: return "NONE";

    case FaultCode::Fault_MissingTemp: return "FAULT_MISSING_TEMP";
    case FaultCode::Fault_MissingPressure: return "FAULT_MISSING_PRESSURE";
    case FaultCode::Fault_MissingSatTemp: return "FAULT_MISSING_SATTEMP";
    case FaultCode::Fault_MissingShSc: return "FAULT_MISSING_SHSC";

    case FaultCode::Fault_StaticNotEqualizing: return "FAULT_STATIC_NOT_EQUALIZING";
    case FaultCode::Fault_HighPsiTooHigh: return "FAULT_HIGH_PSI_TOO_HIGH";
    case FaultCode::Fault_LowPsiTooLow: return "FAULT_LOW_PSI_TOO_LOW";

    case FaultCode::Fault_SuctionLineFreezing: return "FAULT_SUCTION_LINE_FREEZING";
    case FaultCode::Fault_LiquidLineTooHot: return "FAULT_LIQUID_LINE_TOO_HOT";

    case FaultCode::Fault_DeltaTTooLow: return "FAULT_DELTAT_TOO_LOW";
    case FaultCode::Fault_DeltaTTooHigh: return "FAULT_DELTAT_TOO_HIGH";
    case FaultCode::Fault_DeltaTZero: return "FAULT_DELTAT_ZERO";

    case FaultCode::Fault_SubcoolLow: return "FAULT_SUBCOOL_LOW";
    case FaultCode::Fault_SubcoolHigh: return "FAULT_SUBCOOL_HIGH";
    case FaultCode::Fault_SuperheatLow: return "FAULT_SUPERHEAT_LOW";
    case FaultCode::Fault_SuperheatHigh: return "FAULT_SUPERHEAT_HIGH";

    case FaultCode::Fault_LowChargePattern: return "FAULT_LOW_CHARGE_PATTERN";
    case FaultCode::Fault_RestrictionPattern: return "FAULT_RESTRICTION_PATTERN";

    default: return "FAULT_UNKNOWN";
  }
}

static FaultReport evaluateFaultsHeatPumpAll(
  const HvacTemperatures& t,
  const HvacPressures& p,
  const HvacSaturationTemps& sat,
  const HvacShSc& shsc,
  HvacSystemState sys,
  HvacMode mode
) {
  FaultReport r;
  initFaultReport(r);

  // ---------- PLACEHOLDER THRESHOLDS (tune later) ----------
  static constexpr float PSI_HIGH_MAX_COOL = 500.0f;   // X
  static constexpr float PSI_LOW_MIN_COOL  = 40.0f;    // X

  static constexpr float SUCTION_FREEZE_F  = 32.0f;
  static constexpr float LIQUID_TOO_HOT_F  = 130.0f;   // X

  static constexpr float DELTAT_LOW_F      = 10.0f;    // X
  static constexpr float DELTAT_HIGH_F     = 25.0f;    // X
  static constexpr float DELTAT_ZERO_F     = 2.0f;

  static constexpr float SUBCOOL_LOW_F     = 5.0f;     // X
  static constexpr float SUBCOOL_HIGH_F    = 20.0f;    // X
  static constexpr float SUPERHEAT_LOW_F   = 2.0f;
  static constexpr float SUPERHEAT_HIGH_F  = 25.0f;    // X
  //----------------------------------------------------------

  auto ok = [](float v) { return !isnan(v); };

  // Convert needed temps to F for comparisons
  const float suctionF = celsiusToFahrenheit(t.lowPressureLineTempC);
  const float liquidF  = celsiusToFahrenheit(t.highPressureLineTempC);
  const float deltaTF  = deltaCelsiusToDeltaFahrenheit(t.deltaTempC);

  // ---------- DATA VALIDITY (add faults, don't return) ----------
  if (!ok(t.supplyAirTempC) || !ok(t.returnAirTempC) || isnan(t.deltaTempC)) {
    addFault(r, FaultCode::Fault_MissingTemp, DiagnosticState::SensorFault);
  }
  if (!ok(p.lowSidePressurePsi) || !ok(p.highSidePressurePsi)) {
    addFault(r, FaultCode::Fault_MissingPressure, DiagnosticState::SensorFault);
  }
  if (!ok(sat.lowSideSatTempF) || !ok(sat.highSideSatTempF)) {
    addFault(r, FaultCode::Fault_MissingSatTemp, DiagnosticState::SensorFault);
  }
  if (!ok(shsc.superheatF) || !ok(shsc.subcoolingF)) {
    addFault(r, FaultCode::Fault_MissingShSc, DiagnosticState::SensorFault);
  }

  // If system is OFF (true OFF), skip the rest for now (per your plan)
  if (sys == HvacSystemState::Off) {
  // OFF-STATE EQUALIZATION CHECK (placeholder thresholds)
  // If unit is truly off, high/low should drift toward equal over time.
  // If deltaPsi stays high, it can suggest restriction / stuck TXV / valves not bypassing.

  static constexpr float STATIC_EQUALIZE_DELTA_PSI_MAX = 25.0f; // Y placeholder
  // NOTE: later you can add a timer: "must persist for N minutes after shutdown"

  if (ok(p.lowSidePressurePsi) && ok(p.highSidePressurePsi)) {
    const float dpsi = p.highSidePressurePsi - p.lowSidePressurePsi;
    if (!isnan(dpsi) && fabsf(dpsi) > STATIC_EQUALIZE_DELTA_PSI_MAX) {
      addFault(r, FaultCode::Fault_StaticNotEqualizing, DiagnosticState::WeakPerformance);
    }
  }

  return r;
}


  // If we’re missing data, we can still flag those, but avoid nonsense math checks.
  // (Don’t run performance checks unless inputs are valid.)
  const bool tempsOk = ok(t.supplyAirTempC) && ok(t.returnAirTempC) && ok(deltaTF);
  const bool pressuresOk = ok(p.lowSidePressurePsi) && ok(p.highSidePressurePsi);
  const bool satOk = ok(sat.lowSideSatTempF) && ok(sat.highSideSatTempF);
  const bool shscOk = ok(shsc.superheatF) && ok(shsc.subcoolingF);

  // ---------- DELTA-T ----------
  if (tempsOk) {
    if (fabsf(deltaTF) <= DELTAT_ZERO_F) addFault(r, FaultCode::Fault_DeltaTZero, DiagnosticState::WeakPerformance);
    if (deltaTF < DELTAT_LOW_F)          addFault(r, FaultCode::Fault_DeltaTTooLow, DiagnosticState::WeakPerformance);
    if (deltaTF > DELTAT_HIGH_F)         addFault(r, FaultCode::Fault_DeltaTTooHigh, DiagnosticState::WeakPerformance);
  }

  // ---------- PRESSURES ----------
  if (pressuresOk && mode == HvacMode::Cooling) {
    if (p.highSidePressurePsi > PSI_HIGH_MAX_COOL) addFault(r, FaultCode::Fault_HighPsiTooHigh, DiagnosticState::WeakPerformance);
    if (p.lowSidePressurePsi  < PSI_LOW_MIN_COOL)  addFault(r, FaultCode::Fault_LowPsiTooLow, DiagnosticState::WeakPerformance);
  }

  // ---------- LINE TEMPS ----------
  if (ok(suctionF) && suctionF <= SUCTION_FREEZE_F) addFault(r, FaultCode::Fault_SuctionLineFreezing, DiagnosticState::WeakPerformance);
  if (ok(liquidF)  && liquidF  >= LIQUID_TOO_HOT_F) addFault(r, FaultCode::Fault_LiquidLineTooHot, DiagnosticState::WeakPerformance);

  // ---------- SH / SC ----------
  if (shscOk) {
    if (shsc.subcoolingF < SUBCOOL_LOW_F)   addFault(r, FaultCode::Fault_SubcoolLow, DiagnosticState::WeakPerformance);
    if (shsc.subcoolingF > SUBCOOL_HIGH_F)  addFault(r, FaultCode::Fault_SubcoolHigh, DiagnosticState::WeakPerformance);
    if (shsc.superheatF  < SUPERHEAT_LOW_F) addFault(r, FaultCode::Fault_SuperheatLow, DiagnosticState::WeakPerformance);
    if (shsc.superheatF  > SUPERHEAT_HIGH_F)addFault(r, FaultCode::Fault_SuperheatHigh, DiagnosticState::WeakPerformance);
  }

  // ---------- COMBO PATTERNS ----------
  if (shscOk) {
    if (shsc.subcoolingF < SUBCOOL_LOW_F && shsc.superheatF > SUPERHEAT_HIGH_F) {
      addFault(r, FaultCode::Fault_LowChargePattern, DiagnosticState::WeakPerformance);
    }
    if (shsc.subcoolingF > SUBCOOL_HIGH_F && shsc.superheatF > SUPERHEAT_HIGH_F) {
      addFault(r, FaultCode::Fault_RestrictionPattern, DiagnosticState::WeakPerformance);
    }
  }

  return r;
}

static void printFaultReportLines(const FaultReport &r) {
  if (r.count == 0) {
    Serial.println("FAULTS: NONE");
    return;
  }
  Serial.printf("FAULTS (%u):\n", (unsigned)r.count);
  for (uint8_t i = 0; i < r.count; i++) {
    Serial.printf(" - %s\n", faultCodeToString(r.codes[i]));
  }
}

static FaultReport faultReport;
//----------------------------------------------------------------------//

static float computeDeltaPsi(const HvacPressures& p) {
  if (isnan(p.highSidePressurePsi) || isnan(p.lowSidePressurePsi)) return NAN;
  return p.highSidePressurePsi - p.lowSidePressurePsi;
}


// ===================== LOOP =====================
// Basically "int main()" for reading sensors, updating HVAC state, and printing.
// NOTE: These are thermocouple temperatures (TC). If a probe is disconnected,
// thermocoupleTempC will be NAN due to fault (OC/SCG/SCV).
void loop() {
  // Read all sensors (TC + CJ)
  Max31855Reading sensorHighPressure = readMax31855Filtered(MAX31855_CS_HIGH_PRESSURE);
  Max31855Reading sensorLowPressure  = readMax31855Filtered(MAX31855_CS_LOW_PRESSURE);
  Max31855Reading sensorSupplyAir    = readMax31855Filtered(MAX31855_CS_SUPPLY_AIR);
  Max31855Reading sensorReturnAir    = readMax31855Filtered(MAX31855_CS_RETURN_AIR);

  // Map thermocouple temps into HVAC temps
  /**hvacTemps.highPressureLineTempC = sensorHighPressure.thermocoupleTempC;
  hvacTemps.lowPressureLineTempC  = sensorLowPressure.thermocoupleTempC;
  hvacTemps.supplyAirTempC        = sensorSupplyAir.thermocoupleTempC;
  hvacTemps.returnAirTempC        = sensorReturnAir.thermocoupleTempC;**/
  
  hvacTemps.highPressureLineTempC = sensorHighPressure.thermocoupleTempC;
  hvacTemps.lowPressureLineTempC  = sensorLowPressure.thermocoupleTempC;

  // Supply Air (calibrated)
  if (!isnan(sensorSupplyAir.thermocoupleTempC)) {
    hvacTemps.supplyAirTempC =
        sensorSupplyAir.thermocoupleTempC + SUPPLY_TEMP_OFFSET_C;
  } else {
    hvacTemps.supplyAirTempC = NAN;
  }

  // Return Air (calibrated)
  if (!isnan(sensorReturnAir.thermocoupleTempC)) {
    hvacTemps.returnAirTempC =
        sensorReturnAir.thermocoupleTempC + RETURN_TEMP_OFFSET_C;
  } else {
    hvacTemps.returnAirTempC = NAN;
  }


  // Compute deltaTempC = return - supply
  if (!isnan(hvacTemps.returnAirTempC) && !isnan(hvacTemps.supplyAirTempC)) {
    hvacTemps.deltaTempC = hvacTemps.returnAirTempC - hvacTemps.supplyAirTempC;
  } else {
    hvacTemps.deltaTempC = NAN;
  }
  hvacTemps.updatedAtMs = millis();

  // Reset per loop (optional)
  hvacState.diagnostic = DiagnosticState::Unknown;

  // Pressures first (for deltaPsi running detect)
  updateHvacPressures();

  // Decide RUNNING/OFF from deltaT OR deltaPsi
  updateHvacRunStateFromDeltaTOrDeltaPsi(hvacTemps, hvacPressures, hvacState);

  // If running, decide Cooling/Heating from temps
  updateHvacModeFromTemperatures(hvacTemps, hvacState);

  updateHvacSaturationTempsFromPressures();
  updateHvacSuperheatSubcool();

  // Evaluate ALL faults
  faultReport = evaluateFaultsHeatPumpAll(
    hvacTemps, hvacPressures, hvacSatTemps, hvacShSc,
    hvacState.systemState, hvacState.mode
  );

  hvacState.diagnostic = faultReport.diag;

  // Extra ADC voltage prints (optional second read)
  float adcVoltageA0 = NAN;
  float adcVoltageA1 = NAN;

  bool a0Ok = readAds1115ChannelVoltageAtAdcPin(0, adcVoltageA0);
  bool a1Ok = readAds1115ChannelVoltageAtAdcPin(1, adcVoltageA1);

  float sensorVoltageA0 = convertAdcPinVoltageToSensorVoltage(adcVoltageA0);
  float sensorVoltageA1 = convertAdcPinVoltageToSensorVoltage(adcVoltageA1);

  // ---------- PRINTS ----------
  Serial.println("=== BEGIN MONITORING ===");
  Serial.println();
  Serial.printf("SYS=%s  MODE=%s  DIAG=%s\n",
    (hvacState.systemState == HvacSystemState::Running) ? "RUNNING" : "OFF",
    hvacModeToString(hvacState.mode),
    diagToString(hvacState.diagnostic)
  );
  Serial.println();

  Serial.println("---- MAX31855 Raw/Decoded ----");
  printMax31855Reading("HighPressure", sensorHighPressure);
  printMax31855Reading("LowPressure ", sensorLowPressure);
  printMax31855Reading("SupplyAir   ", sensorSupplyAir);
  printMax31855Reading("ReturnAir   ", sensorReturnAir);
  Serial.println();

  Serial.println("---- HVAC Temps (Thermocouple TC) ----");
  printTemperatureOrFaultF("High Pressure Line", hvacTemps.highPressureLineTempC);
  printTemperatureOrFaultF("Low Pressure Line ", hvacTemps.lowPressureLineTempC);
  printTemperatureOrFaultF("Supply Air        ", hvacTemps.supplyAirTempC);
  printTemperatureOrFaultF("Return Air        ", hvacTemps.returnAirTempC);

  if (!isnan(hvacTemps.deltaTempC)) {
    float deltaTempF = deltaCelsiusToDeltaFahrenheit(hvacTemps.deltaTempC);
    Serial.printf("DeltaT (Return-Supply): %.2f F\n", deltaTempF);
  } else {
    Serial.println("DeltaT (Return-Supply): FAULT");
  }

  Serial.println();

  Serial.println("---- ADS1115 (Voltage) ----");
  if (a0Ok) Serial.printf("A0 TDH33 (after divider): %.3f V | sensor: %.3f V\n", adcVoltageA0, sensorVoltageA0);
  else      Serial.println("A0 TDH33 read failed");

  if (a1Ok) Serial.printf("A1 TD1000 (after divider): %.3f V | sensor: %.3f V\n", adcVoltageA1, sensorVoltageA1);
  else      Serial.println("A1 TD1000 read failed");

  Serial.println();
  Serial.println("---- Pressures ----");
  printPressureOrFault("Low Side (A0 0-5V, 0-1000psi)", hvacPressures.lowSidePressurePsi);
  printPressureOrFault("High Side (A1 1-5V, 0-600psi)", hvacPressures.highSidePressurePsi);

  // Helpful: show deltaPsi (why RUNNING)
  float deltaPsi = computeDeltaPsi(hvacPressures);
  if (isnan(deltaPsi)) Serial.println("DeltaPsi (High-Low): FAULT");
  else                 Serial.printf("DeltaPsi (High-Low): %.1f PSI\n", deltaPsi);

  Serial.println();
  Serial.println("---- Saturation Temps (R-22) ----");
  printSatTempOrFault("Low Side Sat Temp",  hvacSatTemps.lowSideSatTempF);
  printSatTempOrFault("High Side Sat Temp", hvacSatTemps.highSideSatTempF);

  Serial.println();
  Serial.println("---- Superheat / Subcooling ----");
  printShScOrFault("Superheat (Suction - SatLow)", hvacShSc.superheatF);
  printShScOrFault("Subcool  (SatHigh - Liquid)", hvacShSc.subcoolingF);

  Serial.println();
  Serial.printf("MODE=%s  DIAG=%s\n", hvacModeToString(hvacState.mode), diagToString(hvacState.diagnostic));
  printFaultReportLines(faultReport);

  Serial.println();
  Serial.println("=== END MONITORING ===");

  delay(2000);
}



// github commit message example:
// git add .
// git commit -m ' xxx '
// git push origin main
