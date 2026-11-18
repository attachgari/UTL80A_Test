#include <Arduino.h>
#include <Wire.h>
#include <bluefruit.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "utl80a_calibration_lab.h"
#include "wrench_motion_filter.h"
#include "wrench_tcn.h"
#include "fz_calibration.h"

/*
  Project: UTL80A
  Version: v1.0.25
  MCU: Seeed XIAO nRF54L15
*/

#define PROJECT_NAME "UTL80A"
#define PROJECT_VERSION "v1.0.25"

enum class WrenchFilter : uint8_t { Tcn, TcnLinear, FzDirect, Impact, Mean, Kalman };

class SerialFrame : public Print {
 public:
  size_t write(uint8_t byte) override {
    if (length_ == sizeof(bytes_)) return 0;
    bytes_[length_++] = byte;
    return 1;
  }

  void send() { Serial.write(bytes_, length_); }

 private:
  uint8_t bytes_[256];
  size_t length_ = 0;
};

static_assert(UTL80A_OUTPUT_COUNT == 6, "This sketch requires the experimental six-row header.");
constexpr uint8_t TZ_AXIS = 5;
float latestWrench[6] = {};
bool wrenchAvailable = false;
uint32_t lastWrenchUs = 0;
uint32_t wrenchUpdates = 0;
uint16_t freshRawHz = 0, freshWrenchHz = 0;

#define TCA_ADDR 0x70
#define LDC_ADDR 0x2A
#define TMP117_ADDR 0x49
#define TMP275_ADDR 0x48
#define TMP117_TEMP_REG 0x00
#define TMP275_TEMP_REG 0x00
#define PIN_SD1 PIN_D0
#define PIN_SD2 PIN_D1

#define REG_DATA0   0x00
#define REG_DATA1   0x02
#define REG_RCOUNT0 0x08
#define REG_SETTLE0 0x10
#define REG_CLOCK0  0x14
#define REG_STATUS  0x18
#define REG_ERROR   0x19
#define REG_CONFIG  0x1A
#define REG_MUX     0x1B
#define REG_DRIVE0  0x1E

#define CONFIG_SLEEP 0x3601
#define CONFIG_RUN   0x1601
#define MUX_TWO_CHANNEL 0x820D   
#define CLOCK_DIV 0x1002         
#define SETTLE_COUNT 0x000A
#define DRIVE_CURRENT 0x9000

const double EXTERNAL_CLOCK_HZ = 40000000.0;
const double NOMINAL_CAP_PF = 1000.0;
const uint32_t PLOT_INTERVAL_US = 10000;  // Target 100 Hz.
const uint32_t TARE_INTERVAL_US = 20000;  // Keep each 50-frame tare window about one second.
const uint16_t RCOUNT_200 = 0x0C00; 
const uint16_t BUFFER_FRAMES = 256;
const double RAW_SCALE = 268435456.0; 

// ==================== 全域變數宣告 ====================
uint32_t i2cClock = 400000;
uint8_t targetBits = 15;
uint16_t rcount = RCOUNT_200;
bool mode200 = true;
uint8_t plotMode = 0; // 0=paused, 1=calibrated wrench, 2=legacy coil diagnostic, 3=RAW, 4=Tz only
uint8_t lastActivePlotMode = 3; 
bool ready[4] = {false, false, false, false};
bool valid[8] = {false};
bool pending[8] = {false};
uint32_t raw28[8] = {0};
uint32_t freshCount[8] = {0};
uint32_t rawUpdateUs[8] = {0};
uint32_t wrenchFrame[8] = {0};
uint32_t wrenchFrameUs = 0, wrenchFrameId = 0;
bool wrenchFrameReady = false;
uint16_t offsetReg[8] = {0};
uint16_t clockReg[8] = {0};
uint32_t frames = 0, plotFrames = 0;
uint32_t i2cErrors = 0, sensorErrors = 0;
uint64_t totalReadUs = 0;
uint32_t readCycles = 0;
uint32_t statsStartMs = 0, lastPlotUs = 0;

char commandBuffer[64];
uint8_t commandIndex = 0;
bool commandOverflow = false;

const uint32_t TEMP_UPDATE_MS = 1000;
float temp117C[4] = {0,0,0,0};
float temp275C = 0;
bool temp117Valid[4] = {false,false,false,false};
bool temp275Valid = false;
uint32_t tempLastUpdateMs = 0;
uint32_t tempI2CErrors = 0;

// ==================== BLE telemetry ====================
BLEService rawService(BLEUuid("9e238080-4e25-4a8a-b0b9-845e2841a6f0"));
BLECharacteristic rawNotify(BLEUuid("9e238081-4e25-4a8a-b0b9-845e2841a6f0"));
BLECharacteristic bleControl(BLEUuid("9e238082-4e25-4a8a-b0b9-845e2841a6f0"));
const uint32_t BLE_SEQ_MASK = 0x3FFFFFFFUL;
const uint32_t BLE_SECOND_HALF = 0x40000000UL;
const uint32_t BLE_DIAGNOSTICS = 0x80000000UL;
const uint32_t BLE_WRENCH = 0xC0000000UL;
const uint32_t BLE_INVALID_RAW = 0xFFFFFFFFUL;
const uint8_t BLE_MAX_BYTES = 20;
const uint32_t BLE_RETRY_US = 2000;
bool bleReady = false;
bool bleStreaming = false;
bool bleRawMode = false;
volatile uint8_t bleRequestedMode = 0;
volatile bool bleTareRequested = false;
volatile uint8_t bleFilterRequest = 0;
uint8_t bleTareResult = 0;
uint32_t bleLastSlotUs = 0, bleLastStatusMs = 0;
uint32_t bleSequence = 0, bleSkippedSlots = 0, bleRejectedFrames = 0;
uint8_t bleFirst[BLE_MAX_BYTES], bleSecond[BLE_MAX_BYTES];
uint8_t bleFirstLength = 0, blePhase = 0, bleStatusPhase = 0;
uint32_t bleLastAttemptUs = 0;

bool wrenchFrameIsFresh();
void executeTare();

void putLE(uint8_t *out, uint32_t value, uint8_t bytes) {
  for (uint8_t i = 0; i < bytes; ++i) {
    out[i] = (uint8_t)value;
    value >>= 8;
  }
}

void onBleControl(uint16_t, BLECharacteristic *, uint8_t *data, uint16_t length) {
  if (length != 1) return;
  if (data[0] == 0 || data[0] == 1) bleRequestedMode = data[0] + 1;
  else if (data[0] == 2) bleTareRequested = true;
  else if (data[0] >= 3 && data[0] <= 9) bleFilterRequest = data[0];
}

bool setupRawBle() {
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  if (!Bluefruit.begin()) return false;
  Bluefruit.setName("UTL80A");
  Bluefruit.Periph.setConnInterval(6, 8); // 7.5–10 ms preference, central negotiates
  if (rawService.begin() != ERROR_NONE) return false;
  rawNotify.setProperties(CHR_PROPS_NOTIFY);
  rawNotify.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  rawNotify.setMaxLen(BLE_MAX_BYTES);
  if (rawNotify.begin() != ERROR_NONE) return false;
  bleControl.setProperties(CHR_PROPS_WRITE);
  bleControl.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  bleControl.setFixedLen(1);
  bleControl.setWriteCallback(onBleControl);
  if (bleControl.begin() != ERROR_NONE) return false;

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(rawService);
  Bluefruit.Advertising.addName();
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  return Bluefruit.Advertising.start(0);
}

bool sendBleTemperature() {
  uint8_t temperature[14] = {};
  uint32_t temperatureMask = 0;
  for (uint8_t i = 0; i < 5; ++i) {
    const bool ok = i < 4 ? temp117Valid[i] : temp275Valid;
    const float value = i < 4 ? temp117C[i] : temp275C;
    // Both temperature sensors are exactly representable in 1/256 °C units.
    const int16_t counts = ok ? (int16_t)(value * 256.0f) : 0;
    if (ok) temperatureMask |= 1UL << i;
    putLE(temperature + 4 + i * 2, (uint16_t)counts, 2);
  }
  putLE(temperature, temperatureMask, 4);
  return rawNotify.notify(temperature, sizeof(temperature));
}

bool sendBleDiagnostics(uint16_t mtu, uint16_t interval) {
  uint8_t diagnostics[20];
  putLE(diagnostics, BLE_DIAGNOSTICS | bleSequence, 4);
  putLE(diagnostics + 4, bleSkippedSlots, 4);
  putLE(diagnostics + 8, bleRejectedFrames, 4);
  putLE(diagnostics + 12, mtu, 2);
  putLE(diagnostics + 14, interval, 2);
  putLE(diagnostics + 16, freshRawHz, 2);
  putLE(diagnostics + 18, freshWrenchHz, 2);
  return rawNotify.notify(diagnostics, sizeof(diagnostics));
}

void prepareBleFrame() {
  if (bleRawMode) {
    bleFirstLength = BLE_MAX_BYTES;
    putLE(bleFirst, bleSequence, 4);
    putLE(bleSecond, bleSequence | BLE_SECOND_HALF, 4);
    for (uint8_t i = 0; i < 8; ++i) {
      uint8_t *packet = i < 4 ? bleFirst : bleSecond;
      putLE(packet + 4 + (i % 4) * 4, valid[i] ? raw28[i] : BLE_INVALID_RAW, 4);
    }
  } else {
    bleFirstLength = 16;
    putLE(bleFirst, bleSequence | BLE_WRENCH, 4);
    const bool fresh = wrenchAvailable && wrenchFrameIsFresh();
    for (uint8_t i = 0; i < 6; ++i) {
      const float counts = latestWrench[i] * (i < 3 ? 20.0f : 1000.0f);
      const int16_t encoded = fresh && isfinite(counts) && fabsf(counts) <= 32767.0f
                              ? (int16_t)lroundf(counts) : INT16_MIN;
      putLE(bleFirst + 4 + 2 * i, (uint16_t)encoded, 2);
    }
  }
  blePhase = 1;
}

void serviceRawBle() {
  if (!bleReady) return;
  if (!Bluefruit.connected() || !rawNotify.notifyEnabled()) {
    bleStreaming = false;
    bleRequestedMode = 0;
    bleTareRequested = false;
    bleFilterRequest = 0;
    bleTareResult = 0;
    return;
  }
  const uint32_t nowUs = micros();
  if (!bleStreaming) {
    bleStreaming = true;
    bleLastSlotUs = nowUs - PLOT_INTERVAL_US;
    bleLastStatusMs = millis();
    bleSequence = bleSkippedSlots = bleRejectedFrames = 0;
    blePhase = bleStatusPhase = 0;
    bleRawMode = false;
    bleLastAttemptUs = nowUs - BLE_RETRY_US;
  }
  const uint8_t mode = bleRequestedMode;
  if (mode != 0) {
    bleRequestedMode = 0;
    const bool raw = mode == 1;
    if (raw != bleRawMode) {
      bleRawMode = raw;
      blePhase = 0;
    }
  }
  const uint32_t periods = (uint32_t)(nowUs - bleLastSlotUs) / PLOT_INTERVAL_US;
  if (periods != 0) {
    if (blePhase != 0) ++bleRejectedFrames;
    bleLastSlotUs += periods * PLOT_INTERVAL_US;
    bleSkippedSlots += periods - 1;
    bleSequence = (bleSequence + periods) & BLE_SEQ_MASK;
    prepareBleFrame();
  }
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - bleLastStatusMs) >= TEMP_UPDATE_MS) {
    bleLastStatusMs = nowMs;
    if (bleStatusPhase == 0) bleStatusPhase = 1;
  }
  if ((uint32_t)(nowUs - bleLastAttemptUs) < BLE_RETRY_US) return;
  if (blePhase == 1) {
    bleLastAttemptUs = nowUs;
    if (rawNotify.notify(bleFirst, bleFirstLength)) blePhase = bleRawMode ? 2 : 0;
  } else if (blePhase == 2) {
    bleLastAttemptUs = nowUs;
    if (rawNotify.notify(bleSecond, sizeof(bleSecond))) blePhase = 0;
  } else if (bleTareResult != 0) {
    bleLastAttemptUs = nowUs;
    uint8_t acknowledgement[5];
    putLE(acknowledgement, BLE_DIAGNOSTICS | bleSequence, 4);
    acknowledgement[4] = bleTareResult;
    if (rawNotify.notify(acknowledgement, sizeof(acknowledgement))) bleTareResult = 0;
  } else if (bleStatusPhase != 0) {
    bleLastAttemptUs = nowUs;
    BLEConnection *link = Bluefruit.Connection(Bluefruit.connHandle());
    if (!link) return;
    if (bleStatusPhase == 1) {
      if (sendBleTemperature()) bleStatusPhase = 2;
    } else if (sendBleDiagnostics(link->getMtu(), link->getConnectionInterval())) {
      bleStatusPhase = 0;
    }
  }
}

// ==================== 雙重濾波器 ====================
#define FILTER_WINDOW_SIZE 32
#define TRIM_COUNT 4  
#define EMA_ALPHA 0.05 

typedef struct {
  uint32_t buffer[FILTER_WINDOW_SIZE];
  uint8_t index;
  uint16_t count;
  float ema_value; 
} AdvancedFilter;

AdvancedFilter coilFilters[8];
float baselineRaw[8] = {0.0}; 

const float SENSITIVITY[8] = {
  3533.92, 3503.35, 2525.66, 4635.25, 
  3412.28, 4796.16, 4462.24, 3317.82
};

void resetFilters() {
  for (uint8_t i = 0; i < 8; ++i) {
    coilFilters[i].index = 0;
    coilFilters[i].count = 0;
    coilFilters[i].ema_value = 0.0;
    for(int j=0; j<FILTER_WINDOW_SIZE; j++) coilFilters[i].buffer[j] = 0;
  }
}

void updateFilter(uint8_t ch, uint32_t rawValue) {
  AdvancedFilter &f = coilFilters[ch];
  f.buffer[f.index] = rawValue;
  f.index = (f.index + 1) % FILTER_WINDOW_SIZE;
  if (f.count < FILTER_WINDOW_SIZE) f.count++;
  
  if (f.count > 0) {
    uint32_t sorted[FILTER_WINDOW_SIZE];
    uint16_t n = f.count;
    for(uint16_t i=0; i<n; i++) sorted[i] = f.buffer[i];
    
    for (uint16_t i = 1; i < n; i++) {
      uint32_t key = sorted[i];
      int16_t j = i - 1;
      while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j = j - 1; }
      sorted[j + 1] = key;
    }
    
    uint16_t trim = (n >= (TRIM_COUNT * 2 + 4)) ? TRIM_COUNT : 0; 
    uint64_t sum = 0;
    uint16_t validCount = 0;
    for(uint16_t i = trim; i < n - trim; i++) {
      sum += sorted[i];
      validCount++;
    }
    float current_trimmed = (validCount == 0) ? 0 : ((float)sum / validCount);
    
    if (f.ema_value == 0.0) {
      f.ema_value = current_trimmed;
    } else {
      f.ema_value = (EMA_ALPHA * current_trimmed) + ((1.0 - EMA_ALPHA) * f.ema_value);
    }
  }
}

float getFilteredValue(uint8_t ch) {
  return coilFilters[ch].ema_value;
}

// The calibration was fitted on means of eight-channel RAW28 readings.
constexpr uint8_t WRENCH_MAX_WINDOW = 100;
uint8_t wrenchWindowSize = 24;
constexpr uint8_t TARE_FRAME_COUNT = 50;
constexpr uint32_t WRENCH_MAX_AGE_US = 3 * PLOT_INTERVAL_US;
float wrenchWindow[WRENCH_MAX_WINDOW][TZ_AXIS] = {};
double wrenchWindowSum[TZ_AXIS] = {};
uint64_t wrenchTareSum[8] = {};
uint8_t wrenchWindowIndex = 0, wrenchWindowCount = 0;
uint32_t wrenchLastUsedFrame = 0;
bool wrenchTared = false;
bool tareActive = false;
bool tareHasReference = false, tareSawMovement = false;
uint8_t tareCount = 0;
uint32_t tareStartUs = 0, tareSampleUs = 0, tareLastFrame = 0;
uint64_t tareCandidate[8] = {}, tareReference[8] = {};
double tareLegacy[8] = {};
WrenchFilter wrenchFilter = WrenchFilter::Tcn;
bool kalmanReady = false;
float kalmanState[5] = {}, kalmanVariance = 1.0f;
WrenchMotionFilter motionFilter;
WrenchTcn wrenchTcn;

void acquire();

void clearWrenchWindow() {
  memset(wrenchWindowSum, 0, sizeof(wrenchWindowSum));
  wrenchWindowIndex = wrenchWindowCount = 0;
  wrenchLastUsedFrame = wrenchFrameId;
  wrenchAvailable = false;
  kalmanReady = false;
  kalmanVariance = 1.0f;
  motionFilter.reset();
  wrenchTcn.reset();
}

bool wrenchFrameIsFresh() {
  const uint32_t now = micros();
  if (!wrenchFrameReady || (uint32_t)(now - wrenchFrameUs) > WRENCH_MAX_AGE_US) return false;
  for (uint8_t i = 0; i < 8; ++i) {
    if (!valid[i] || (uint32_t)(now - rawUpdateUs[i]) > WRENCH_MAX_AGE_US) return false;
  }
  return true;
}

void addWrenchFrame(const float clean[6]) {
  const bool full = wrenchWindowCount == wrenchWindowSize;
  for (uint8_t i = 0; i < TZ_AXIS; ++i) {
    if (full) wrenchWindowSum[i] -= wrenchWindow[wrenchWindowIndex][i];
    wrenchWindow[wrenchWindowIndex][i] = clean[i];
    wrenchWindowSum[i] += clean[i];
  }
  if (!full) ++wrenchWindowCount;
  wrenchWindowIndex = (wrenchWindowIndex + 1) % wrenchWindowSize;
}

void executeTare() {
  if (tareActive) return;
  Serial.println("\n[TARE] Keep the mounted sensor unloaded and still.");
  tareActive = true;
  tareHasReference = tareSawMovement = false;
  bleTareResult = 0;
  tareCount = 0;
  tareStartUs = tareSampleUs = micros();
  tareLastFrame = wrenchFrameId;
  memset(tareCandidate, 0, sizeof(tareCandidate));
  memset(tareLegacy, 0, sizeof(tareLegacy));
  clearWrenchWindow();
}

bool tareWindowsAgree() {
  float delta[8], wrench[6];
  for (uint8_t i = 0; i < 8; ++i) {
    delta[i] = (float)((int64_t)tareCandidate[i] - (int64_t)tareReference[i]) / TARE_FRAME_COUNT;
  }
  utl80a_predict_delta(delta, wrench);
  static const float tolerance[6] = {1.1f, 1.1f, .50f, .06f, .06f, .08f};
  for (uint8_t i = 0; i < 6; ++i) {
    if (fabsf(wrench[i]) > tolerance[i]) return false;
  }
  return true;
}

void serviceTare(uint32_t now) {
  if (!tareActive) return;
  const uint32_t elapsed = now - tareStartUs;
  if (elapsed >= 8500000UL) {
    tareActive = false;
    bleTareResult = tareSawMovement ? 3 : 2;
    if (tareSawMovement) Serial.println("[TARE] Failed: unstable readings. Previous zero retained.");
    else Serial.println("[TARE] Failed: not enough fresh eight-channel frames. Previous zero retained.");
    return;
  }
  if (elapsed < 500000UL || (uint32_t)(now - tareSampleUs) < TARE_INTERVAL_US
      || wrenchFrameId == tareLastFrame || !wrenchFrameIsFresh()) return;
  tareSampleUs = now;
  tareLastFrame = wrenchFrameId;
  for (uint8_t i = 0; i < 8; ++i) {
    tareCandidate[i] += wrenchFrame[i];
    tareLegacy[i] += getFilteredValue(i);
  }
  if (++tareCount < TARE_FRAME_COUNT) return;
  if (!tareHasReference || !tareWindowsAgree()) {
    tareSawMovement = tareHasReference;
    tareHasReference = true;
    memcpy(tareReference, tareCandidate, sizeof(tareReference));
    memset(tareCandidate, 0, sizeof(tareCandidate));
    memset(tareLegacy, 0, sizeof(tareLegacy));
    tareCount = 0;
    return;
  }
  for (uint8_t i = 0; i < 8; ++i) {
    wrenchTareSum[i] = tareCandidate[i];
    baselineRaw[i] = (float)(tareLegacy[i] / TARE_FRAME_COUNT);
  }
  clearWrenchWindow();
  wrenchTared = true;
  tareActive = false;
  bleTareResult = 1;
  Serial.println("[TARE] OK");
}

void printFilter() {
  if (wrenchFilter == WrenchFilter::Tcn) Serial.println("[FILTER] Residual TCN (all axes)");
  else if (wrenchFilter == WrenchFilter::TcnLinear) Serial.println("[FILTER] TCN linear baseline (all axes)");
  else if (wrenchFilter == WrenchFilter::FzDirect) Serial.println("[FILTER] Fz deadweight fit (Fz tests)");
  else if (wrenchFilter == WrenchFilter::Impact) Serial.println("[FILTER] Legacy linear (calibrated snapshots)");
  else if (wrenchFilter == WrenchFilter::Kalman) Serial.println("[FILTER] Kalman (first five axes)");
  else {
    Serial.print("[FILTER] Window: ");
    Serial.println(wrenchWindowSize);
  }
}

void setWrenchFilter(WrenchFilter mode) {
  wrenchFilter = mode;
  clearWrenchWindow();
  printFilter();
}

void updateRates() {
  static uint32_t previousMs = 0, previousRaw = 0, previousWrench = 0;
  const uint32_t now = millis();
  const uint32_t duration = now - previousMs;
  if (duration < 1000) return;
  freshRawHz = (uint16_t)(((wrenchFrameId - previousRaw) * 1000UL + duration / 2) / duration);
  freshWrenchHz = (uint16_t)(((wrenchUpdates - previousWrench) * 1000UL + duration / 2) / duration);
  previousMs = now;
  previousRaw = wrenchFrameId;
  previousWrench = wrenchUpdates;
}

// ==================== I2C ====================
uint16_t calculateRCOUNT(uint8_t bits) {
  if (bits < 12) bits = 12;
  if (bits > 20) bits = 20;
  uint32_t v = 1UL << (bits - 4);
  return (uint16_t)((v > 65535UL) ? 65535UL : v);
}

bool ping(uint8_t addr) { Wire.beginTransmission(addr); return Wire.endTransmission() == 0; }
bool selectTCA(uint8_t ch) { if (ch >= 8) return false; Wire.beginTransmission(TCA_ADDR); Wire.write((uint8_t)(1U << ch)); return Wire.endTransmission() == 0; }
bool disableTCA() { Wire.beginTransmission(TCA_ADDR); Wire.write((uint8_t)0); return Wire.endTransmission() == 0; }

bool write16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(LDC_ADDR);
  Wire.write(reg); Wire.write((uint8_t)(value >> 8)); Wire.write((uint8_t)(value & 0xFF));
  return Wire.endTransmission() == 0;
}

bool read16(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(LDC_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t n = Wire.requestFrom((uint8_t)LDC_ADDR, (uint8_t)2);
  if (n != 2 || Wire.available() < 2) { while (Wire.available()) Wire.read(); return false; }
  value = ((uint8_t)Wire.read() << 8) | (uint8_t)Wire.read();
  return true;
}

bool readTempRegister(uint8_t addr, uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t n = Wire.requestFrom(addr, (uint8_t)2);
  if (n != 2 || Wire.available() < 2) { while (Wire.available()) Wire.read(); return false; }
  value = ((uint8_t)Wire.read() << 8) | (uint8_t)Wire.read();
  return true;
}
bool readTMP117(uint8_t ch, float &value) {
  if (!selectTCA(ch)) return false;
  uint16_t raw = 0;
  bool ok = readTempRegister(TMP117_ADDR, TMP117_TEMP_REG, raw);
  disableTCA();
  if (ok) value = (float)((int16_t)raw * 0.0078125);
  return ok;
}
bool readTMP275(float &value) {
  if (!disableTCA()) return false;
  uint16_t raw = 0;
  if (!readTempRegister(TMP275_ADDR, TMP275_TEMP_REG, raw)) return false;
  value = (float)((int16_t)raw / 256.0);
  return true;
}
void refreshTemperatures(bool force = false) {
  uint32_t now = millis();
  if (!force && (uint32_t)(now - tempLastUpdateMs) < TEMP_UPDATE_MS) return;
  tempLastUpdateMs = now;
  for (uint8_t ch = 0; ch < 4; ++ch) {
    float v = 0; bool ok = readTMP117(ch, v);
    temp117Valid[ch] = ok; if (ok) temp117C[ch] = v;
  }
  float v = 0; temp275Valid = readTMP275(v); if (temp275Valid) temp275C = v;
}

bool configureLDC(uint8_t dev) {
  ready[dev] = false;
  if (!selectTCA(dev) || !ping(LDC_ADDR)) return false;
  uint16_t id = 0, mfr = 0;
  if (!read16(0x7F, id) || !read16(0x7E, mfr)) return false;
  if (!write16(REG_CONFIG, CONFIG_SLEEP)) return false;
  delay(2);
  const struct {uint8_t reg; uint16_t value;} settings[] = {
    {0x08, rcount}, {0x09, rcount}, {0x0C, 0x0000}, {0x0D, 0x0000},
    {0x10, SETTLE_COUNT}, {0x11, SETTLE_COUNT}, {0x14, CLOCK_DIV}, {0x15, CLOCK_DIV},
    {REG_ERROR, 0x0000}, {REG_MUX, MUX_TWO_CHANNEL}, {0x1E, DRIVE_CURRENT}, {0x1F, DRIVE_CURRENT}
  };
  for (unsigned i = 0; i < sizeof(settings)/sizeof(settings[0]); ++i) {
    if (!write16(settings[i].reg, settings[i].value)) return false;
  }
  if (!write16(REG_CONFIG, CONFIG_RUN)) return false;
  delay(15);
  ready[dev] = true; return true;
}

void resetStats() {
  frames = plotFrames = i2cErrors = sensorErrors = 0;
  totalReadUs = 0; readCycles = 0; resetFilters();
  for (uint8_t i = 0; i < 8; ++i) { freshCount[i] = 0; pending[i] = false; }
  statsStartMs = millis(); lastPlotUs = micros();
}

void initializeAll() {
  plotMode = 0; 
  rcount = mode200 ? RCOUNT_200 : calculateRCOUNT(targetBits);
  digitalWrite(PIN_SD1, LOW); digitalWrite(PIN_SD2, LOW);
  delay(30);
  Serial.println();
  Serial.print(PROJECT_NAME); Serial.print(" "); Serial.println(PROJECT_VERSION);
  for (uint8_t i = 0; i < 4; ++i) ready[i] = false;
  if (!ping(TCA_ADDR)) return;
  disableTCA();
  for (uint8_t dev = 0; dev < 4; ++dev) { configureLDC(dev); }
  disableTCA();
  for (uint8_t i = 0; i < 8; ++i) { valid[i] = pending[i] = false; raw28[i] = 0; }
  resetStats();
}

bool readRaw28(uint8_t ch, uint32_t &raw, uint8_t &flags) {
  uint8_t reg = ch ? REG_DATA1 : REG_DATA0;
  uint16_t msb = 0, lsb = 0;
  if (!read16(reg, msb) || !read16((uint8_t)(reg+1), lsb)) return false;
  flags = (uint8_t)((msb >> 12) & 0x0F);
  raw = ((uint32_t)(msb & 0x0FFF) << 16) | (uint32_t)lsb;
  return true;
}

void acquire() {
  uint32_t startUs = micros();
  for (uint8_t dev = 0; dev < 4; ++dev) {
    if (!ready[dev]) continue;
    if (!selectTCA(dev)) { ++i2cErrors; continue; }
    uint16_t status = 0;
    if (!read16(REG_STATUS, status)) { ++i2cErrors; continue; }
    for (uint8_t ch = 0; ch < 2; ++ch) {
      uint8_t i = dev*2 + ch;
      uint16_t mask = ch ? 0x0004 : 0x0008;
      if (!(status & mask)) continue;
      uint32_t raw = 0; uint8_t flags = 0;
      if (!readRaw28(ch, raw, flags)) { ++i2cErrors; continue; }
      if (flags != 0 || raw == 0 || raw == 0x0FFFFFFFUL) {
        ++sensorErrors; valid[i] = false; pending[i] = false; wrenchFrameReady = false; continue;
      }
      raw28[i] = raw; valid[i] = pending[i] = true; ++freshCount[i];
      rawUpdateUs[i] = micros();
      updateFilter(i, raw); 
    }
  }
  disableTCA();
  totalReadUs += (uint32_t)(micros()-startUs);
  ++readCycles;
  bool complete = true;
  for (uint8_t i = 0; i < 8; ++i) { if (!pending[i]) { complete = false; break; } }
  if (complete) {
    ++frames;
    const uint32_t now = micros();
    bool fresh = true;
    for (uint8_t i = 0; i < 8; ++i) {
      if ((uint32_t)(now - rawUpdateUs[i]) > WRENCH_MAX_AGE_US) fresh = false;
      pending[i] = false;
    }
    wrenchFrameReady = fresh;
    if (fresh) {
      memcpy(wrenchFrame, raw28, sizeof(wrenchFrame));
      wrenchFrameUs = now;
      ++wrenchFrameId;
    }
  }
}

// -------------------- 繪圖函數群 --------------------

void updateWrench() {
  const bool fresh = wrenchTared && !tareActive && wrenchFrameIsFresh();
  if (!fresh) {
    clearWrenchWindow();
    return;
  }
  if (wrenchFrameId == wrenchLastUsedFrame) return;
  wrenchLastUsedFrame = wrenchFrameId;
  float delta[8], instantaneous[6], clean[6];
  for (uint8_t i = 0; i < 8; ++i) {
    const int64_t numerator = (int64_t)wrenchFrame[i] * TARE_FRAME_COUNT - (int64_t)wrenchTareSum[i];
    delta[i] = (float)numerator / TARE_FRAME_COUNT;
  }
  if (wrenchFilter == WrenchFilter::Tcn || wrenchFilter == WrenchFilter::TcnLinear ||
      wrenchFilter == WrenchFilter::FzDirect) {
    if (wrenchFilter == WrenchFilter::TcnLinear) wrenchTcn.linear(delta, latestWrench);
    else wrenchTcn.update(wrenchFrameUs, delta, latestWrench);
    if (wrenchFilter == WrenchFilter::FzDirect) {
      float fz = 0;
      for (uint8_t i = 0; i < 8; ++i) fz += FZ_DIRECT_COEFFICIENTS[i] * delta[i];
      latestWrench[2] = fz;
    }
    wrenchAvailable = true;
    ++wrenchUpdates;
    return;
  }
  // A shared signed slope per axis about tare maps opposite Fz or Tz deltas to opposite values.
  utl80a_predict_delta(delta, instantaneous);
  const float tx = instantaneous[3];
  instantaneous[3] = instantaneous[4];
  instantaneous[4] = tx;
  if (wrenchFilter == WrenchFilter::Impact) {
    memcpy(latestWrench, instantaneous, sizeof(latestWrench));
    wrenchAvailable = true;
    ++wrenchUpdates;
    return;
  }
  if (!motionFilter.update(instantaneous, clean)) return;
  if (wrenchFilter == WrenchFilter::Kalman) {
    // Scalar random-walk filter. R=1 fixes the scale; Q/R=0.01 sets response.
    const float priorVariance = kalmanVariance + 0.01f;
    const float gain = priorVariance / (priorVariance + 1.0f);
    for (uint8_t a = 0; a < TZ_AXIS; ++a) {
      if (!kalmanReady) kalmanState[a] = clean[a];
      else kalmanState[a] += gain * (clean[a] - kalmanState[a]);
      latestWrench[a] = kalmanState[a];
    }
    kalmanVariance = (1.0f - gain) * priorVariance;
    kalmanReady = true;
  } else {
    addWrenchFrame(clean);
    for (uint8_t a = 0; a < TZ_AXIS; ++a) {
      latestWrench[a] = (float)(wrenchWindowSum[a] / wrenchWindowCount);
    }
  }
  latestWrench[TZ_AXIS] = clean[TZ_AXIS];
  wrenchAvailable = true;
  ++wrenchUpdates;
}

void printWrench(bool tzOnly) {
  const bool available = wrenchAvailable && wrenchFrameIsFresh();
  SerialFrame line;
  if (tzOnly) {
    line.print("Tz:");
    if (available) line.println(latestWrench[TZ_AXIS], 4);
    else line.println("nan");
    line.send();
    ++plotFrames;
    return;
  }
  static const char *const labels[] = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
  for (uint8_t i = 0; i < 6; ++i) {
    if (i != 0) line.print('\t');
    line.print(labels[i]); line.print(':');
    if (available) line.print(latestWrench[i], 4);
    else line.print("nan");
  }
  line.println();
  line.send();
  ++plotFrames;
}

void printWeightPlot() {
  SerialFrame line;
  for (uint8_t i = 0; i < 8; ++i) {
    line.print("W"); line.print(i + 1); line.print(":");
    float current_raw = getFilteredValue(i);
    if (tareActive || !wrenchTared || !valid[i] || current_raw == 0) line.print("nan");
    else {
      float delta_counts = current_raw - baselineRaw[i];
      line.print(delta_counts / SENSITIVITY[i], 4);
    }
    line.print('\t');
  }
  line.println();
  line.send();
  ++plotFrames;
}

void printPlot() {
  SerialFrame line;
  for (uint8_t i = 0; i < 8; ++i) {
    line.print("L"); line.print(i + 1); line.print(":");
    line.print(valid[i] ? raw28[i] : -1); line.print('\t');
  }
  for (uint8_t ch = 0; ch < 4; ++ch) {
    line.print("TMP117_"); line.print(ch + 1); line.print(":");
    if (temp117Valid[ch]) line.print(temp117C[ch], 3);
    else line.print("nan");
    line.print('\t');
  }
  line.print("TMP275:");
  if (temp275Valid) line.print(temp275C, 3);
  else line.print("nan");
  line.println();
  line.send();
  ++plotFrames;
}

void printHelp() {
  Serial.println();
  Serial.println("========== " PROJECT_NAME " " PROJECT_VERSION " COMMANDS ==========");
  Serial.println("tare    : Zero at the current unloaded reference (also bias or bias on)");
  Serial.println("w / pw  : Fx/Fy/Fz in N, Tx/Ty in N*m, experimental Tz in N*m");
  Serial.println("tz      : Plot only the signed Tz estimate");
  Serial.println("filter  : Show mode; filter fz, filter tcn, filter tcn-linear, filter impact, filter kalman, or filter 1-100.");
  Serial.println("rate    : Target telemetry rate and actual fresh RAW/wrench rates.");
  Serial.println("p       : RAW28 and temperature");
  Serial.println("pw8     : Legacy per-coil diagnostic");
  Serial.println("s       : Pause stream");
  Serial.println("c       : Continue stream");
  Serial.println("=============================================");
}

void processCommand(const char *input) {
  char command[64];
  strncpy(command, input, sizeof(command)-1);
  command[sizeof(command)-1] = '\0';
  char *p = command;
  
  // 跳過空白字元
  while (*p == ' ' || *p == '\t') ++p;
  
  size_t len = strlen(p);
  while (len && (p[len-1]==' ' || p[len-1]=='\t' ||
                 p[len-1]=='\r' || p[len-1]=='\n')) p[--len]='\0';
  for (size_t i=0; p[i]; ++i) if (p[i]>='A' && p[i]<='Z') p[i] = (char)(p[i]-'A'+'a');
  if (!*p) return;
  
  if (!strcmp(p, "help")) printHelp();
  else if (!strcmp(p, "bias") || !strcmp(p, "bias on") || !strcmp(p, "tare")) executeTare();
  else if (!strcmp(p, "filter fz")) setWrenchFilter(WrenchFilter::FzDirect);
  else if (!strcmp(p, "filter tcn")) setWrenchFilter(WrenchFilter::Tcn);
  else if (!strcmp(p, "filter tcn-linear")) setWrenchFilter(WrenchFilter::TcnLinear);
  else if (!strcmp(p, "filter impact")) setWrenchFilter(WrenchFilter::Impact);
  else if (!strcmp(p, "filter kalman")) setWrenchFilter(WrenchFilter::Kalman);
  else if (!strcmp(p, "rate")) {
    Serial.print("[RATE] target:100 raw:"); Serial.print(freshRawHz);
    Serial.print(" wrench:"); Serial.println(freshWrenchHz);
  }
  else if (!strcmp(p, "filter") || !strncmp(p, "filter ", 7)) {
    if (p[6] != '\0') {
      char *end;
      const long window = strtol(p + 7, &end, 10);
      if (*end || window < 1 || window > WRENCH_MAX_WINDOW) {
        Serial.println("[FILTER] Use an integer from 1 to 100.");
        return;
      }
      wrenchWindowSize = (uint8_t)window;
      setWrenchFilter(WrenchFilter::Mean);
    } else {
      printFilter();
    }
  }
  else if (!strcmp(p, "s")) { 
    if (plotMode != 0) lastActivePlotMode = plotMode; 
    plotMode = 0; 
    Serial.println("Stream Paused."); 
  }
  else if (!strcmp(p, "c")) {
    if (plotMode == 0) plotMode = lastActivePlotMode; 
    if (plotMode == 1 || plotMode == 4) clearWrenchWindow();
    lastPlotUs = micros();
  }
  // 進階指令
  else if (!strcmp(p, "pw") || !strcmp(p, "w")) {
    clearWrenchWindow();
    plotMode = 1; lastActivePlotMode = 1; lastPlotUs = micros();
  }
  else if (!strcmp(p, "tz")) {
    clearWrenchWindow();
    plotMode = 4; lastActivePlotMode = 4; lastPlotUs = micros();
  }
  else if (!strcmp(p, "pw8")) { plotMode = 2; lastActivePlotMode = 2; lastPlotUs = micros(); } 
  else if (!strcmp(p, "p"))   { plotMode = 3; lastActivePlotMode = 3; lastPlotUs = micros(); } 
}

void readCommands() {
  while (Serial.available()) {
    char c=(char)Serial.read();
    if (c=='\n' || c=='\r') {
      if (!commandOverflow && commandIndex>0) {
        commandBuffer[commandIndex]='\0';
        processCommand(commandBuffer);
      }
      commandIndex=0;
      commandOverflow=false;
    } else if (!commandOverflow) {
      if (commandIndex<sizeof(commandBuffer)-1) commandBuffer[commandIndex++]=c;
      else { commandOverflow=true; commandIndex=0; }
    }
  }
}

void setup() {
  pinMode(PIN_SD1, OUTPUT);
  pinMode(PIN_SD2, OUTPUT);
  digitalWrite(PIN_SD1, LOW);
  digitalWrite(PIN_SD2, LOW);
  Serial.begin(115200);
  uint32_t start=millis();
  while (!Serial && (uint32_t)(millis()-start)<5000UL) delay(10);
  delay(500);
  Wire.begin(); 
  Wire.setClock(i2cClock);
  delay(100);
  
  initializeAll();

  // 開機自動歸零
  Serial.println("\n[SYSTEM] Collecting initial samples for Auto-Tare...");
  uint32_t collectStart = millis();
  while(millis() - collectStart < 1500) {
    acquire();
    delay(1);
  }
  executeTare();
  
  bleReady = setupRawBle();
  Serial.println(bleReady ? "[BLE] Advertising UTL80A" : "[BLE] Initialization failed");
  printHelp();
  
  // 開機直接狂噴 RAW 供 Python 讀取
  plotMode = 3; 
  lastActivePlotMode = 3;
  lastPlotUs = micros();
}

void loop() {
  readCommands();
  if (bleTareRequested) {
    bleTareRequested = false;
    if (Bluefruit.connected()) executeTare();
  }
  const uint8_t filterRequest = bleFilterRequest;
  if (filterRequest) {
    bleFilterRequest = 0;
    if (filterRequest == 4) wrenchWindowSize = 24;
    if (filterRequest == 6) wrenchWindowSize = 4;
    setWrenchFilter(filterRequest == 9 ? WrenchFilter::FzDirect :
                    filterRequest == 7 ? WrenchFilter::Tcn :
                    filterRequest == 8 ? WrenchFilter::TcnLinear :
                    filterRequest == 5 ? WrenchFilter::Impact :
                    filterRequest == 3 ? WrenchFilter::Kalman : WrenchFilter::Mean);
  }
  acquire();
  serviceTare(micros());
  refreshTemperatures();
  const uint32_t now = micros();
  if ((uint32_t)(now - lastWrenchUs) >= PLOT_INTERVAL_US) {
    lastWrenchUs += ((uint32_t)(now - lastWrenchUs) / PLOT_INTERVAL_US) * PLOT_INTERVAL_US;
    updateWrench();
  }
  updateRates();
  
  if (Serial && plotMode > 0 && (uint32_t)(now-lastPlotUs)>=PLOT_INTERVAL_US) {
    lastPlotUs += ((uint32_t)(now - lastPlotUs) / PLOT_INTERVAL_US) * PLOT_INTERVAL_US;
    if (plotMode == 1 || plotMode == 4) printWrench(plotMode == 4);
    else if (plotMode == 2) printWeightPlot();
    else if (plotMode == 3) printPlot();
  }
  serviceRawBle();
}
