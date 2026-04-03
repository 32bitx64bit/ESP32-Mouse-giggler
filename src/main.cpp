#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BleMouse.h>
#include <M5Unified.h>

namespace {

constexpr uint32_t kActiveLoopDelayMs = 20;
constexpr uint32_t kIdleLoopDelayMs = 100;
constexpr uint32_t kDimmedLoopDelayMs = 300;
constexpr uint32_t kPairHoldMs = 2000;
constexpr uint32_t kPairingWindowMs = 60000;
constexpr uint32_t kPairingLockDelayMs = 3000;
constexpr uint32_t kGiggleIntervalMs = 15000;
constexpr uint32_t kReturnMoveDelayMs = 120;
constexpr uint32_t kUiRefreshMs = 1000;
constexpr uint32_t kUiRefreshSlowMs = 10000;
constexpr uint32_t kBatterySampleMs = 10000;
constexpr uint32_t kScreenDimTimeoutMs = 30000;
constexpr uint32_t kBondResetHoldMs = 5000;
constexpr uint32_t kBatteryCalibrationSettleMs = 90000;
constexpr uint32_t kBatteryCalibrationSaveMs = 21600000;
constexpr int16_t kBatteryDefaultEmptyMv = 3300;
constexpr int16_t kBatteryDefaultFullMv = 4180;
constexpr int16_t kBatteryDefaultChargeBiasMv = 60;
constexpr int16_t kBatteryMinEmptyMv = 3200;
constexpr int16_t kBatteryMaxEmptyMv = 3400;
constexpr int16_t kBatteryMinFullMv = 4100;
constexpr int16_t kBatteryMaxFullMv = 4220;
constexpr int16_t kBatteryNearEmptyMv = 3380;
constexpr int16_t kBatteryNearFullMv = 4090;
constexpr int16_t kBatteryMinSpanMv = 700;
constexpr int16_t kBatteryMaxSpanMv = 1000;
constexpr int16_t kBatteryCalibrationSaveDeltaMv = 8;
constexpr float kBatteryRiseFilterAlpha = 0.18f;
constexpr float kBatteryFallFilterAlpha = 0.35f;
constexpr int8_t kGigglePixels = 1;
constexpr uint8_t kScreenBrightLevel = 96;
constexpr uint8_t kScreenDimLevel = 8;

struct BatteryCurvePoint {
  int16_t voltageMv;
  int8_t percent;
};

constexpr BatteryCurvePoint kBatteryCurve[] = {
    {3300, 0},  {3450, 5},  {3600, 15}, {3700, 30}, {3750, 45},
    {3790, 60}, {3830, 72}, {3870, 82}, {3920, 89}, {3980, 94},
    {4050, 97}, {4110, 99}, {4180, 100},
};

struct BatteryStatus {
  int32_t level = -1;
  int16_t voltageMv = 0;
  int16_t filteredVoltageMv = 0;
  bool usbPowered = false;
  bool initialized = false;
};

struct BatteryCalibration {
  int16_t emptyMv = kBatteryDefaultEmptyMv;
  int16_t fullMv = kBatteryDefaultFullMv;
  int16_t chargeBiasMv = kBatteryDefaultChargeBiasMv;
};

struct BatteryCalibrationRuntime {
  int16_t chargePeakMv = 0;
  int16_t settleLowMv = 0;
  uint32_t settleDeadlineMs = 0;
  uint32_t nextSaveMs = 0;
  bool lastUsbPowered = false;
  bool settlePending = false;
  bool dirty = false;
};

bool timeReached(const uint32_t now, const uint32_t target) {
  return static_cast<int32_t>(now - target) >= 0;
}

class ManagedBleMouse : public BleMouse {
 public:
  ManagedBleMouse()
    : BleMouse("M5StickC Plus2 Giggler", "M5Stack", 100) {
  }

  bool ready() const {
    return server_ != nullptr;
  }

  bool advertising() const {
    return advertising_;
  }

  void restartAdvertising() {
    if (!server_) {
      return;
    }

    auto* advertising = server_->getAdvertising();
    if (!advertising) {
      return;
    }

    advertising->stop();
    delay(50);
    advertising->start();
    advertising_ = true;
  }

  void setAdvertising(const bool enabled) {
    if (!server_) {
      return;
    }

    auto* advertising = server_->getAdvertising();
    if (!advertising) {
      return;
    }

    if (enabled) {
      advertising->start();
    } else {
      advertising->stop();
    }

    advertising_ = enabled;
  }

 protected:
  void onStarted(BLEServer* server) override {
    server_ = server;
    advertising_ = true;
  }

 private:
  BLEServer* server_ = nullptr;
  bool advertising_ = false;
};

ManagedBleMouse bleMouse;
M5Canvas uiCanvas(&M5.Display);
Preferences preferences;

bool gigglerEnabled = false;
bool pairingMode = false;
bool displayDirty = true;
bool initialAdvertisingStopped = false;
bool wasConnected = false;
bool returnMovePending = false;
bool screenDimmed = false;
bool ignoreNextAClick = false;
bool pairingResetRequested = false;

int8_t cycleDirection = kGigglePixels;
uint32_t pairingDeadlineMs = 0;
uint32_t connectedSinceMs = 0;
uint32_t nextGiggleMs = 0;
uint32_t returnMoveMs = 0;
uint32_t nextUiRefreshMs = 0;
uint32_t lastUserActivityMs = 0;
uint32_t nextBatterySampleMs = 0;

BatteryStatus batteryStatus;
BatteryCalibration batteryCalibration;
BatteryCalibration persistedBatteryCalibration;
BatteryCalibrationRuntime batteryCalibrationRuntime;

void setScreenDimmed(const bool dimmed) {
  if (screenDimmed == dimmed) {
    return;
  }

  screenDimmed = dimmed;
  M5.Display.setBrightness(dimmed ? kScreenDimLevel : kScreenBrightLevel);
  displayDirty = true;
}

void registerUserActivity() {
  lastUserActivityMs = millis();
  if (screenDimmed) {
    setScreenDimmed(false);
  }
}

void normalizeBatteryCalibration() {
  batteryCalibration.emptyMv = constrain(
      batteryCalibration.emptyMv, kBatteryMinEmptyMv, kBatteryMaxEmptyMv);
  batteryCalibration.fullMv = constrain(
      batteryCalibration.fullMv, kBatteryMinFullMv, kBatteryMaxFullMv);

  const int16_t spanMv = batteryCalibration.fullMv - batteryCalibration.emptyMv;
  if (spanMv < kBatteryMinSpanMv) {
    batteryCalibration.emptyMv = batteryCalibration.fullMv - kBatteryMinSpanMv;
  } else if (spanMv > kBatteryMaxSpanMv) {
    batteryCalibration.emptyMv = batteryCalibration.fullMv - kBatteryMaxSpanMv;
  }

  batteryCalibration.emptyMv = constrain(
      batteryCalibration.emptyMv, kBatteryMinEmptyMv, kBatteryMaxEmptyMv);
  batteryCalibration.chargeBiasMv = constrain(
      batteryCalibration.chargeBiasMv, 20, 120);
}

void loadBatteryCalibration() {
  preferences.begin("mousegiggler", false);

  batteryCalibration.emptyMv = static_cast<int16_t>(preferences.getShort(
      "batEmpty", kBatteryDefaultEmptyMv));
  batteryCalibration.fullMv = static_cast<int16_t>(preferences.getShort(
      "batFull", kBatteryDefaultFullMv));
  batteryCalibration.chargeBiasMv = static_cast<int16_t>(preferences.getShort(
      "batBias", kBatteryDefaultChargeBiasMv));
  normalizeBatteryCalibration();
  persistedBatteryCalibration = batteryCalibration;
}

void saveBatteryCalibrationIfNeeded(const uint32_t now) {
  if (!batteryCalibrationRuntime.dirty ||
      !timeReached(now, batteryCalibrationRuntime.nextSaveMs)) {
    return;
  }

  normalizeBatteryCalibration();

  if (abs(batteryCalibration.emptyMv - persistedBatteryCalibration.emptyMv) >=
      kBatteryCalibrationSaveDeltaMv) {
    preferences.putShort("batEmpty", batteryCalibration.emptyMv);
    persistedBatteryCalibration.emptyMv = batteryCalibration.emptyMv;
  }

  if (abs(batteryCalibration.fullMv - persistedBatteryCalibration.fullMv) >=
      kBatteryCalibrationSaveDeltaMv) {
    preferences.putShort("batFull", batteryCalibration.fullMv);
    persistedBatteryCalibration.fullMv = batteryCalibration.fullMv;
  }

  if (abs(batteryCalibration.chargeBiasMv -
          persistedBatteryCalibration.chargeBiasMv) >=
      4) {
    preferences.putShort("batBias", batteryCalibration.chargeBiasMv);
    persistedBatteryCalibration.chargeBiasMv = batteryCalibration.chargeBiasMv;
  }

  batteryCalibrationRuntime.dirty = false;
  batteryCalibrationRuntime.nextSaveMs = now + kBatteryCalibrationSaveMs;
}

int16_t remapBatteryVoltage(const int16_t voltageMv) {
  const int32_t calibratedSpanMv =
      batteryCalibration.fullMv - batteryCalibration.emptyMv;
  if (calibratedSpanMv <= 0) {
    return voltageMv;
  }

  constexpr int32_t kReferenceSpanMv =
      kBatteryDefaultFullMv - kBatteryDefaultEmptyMv;
  const int32_t adjustedMv =
      kBatteryDefaultEmptyMv +
      ((static_cast<int32_t>(voltageMv - batteryCalibration.emptyMv) *
        kReferenceSpanMv) /
       calibratedSpanMv);

  return constrain(adjustedMv, kBatteryDefaultEmptyMv,
                   kBatteryDefaultFullMv);
}

int32_t estimateBatteryPercentFromVoltage(const int16_t voltageMv) {
  const int16_t adjustedVoltageMv = remapBatteryVoltage(voltageMv);
  constexpr size_t kPointCount = sizeof(kBatteryCurve) / sizeof(kBatteryCurve[0]);

  if (adjustedVoltageMv <= kBatteryCurve[0].voltageMv) {
    return kBatteryCurve[0].percent;
  }

  if (adjustedVoltageMv >= kBatteryCurve[kPointCount - 1].voltageMv) {
    return kBatteryCurve[kPointCount - 1].percent;
  }

  for (size_t i = 1; i < kPointCount; ++i) {
    const auto& lower = kBatteryCurve[i - 1];
    const auto& upper = kBatteryCurve[i];
    if (adjustedVoltageMv <= upper.voltageMv) {
      const int32_t spanMv = upper.voltageMv - lower.voltageMv;
      const int32_t offsetMv = adjustedVoltageMv - lower.voltageMv;
      const int32_t spanPercent = upper.percent - lower.percent;
      return lower.percent + ((offsetMv * spanPercent) / spanMv);
    }
  }

  return 100;
}

void updateBatteryCalibration(const uint32_t now, const int16_t batteryVoltageMv,
                              const bool usbPowered) {
  if (batteryVoltageMv <= 0) {
    batteryCalibrationRuntime.lastUsbPowered = usbPowered;
    return;
  }

  if (usbPowered) {
    if (!batteryCalibrationRuntime.lastUsbPowered) {
      batteryCalibrationRuntime.chargePeakMv = batteryVoltageMv;
      batteryCalibrationRuntime.settlePending = false;
    } else if (batteryVoltageMv > batteryCalibrationRuntime.chargePeakMv) {
      batteryCalibrationRuntime.chargePeakMv = batteryVoltageMv;
    }

    if (batteryVoltageMv >= kBatteryNearFullMv) {
      const int16_t candidateFullMv = constrain(
          batteryVoltageMv - batteryCalibration.chargeBiasMv,
          kBatteryMinFullMv, kBatteryMaxFullMv);
      batteryCalibration.fullMv = static_cast<int16_t>(
          ((batteryCalibration.fullMv * 7) + candidateFullMv) / 8);
      batteryCalibrationRuntime.dirty = true;
    }
  } else {
    if (batteryCalibrationRuntime.lastUsbPowered) {
      batteryCalibrationRuntime.settlePending = true;
      batteryCalibrationRuntime.settleLowMv = batteryVoltageMv;
      batteryCalibrationRuntime.settleDeadlineMs =
          now + kBatteryCalibrationSettleMs;
    }

    if (batteryCalibrationRuntime.settlePending) {
      if (batteryVoltageMv < batteryCalibrationRuntime.settleLowMv) {
        batteryCalibrationRuntime.settleLowMv = batteryVoltageMv;
      }

      if (timeReached(now, batteryCalibrationRuntime.settleDeadlineMs)) {
        const int16_t observedBiasMv = max<int16_t>(
            0, batteryCalibrationRuntime.chargePeakMv -
                   batteryCalibrationRuntime.settleLowMv);
        batteryCalibration.chargeBiasMv = static_cast<int16_t>(
            ((batteryCalibration.chargeBiasMv * 3) + observedBiasMv) / 4);

        if (batteryCalibrationRuntime.settleLowMv >= kBatteryNearFullMv) {
          batteryCalibration.fullMv = static_cast<int16_t>(
              ((batteryCalibration.fullMv * 3) +
               batteryCalibrationRuntime.settleLowMv) /
              4);
        }

        batteryCalibrationRuntime.settlePending = false;
        batteryCalibrationRuntime.dirty = true;
      }
    }

    if (batteryVoltageMv <= kBatteryNearEmptyMv) {
      const int16_t candidateEmptyMv = constrain(
          batteryVoltageMv - 10, kBatteryMinEmptyMv, kBatteryMaxEmptyMv);
      batteryCalibration.emptyMv = static_cast<int16_t>(
          ((batteryCalibration.emptyMv * 15) + candidateEmptyMv) / 16);
      batteryCalibrationRuntime.dirty = true;
    }
  }

  normalizeBatteryCalibration();
  saveBatteryCalibrationIfNeeded(now);
  batteryCalibrationRuntime.lastUsbPowered = usbPowered;
}

void refreshBatteryStatus(const uint32_t now, const bool force = false) {
  if (!force && !timeReached(now, nextBatterySampleMs)) {
    return;
  }

  const int16_t batteryVoltageMv = M5.Power.getBatteryVoltage();
  const bool usbPowered = M5.Power.getVBUSVoltage() > 0;

  updateBatteryCalibration(now, batteryVoltageMv, usbPowered);

  if (batteryVoltageMv > 0) {
    int16_t effectiveVoltageMv = batteryVoltageMv;

    if (batteryStatus.initialized && batteryVoltageMv > batteryStatus.filteredVoltageMv) {
      const int16_t riseBiasMv = constrain(
          batteryVoltageMv - batteryStatus.filteredVoltageMv, 0,
          batteryCalibration.chargeBiasMv);
      effectiveVoltageMv -= riseBiasMv;
    }

    if (!batteryStatus.initialized) {
      batteryStatus.filteredVoltageMv = effectiveVoltageMv;
      batteryStatus.initialized = true;
    } else {
      const float alpha = (effectiveVoltageMv >= batteryStatus.filteredVoltageMv)
                              ? kBatteryRiseFilterAlpha
                              : kBatteryFallFilterAlpha;
      batteryStatus.filteredVoltageMv = static_cast<int16_t>(
          batteryStatus.filteredVoltageMv +
          ((effectiveVoltageMv - batteryStatus.filteredVoltageMv) * alpha));
    }

    batteryStatus.level = estimateBatteryPercentFromVoltage(
        batteryStatus.filteredVoltageMv);
  } else {
    batteryStatus.level = -1;
    batteryStatus.filteredVoltageMv = 0;
    batteryStatus.initialized = false;
  }

  batteryStatus.voltageMv = batteryVoltageMv;
  batteryStatus.usbPowered = usbPowered;
  nextBatterySampleMs = now + kBatterySampleMs;
}

void clearBondedDevices() {
  const int bondCount = esp_ble_get_bond_device_num();
  if (bondCount <= 0) {
    return;
  }

  constexpr int kMaxBondDevices = 15;
  esp_ble_bond_dev_t bondedDevices[kMaxBondDevices];
  int deviceCount = min(bondCount, kMaxBondDevices);
  if (esp_ble_get_bond_device_list(&deviceCount, bondedDevices) == ESP_OK) {
    for (int i = 0; i < deviceCount; ++i) {
      esp_ble_remove_bond_device(bondedDevices[i].bd_addr);
    }
  }
}

[[noreturn]] void resetPairingAndRestart() {
  bleMouse.setAdvertising(false);
  clearBondedDevices();
  delay(250);
  ESP.restart();
  while (true) {
    delay(1000);
  }
}

uint16_t batteryFillColor(const int32_t level) {
  if (level >= 60) {
    return TFT_GREEN;
  }
  if (level >= 25) {
    return TFT_ORANGE;
  }
  return TFT_RED;
}

void drawBatteryWidget(const int16_t x, const int16_t y, const int32_t level,
                       const int16_t voltageMv, const bool usbPowered) {
  constexpr int16_t kBodyW = 44;
  constexpr int16_t kBodyH = 18;
  constexpr int16_t kTipW = 4;
  constexpr int16_t kInnerPad = 3;

  uiCanvas.drawRoundRect(x, y, kBodyW, kBodyH, 4, TFT_WHITE);
  uiCanvas.fillRect(x + kBodyW, y + 5, kTipW, kBodyH - 10, TFT_WHITE);

  int16_t fillWidth = 0;
  if (level >= 0) {
    fillWidth = (kBodyW - (kInnerPad * 2)) * constrain(level, 0, 100) / 100;
  }

  if (fillWidth > 0) {
    uiCanvas.fillRoundRect(x + kInnerPad, y + kInnerPad, fillWidth,
                           kBodyH - (kInnerPad * 2), 2,
                           batteryFillColor(level));
  }

  uiCanvas.setTextColor(TFT_WHITE, TFT_NAVY);
  uiCanvas.setCursor(x - 2, y + 24);
  if (level >= 0) {
    uiCanvas.printf("%3ld%%", static_cast<long>(level));
  } else {
    uiCanvas.print("N/A");
  }

  uiCanvas.setCursor(x - 4, y + 38);
  if (voltageMv > 0) {
    uiCanvas.printf("%.2fV", voltageMv / 1000.0f);
  } else {
    uiCanvas.print("--.--V");
  }

  if (usbPowered) {
    uiCanvas.setTextColor(TFT_CYAN, TFT_NAVY);
    uiCanvas.setCursor(x - 2, y + 52);
    uiCanvas.print("USB");
  }
}

void drawCard(const int16_t x, const int16_t y, const int16_t w, const int16_t h,
              const char* title, const uint16_t accent) {
  uiCanvas.fillRoundRect(x, y, w, h, 10, 0x18E3);
  uiCanvas.drawRoundRect(x, y, w, h, 10, 0x3186);
  uiCanvas.fillRoundRect(x + 6, y + 6, 8, 8, 4, accent);
  uiCanvas.setTextColor(TFT_WHITE, 0x18E3);
  uiCanvas.setCursor(x + 20, y + 4);
  uiCanvas.print(title);
}

void drawProgressBar(const int16_t x, const int16_t y, const int16_t w,
                     const int16_t h, const int32_t percent,
                     const uint16_t color) {
  uiCanvas.drawRoundRect(x, y, w, h, h / 2, TFT_DARKGREY);
  const int16_t fill = (w - 4) * constrain(percent, 0, 100) / 100;
  if (fill > 0) {
    uiCanvas.fillRoundRect(x + 2, y + 2, fill, h - 4, (h - 4) / 2, color);
  }
}

void drawStatus() {
  if (!displayDirty) {
    return;
  }

  if (screenDimmed) {
    displayDirty = false;
    return;
  }

  displayDirty = false;

  const uint32_t now = millis();
  const bool connected = bleMouse.isConnected();
  const uint32_t pairRemainingMs =
      (pairingMode && !timeReached(now, pairingDeadlineMs))
          ? (pairingDeadlineMs - now)
          : 0;
  uint32_t nextMoveMs = 0;
  if (gigglerEnabled && connected) {
    if (returnMovePending) {
      nextMoveMs = timeReached(now, returnMoveMs) ? 0 : (returnMoveMs - now);
    } else {
      nextMoveMs = timeReached(now, nextGiggleMs) ? 0 : (nextGiggleMs - now);
    }
  }

  uiCanvas.fillScreen(0x0841);
  uiCanvas.fillRect(0, 0, uiCanvas.width(), 34, TFT_NAVY);

  uiCanvas.setTextColor(TFT_WHITE, TFT_NAVY);
  uiCanvas.setCursor(10, 7);
  uiCanvas.setTextSize(1);
  uiCanvas.print("M5StickC Plus2");
  uiCanvas.setCursor(10, 18);
  uiCanvas.print("Mouse Giggler");

  drawBatteryWidget(186, 8, batteryStatus.level, batteryStatus.voltageMv,
                    batteryStatus.usbPowered);

  drawCard(8, 42, 108, 42, "Bluetooth", connected ? TFT_GREEN : (pairingMode ? TFT_YELLOW : TFT_DARKGREY));
  uiCanvas.setTextColor(TFT_WHITE, 0x18E3);
  uiCanvas.setCursor(16, 60);
  if (connected) {
    uiCanvas.print("Connected");
  } else if (pairingMode) {
    uiCanvas.printf("Pairing %lus", pairRemainingMs / 1000UL);
  } else {
    uiCanvas.print("Hidden");
  }

  drawCard(124, 42, 108, 42, "Giggler", gigglerEnabled ? TFT_GREEN : TFT_RED);
  uiCanvas.setTextColor(TFT_WHITE, 0x18E3);
  uiCanvas.setCursor(132, 60);
  uiCanvas.print(gigglerEnabled ? "Enabled" : "Disabled");

  drawCard(8, 90, 224, 37, "Live Status", TFT_CYAN);
  uiCanvas.setCursor(16, 108);
  uiCanvas.setTextColor(TFT_WHITE, 0x18E3);

  if (connected && gigglerEnabled) {
    uiCanvas.printf("Next move in %lus", nextMoveMs / 1000UL);
  } else if (pairingMode) {
    uiCanvas.printf("Discoverable for %lus", pairRemainingMs / 1000UL);
  } else if (connected) {
    uiCanvas.print("Connected and waiting");
  } else {
    uiCanvas.print("Hold B for pairing mode");
  }

  uiCanvas.setCursor(16, 119);
  uiCanvas.setTextColor(TFT_LIGHTGREY, 0x18E3);
  if (screenDimmed) {
    uiCanvas.printf("A: Wake   Hold A %lus: Reset Pair",
                    kBondResetHoldMs / 1000UL);
  } else {
    uiCanvas.printf("A: Toggle   Hold A %lus: Reset Pair",
                    kBondResetHoldMs / 1000UL);
  }

  if (connected && gigglerEnabled) {
    const int32_t progress = returnMovePending
                                 ? 100 - static_cast<int32_t>((nextMoveMs * 100UL) / kReturnMoveDelayMs)
                                 : 100 - static_cast<int32_t>((nextMoveMs * 100UL) / kGiggleIntervalMs);
    drawProgressBar(128, 70, 92, 8, progress, TFT_GREEN);
  } else if (pairingMode) {
    const int32_t progress = static_cast<int32_t>((pairRemainingMs * 100UL) / kPairingWindowMs);
    drawProgressBar(16, 70, 92, 8, progress, TFT_YELLOW);
  }

  M5.Display.startWrite();
  uiCanvas.pushSprite(0, 0);
  M5.Display.endWrite();
}

void stopGigglerMotion() {
  returnMovePending = false;
  nextGiggleMs = millis() + kGiggleIntervalMs;
}

void setPairingMode(const bool enabled) {
  pairingMode = enabled;
  pairingDeadlineMs = enabled ? millis() + kPairingWindowMs : 0;
  connectedSinceMs = 0;

  if (bleMouse.ready()) {
    if (enabled) {
      if (!bleMouse.isConnected()) {
        bleMouse.restartAdvertising();
      }
    } else {
      bleMouse.setAdvertising(false);
    }
  }

  displayDirty = true;
}

void handleConnectionState() {
  const bool connected = bleMouse.isConnected();

  if (connected == wasConnected) {
    return;
  }

  wasConnected = connected;

  if (connected) {
    connectedSinceMs = millis();
    returnMovePending = false;
    nextGiggleMs = millis() + kGiggleIntervalMs;
  } else {
    connectedSinceMs = 0;
    stopGigglerMotion();
    if (pairingMode && bleMouse.ready()) {
      bleMouse.setAdvertising(true);
    }
  }

  displayDirty = true;
}

void handleButtons() {
  if (M5.BtnA.wasPressed()) {
    if (screenDimmed) {
      setScreenDimmed(false);
      ignoreNextAClick = true;
    }
    lastUserActivityMs = millis();
  }

  if (M5.BtnB.wasPressed()) {
    lastUserActivityMs = millis();
  }

  if (!pairingResetRequested && M5.BtnA.pressedFor(kBondResetHoldMs)) {
    pairingResetRequested = true;
    resetPairingAndRestart();
  }

  if (M5.BtnA.wasReleased()) {
    pairingResetRequested = false;
  }

  if (M5.BtnA.wasClicked()) {
    if (ignoreNextAClick) {
      ignoreNextAClick = false;
    } else {
      gigglerEnabled = !gigglerEnabled;
      stopGigglerMotion();
      displayDirty = true;
      registerUserActivity();
    }
  }

  if (M5.BtnB.wasHold()) {
    setPairingMode(true);
    lastUserActivityMs = millis();
  }
}

void maintainPairingLock() {
  if (!pairingMode || !bleMouse.isConnected()) {
    return;
  }

  if (connectedSinceMs != 0 && timeReached(millis(), connectedSinceMs + kPairingLockDelayMs)) {
    setPairingMode(false);
  }
}

void maintainScreenDimming() {
  if (screenDimmed) {
    return;
  }

  if (timeReached(millis(), lastUserActivityMs + kScreenDimTimeoutMs)) {
    setScreenDimmed(true);
  }
}

void maintainPairingWindow() {
  if (!pairingMode || bleMouse.isConnected()) {
    return;
  }

  const uint32_t now = millis();
  if (timeReached(now, pairingDeadlineMs)) {
    setPairingMode(false);
    return;
  }

  if (bleMouse.ready() && !bleMouse.advertising()) {
    bleMouse.setAdvertising(true);
  }
}

void runGiggler() {
  if (!gigglerEnabled || !bleMouse.isConnected()) {
    returnMovePending = false;
    return;
  }

  const uint32_t now = millis();

  if (returnMovePending) {
    if (timeReached(now, returnMoveMs)) {
      bleMouse.move(-cycleDirection, 0);
      returnMovePending = false;
      cycleDirection = -cycleDirection;
      nextGiggleMs = now + kGiggleIntervalMs;
    }
    return;
  }

  if (timeReached(now, nextGiggleMs)) {
    bleMouse.move(cycleDirection, 0);
    returnMovePending = true;
    returnMoveMs = now + kReturnMoveDelayMs;
  }
}

uint32_t computeLoopDelayMs(const uint32_t now) {
  if (returnMovePending) {
    return kActiveLoopDelayMs;
  }

  if (pairingMode || bleMouse.isConnected()) {
    return kActiveLoopDelayMs;
  }

  if (screenDimmed) {
    return kDimmedLoopDelayMs;
  }

  const uint32_t screenDimAtMs = lastUserActivityMs + kScreenDimTimeoutMs;
  const uint32_t untilDim = timeReached(now, screenDimAtMs)
                               ? 0
                               : (screenDimAtMs - now);

  return untilDim <= kIdleLoopDelayMs ? kActiveLoopDelayMs : kIdleLoopDelayMs;
}

}  // namespace

void setup() {
  setCpuFrequencyMhz(80);
  auto cfg = M5.config();
  M5.begin(cfg);
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true, true);
  esp_wifi_deinit();
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  M5.Display.setRotation(1);
  M5.Display.setFont(&fonts::Font2);
  M5.Display.setBrightness(kScreenBrightLevel);
  M5.BtnB.setHoldThresh(kPairHoldMs);

  uiCanvas.setColorDepth(16);
  uiCanvas.createSprite(M5.Display.width(), M5.Display.height());
  uiCanvas.setFont(&fonts::Font2);
  uiCanvas.setTextWrap(false);

  Serial.begin(115200);
  Serial.println("Starting mouse giggler...");

  loadBatteryCalibration();
  bleMouse.begin();
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P3);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P3);
  nextGiggleMs = millis() + kGiggleIntervalMs;
  lastUserActivityMs = millis();
  refreshBatteryStatus(lastUserActivityMs, true);
  drawStatus();
}

void loop() {
  M5.update();

  const uint32_t now = millis();

  if (bleMouse.ready() && !initialAdvertisingStopped && !bleMouse.isConnected()) {
    bleMouse.setAdvertising(false);
    initialAdvertisingStopped = true;
    displayDirty = true;
  }

  handleConnectionState();
  handleButtons();
  maintainPairingWindow();
  maintainPairingLock();
  maintainScreenDimming();
  runGiggler();

  refreshBatteryStatus(now);

  if (!screenDimmed && timeReached(now, nextUiRefreshMs)) {
    const bool dynamicContent =
        (bleMouse.isConnected() && gigglerEnabled) || pairingMode;
    nextUiRefreshMs = now + (dynamicContent ? kUiRefreshMs : kUiRefreshSlowMs);
    displayDirty = true;
  }

  drawStatus();

  delay(computeLoopDelayMs(now));
}