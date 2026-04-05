#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_system.h>
#include <M5Unified.h>

namespace {

constexpr uint32_t kActiveLoopDelayMs = 40;
constexpr uint32_t kConnectedLoopDelayMs = 180;
constexpr uint32_t kIdleLoopDelayMs = 250;
constexpr uint32_t kDimmedLoopDelayMs = 350;
constexpr uint32_t kPairHoldMs = 2000;
constexpr uint32_t kPairingWindowMs = 60000;
constexpr uint32_t kPairingLockDelayMs = 3000;
constexpr uint32_t kSecurityHandshakeTimeoutMs = 15000;
constexpr uint32_t kGiggleIntervalMs = 15000;
constexpr uint32_t kReturnMoveDelayMs = 120;
constexpr uint32_t kUiRefreshMs = 10000;
constexpr uint32_t kUiRefreshSlowMs = 60000;
constexpr uint32_t kBatterySampleMs = 120000;
constexpr uint32_t kScreenDimTimeoutMs = 10000;
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
constexpr int8_t kBleTxPowerDbm = -3;
constexpr uint16_t kBleAppearanceMouse = 0x03C2;
constexpr uint16_t kBleConnIntervalMin = 72;
constexpr uint16_t kBleConnIntervalMax = 96;
constexpr uint16_t kBleConnLatency = 12;
constexpr uint16_t kBleConnTimeout = 600;

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

constexpr uint8_t kMouseReportDescriptor[] = {
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,        // USAGE (Mouse)
    0xA1, 0x01,        // COLLECTION (Application)
    0x09, 0x01,        //   USAGE (Pointer)
    0xA1, 0x00,        //   COLLECTION (Physical)
    0x05, 0x09,        //     USAGE_PAGE (Button)
    0x19, 0x01,        //     USAGE_MINIMUM (Button 1)
    0x29, 0x05,        //     USAGE_MAXIMUM (Button 5)
    0x15, 0x00,        //     LOGICAL_MINIMUM (0)
    0x25, 0x01,        //     LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //     REPORT_SIZE (1)
    0x95, 0x05,        //     REPORT_COUNT (5)
    0x81, 0x02,        //     INPUT (Data,Var,Abs)
    0x75, 0x03,        //     REPORT_SIZE (3)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0x81, 0x03,        //     INPUT (Const,Var,Abs)
    0x05, 0x01,        //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,        //     USAGE (X)
    0x09, 0x31,        //     USAGE (Y)
    0x09, 0x38,        //     USAGE (Wheel)
    0x15, 0x81,        //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F,        //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,        //     REPORT_SIZE (8)
    0x95, 0x03,        //     REPORT_COUNT (3)
    0x81, 0x06,        //     INPUT (Data,Var,Rel)
    0x05, 0x0C,        //     USAGE_PAGE (Consumer)
    0x0A, 0x38, 0x02,  //     USAGE (AC Pan)
    0x15, 0x81,        //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F,        //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,        //     REPORT_SIZE (8)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0x81, 0x06,        //     INPUT (Data,Var,Rel)
    0xC0,              //   END_COLLECTION
    0xC0,              // END_COLLECTION
};

class ManagedBleMouse {
 public:
  ManagedBleMouse() = default;

  void begin() {
    NimBLEDevice::init("M5StickC Plus2 Giggler");
    NimBLEDevice::setPower(kBleTxPowerDbm);
    NimBLEDevice::setSecurityAuth(true, false, true);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

    server_ = NimBLEDevice::createServer();
    server_->setCallbacks(&serverCallbacks_, false);
    server_->advertiseOnDisconnect(false);

    hid_ = new NimBLEHIDDevice(server_);
    hid_->setManufacturer("M5Stack");
    hid_->setPnp(0x02, 0xE502, 0xA111, 0x0210);
    hid_->setHidInfo(0x00, 0x02);
    hid_->setReportMap(const_cast<uint8_t*>(kMouseReportDescriptor),
                       sizeof(kMouseReportDescriptor));
    inputReport_ = hid_->getInputReport(0);
    inputReport_->setCallbacks(&inputCallbacks_);
    hid_->setBatteryLevel(batteryLevel_);

    server_->start();

    advertisingInstance_ = NimBLEDevice::getAdvertising();
    advertisingInstance_->setName("M5StickC Plus2 Giggler");
    advertisingInstance_->setAppearance(kBleAppearanceMouse);
    advertisingInstance_->addServiceUUID(hid_->getHidService()->getUUID());
    advertisingInstance_->setPreferredParams(kBleConnIntervalMin,
                                             kBleConnIntervalMax);
    advertisingInstance_->enableScanResponse(false);

    ready_ = true;
  }

  bool ready() const {
    return ready_;
  }

  bool advertising() const {
    return advertising_;
  }

  bool hasLink() const {
    return linkConnected_;
  }

  bool bonded() const {
    return bonded_;
  }

  bool secure() const {
    return encrypted_ && bonded_;
  }

  bool isConnected() const {
    return linkConnected_ && subscribed_ && secure();
  }

  uint32_t passkey() const {
    return passkey_;
  }

  void setPasskey(const uint32_t passkey) {
    passkey_ = passkey;
  }

  void restartAdvertising() {
    if (!ready_) {
      return;
    }

    if (!advertisingInstance_) {
      return;
    }

    advertisingInstance_->stop();
    delay(20);
    advertising_ = advertisingInstance_->start();
  }

  void setAdvertising(const bool enabled) {
    if (!ready_ || !advertisingInstance_) {
      return;
    }

    if (enabled) {
      advertising_ = advertisingInstance_->start();
    } else {
      advertisingInstance_->stop();
      advertising_ = false;
    }
  }

  void disconnect() {
    if (server_ && connHandle_ != BLE_HS_CONN_HANDLE_NONE) {
      server_->disconnect(connHandle_);
    }
  }

  void setBatteryLevel(const uint8_t level, const bool notify = false) {
    batteryLevel_ = level;
    if (hid_) {
      hid_->setBatteryLevel(level, notify);
    }
  }

  bool move(const int8_t x, const int8_t y, const int8_t wheel = 0,
            const int8_t hWheel = 0) {
    if (!isConnected() || inputReport_ == nullptr) {
      return false;
    }

    const uint8_t report[5] = {
        buttons_,
        static_cast<uint8_t>(x),
        static_cast<uint8_t>(y),
        static_cast<uint8_t>(wheel),
        static_cast<uint8_t>(hWheel),
    };

    return inputReport_->notify(report, sizeof(report), connHandle_);
  }

 private:
  class ServerCallbacks : public NimBLEServerCallbacks {
   public:
    explicit ServerCallbacks(ManagedBleMouse& owner) : owner_(owner) {
    }

    void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
      owner_.handleConnect(server, connInfo);
    }

    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo,
                      const int reason) override {
      owner_.handleDisconnect(server, connInfo, reason);
    }

    uint32_t onPassKeyDisplay() override {
      return owner_.passkey_;
    }

    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
      owner_.handleAuthenticationComplete(connInfo);
    }

   private:
    ManagedBleMouse& owner_;
  };

  class InputCallbacks : public NimBLECharacteristicCallbacks {
   public:
    explicit InputCallbacks(ManagedBleMouse& owner) : owner_(owner) {
    }

    void onSubscribe(NimBLECharacteristic* characteristic,
                     NimBLEConnInfo& connInfo, const uint16_t subValue) override {
      (void)characteristic;
      owner_.handleSubscribe(connInfo, subValue);
    }

   private:
    ManagedBleMouse& owner_;
  };

  void resetSessionState() {
    connHandle_ = BLE_HS_CONN_HANDLE_NONE;
    linkConnected_ = false;
    encrypted_ = false;
    authenticated_ = false;
    bonded_ = false;
    subscribed_ = false;
  }

  void refreshSecurityState(const NimBLEConnInfo& connInfo) {
    const NimBLEAddress idAddress = connInfo.getIdAddress();
    encrypted_ = connInfo.isEncrypted();
    authenticated_ = connInfo.isAuthenticated();
    bonded_ = connInfo.isBonded() ||
              (!idAddress.isNull() && NimBLEDevice::isBonded(idAddress)) ||
              NimBLEDevice::isBonded(connInfo.getAddress());
  }

  void handleConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) {
    connHandle_ = connInfo.getConnHandle();
    linkConnected_ = true;
    subscribed_ = false;
    refreshSecurityState(connInfo);
    advertising_ = true;
    server->updateConnParams(connHandle_, kBleConnIntervalMin,
                             kBleConnIntervalMax, kBleConnLatency,
                             kBleConnTimeout);
    NimBLEDevice::startSecurity(connHandle_);
  }

  void handleDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo,
                        const int reason) {
    (void)server;
    (void)connInfo;
    (void)reason;
    advertising_ = false;
    resetSessionState();
  }

  void handleAuthenticationComplete(NimBLEConnInfo& connInfo) {
    refreshSecurityState(connInfo);
    if (!secure()) {
      disconnect();
    }
  }

  void handleSubscribe(NimBLEConnInfo& connInfo, const uint16_t subValue) {
    if (connInfo.getConnHandle() != connHandle_) {
      return;
    }

    subscribed_ = (subValue & 0x0001U) != 0;
    refreshSecurityState(connInfo);
  }

  NimBLEServer* server_ = nullptr;
  NimBLEHIDDevice* hid_ = nullptr;
  NimBLEAdvertising* advertisingInstance_ = nullptr;
  NimBLECharacteristic* inputReport_ = nullptr;
  ServerCallbacks serverCallbacks_{*this};
  InputCallbacks inputCallbacks_{*this};
  uint16_t connHandle_ = BLE_HS_CONN_HANDLE_NONE;
  uint8_t buttons_ = 0;
  uint8_t batteryLevel_ = 100;
  uint32_t passkey_ = 123456;
  bool ready_ = false;
  bool advertising_ = false;
  bool linkConnected_ = false;
  bool encrypted_ = false;
  bool authenticated_ = false;
  bool bonded_ = false;
  bool subscribed_ = false;
};

ManagedBleMouse bleMouse;
M5Canvas uiCanvas(&M5.Display);
Preferences preferences;

bool gigglerEnabled = false;
bool pairingMode = false;
bool displayDirty = true;
bool wasConnected = false;
bool wasLinkConnected = false;
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
uint32_t securityHandshakeDeadlineMs = 0;

BatteryStatus batteryStatus;
BatteryCalibration batteryCalibration;
BatteryCalibration persistedBatteryCalibration;
BatteryCalibrationRuntime batteryCalibrationRuntime;
int32_t lastPublishedBatteryLevel = -1;

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

  const int32_t previousLevel = batteryStatus.level;

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
  if (bleMouse.ready() && batteryStatus.level >= 0) {
    const uint8_t level =
        static_cast<uint8_t>(constrain(batteryStatus.level, 0L, 100L));
    const bool shouldNotify =
        bleMouse.isConnected() && batteryStatus.level != previousLevel;
    bleMouse.setBatteryLevel(level, shouldNotify);
    lastPublishedBatteryLevel = batteryStatus.level;
  }
  nextBatterySampleMs = now + kBatterySampleMs;
}

void clearBondedDevices() {
  NimBLEDevice::deleteAllBonds();
}

[[noreturn]] void resetPairingAndRestart() {
  bleMouse.disconnect();
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
  const bool linkConnected = bleMouse.hasLink();
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

  drawCard(8, 42, 108, 42, "Bluetooth",
           connected ? TFT_GREEN
                     : (linkConnected ? TFT_CYAN
                                      : (pairingMode ? TFT_YELLOW : TFT_DARKGREY)));
  uiCanvas.setTextColor(TFT_WHITE, 0x18E3);
  uiCanvas.setCursor(16, 60);
  if (connected) {
    uiCanvas.print("Connected");
  } else if (linkConnected) {
    uiCanvas.print("Securing");
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
  } else if (linkConnected) {
    uiCanvas.print("Bonding...");
  } else if (pairingMode) {
    uiCanvas.printf("Discoverable %lus", pairRemainingMs / 1000UL);
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
  const bool wasPairingMode = pairingMode;
  pairingMode = enabled;
  pairingDeadlineMs = enabled ? millis() + kPairingWindowMs : 0;
  connectedSinceMs = 0;

  if (bleMouse.ready()) {
    if (enabled) {
      if (!bleMouse.hasLink()) {
        bleMouse.restartAdvertising();
      }
    } else {
      bleMouse.setAdvertising(false);
    }
  }

  displayDirty = true;
}

void handleConnectionState() {
  const bool linkConnected = bleMouse.hasLink();
  const bool connected = bleMouse.isConnected();

  if (linkConnected != wasLinkConnected) {
    wasLinkConnected = linkConnected;

    if (linkConnected) {
      securityHandshakeDeadlineMs = millis() + kSecurityHandshakeTimeoutMs;
      returnMovePending = false;
    } else {
      securityHandshakeDeadlineMs = 0;
      connectedSinceMs = 0;
      stopGigglerMotion();
      if (pairingMode && bleMouse.ready()) {
        bleMouse.setAdvertising(true);
      }
    }

    displayDirty = true;
  }

  if (connected == wasConnected) {
    return;
  }

  wasConnected = connected;

  if (connected) {
    connectedSinceMs = millis();
    securityHandshakeDeadlineMs = 0;
    returnMovePending = false;
    nextGiggleMs = millis() + kGiggleIntervalMs;
    if (batteryStatus.level >= 0) {
      bleMouse.setBatteryLevel(
          static_cast<uint8_t>(constrain(batteryStatus.level, 0L, 100L)),
          true);
      lastPublishedBatteryLevel = batteryStatus.level;
    }
  } else {
    connectedSinceMs = 0;
    stopGigglerMotion();
    if (pairingMode && bleMouse.ready() && !bleMouse.hasLink()) {
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

void maintainSecureLink() {
  if (!bleMouse.hasLink() || bleMouse.isConnected()) {
    return;
  }

  const uint32_t now = millis();
  if (!pairingMode ||
      (securityHandshakeDeadlineMs != 0 &&
       timeReached(now, securityHandshakeDeadlineMs))) {
    bleMouse.disconnect();
    securityHandshakeDeadlineMs = now + kSecurityHandshakeTimeoutMs;
    displayDirty = true;
  }
}

void maintainPairingWindow() {
  if (!pairingMode || bleMouse.hasLink()) {
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
  uint32_t maxDelayMs = kIdleLoopDelayMs;

  if (returnMovePending) {
    maxDelayMs = kActiveLoopDelayMs;
  } else if (pairingMode || (bleMouse.hasLink() && !bleMouse.isConnected())) {
    maxDelayMs = kActiveLoopDelayMs;
  } else if (bleMouse.isConnected()) {
    maxDelayMs = kConnectedLoopDelayMs;
  } else if (screenDimmed) {
    maxDelayMs = kDimmedLoopDelayMs;
  }

  const uint32_t screenDimAtMs = lastUserActivityMs + kScreenDimTimeoutMs;
  const uint32_t untilDim =
      timeReached(now, screenDimAtMs) ? 0 : (screenDimAtMs - now);

  uint32_t untilNextWorkMs = maxDelayMs;

  if (returnMovePending) {
    const uint32_t untilReturnMs =
        timeReached(now, returnMoveMs) ? 0 : (returnMoveMs - now);
    untilNextWorkMs = min(untilNextWorkMs, untilReturnMs);
  } else if (gigglerEnabled && bleMouse.isConnected()) {
    const uint32_t untilGiggleMs =
        timeReached(now, nextGiggleMs) ? 0 : (nextGiggleMs - now);
    untilNextWorkMs = min(untilNextWorkMs, untilGiggleMs);
  }

  if (!screenDimmed && nextUiRefreshMs != 0) {
    const uint32_t untilUiRefreshMs =
        timeReached(now, nextUiRefreshMs) ? 0 : (nextUiRefreshMs - now);
    untilNextWorkMs = min(untilNextWorkMs, untilUiRefreshMs);
  }

  if (nextBatterySampleMs != 0) {
    const uint32_t untilBatterySampleMs =
        timeReached(now, nextBatterySampleMs) ? 0 : (nextBatterySampleMs - now);
    untilNextWorkMs = min(untilNextWorkMs, untilBatterySampleMs);
  }

  if (!screenDimmed && untilDim <= untilNextWorkMs) {
    return min(kActiveLoopDelayMs, untilDim);
  }

  return untilNextWorkMs;
}

}  // namespace

void setup() {
  setCpuFrequencyMhz(80);
  auto cfg = M5.config();
  cfg.internal_imu = false;
  cfg.internal_mic = false;
  cfg.internal_spk = false;
  cfg.led_brightness = 0;
  M5.begin(cfg);
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true, true);
  esp_wifi_deinit();
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  M5.Power.setLed(0);
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
  nextGiggleMs = millis() + kGiggleIntervalMs;
  lastUserActivityMs = millis();
  refreshBatteryStatus(lastUserActivityMs, true);
  drawStatus();
}

void loop() {
  M5.update();

  const uint32_t now = millis();

  handleConnectionState();
  handleButtons();
  maintainPairingWindow();
  maintainPairingLock();
  maintainSecureLink();
  maintainScreenDimming();
  runGiggler();

  refreshBatteryStatus(now);

  if (!screenDimmed && timeReached(now, nextUiRefreshMs)) {
    const uint32_t refreshMs = pairingMode
                                   ? 1000
                                   : ((bleMouse.isConnected() && gigglerEnabled)
                                          ? kUiRefreshMs
                                          : kUiRefreshSlowMs);
    nextUiRefreshMs = now + refreshMs;
    displayDirty = true;
  }

  drawStatus();

  delay(computeLoopDelayMs(now));
}