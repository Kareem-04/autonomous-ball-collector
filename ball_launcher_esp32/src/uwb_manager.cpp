#include "uwb_manager.h"
#include <math.h>

// Matches the user's working config project exactly:
//   BU04_CMD  = Serial2 (UART2) for AT commands — pins 16/17
//   BU04_DATA = HardwareSerial(1) (UART1) for data stream — RX=18, TX=19
// The working config project uses this exact setup without corruption.
HardwareSerial BU04_DATA(1);

void UWBManager::begin() {
  // AT command serial — same as always
  Serial2.begin(UWB_BAUD_RATE, SERIAL_8N1, UWB_CMD_RX_PIN, UWB_TX_PIN);

  // Data stream serial — match working config project exactly
  // Key: use TX pin 19 (not -1), same as working config
  BU04_DATA.begin(UWB_BAUD_RATE, SERIAL_8N1, UWB_DATA_RX_PIN, 19);

  _data    = {0, 0, 0, 0, 0, false, 0};
  _rawData = {0, 0, 0, 0, 0, false, 0};
  _passthrough    = false;
  _initialized    = false;
  _lastRead       = 0;
  _rxPos          = 0;

  _filteredDistance = 0;
  _filteredAngle   = 0;
  _filteredX       = 0;
  _filteredY       = 0;
  _filterInitialized = false;

  memset(_rxBuf, 0, RX_BUF_SIZE);

  delay(100);
  flushInput();

  Serial.println("[UWB] Initialized:");
  Serial.printf("[UWB]   AT cmds  → Serial2 (TX=%d, RX=%d)\n", UWB_TX_PIN, UWB_CMD_RX_PIN);
  Serial.printf("[UWB]   Data     → UART1   (RX=%d, from BU04 P2)\n", UWB_DATA_RX_PIN);
  Serial.printf("[UWB]   Baud: %d\n", UWB_BAUD_RATE);
}

// AT commands go through Serial2
bool UWBManager::sendCommand(const char* cmd, char* response, size_t respLen, unsigned long timeoutMs) {
  while (Serial2.available()) Serial2.read();

  Serial2.print(cmd);
  Serial2.print("\r\n");

  Serial.printf("[UWB] >> %s\n", cmd);

  unsigned long start = millis();
  size_t pos = 0;
  char tempBuf[256];
  memset(tempBuf, 0, sizeof(tempBuf));

  while (millis() - start < timeoutMs) {
    if (Serial2.available()) {
      char c = Serial2.read();
      if (pos < sizeof(tempBuf) - 1) {
        tempBuf[pos++] = c;
      }
      if (strstr(tempBuf, "OK") || strstr(tempBuf, "ERR")) {
        delay(20);
        while (Serial2.available() && pos < sizeof(tempBuf) - 1) {
          tempBuf[pos++] = Serial2.read();
        }
        break;
      }
    }
  }

  tempBuf[pos] = '\0';

  if (response && respLen > 0) {
    strncpy(response, tempBuf, respLen - 1);
    response[respLen - 1] = '\0';
  }

  if (pos > 0) {
    Serial.printf("[UWB] << %s\n", tempBuf);
  } else {
    Serial.println("[UWB] << (no response / timeout)");
  }

  return (strstr(tempBuf, "OK") != nullptr);
}

bool UWBManager::setupAnchor() {
  Serial.println("[UWB] ========================================");
  Serial.println("[UWB]  BU04 G451 ANCHOR SETUP (PDoA MODE)");
  Serial.println("[UWB] ========================================");

  Serial.println("[UWB] Step 1: Testing communication...");
  if (!sendCommand("AT")) {
    Serial.println("[UWB] ERROR: No response from BU04!");
    return false;
  }

  Serial.println("[UWB] Step 2: Firmware version...");
  sendCommand("AT+GETVER");

  Serial.println("[UWB] Step 3: Setting PDoA mode...");
  sendCommand("AT+SETUWBMODE=1");

  Serial.println("[UWB] Step 4: Configure as Anchor...");
  sendCommand("AT+SETCFG=0,1,1,1,1");

  Serial.println("[UWB] Step 5: Enable filtering...");
  sendCommand("AT+FILTER=1");

  Serial.println("[UWB] Step 6: Set output rate (100ms)...");
  sendCommand("AT+UARTRATE=100");

  Serial.println("[UWB] Step 7: Saving...");
  sendCommand("AT+SAVE");

  Serial.println("[UWB] Step 8: Restarting...");
  sendCommand("AT+RESTART");
  delay(2000);
  flushInput();

  _initialized = true;
  Serial.println("[UWB] SETUP COMPLETE");
  return true;
}

// Data reading — from BU04_DATA (UART1, RX=GPIO18)
bool UWBManager::update() {
  if (_passthrough) return false;

  bool gotData = false;

  while (BU04_DATA.available()) {
    char c = BU04_DATA.read();

    if (c == '\n' || c == '\r') {
      if (_rxPos > 0) {
        _rxBuf[_rxPos] = '\0';
        if (processLine(_rxBuf)) {
          gotData = true;
        }
        _rxPos = 0;
      }
    } else if (_rxPos < RX_BUF_SIZE - 1) {
      _rxBuf[_rxPos++] = c;
    } else {
      _rxPos = 0;
    }
  }

  if (gotData) {
    _lastRead = millis();
    applyFilter();
  }

  return gotData;
}

bool UWBManager::processLine(const char* line) {
  if (strlen(line) < 5) return false;

  if (strncmp(line, "OK", 2) == 0) return false;
  if (strncmp(line, "ERR", 3) == 0) return false;
  if (strstr(line, "UWB Module") != nullptr) return false;
  if (strstr(line, "IIC Error") != nullptr) return false;

  if (parseTWRJson(line)) return true;
  if (parseDistanceCmd(line)) return true;

  return false;
}

bool UWBManager::parseTWRJson(const char* line) {
  const char* jsonStart = strstr(line, "{\"TWR\"");
  if (!jsonStart) return false;

  int D = 0, P = 0, Xcm = 0, Ycm = 0;

  bool hasD   = jsonGetInt(jsonStart, "\"D\":",   &D);
  bool hasP   = jsonGetInt(jsonStart, "\"P\":",   &P);
  bool hasXcm = jsonGetInt(jsonStart, "\"Xcm\":", &Xcm);
  bool hasYcm = jsonGetInt(jsonStart, "\"Ycm\":", &Ycm);

  if (!hasD) return false;

  float distMm = (float)D * 10.0f;

  float angleDeg = 0.0f;
  if (hasXcm && hasYcm && (Xcm != 0 || Ycm != 0)) {
    angleDeg = atan2((float)Xcm, (float)Ycm) * 180.0f / PI;
  }

  _rawData.distance_mm = distMm;
  _rawData.angle_deg   = angleDeg;
  _rawData.pdoa_raw    = P;
  _rawData.x_cm        = (float)Xcm;
  _rawData.y_cm        = (float)Ycm;
  _rawData.valid        = true;
  _rawData.timestamp    = millis();

  if (distMm >= UWB_MIN_DISTANCE && distMm <= UWB_MAX_DISTANCE) {
    _data = _rawData;
    return true;
  }

  return false;
}

bool UWBManager::jsonGetInt(const char* json, const char* key, int* out) {
  const char* p = strstr(json, key);
  if (!p) return false;

  p += strlen(key);
  while (*p == ' ') p++;

  bool negative = false;
  if (*p == '-') {
    negative = true;
    p++;
  }

  if (!isdigit(*p)) return false;

  int val = 0;
  while (isdigit(*p)) {
    val = val * 10 + (*p - '0');
    p++;
  }

  *out = negative ? -val : val;
  return true;
}

bool UWBManager::parseDistanceCmd(const char* line) {
  const char* distPtr = strstr(line, "distance:");
  if (!distPtr) distPtr = strstr(line, "Distance:");
  if (!distPtr) return false;

  distPtr += 9;
  while (*distPtr == ' ') distPtr++;

  float distMeters = atof(distPtr);
  if (distMeters <= 0.0f) return false;

  float distMm = distMeters * 1000.0f;

  _rawData.distance_mm = distMm;
  _rawData.valid     = true;
  _rawData.timestamp = millis();

  if (distMm >= UWB_MIN_DISTANCE && distMm <= UWB_MAX_DISTANCE) {
    _data = _rawData;
    return true;
  }
  return false;
}

void UWBManager::applyFilter() {
  if (!_filterInitialized) {
    _filteredDistance = _data.distance_mm;
    _filteredAngle   = _data.angle_deg;
    _filteredX       = _data.x_cm;
    _filteredY       = _data.y_cm;
    _filterInitialized = true;
    return;
  }

  _filteredDistance = FILTER_ALPHA_DIST  * _data.distance_mm + (1.0f - FILTER_ALPHA_DIST)  * _filteredDistance;
  _filteredAngle   = FILTER_ALPHA_ANGLE * _data.angle_deg   + (1.0f - FILTER_ALPHA_ANGLE) * _filteredAngle;
  _filteredX       = FILTER_ALPHA_ANGLE * _data.x_cm        + (1.0f - FILTER_ALPHA_ANGLE) * _filteredX;
  _filteredY       = FILTER_ALPHA_ANGLE * _data.y_cm        + (1.0f - FILTER_ALPHA_ANGLE) * _filteredY;
}

UWBData UWBManager::getData() const { return _data; }

bool UWBManager::hasValidData(unsigned long maxAgeMs) const {
  if (!_data.valid) return false;
  return (millis() - _data.timestamp) < maxAgeMs;
}

float UWBManager::getFilteredDistance() const { return _filteredDistance; }
float UWBManager::getFilteredAngle() const   { return _filteredAngle; }
float UWBManager::getFilteredX() const       { return _filteredX; }
float UWBManager::getFilteredY() const       { return _filteredY; }

// Passthrough uses Serial2 (AT command port)
void UWBManager::enablePassthrough() {
  _passthrough = true;
  Serial.println("[UWB] === PASSTHROUGH MODE ===");
  Serial.println("[UWB] Type EXIT to leave.");
  Serial.println("[UWB]   AT+GETCFG / AT+GETUWBMODE / AT+GETKLIST");
  Serial.println("[UWB]   AT+ADDTAG / AT+DISTANCE / AT+SAVE / AT+RESTART");
  Serial.println("[UWB] ========================");
  while (Serial2.available()) Serial2.read();
}

void UWBManager::disablePassthrough() {
  _passthrough = false;
  Serial.println("[UWB] Passthrough disabled.");
}

bool UWBManager::isPassthrough() const { return _passthrough; }

void UWBManager::handlePassthrough() {
  if (!_passthrough) return;

  while (Serial.available()) {
    char c = Serial.read();
    Serial2.write(c);
    Serial.write(c);
  }

  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);
  }
}

void UWBManager::debugRawData() {
  Serial.println("[UWB-DBG] === RAW DATA DUMP (3 seconds) ===");
  Serial.printf("[UWB-DBG] Reading UART1, RX=GPIO%d (BU04 P2)\n", UWB_DATA_RX_PIN);
  Serial.printf("[UWB-DBG] BU04_DATA.available() = %d\n", BU04_DATA.available());
  Serial.println("[UWB-DBG] --- START ---");

  unsigned long start = millis();
  int byteCount = 0;

  while (millis() - start < 3000) {
    if (BU04_DATA.available()) {
      char c = BU04_DATA.read();
      Serial.write(c);
      byteCount++;
    }
  }

  Serial.println();
  Serial.printf("[UWB-DBG] --- END --- (%d bytes in 3 sec)\n", byteCount);

  if (byteCount == 0) {
    Serial.println("[UWB-DBG] NO DATA! Check BU04 P2 → GPIO 18 wire.");
  }
}

void UWBManager::printStatus() {
  Serial.println("[UWB] --- Status ---");
  Serial.printf("[UWB] Initialized: %s\n", _initialized ? "YES" : "NO");
  Serial.printf("[UWB] Passthrough: %s\n", _passthrough ? "YES" : "NO");
  Serial.printf("[UWB] Valid data:  %s\n", _data.valid ? "YES" : "NO");
  Serial.printf("[UWB] AT cmd: Serial2 (TX=%d, RX=%d)\n", UWB_TX_PIN, UWB_CMD_RX_PIN);
  Serial.printf("[UWB] Data:   UART1   (RX=%d)\n", UWB_DATA_RX_PIN);

  if (_data.valid) {
    Serial.printf("[UWB] Raw   — D:%dcm  P:%d  Xcm:%.0f  Ycm:%.0f  Angle:%.1f°\n",
      (int)(_data.distance_mm / 10.0f), _data.pdoa_raw,
      _data.x_cm, _data.y_cm, _data.angle_deg);
    Serial.printf("[UWB] Filt. — D:%.0fmm (%.2fm)  Angle:%.1f°  X:%.0fcm  Y:%.0fcm\n",
      _filteredDistance, _filteredDistance / 1000.0f,
      _filteredAngle, _filteredX, _filteredY);
    Serial.printf("[UWB] Age: %lu ms\n", millis() - _data.timestamp);
  } else {
    Serial.println("[UWB] No data received yet.");
    Serial.println("[UWB] Try RAWUWB to check raw bytes.");
  }
  Serial.println("[UWB] ----------------");
}

void UWBManager::flushInput() {
  while (Serial2.available()) Serial2.read();
  while (BU04_DATA.available()) BU04_DATA.read();
  _rxPos = 0;
}
