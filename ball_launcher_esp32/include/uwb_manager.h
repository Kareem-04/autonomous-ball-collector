#ifndef UWB_MANAGER_H
#define UWB_MANAGER_H

#include <Arduino.h>
#include "config.h"

struct UWBData {
  float    distance_mm;
  float    angle_deg;
  int      pdoa_raw;
  float    x_cm;
  float    y_cm;
  bool     valid;
  unsigned long timestamp;
};

class UWBManager {
public:
  void begin();

  bool sendCommand(const char* cmd, char* response = nullptr, size_t respLen = 0,
                   unsigned long timeoutMs = UWB_TIMEOUT_MS);

  bool setupAnchor();
  bool update();

  UWBData getData() const;
  bool hasValidData(unsigned long maxAgeMs = 500) const;

  float getFilteredDistance() const;
  float getFilteredAngle() const;
  float getFilteredX() const;
  float getFilteredY() const;

  void enablePassthrough();
  void disablePassthrough();
  bool isPassthrough() const;
  void handlePassthrough();

  void printStatus();
  void debugRawData();

private:
  UWBData  _data;
  UWBData  _rawData;
  bool     _passthrough;
  bool     _initialized;
  unsigned long _lastRead;

  float _filteredDistance;
  float _filteredAngle;
  float _filteredX;
  float _filteredY;
  bool  _filterInitialized;

  static const size_t RX_BUF_SIZE = 512;
  char    _rxBuf[RX_BUF_SIZE];
  size_t  _rxPos;

  bool parseTWRJson(const char* line);
  bool parseDistanceCmd(const char* line);
  bool processLine(const char* line);

  void applyFilter();
  void flushInput();

  bool jsonGetInt(const char* json, const char* key, int* out);
};

#endif // UWB_MANAGER_H
