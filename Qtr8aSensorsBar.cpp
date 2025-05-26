#include "Qtr8aSensorsBar.h"

namespace Qtr8aSensorsBar {

QtrSensorBar::QtrSensorBar(const int* sensorPins,
                           uint8_t    sensorCount,
                           uint8_t    filterSize)
  : _sensorCount(sensorCount),
    _filterSize(filterSize),
    _pins(sensorPins, sensorPins + sensorCount),
    _raw(sensorCount, 0),
    _norm(sensorCount, 0),
    _minValues(sensorCount, 4095),
    _maxValues(sensorCount, 0),
    _weights(sensorCount, 0.0f),
    _buf(sensorCount, std::vector<uint16_t>(filterSize, 0)),
    _bufIdx(sensorCount, 0)
{
  // Default symmetric weights centered around zero
  float center = (sensorCount - 1) / 2.0f;
  for (uint8_t i = 0; i < _sensorCount; ++i) {
    _weights[i] = static_cast<float>(i) - center;
  }
  analogReadResolution(12); // ensure 12-bit ADC on ESP32
}

QtrSensorBar::~QtrSensorBar() {}

void QtrSensorBar::calibrate(uint32_t durationMs) {
  uint32_t start = millis();
  // reset bounds
  for (uint8_t i = 0; i < _sensorCount; ++i) {
    _minValues[i] = 4095;
    _maxValues[i] = 0;
  }
  // sample until timeout
  while (millis() - start < durationMs) {
    readRaw(); // updates _raw and filters
    for (uint8_t i = 0; i < _sensorCount; ++i) {
      uint16_t v = _raw[i];
      if (v < _minValues[i]) _minValues[i] = v;
      if (v > _maxValues[i]) _maxValues[i] = v;
    }
    delay(5);
  }
}

std::vector<uint16_t> QtrSensorBar::readRaw() {
  for (uint8_t i = 0; i < _sensorCount; ++i) {
    uint16_t v = analogRead(_pins[i]);
    _raw[i] = v;
    pushFilter(i, v);
  }
  return _raw;
}

void QtrSensorBar::pushFilter(uint8_t idx, uint16_t sample) {
  _buf[idx][_bufIdx[idx]] = sample;
  _bufIdx[idx] = (_bufIdx[idx] + 1) % _filterSize;
}

float QtrSensorBar::filtered(uint8_t idx) const {
  uint32_t sum = 0;
  for (auto x : _buf[idx]) sum += x;
  return static_cast<float>(sum) / _filterSize;
}

std::vector<uint16_t> QtrSensorBar::readNormalized() {
  std::vector<uint16_t> out(_sensorCount, 0);
  for (uint8_t i = 0; i < _sensorCount; ++i) {
    float    filt = filtered(i);
    uint16_t minv = _minValues[i], maxv = _maxValues[i];
    int32_t  v    = 0;
    if (maxv > minv) {
      v = static_cast<int32_t>((filt - minv) * 1000.0f / (maxv - minv));
      v = constrain(v, 0, 1000);
    }
    _norm[i] = static_cast<uint16_t>(v);
    out[i]   = _norm[i];
  }
  return out;
}

float QtrSensorBar::getPosition() {
  auto norm = readNormalized();
  float wSum = 0.0f, vSum = 0.0f;
  for (uint8_t i = 0; i < _sensorCount; ++i) {
    wSum += static_cast<float>(norm[i]) * _weights[i];
    vSum += norm[i];
  }
  return (vSum > 0.0f) ? (wSum / vSum) : 0.0f;
}

void QtrSensorBar::setWeights(const float* weights) {
  for (uint8_t i = 0; i < _sensorCount; ++i) {
    _weights[i] = weights[i];
  }
}

} // namespace Qtr8aSensorsBar
