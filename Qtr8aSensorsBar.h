#ifndef QTR8A_SENSORS_BAR_H
#define QTR8A_SENSORS_BAR_H

#include <Arduino.h>
#include <vector>
#include <cstdint>

namespace Qtr8aSensorsBar {

/**
 * @brief Handles a bar of analog QTR-8A sensors on an ESP32.
 */
class QtrSensorBar {
public:
  /**
   * @brief Construct the sensor bar.
   * @param sensorPins Array of analog pin numbers (e.g. {34,35,…})
   * @param sensorCount Number of sensors
   * @param filterSize Samples for the moving-average window
   */
  QtrSensorBar(const int* sensorPins,
               uint8_t     sensorCount,
               uint8_t     filterSize = 8);

  ~QtrSensorBar();

  /**
   * @brief Calibrate each sensor’s min/max over a time window.
   * @param durationMs Milliseconds to read and update bounds.
   */
  void calibrate(uint32_t durationMs = 3000);

  /**
   * @brief Read raw 12-bit ADC values from every sensor.
   * @return vector of readings [0…4095].
   */
  std::vector<uint16_t> readRaw();

  /**
   * @brief Read normalized sensor values, scaled to 0–1000.
   * @return vector of normalized [0…1000].
   */
  std::vector<uint16_t> readNormalized();

  /**
   * @brief Get the filtered, weighted line position.
   * @return position = (Σ value_i·weight_i)/(Σ value_i)
   */
  float getPosition();

  /**
   * @brief Override the per-sensor weights.
   * @param weights Array of floats, length == sensorCount.
   */
  void setWeights(const float* weights);

private:
  void     pushFilter(uint8_t idx, uint16_t sample);
  float    filtered(uint8_t idx) const;

  const uint8_t               _sensorCount;
  const uint8_t               _filterSize;
  std::vector<uint8_t>        _pins;
  std::vector<uint16_t>       _raw;          ///< last raw ADC readings
  std::vector<uint16_t>       _norm;         ///< last normalized [0–1000]
  std::vector<uint16_t>       _minValues;    ///< calibration mins
  std::vector<uint16_t>       _maxValues;    ///< calibration maxes
  std::vector<float>          _weights;      ///< custom +/– weights

  // moving-average buffers
  std::vector<std::vector<uint16_t>> _buf;
  std::vector<uint8_t>               _bufIdx;
};

} // namespace Qtr8aSensorsBar

#endif // QTR8A_SENSORS_BAR_H
