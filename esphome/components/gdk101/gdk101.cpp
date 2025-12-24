#include "gdk101.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gdk101 {

static const char *const TAG = "gdk101";

void GDK101Component::update() {
  uint8_t data[2];
  if (!this->read_dose_1m_(data)) {
    this->status_set_warning(LOG_STR("Failed to read dose 1m"));
    return;
  }

  if (!this->read_dose_10m_(data)) {
    this->status_set_warning(LOG_STR("Failed to read dose 10m"));
    return;
  }

  if (!this->read_status_(data)) {
    this->status_set_warning(LOG_STR("Failed to read status"));
    return;
  }

  if (!this->read_measurement_duration_(data)) {
    this->status_set_warning(LOG_STR("Failed to read measurement duration"));
    return;
  }
  this->status_clear_warning();
}

void GDK101Component::setup() {
  uint8_t data[2];
  ESP_LOGD(TAG, "Starting setup, address=0x%02X", this->address_);
  delay(300);
  if (!this->reset_sensor_(data)) {
    this->status_set_error(LOG_STR("Reset failed!"));
    this->mark_failed();
    return;
  }
  delay(50);
  if (!this->read_fw_version_(data)) {
    this->status_set_error(LOG_STR("Failed to read firmware version"));
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "Setup complete, fw=%u.%u", data[0], data[1]);
}

void GDK101Component::dump_config() {
  ESP_LOGCONFIG(TAG, "GDK101:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
  }
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Average Radaition Dose per 1 minute", this->rad_1m_sensor_);
  LOG_SENSOR("  ", "Average Radaition Dose per 10 minutes", this->rad_10m_sensor_);
  LOG_SENSOR("  ", "Status", this->status_sensor_);
  LOG_SENSOR("  ", "Measurement Duration", this->measurement_duration_sensor_);
#endif  // USE_SENSOR

#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Vibration Status", this->vibration_binary_sensor_);
#endif  // USE_BINARY_SENSOR

#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Firmware Version", this->fw_version_text_sensor_);
#endif  // USE_TEXT_SENSOR
}

float GDK101Component::get_setup_priority() const { return setup_priority::DATA; }

bool GDK101Component::read_data_(uint8_t a_register, uint8_t *data, uint8_t len) {
  ESP_LOGVV(TAG, "Reading reg 0x%02X (len=%u)", a_register, len);
  if (this->write(&a_register, 1) != i2c::ERROR_OK) {
    ESP_LOGD(TAG, "Write for reg=0x%02X failed", a_register);
    return false;
  }
  delay(2);
  if (this->read(data, len) != i2c::ERROR_OK) {
    ESP_LOGD(TAG, "Read for reg=0x%02X failed after write", a_register);
    return false;
  }
  ESP_LOGVV(TAG, "Read ok reg=0x%02X data[0]=0x%02X data[1]=0x%02X", a_register, data[0],
            len > 1 ? data[1] : 0);
  return true;
}

bool GDK101Component::reset_sensor_(uint8_t *data) {
  ESP_LOGD(TAG, "Issuing reset sequence");
  if (this->read_data_(GDK101_REG_RESET, data, 1)) {
    ESP_LOGD(TAG, "Reset read command acknowledged");
    return true;
  }

  const uint8_t reset_cmd = GDK101_REG_RESET;
  if (this->write(&reset_cmd, 1) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Reset command failed!");
    return false;
  }
  delay(50);
  ESP_LOGD(TAG, "Reset write command sent");
  return true;
}

bool GDK101Component::read_dose_1m_(uint8_t *data) {
#ifdef USE_SENSOR
  if (this->rad_1m_sensor_ != nullptr) {
    if (!this->read_data_(GDK101_REG_READ_1MIN_AVG, data, 2)) {
      ESP_LOGE(TAG, "Updating GDK101 failed!");
      return false;
    }

    const float dose = data[0] + (data[1] / 100.0f);

    this->rad_1m_sensor_->publish_state(dose);
  }
#endif  // USE_SENSOR
  return true;
}

bool GDK101Component::read_dose_10m_(uint8_t *data) {
#ifdef USE_SENSOR
  if (this->rad_10m_sensor_ != nullptr) {
    if (!this->read_data_(GDK101_REG_READ_10MIN_AVG, data, 2)) {
      ESP_LOGE(TAG, "Updating GDK101 failed!");
      return false;
    }

    const float dose = data[0] + (data[1] / 100.0f);

    this->rad_10m_sensor_->publish_state(dose);
  }
#endif  // USE_SENSOR
  return true;
}

bool GDK101Component::read_status_(uint8_t *data) {
  if (!this->read_data_(GDK101_REG_READ_STATUS, data, 2)) {
    ESP_LOGE(TAG, "Updating GDK101 failed!");
    return false;
  }

#ifdef USE_SENSOR
  if (this->status_sensor_ != nullptr) {
    this->status_sensor_->publish_state(data[0]);
  }
#endif  // USE_SENSOR

#ifdef USE_BINARY_SENSOR
  if (this->vibration_binary_sensor_ != nullptr) {
    this->vibration_binary_sensor_->publish_state(data[1]);
  }
#endif  // USE_BINARY_SENSOR

  return true;
}

bool GDK101Component::read_fw_version_(uint8_t *data) {
#ifdef USE_TEXT_SENSOR
  if (this->fw_version_text_sensor_ != nullptr) {
    if (!this->read_data_(GDK101_REG_READ_FIRMWARE, data, 2)) {
      ESP_LOGE(TAG, "Updating GDK101 failed!");
      return false;
    }

    const std::string fw_version_str = str_sprintf("%d.%d", data[0], data[1]);

    this->fw_version_text_sensor_->publish_state(fw_version_str);
  }
#endif  // USE_TEXT_SENSOR
  return true;
}

bool GDK101Component::read_measurement_duration_(uint8_t *data) {
#ifdef USE_SENSOR
  if (this->measurement_duration_sensor_ != nullptr) {
    if (!this->read_data_(GDK101_REG_READ_MEASURING_TIME, data, 2)) {
      ESP_LOGE(TAG, "Updating GDK101 failed!");
      return false;
    }

    const float meas_time = (data[0] * 60) + data[1];

    this->measurement_duration_sensor_->publish_state(meas_time);
  }
#endif  // USE_SENSOR
  return true;
}

}  // namespace gdk101
}  // namespace esphome
