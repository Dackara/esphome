#include "teleruptor_binary_sensor.h"

#include "esphome/core/log.h"

namespace esphome {
namespace teleruptor {

static const char *const TAG = "teleruptor.binary_sensor";

void TeleruptorBinarySensor::setup() {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Aucun teleruptor_id configure");
    return;
  }

  this->parent_->add_state_callback([this](bool state) { this->publish_state(state); });
}

void TeleruptorBinarySensor::dump_config() {
  LOG_BINARY_SENSOR("", "Teleruptor Binary Sensor", this);
}

}  // namespace teleruptor
}  // namespace esphome
