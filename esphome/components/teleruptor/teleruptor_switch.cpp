#include "teleruptor_switch.h"

#include "esphome/core/log.h"

namespace esphome {
namespace teleruptor {

static const char *const TAG = "teleruptor.switch";

void TeleruptorSwitch::setup() {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Aucun teleruptor_id configure");
    return;
  }

  this->parent_->add_state_callback([this](bool state) { this->publish_state(state); });
}

void TeleruptorSwitch::dump_config() {
  LOG_SWITCH("", "Teleruptor Switch", this);
}

void TeleruptorSwitch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Commande ignoree: aucun parent teleruptor");
    return;
  }

  this->parent_->request_state(state);
}

}  // namespace teleruptor
}  // namespace esphome
