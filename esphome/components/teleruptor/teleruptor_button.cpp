#include "teleruptor_button.h"

#include "esphome/core/log.h"

namespace esphome {
namespace teleruptor {

static const char *const TAG = "teleruptor.button";

void TeleruptorButton::dump_config() {
  LOG_BUTTON("", "Teleruptor Button", this);
}

void TeleruptorButton::press_action() {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Commande ignoree: aucun parent teleruptor");
    return;
  }

  this->parent_->request_toggle_pulse();
}

}  // namespace teleruptor
}  // namespace esphome
