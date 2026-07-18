#include "teleruptor_light.h"

#include "esphome/core/log.h"

namespace esphome {
namespace teleruptor {

static const char *const TAG = "teleruptor.light";

light::LightTraits TeleruptorLightOutput::get_traits() {
  auto traits = light::LightTraits();
  traits.set_supported_color_modes({light::ColorMode::ON_OFF});
  return traits;
}

void TeleruptorLightOutput::setup_state(light::LightState *state) {
  this->light_state_ = state;
  this->ignore_first_write_ = true;

  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Aucun teleruptor_id configure");
    return;
  }

  this->parent_->add_state_callback([this](bool state) { this->publish_from_parent_(state); });
}

void TeleruptorLightOutput::write_state(light::LightState *state) {
  bool target_state = false;
  state->current_values_as_binary(&target_state);

  if (this->ignore_first_write_) {
    this->ignore_first_write_ = false;
    ESP_LOGD(TAG, "Initial light restore ignored; physical feedback is source of truth");
    if (this->parent_ != nullptr && this->parent_->has_state()) {
      this->publish_from_parent_(this->parent_->state());
    }
    return;
  }

  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Commande ignoree: aucun parent teleruptor");
    return;
  }

  this->parent_->request_state(target_state);
}

void TeleruptorLightOutput::publish_from_parent_(bool state) {
  if (this->light_state_ == nullptr) {
    return;
  }

  light::LightColorValues values;
  values.set_color_mode(light::ColorMode::ON_OFF);
  values.set_state(state);
  values.set_brightness(1.0f);

  this->light_state_->current_values = values;
  this->light_state_->remote_values = values;
  this->light_state_->publish_state();
}

}  // namespace teleruptor
}  // namespace esphome
