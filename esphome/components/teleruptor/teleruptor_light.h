#pragma once

#include "teleruptor.h"
#include "esphome/components/light/light_output.h"

namespace esphome {
namespace teleruptor {

class TeleruptorLightOutput : public light::LightOutput {
 public:
  void set_parent(Teleruptor *parent) { this->parent_ = parent; }

  light::LightTraits get_traits() override;
  void setup_state(light::LightState *state) override;
  void write_state(light::LightState *state) override;

 protected:
  Teleruptor *parent_{nullptr};
  light::LightState *light_state_{nullptr};
  bool ignore_first_write_{true};

  void publish_from_parent_(bool state);
};

}  // namespace teleruptor
}  // namespace esphome
