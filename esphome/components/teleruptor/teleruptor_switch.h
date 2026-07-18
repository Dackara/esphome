#pragma once

#include "teleruptor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome {
namespace teleruptor {

class TeleruptorSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(Teleruptor *parent) { this->parent_ = parent; }

  void setup() override;
  void dump_config() override;
  void write_state(bool state) override;

 protected:
  Teleruptor *parent_{nullptr};
};

}  // namespace teleruptor
}  // namespace esphome
