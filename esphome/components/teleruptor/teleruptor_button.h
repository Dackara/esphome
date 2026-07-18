#pragma once

#include "teleruptor.h"
#include "esphome/components/button/button.h"
#include "esphome/core/component.h"

namespace esphome {
namespace teleruptor {

class TeleruptorButton : public button::Button, public Component {
 public:
  void set_parent(Teleruptor *parent) { this->parent_ = parent; }

  void dump_config() override;

 protected:
  Teleruptor *parent_{nullptr};

  void press_action() override;
};

}  // namespace teleruptor
}  // namespace esphome
