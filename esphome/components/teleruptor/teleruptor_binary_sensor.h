#pragma once

#include "teleruptor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace teleruptor {

class TeleruptorBinarySensor : public binary_sensor::BinarySensor, public Component {
 public:
  void set_parent(Teleruptor *parent) { this->parent_ = parent; }

  void setup() override;
  void dump_config() override;

 protected:
  Teleruptor *parent_{nullptr};
};

}  // namespace teleruptor
}  // namespace esphome
