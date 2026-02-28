#pragma once

#include "esphome/components/fan/fan.h"
#include "esphome/core/component.h"

namespace esphome {
namespace kocom_rs485 {

class KocomRS485;  // forward declaration

class KocomFan : public fan::Fan, public Component {
 public:
  void set_parent(KocomRS485 *parent) { parent_ = parent; }

  void setup() override;
  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

  fan::FanTraits get_traits() override;
  void control(const fan::FanCall &call) override;

 protected:
  KocomRS485 *parent_{nullptr};
};

}  // namespace kocom_rs485
}  // namespace esphome
