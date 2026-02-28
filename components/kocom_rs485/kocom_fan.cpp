#include "kocom_fan.h"
#include "kocom_rs485.h"
#include "esphome/core/log.h"

namespace esphome {
namespace kocom_rs485 {

static const char *const TAG = "kocom_fan";

void KocomFan::setup() {
  if (parent_ != nullptr)
    parent_->register_fan(this);
}

fan::FanTraits KocomFan::get_traits() {
  auto traits = fan::FanTraits();
  traits.set_supported_speed_count(3);
  return traits;
}

void KocomFan::control(const fan::FanCall &call) {
  if (parent_ == nullptr) return;

  bool on = this->state;
  uint8_t spd = this->speed;

  if (call.get_state().has_value())
    on = *call.get_state();
  if (call.get_speed().has_value())
    spd = *call.get_speed();

  // Turning on with no speed defaults to low
  if (on && spd == 0)
    spd = 1;

  parent_->set_fan(on, spd);

  this->state = on;
  this->speed = on ? spd : 0;
  this->publish_state();
}

}  // namespace kocom_rs485
}  // namespace esphome
