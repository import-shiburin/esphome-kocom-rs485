#include "kocom_climate.h"
#include "kocom_rs485.h"
#include "esphome/core/log.h"

namespace esphome {
namespace kocom_rs485 {

static const char *const TAG = "kocom_climate";

void KocomClimate::setup() {
  if (parent_ != nullptr)
    parent_->register_climate(room_, this);
}

void KocomClimate::control(const climate::ClimateCall &call) {
  if (parent_ == nullptr) return;

  uint8_t mode = 0;
  uint8_t target = this->target_temperature;

  if (call.get_mode().has_value()) {
    switch (*call.get_mode()) {
      case climate::CLIMATE_MODE_OFF: mode = 0; break;
      case climate::CLIMATE_MODE_HEAT: mode = 1; break;
      case climate::CLIMATE_MODE_FAN_ONLY: mode = 2; break;
      default: mode = 0; break;
    }
    this->mode = *call.get_mode();
  } else {
    // keep current kocom mode
    mode = parent_->get_thermo_mode(room_);
  }

  if (call.get_target_temperature().has_value()) {
    target = (uint8_t)*call.get_target_temperature();
    this->target_temperature = *call.get_target_temperature();
  }

  parent_->set_thermostat(room_, mode, target);
  this->publish_state();
}

}  // namespace kocom_rs485
}  // namespace esphome
