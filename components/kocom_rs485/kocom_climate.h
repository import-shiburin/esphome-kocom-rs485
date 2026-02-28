#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/core/component.h"

namespace esphome {
namespace kocom_rs485 {

class KocomRS485;  // forward declaration

class KocomClimate : public climate::Climate, public Component {
 public:
  void set_parent(KocomRS485 *parent) { parent_ = parent; }
  void set_room(uint8_t room) { room_ = room; }

  void setup() override;
  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

  climate::ClimateTraits traits() override {
    auto traits = climate::ClimateTraits();
    traits.set_supports_current_temperature(true);
    traits.set_visual_min_temperature(5.0f);
    traits.set_visual_max_temperature(40.0f);
    traits.set_visual_temperature_step(1.0f);
    traits.set_supported_modes({
        climate::CLIMATE_MODE_OFF,
        climate::CLIMATE_MODE_HEAT,
        climate::CLIMATE_MODE_FAN_ONLY,
    });
    return traits;
  }

  void control(const climate::ClimateCall &call) override;

 protected:
  KocomRS485 *parent_{nullptr};
  uint8_t room_{0};
};

}  // namespace kocom_rs485
}  // namespace esphome
