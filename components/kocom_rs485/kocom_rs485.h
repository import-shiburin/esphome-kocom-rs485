#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/fan/fan.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>
#include <deque>

namespace esphome {
namespace kocom_rs485 {

// Device codes
static const uint8_t DEV_WALLPAD    = 0x01;
static const uint8_t DEV_LIGHT      = 0x0E;
static const uint8_t DEV_THERMOSTAT = 0x36;
static const uint8_t DEV_PLUG       = 0x3B;
static const uint8_t DEV_FAN        = 0x48;
static const uint8_t DEV_ELEVATOR   = 0x44;

// Commands
static const uint8_t CMD_STATE = 0x00;
static const uint8_t CMD_ON    = 0x01;
static const uint8_t CMD_QUERY = 0x3A;

// Packet constants
static const uint8_t PKT_SYNC0 = 0xAA;
static const uint8_t PKT_SYNC1 = 0x55;
static const uint8_t PKT_TAIL  = 0x0D;
static const size_t  PKT_LEN   = 21;

// Timing
static const uint32_t MIN_SEND_INTERVAL_MS = 100;
static const uint32_t POLL_INTERVAL_S      = 300;
static const uint32_t POLL_SPACING_MS      = 800;
static const uint32_t ELEVATOR_ARRIVED_TIMEOUT_MS = 30000;
static const uint32_t ELEVATOR_CALLED_TIMEOUT_MS  = 300000;

// Rooms
static const uint8_t NUM_ROOMS = 5;
static const uint8_t MAX_SUB   = 8;
static const uint8_t NUM_THERMO_ROOMS = 4;

struct FanState {
  bool on;         // value[0] == 0x11
  uint8_t speed;   // 1=low, 2=mid, 3=high, 0=off
};

struct ThermoState {
  uint8_t mode;        // 0=off, 1=heat, 2=fan_only(away)
  uint8_t target_temp;
  uint8_t current_temp;
};

class KocomRS485 : public Component, public uart::UARTDevice {
 public:
  // State arrays: [room][sub_device], sub 0..7
  bool light_state_[NUM_ROOMS][MAX_SUB]{};
  bool plug_state_[NUM_ROOMS][MAX_SUB]{};
  ThermoState thermo_state_[NUM_THERMO_ROOMS]{};
  FanState fan_state_{};

  // Light counts per room, plug counts per room
  uint8_t light_count_[NUM_ROOMS]{};
  uint8_t plug_count_[NUM_ROOMS]{};

  // Light entity pointers for pushing state back to HA
  light::LightState *light_entities_[NUM_ROOMS][MAX_SUB]{};
  bool suppress_write_ = false;

  // Climate entity pointers for pushing state back to HA
  climate::Climate *climate_entities_[NUM_THERMO_ROOMS]{};
  void register_climate(uint8_t room, climate::Climate *c);

  // Fan entity pointer
  fan::Fan *fan_entity_{nullptr};
  void register_fan(fan::Fan *f);
  void set_fan(bool on, uint8_t speed);

  // Elevator
  binary_sensor::BinarySensor *elevator_arrived_sensor_{nullptr};
  bool elevator_called_{false};
  bool elevator_arrived_{false};
  uint32_t elevator_arrived_ms_{0};
  uint32_t elevator_called_ms_{0};

  void register_elevator_arrived(binary_sensor::BinarySensor *s) { elevator_arrived_sensor_ = s; }
  void call_elevator();
  bool is_elevator_called() { return elevator_called_; }
  void clear_elevator_called() { elevator_called_ = false; }

  void set_light_count(uint8_t room, uint8_t count);
  void set_plug_count(uint8_t room, uint8_t count);
  void register_light(uint8_t room, uint8_t sub, light::LightState *light);

  float get_setup_priority() const override;
  void setup() override;
  void loop() override;
  void dump_config() override;

  // --- Public API for YAML lambdas ---
  bool get_light(uint8_t room, uint8_t sub);
  void set_light(uint8_t room, uint8_t sub, bool on);
  bool get_plug(uint8_t room, uint8_t sub);
  void set_plug(uint8_t room, uint8_t sub, bool on);
  uint8_t get_thermo_mode(uint8_t room);
  uint8_t get_thermo_target(uint8_t room);
  uint8_t get_thermo_current(uint8_t room);
  void set_thermostat(uint8_t room, uint8_t mode, uint8_t target_temp);

 private:
  // Receive state machine
  enum RxState { WAIT_SYNC0, WAIT_SYNC1, READ_BODY };
  RxState rx_state_ = WAIT_SYNC0;
  uint8_t rx_buf_[PKT_LEN]{};
  uint8_t rx_pos_ = 0;
  bool logged_avail_ = false;

  // Buffer for non-Kocom bytes
  static const size_t UNKNOWN_BUF_SIZE = 256;
  uint8_t unknown_buf_[UNKNOWN_BUF_SIZE]{};
  size_t unknown_pos_ = 0;

  void unknown_push(uint8_t byte);
  void unknown_flush();

  // Command queue
  std::deque<std::vector<uint8_t>> cmd_queue_;
  uint32_t last_send_ms_ = 0;

  // Polling state
  uint32_t last_poll_ms_ = 0;
  uint32_t last_poll_item_ms_ = 0;
  uint8_t poll_index_ = 0;
  bool first_poll_done_ = false;

  struct PollItem {
    uint8_t device;
    uint8_t room_code;
  };

  std::vector<PollItem> get_poll_list();
  void run_poll(uint32_t now);

  static uint8_t light_plug_room_code(uint8_t room_idx);
  static uint8_t light_plug_room_index(uint8_t code);
  static uint8_t thermo_room_code(uint8_t room_idx);
  static uint8_t thermo_room_index(uint8_t code);

  void read_uart();
  void process_packet(const uint8_t *pkt);
  void handle_elevator(uint8_t cmd, const uint8_t *value);
  void push_light_states(uint8_t room);
  void push_climate_states(uint8_t room);
  void push_fan_state();
  void send_query(uint8_t device_code, uint8_t room_code);
  void send_command(uint8_t device_code, uint8_t room_code, uint8_t cmd, const uint8_t *value);
  void enqueue_packet(uint8_t device_code, uint8_t room_code, uint8_t cmd, const uint8_t *value);
};

}  // namespace kocom_rs485
}  // namespace esphome
