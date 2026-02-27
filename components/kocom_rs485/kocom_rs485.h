#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/light/light_state.h"
#include <vector>
#include <deque>

namespace esphome {
namespace kocom_rs485 {

// Device codes
static const uint8_t DEV_WALLPAD    = 0x01;
static const uint8_t DEV_LIGHT      = 0x0E;
static const uint8_t DEV_THERMOSTAT = 0x36;
static const uint8_t DEV_PLUG       = 0x3B;

// Commands
static const uint8_t CMD_STATE = 0x00;
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

// Rooms
static const uint8_t NUM_ROOMS = 5;
static const uint8_t MAX_SUB   = 8;
static const uint8_t NUM_THERMO_ROOMS = 4;

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

  // Light counts per room, plug counts per room
  uint8_t light_count_[NUM_ROOMS]{};
  uint8_t plug_count_[NUM_ROOMS]{};

  // Light entity pointers for pushing state back to HA
  light::LightState *light_entities_[NUM_ROOMS][MAX_SUB]{};
  bool suppress_write_ = false;

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
  void push_light_states(uint8_t room);
  void send_query(uint8_t device_code, uint8_t room_code);
  void send_command(uint8_t device_code, uint8_t room_code, uint8_t cmd, const uint8_t *value);
  void enqueue_packet(uint8_t device_code, uint8_t room_code, uint8_t cmd, const uint8_t *value);
};

}  // namespace kocom_rs485
}  // namespace esphome
