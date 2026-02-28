#include "kocom_rs485.h"
#include "esphome/core/log.h"

namespace esphome {
namespace kocom_rs485 {

static const char *const TAG = "kocom_rs485";

// --- Configuration ---

void KocomRS485::set_light_count(uint8_t room, uint8_t count) {
  if (room < NUM_ROOMS) light_count_[room] = count;
}

void KocomRS485::set_plug_count(uint8_t room, uint8_t count) {
  if (room < NUM_ROOMS) plug_count_[room] = count;
}

void KocomRS485::register_light(uint8_t room, uint8_t sub, light::LightState *light) {
  if (room < NUM_ROOMS && sub < MAX_SUB)
    light_entities_[room][sub] = light;
}

void KocomRS485::register_climate(uint8_t room, climate::Climate *c) {
  if (room < NUM_THERMO_ROOMS)
    climate_entities_[room] = c;
}

void KocomRS485::register_fan(fan::Fan *f) { fan_entity_ = f; }

// --- Component lifecycle ---

float KocomRS485::get_setup_priority() const { return setup_priority::DATA; }

void KocomRS485::setup() {
  ESP_LOGI(TAG, "KocomRS485 setup");
  for (int i = 0; i < NUM_THERMO_ROOMS; i++) {
    thermo_state_[i].mode = 0;
    thermo_state_[i].target_temp = 22;
    thermo_state_[i].current_temp = 0;
  }
  last_send_ms_ = 0;
  last_poll_ms_ = 0;
  poll_index_ = 0;
  last_poll_item_ms_ = 0;
  first_poll_done_ = false;
}

void KocomRS485::loop() {
  read_uart();

  uint32_t now = millis();
  if (!cmd_queue_.empty() && (now - last_send_ms_ >= MIN_SEND_INTERVAL_MS)) {
    auto &pkt = cmd_queue_.front();
    write_array(pkt.data(), pkt.size());
    flush();
    last_send_ms_ = now;
    cmd_queue_.pop_front();
  }

  if (!first_poll_done_ || (now - last_poll_ms_ >= POLL_INTERVAL_S * 1000)) {
    run_poll(now);
  }

  // Auto-clear elevator arrived state after timeout
  if (elevator_arrived_ && (now - elevator_arrived_ms_ >= ELEVATOR_ARRIVED_TIMEOUT_MS)) {
    elevator_arrived_ = false;
    if (elevator_arrived_sensor_ != nullptr)
      elevator_arrived_sensor_->publish_state(false);
  }
  // Auto-clear elevator called state after timeout (no arrival received)
  if (elevator_called_ && (now - elevator_called_ms_ >= ELEVATOR_CALLED_TIMEOUT_MS)) {
    elevator_called_ = false;
  }
}

void KocomRS485::dump_config() {
  ESP_LOGCONFIG(TAG, "KocomRS485:");
  for (uint8_t r = 0; r < NUM_ROOMS; r++) {
    if (light_count_[r] > 0 || plug_count_[r] > 0) {
      ESP_LOGCONFIG(TAG, "  Room %d: lights=%d, plugs=%d", r, light_count_[r], plug_count_[r]);
    }
  }
}

// --- Public API ---

bool KocomRS485::get_light(uint8_t room, uint8_t sub) {
  if (room >= NUM_ROOMS || sub >= MAX_SUB) return false;
  return light_state_[room][sub];
}

void KocomRS485::set_light(uint8_t room, uint8_t sub, bool on) {
  if (!first_poll_done_) return;
  if (room >= NUM_ROOMS || sub >= MAX_SUB) return;
  uint8_t value[8]{};
  for (uint8_t i = 0; i < light_count_[room] && i < MAX_SUB; i++) {
    value[i] = (i == sub) ? (on ? 0xFF : 0x00)
                           : (light_state_[room][i] ? 0xFF : 0x00);
  }
  uint8_t room_code = light_plug_room_code(room);
  send_command(DEV_LIGHT, room_code, CMD_STATE, value);
}

bool KocomRS485::get_plug(uint8_t room, uint8_t sub) {
  if (room >= NUM_ROOMS || sub >= MAX_SUB) return false;
  return plug_state_[room][sub];
}

void KocomRS485::set_plug(uint8_t room, uint8_t sub, bool on) {
  if (!first_poll_done_) return;
  if (room >= NUM_ROOMS || sub >= MAX_SUB) return;
  uint8_t value[8]{};
  for (uint8_t i = 0; i < plug_count_[room] && i < MAX_SUB; i++) {
    value[i] = (i == sub) ? (on ? 0xFF : 0x00)
                           : (plug_state_[room][i] ? 0xFF : 0x00);
  }
  uint8_t room_code = light_plug_room_code(room);
  send_command(DEV_PLUG, room_code, CMD_STATE, value);
}

uint8_t KocomRS485::get_thermo_mode(uint8_t room) {
  if (room >= NUM_THERMO_ROOMS) return 0;
  return thermo_state_[room].mode;
}

uint8_t KocomRS485::get_thermo_target(uint8_t room) {
  if (room >= NUM_THERMO_ROOMS) return 22;
  return thermo_state_[room].target_temp;
}

uint8_t KocomRS485::get_thermo_current(uint8_t room) {
  if (room >= NUM_THERMO_ROOMS) return 0;
  return thermo_state_[room].current_temp;
}

void KocomRS485::set_thermostat(uint8_t room, uint8_t mode, uint8_t target_temp) {
  if (!first_poll_done_) return;
  if (room >= NUM_THERMO_ROOMS) return;
  uint8_t value[8]{};
  if (mode == 1) {        // heat
    value[0] = 0x11; value[1] = 0x00;
  } else if (mode == 0) { // off
    value[0] = 0x01; value[1] = 0x00;
  } else {                // fan_only (away)
    value[0] = 0x11; value[1] = 0x01;
  }
  value[2] = target_temp;
  uint8_t room_code = thermo_room_code(room);
  send_command(DEV_THERMOSTAT, room_code, CMD_STATE, value);
}

void KocomRS485::set_fan(bool on, uint8_t speed) {
  if (!first_poll_done_) return;
  uint8_t value[8]{};
  if (on) {
    value[0] = 0x11;
    value[1] = 0x01;
    static const uint8_t speed_map[] = {0x00, 0x40, 0x80, 0xC0};
    value[2] = (speed >= 1 && speed <= 3) ? speed_map[speed] : 0x40;
  }
  send_command(DEV_FAN, 0x00, CMD_STATE, value);
}

// --- Room code mappings ---

uint8_t KocomRS485::light_plug_room_code(uint8_t room_idx) {
  static const uint8_t codes[] = {0x00, 0x01, 0x03, 0x02, 0x04};
  return (room_idx < NUM_ROOMS) ? codes[room_idx] : 0x00;
}

uint8_t KocomRS485::light_plug_room_index(uint8_t code) {
  switch (code) {
    case 0x00: return 0;
    case 0x01: return 1;
    case 0x03: return 2;
    case 0x02: return 3;
    case 0x04: return 4;
    default: return 0xFF;
  }
}

uint8_t KocomRS485::thermo_room_code(uint8_t room_idx) {
  static const uint8_t codes[] = {0x00, 0x01, 0x02, 0x03};
  return (room_idx < NUM_THERMO_ROOMS) ? codes[room_idx] : 0x00;
}

uint8_t KocomRS485::thermo_room_index(uint8_t code) {
  if (code < NUM_THERMO_ROOMS) return code;
  return 0xFF;
}

// --- Polling ---

std::vector<KocomRS485::PollItem> KocomRS485::get_poll_list() {
  std::vector<PollItem> list;
  for (uint8_t r = 0; r < NUM_ROOMS; r++) {
    if (light_count_[r] > 0)
      list.push_back({DEV_LIGHT, light_plug_room_code(r)});
  }
  for (uint8_t r = 0; r < NUM_ROOMS; r++) {
    if (plug_count_[r] > 0)
      list.push_back({DEV_PLUG, light_plug_room_code(r)});
  }
  for (uint8_t r = 0; r < NUM_THERMO_ROOMS; r++) {
    list.push_back({DEV_THERMOSTAT, thermo_room_code(r)});
  }
  if (fan_entity_ != nullptr)
    list.push_back({DEV_FAN, 0x00});
  return list;
}

void KocomRS485::run_poll(uint32_t now) {
  auto list = get_poll_list();
  if (list.empty()) return;

  if (poll_index_ >= list.size()) {
    poll_index_ = 0;
    last_poll_ms_ = now;
    if (!first_poll_done_) {
      first_poll_done_ = true;
      ESP_LOGI(TAG, "Initial poll complete, ready for commands");
    }
    return;
  }

  if (poll_index_ == 0 && last_poll_item_ms_ == 0) {
    last_poll_item_ms_ = now;
  }

  if (now - last_poll_item_ms_ >= POLL_SPACING_MS) {
    auto &item = list[poll_index_];
    send_query(item.device, item.room_code);
    last_poll_item_ms_ = now;
    poll_index_++;
  }
}

// --- Unknown (non-Kocom) byte buffer ---

void KocomRS485::unknown_push(uint8_t byte) {
  if (unknown_pos_ < UNKNOWN_BUF_SIZE) {
    unknown_buf_[unknown_pos_++] = byte;
  } else {
    unknown_flush();
    unknown_buf_[0] = byte;
    unknown_pos_ = 1;
  }
}

void KocomRS485::unknown_flush() {
  if (unknown_pos_ == 0) return;
  // Format hex string: up to 256 bytes = 768 chars (2 hex + space each)
  char hex[UNKNOWN_BUF_SIZE * 3 + 1];
  size_t off = 0;
  for (size_t i = 0; i < unknown_pos_; i++) {
    off += snprintf(hex + off, sizeof(hex) - off, "%02X ", unknown_buf_[i]);
  }
  ESP_LOGW(TAG, "UNKNOWN[%d]: %s", unknown_pos_, hex);
  unknown_pos_ = 0;
}

// --- UART read state machine ---

void KocomRS485::read_uart() {
  uint8_t byte;
  int avail = available();
  if (avail > 0 && !logged_avail_) {
    ESP_LOGI(TAG, "UART available: %d bytes", avail);
    logged_avail_ = true;
  }
  while (available()) {
    read_byte(&byte);
    switch (rx_state_) {
      case WAIT_SYNC0:
        if (byte == PKT_SYNC0) {
          unknown_flush();  // flush any accumulated non-Kocom data
          rx_buf_[0] = byte;
          rx_state_ = WAIT_SYNC1;
        } else {
          unknown_push(byte);
        }
        break;
      case WAIT_SYNC1:
        if (byte == PKT_SYNC1) {
          rx_buf_[1] = byte;
          rx_pos_ = 2;
          rx_state_ = READ_BODY;
        } else if (byte == PKT_SYNC0) {
          // Previous 0xAA was not a sync - push it as unknown
          unknown_push(PKT_SYNC0);
          // Stay in WAIT_SYNC1 with this new 0xAA
        } else {
          // Neither 0x55 nor 0xAA - previous 0xAA + this byte are unknown
          unknown_push(PKT_SYNC0);
          unknown_push(byte);
          rx_state_ = WAIT_SYNC0;
        }
        break;
      case READ_BODY:
        rx_buf_[rx_pos_++] = byte;
        if (rx_pos_ >= PKT_LEN) {
          process_packet(rx_buf_);
          rx_state_ = WAIT_SYNC0;
        }
        break;
    }
  }
  // Flush any remaining unknown bytes when no more data available
  unknown_flush();
}

// --- Packet processing ---

void KocomRS485::process_packet(const uint8_t *pkt) {
  ESP_LOGI(TAG, "PKT: %02X%02X %02X%02X%02X %02X%02X %02X%02X %02X %02X%02X%02X%02X%02X%02X%02X%02X %02X %02X%02X",
           pkt[0],pkt[1],pkt[2],pkt[3],pkt[4],pkt[5],pkt[6],pkt[7],pkt[8],pkt[9],
           pkt[10],pkt[11],pkt[12],pkt[13],pkt[14],pkt[15],pkt[16],pkt[17],pkt[18],pkt[19],pkt[20]);

  if (pkt[19] != PKT_TAIL || pkt[20] != PKT_TAIL) {
    ESP_LOGW(TAG, "Bad tail: %02X %02X", pkt[19], pkt[20]);
    return;
  }

  uint16_t sum = 0;
  for (int i = 2; i < 18; i++) sum += pkt[i];
  uint8_t calc_chk = (uint8_t)(sum & 0xFF);
  if (calc_chk != pkt[18]) {
    ESP_LOGW(TAG, "Bad checksum: calc=%02X pkt=%02X", calc_chk, pkt[18]);
    return;
  }

  uint8_t type_hi = pkt[2];
  uint8_t type_lo = pkt[3] >> 4;

  if (type_hi != 0x30) {
    ESP_LOGW(TAG, "UNHANDLED type_hi=%02X (expected 0x30)", type_hi);
    return;
  }

  bool is_send = (type_lo == 0x0B);
  bool is_ack  = (type_lo == 0x0D);
  if (!is_send && !is_ack) {
    ESP_LOGW(TAG, "UNHANDLED type_lo=%01X (expected 0xB or 0xD)", type_lo);
    return;
  }

  uint8_t dev_a  = pkt[5];
  uint8_t room_a = pkt[6];
  uint8_t dev_b  = pkt[7];
  uint8_t room_b = pkt[8];
  uint8_t cmd    = pkt[9];
  const uint8_t *value = &pkt[10];

  uint8_t src_dev, src_room, dst_dev, dst_room;
  if (is_send) {
    dst_dev = dev_a; dst_room = room_a;
    src_dev = dev_b; src_room = room_b;
  } else {
    src_dev = dev_a; src_room = room_a;
    dst_dev = dev_b; dst_room = room_b;
  }

  ESP_LOGD(TAG, "RX %s src=%02X/%02X dst=%02X/%02X cmd=%02X val=%02X%02X%02X%02X%02X%02X%02X%02X",
           is_send ? "SND" : "ACK",
           src_dev, src_room, dst_dev, dst_room, cmd,
           value[0], value[1], value[2], value[3],
           value[4], value[5], value[6], value[7]);

  if (cmd != CMD_STATE) {
    ESP_LOGW(TAG, "UNHANDLED cmd=%02X (not STATE) src=%02X/%02X dst=%02X/%02X val=%02X%02X%02X%02X%02X%02X%02X%02X",
             cmd, src_dev, src_room, dst_dev, dst_room,
             value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7]);
    return;
  }

  // Determine the non-wallpad device
  uint8_t device_code, device_room;
  if (src_dev == DEV_WALLPAD) {
    device_code = dst_dev;
    device_room = dst_room;
  } else if (dst_dev == DEV_WALLPAD) {
    device_code = src_dev;
    device_room = src_room;
  } else {
    ESP_LOGW(TAG, "UNHANDLED no wallpad: src=%02X/%02X dst=%02X/%02X cmd=%02X val=%02X%02X%02X%02X%02X%02X%02X%02X",
             src_dev, src_room, dst_dev, dst_room, cmd,
             value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7]);
    return;
  }

  // Elevator uses cmd=01 (ON) and has special addressing — handle before generic cmd check
  if (device_code == DEV_ELEVATOR) {
    handle_elevator(cmd, value);
    return;
  }

  if (cmd != CMD_STATE) {
    ESP_LOGW(TAG, "UNHANDLED cmd=%02X (not STATE) src=%02X/%02X dst=%02X/%02X val=%02X%02X%02X%02X%02X%02X%02X%02X",
             cmd, src_dev, src_room, dst_dev, dst_room,
             value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7]);
    return;
  }

  if (device_code == DEV_LIGHT) {
    uint8_t ri = light_plug_room_index(device_room);
    if (ri == 0xFF || ri >= NUM_ROOMS) return;
    for (uint8_t i = 0; i < light_count_[ri] && i < MAX_SUB; i++) {
      light_state_[ri][i] = (value[i] != 0x00);
    }
    push_light_states(ri);
    ESP_LOGD(TAG, "Light room %d updated", ri);
  } else if (device_code == DEV_PLUG) {
    uint8_t ri = light_plug_room_index(device_room);
    if (ri == 0xFF || ri >= NUM_ROOMS) return;
    for (uint8_t i = 0; i < plug_count_[ri] && i < MAX_SUB; i++) {
      plug_state_[ri][i] = (value[i] != 0x00);
    }
    ESP_LOGD(TAG, "Plug room %d updated", ri);
  } else if (device_code == DEV_THERMOSTAT) {
    uint8_t ri = thermo_room_index(device_room);
    if (ri == 0xFF || ri >= NUM_THERMO_ROOMS) return;
    bool heat_on = (value[0] == 0x11);
    bool away_on = (value[1] == 0x01);
    uint8_t target = value[2];
    uint8_t current = value[4];

    thermo_state_[ri].current_temp = current;
    if (heat_on && away_on) {
      thermo_state_[ri].mode = 2; // fan_only
    } else if (heat_on) {
      thermo_state_[ri].mode = 1; // heat
      thermo_state_[ri].target_temp = target;
    } else {
      thermo_state_[ri].mode = 0; // off
    }
    ESP_LOGD(TAG, "Thermo room %d: mode=%d target=%d current=%d",
             ri, thermo_state_[ri].mode, thermo_state_[ri].target_temp, thermo_state_[ri].current_temp);
    push_climate_states(ri);
  } else if (device_code == DEV_FAN) {
    fan_state_.on = (value[0] == 0x11);
    switch (value[2]) {
      case 0x40: fan_state_.speed = 1; break;
      case 0x80: fan_state_.speed = 2; break;
      case 0xC0: fan_state_.speed = 3; break;
      default:   fan_state_.speed = 0; break;
    }
    ESP_LOGD(TAG, "Fan: on=%d speed=%d", fan_state_.on, fan_state_.speed);
    push_fan_state();
  }
}

void KocomRS485::push_light_states(uint8_t room) {
  suppress_write_ = true;
  for (uint8_t i = 0; i < light_count_[room] && i < MAX_SUB; i++) {
    if (light_entities_[room][i] != nullptr) {
      bool current_on = light_entities_[room][i]->current_values.is_on();
      if (current_on != light_state_[room][i]) {
        auto call = light_entities_[room][i]->make_call();
        call.set_state(light_state_[room][i]);
        call.perform();
      }
    }
  }
  suppress_write_ = false;
}

void KocomRS485::push_climate_states(uint8_t room) {
  if (room >= NUM_THERMO_ROOMS) return;
  auto *c = climate_entities_[room];
  if (c == nullptr) return;

  auto &ts = thermo_state_[room];
  switch (ts.mode) {
    case 0: c->mode = climate::CLIMATE_MODE_OFF; break;
    case 1: c->mode = climate::CLIMATE_MODE_HEAT; break;
    case 2: c->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    default: c->mode = climate::CLIMATE_MODE_OFF; break;
  }
  c->current_temperature = ts.current_temp;
  c->target_temperature = ts.target_temp;

  if (ts.mode == 1 && ts.current_temp < ts.target_temp) {
    c->action = climate::CLIMATE_ACTION_HEATING;
  } else if (ts.mode == 1) {
    c->action = climate::CLIMATE_ACTION_IDLE;
  } else if (ts.mode == 2) {
    c->action = climate::CLIMATE_ACTION_FAN;
  } else {
    c->action = climate::CLIMATE_ACTION_OFF;
  }

  c->publish_state();
}

void KocomRS485::push_fan_state() {
  if (fan_entity_ == nullptr) return;
  fan_entity_->state = fan_state_.on;
  fan_entity_->speed = fan_state_.on ? fan_state_.speed : 0;
  fan_entity_->publish_state();
}

// --- Elevator ---

void KocomRS485::handle_elevator(uint8_t cmd, const uint8_t *value) {
  if (cmd == CMD_ON) {
    if (value[0] != 0x00) {
      // Phase 2: elevator has arrived (cmd=01, value[0]=03)
      if (!elevator_arrived_) {
        elevator_arrived_ = true;
        elevator_arrived_ms_ = millis();
        if (elevator_arrived_sensor_ != nullptr)
          elevator_arrived_sensor_->publish_state(true);
        ESP_LOGI(TAG, "Elevator arrived (val[0]=%02X)", value[0]);
      }
      elevator_called_ = false;
    } else {
      // Elevator call detected on bus (our echo or physical wallpad call)
      if (!elevator_called_) {
        elevator_called_ = true;
        elevator_called_ms_ = millis();
        ESP_LOGI(TAG, "Elevator call detected on bus");
      }
    }
  } else if (cmd == CMD_STATE && value[0] != 0x00) {
    // Phase 1: elevator approaching (cmd=00, value[0]=03)
    ESP_LOGI(TAG, "Elevator approaching (val[0]=%02X)", value[0]);
    if (!elevator_called_) {
      elevator_called_ = true;
      elevator_called_ms_ = millis();
    }
  }
}

void KocomRS485::call_elevator() {
  if (!first_poll_done_) return;

  std::vector<uint8_t> pkt(PKT_LEN);
  pkt[0] = 0xAA;
  pkt[1] = 0x55;
  pkt[2] = 0x30;
  pkt[3] = 0xBC;
  pkt[4] = 0x00;
  // Elevator uses REVERSED addressing: PosA = wallpad, PosB = elevator
  pkt[5] = DEV_WALLPAD;
  pkt[6] = 0x00;
  pkt[7] = DEV_ELEVATOR;
  pkt[8] = 0x00;
  pkt[9] = CMD_ON;
  // Value: all zeros (already zero-initialized by vector)

  uint16_t sum = 0;
  for (int i = 2; i < 18; i++) sum += pkt[i];
  pkt[18] = (uint8_t)(sum & 0xFF);
  pkt[19] = 0x0D;
  pkt[20] = 0x0D;

  elevator_called_ = true;
  elevator_called_ms_ = millis();
  cmd_queue_.push_back(std::move(pkt));
  ESP_LOGI(TAG, "Elevator call queued");
}

// --- Packet construction ---

void KocomRS485::send_query(uint8_t device_code, uint8_t room_code) {
  uint8_t value[8]{};
  enqueue_packet(device_code, room_code, CMD_QUERY, value);
}

void KocomRS485::send_command(uint8_t device_code, uint8_t room_code, uint8_t cmd, const uint8_t *value) {
  enqueue_packet(device_code, room_code, cmd, value);
}

void KocomRS485::enqueue_packet(uint8_t device_code, uint8_t room_code, uint8_t cmd, const uint8_t *value) {
  std::vector<uint8_t> pkt(PKT_LEN);
  pkt[0] = 0xAA;
  pkt[1] = 0x55;
  pkt[2] = 0x30;
  pkt[3] = 0xBC;
  pkt[4] = 0x00;
  pkt[5] = device_code;
  pkt[6] = room_code;
  pkt[7] = DEV_WALLPAD;
  pkt[8] = 0x00;
  pkt[9] = cmd;
  for (int i = 0; i < 8; i++) pkt[10 + i] = value[i];

  uint16_t sum = 0;
  for (int i = 2; i < 18; i++) sum += pkt[i];
  pkt[18] = (uint8_t)(sum & 0xFF);
  pkt[19] = 0x0D;
  pkt[20] = 0x0D;

  ESP_LOGD(TAG, "TX dev=%02X room=%02X cmd=%02X val=%02X%02X%02X%02X%02X%02X%02X%02X chk=%02X",
           device_code, room_code, cmd,
           value[0], value[1], value[2], value[3],
           value[4], value[5], value[6], value[7], pkt[18]);

  cmd_queue_.push_back(std::move(pkt));
}

}  // namespace kocom_rs485
}  // namespace esphome
