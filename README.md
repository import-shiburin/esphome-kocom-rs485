# ESPHome Kocom RS485

ESPHome custom component for controlling Kocom wallpad devices over RS485, commonly found in Korean apartment building automation systems.

## Supported Devices

| Device | Description | Qty |
|--------|-------------|-----|
| **Light** | On/off control per room | Up to 8 lights x 5 rooms |
| **Plug** | Smart outlet on/off per room | Up to 8 plugs x 5 rooms |
| **Thermostat** | Floor heating with OFF / HEAT / FAN_ONLY modes | Up to 4 rooms |
| **Fan** | Ventilation fan with 3-speed control (low/medium/high) | 1 unit |
| **Elevator** | Call elevator + arrival detection | 1 unit |

## Hardware Setup

Connect your ESP device to the Kocom wallpad RS485 bus using an RS485 transceiver (e.g. MAX485, SP3485). The UART settings are **9600 baud, 8N1**.

```
ESP32/ESP8266          RS485 Transceiver          Kocom RS485 Bus
  TX  ──────────────►  DI                A ──────── A (+)
  RX  ◄──────────────  RO                B ──────── B (-)
  GPIO ─────────────►  DE/RE             GND ────── GND
```

## Installation

Add this repository as an external component in your ESPHome YAML:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/import-shiburin/esphome-kocom-rs485
    components: [kocom_rs485]
```

Or use a local path during development:

```yaml
external_components:
  - source:
      type: local
      path: components
```

## Configuration

### UART

```yaml
uart:
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600
```

### Main Component

```yaml
kocom_rs485:
  id: kocom
  uart_id: uart_bus  # optional if only one UART
  rooms:
    - index: 0
      light_count: 3
      light_ids:
        - living_light_1
        - living_light_2
        - living_light_3
      plug_count: 2
    - index: 1
      light_count: 2
      light_ids:
        - bedroom_light_1
        - bedroom_light_2
      plug_count: 1
  elevator_arrived:
    name: "Elevator Arrived"
```

#### Room Configuration

| Option | Type | Required | Description |
|--------|------|----------|-------------|
| `index` | int (0-4) | yes | Room index |
| `light_count` | int (0-8) | no | Number of lights in the room (default: 0) |
| `light_ids` | list | no | Light entity IDs to bind for state sync |
| `plug_count` | int (0-8) | no | Number of plugs in the room (default: 0) |

#### Elevator

| Option | Type | Required | Description |
|--------|------|----------|-------------|
| `elevator_arrived` | binary_sensor | no | Binary sensor that turns on when the elevator arrives (auto-clears after 30s) |

### Climate (Thermostat)

```yaml
climate:
  - platform: kocom_rs485
    kocom_rs485_id: kocom
    room: 0
    name: "Living Room Thermostat"

  - platform: kocom_rs485
    kocom_rs485_id: kocom
    room: 1
    name: "Bedroom Thermostat"
```

| Option | Type | Required | Description |
|--------|------|----------|-------------|
| `kocom_rs485_id` | id | yes | ID of the main kocom_rs485 component |
| `room` | int (0-3) | yes | Thermostat room index |
| `name` | string | yes | Entity name in Home Assistant |

Supported climate modes: **OFF**, **HEAT**, **FAN_ONLY**. Temperature range: 5-40 C with 1 C step.

### Fan (Ventilation)

```yaml
fan:
  - platform: kocom_rs485
    kocom_rs485_id: kocom
    name: "Ventilation Fan"
    speed_count: 3
```

| Option | Type | Required | Description |
|--------|------|----------|-------------|
| `kocom_rs485_id` | id | yes | ID of the main kocom_rs485 component |
| `name` | string | yes | Entity name in Home Assistant |
| `speed_count` | int | no | Number of speed levels (default: 3 - low/medium/high) |

### Lights

Lights are declared as standard ESPHome binary light entities and registered via `light_ids` in the room config:

```yaml
light:
  - platform: binary
    id: living_light_1
    name: "Living Room Light 1"
    output: living_light_1_out

output:
  - platform: template
    id: living_light_1_out
    type: binary
    write_action:
      - lambda: |-
          id(kocom).set_light(0, 0, state);
```

## Elevator Control

The elevator call function is exposed through the component's public API. Use a button or automation:

```yaml
button:
  - platform: template
    name: "Call Elevator"
    on_press:
      - lambda: |-
          id(kocom).call_elevator();
```

You can also track whether the elevator has been called:

```yaml
binary_sensor:
  - platform: template
    name: "Elevator Called"
    lambda: |-
      return id(kocom).is_elevator_called();
```

## Plug Control

Plugs are controlled via lambda since there is no dedicated plug platform:

```yaml
switch:
  - platform: template
    name: "Living Room Plug 1"
    lambda: |-
      return id(kocom).get_plug(0, 0);
    turn_on_action:
      - lambda: |-
          id(kocom).set_plug(0, 0, true);
    turn_off_action:
      - lambda: |-
          id(kocom).set_plug(0, 0, false);
```

## Behavior Notes

- The component performs an initial poll of all configured devices on startup. Commands are held until this completes.
- All devices are re-polled every **5 minutes** to keep state in sync.
- Commands are rate-limited with a minimum **100ms** gap between RS485 packets.
- Non-Kocom bytes on the bus are buffered and logged as warnings for debugging.
- Elevator arrival state auto-clears after **30 seconds**. Elevator call state auto-clears after **5 minutes** if no arrival is detected.

## Protocol Reference

The Kocom RS485 protocol uses fixed 21-byte packets:

```
Offset  Bytes  Description
  0       2    Sync: 0xAA 0x55
  2       3    Type: 0x30, direction (0xBC=send, 0xDC=ack), 0x00
  5       2    Device A: device code + room code
  7       2    Device B: device code + room code
  9       1    Command: 0x00=state, 0x01=on, 0x3A=query
 10       8    Value: device-specific payload
 18       1    Checksum: sum of bytes 2-17, masked 0xFF
 19       2    Tail: 0x0D 0x0D
```

Device codes: `0x0E` light, `0x36` thermostat, `0x3B` plug, `0x48` fan, `0x44` elevator, `0x01` wallpad.
