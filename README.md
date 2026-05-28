# OBD2 Bridge for ESP32-S3 Super Mini

An ESP32-S3 sketch that connects to a BLE ELM327-compatible OBD adapter, optionally reads GPS data from a NEO-7 module, and pushes the collected telemetry to a webhook.

The device normally works headless, but if it cannot connect to the configured Wi-Fi network during boot, it starts its own setup access point and serves a configuration portal where you can:

- enable or disable GPS,
- scan and select a Wi-Fi network,
- scan and select a BLE OBD adapter,
- configure the default OBD request header,
- enable or disable built-in PIDs,
- add custom PIDs with parser, formula, header and frame settings,
- set the webhook URL and bearer token.

This project is designed for:

- `ESP32-S3 Super Mini`
- `BLE ELM327 / Vgate / vLinker` style OBD adapters
- `NEO-7` GPS modules
- `ESP32 4MB flash` with `Huge APP (3MB app + 1MB SPIFFS)`

## Features

- BLE connection to OBD adapter
- Wi-Fi fallback setup portal
- persistent configuration stored in `Preferences` (NVS)
- built-in telemetry set with simple enable/disable control
- custom PID support from the web panel
- parser selection for custom PIDs
- GPS coordinates included in webhook payload
- shared JSON payload with built-in and custom values

## Hardware

### Main board

- `ESP32-S3 Super Mini`

### GPS

- `u-blox NEO-7` module or compatible breakout

### OBD adapter

- BLE ELM327-compatible device
- the current sketch expects the common service/characteristics used by many BLE OBD dongles:
  - service: `18F0`
  - write characteristic: `2AF1`
  - notify characteristic: `2AF0`

## GPS wiring for ESP32-S3 Super Mini

The sketch uses:

- `GPIO5` as `UART1 RX`
- `GPIO6` as `UART1 TX`
- GPS baud rate: `9600`

That means the practical GPS wiring is:

| NEO-7 pin | ESP32-S3 Super Mini pin | Notes |
|---|---|---|
| `TX` | `GPIO5` | Required. GPS sends NMEA data to ESP32. |
| `RX` | `GPIO6` | Optional. Not required for normal NMEA receive-only operation. |
| `GND` | `GND` | Required. |
| `VCC` | `3V3` or module-specific `5V` input | Check your exact NEO-7 breakout board. |

### Important power note

NEO-7 breakout boards vary:

- some are `3.3V-only`,
- some include a regulator and accept `5V` on `VCC`,
- the ESP32-S3 logic is `3.3V`.

Safest rule:

- if your NEO-7 module supports `3.3V` power, use `3V3`,
- if your module is labeled for `5V` input, confirm its logic level behavior before wiring.

### Minimal GPS wiring

If you only want standard location reading, you can usually connect just:

- `GPS TX -> ESP GPIO5`
- `GPS GND -> ESP GND`
- `GPS VCC -> power`

The `GPS RX -> ESP GPIO6` line is optional for this sketch.

## Arduino IDE libraries

To compile this sketch in Arduino IDE, install the `ESP32 by Espressif Systems` board package from Boards Manager.

Tested local environment:

- `ESP32 by Espressif Systems` version `3.3.8`

The following libraries are required from Library Manager:

| Library Manager name | Tested version | Used for |
|---|---|---|
| `NimBLE-Arduino` | `2.5.0` | BLE connection to the OBD adapter |
| `TinyGPSPlus` | `1.0.3` | NMEA GPS parsing |
| `ArduinoJson` | `7.4.3` | webhook payload and internal JSON handling |

## Flash / board settings

Recommended Arduino IDE settings:

- Board: `ESP32S3 Dev Module` or your vendor's `ESP32-S3 Super Mini` profile
- Flash size: `4MB`
- Partition scheme: `Huge APP (3MB No OTA / 1MB SPIFFS)`


## First boot and setup portal

On boot, the ESP32 tries to connect to the saved Wi-Fi network.

Current default timing in the sketch:

- startup Wi-Fi wait: `60 seconds`
- background retry while portal is active: every `15 seconds`
- normal reconnect interval after boot: every `15 seconds`

If Wi-Fi connection fails, it starts a setup AP:

- SSID pattern: `OBD2-Setup-XXXX`
- password: `9876543210`
- portal address: usually `http://192.168.4.1/`

If your phone or laptop does not open the portal automatically, open `192.168.4.1` manually.

### Important behavior with slow car hotspots

The setup portal is not a dead end.

If the captive portal starts because the car Wi-Fi hotspot is still booting, the ESP32 will continue trying to connect to the saved Wi-Fi in the background.

When the saved Wi-Fi finally appears and the ESP32 connects successfully:

- the setup portal is stopped automatically,
- the temporary AP is turned off,
- the device continues in normal telemetry mode.

This is especially useful for in-car Wi-Fi systems that appear `30-60 seconds` after power-up.

### Tuning Wi-Fi timing

If your car hotspot starts slower or faster, you can tune these constants near the top of the sketch:

- `kWifiStartupConnectTimeoutMs`
  Time the ESP32 waits on boot before it gives up and starts the setup portal.
- `kWifiPortalRetryIntervalMs`
  How often the ESP32 retries the saved Wi-Fi while the setup portal is already running.
- `kWifiReconnectIntervalMs`
  Normal reconnect interval used later during runtime if Wi-Fi drops after the device has already started working.

For the typical in-car hotspot case, the current defaults are a good starting point:

- `60000` ms startup wait
- `15000` ms portal retry interval
- `15000` ms normal reconnect interval

## Configuration portal

The configuration portal is split into a few sections.

### 1. Wi-Fi & GPS

#### Wi-Fi SSID

The Wi-Fi network the ESP32 should connect to during normal operation.

#### Wi-Fi password

Password for the selected Wi-Fi network.

#### Default OBD request header

Default CAN header used for OBD requests when a PID does not define its own custom header.

Typical example:

- `7E0` for the main engine ECU

If a custom PID leaves its `Request header` field empty, this default header is used.

#### Run slow PIDs every N loops

Controls how often `Slow` PIDs are polled.

The sketch main loop runs roughly every `20 seconds`, so:

- `1` means every loop, about every `20s`
- `2` means about every `40s`
- `3` means about every `60s`
- `6` means about every `120s`

#### Enable GPS reading

Turns GPS reading on or off.

When enabled:

- the ESP32 listens for NMEA data on `GPIO5`,
- GPS coordinates are added to the webhook payload when a valid fix exists,
- `gps_sats`, `latitude`, and `longitude` are reported.

When disabled:

- GPS is ignored,
- no location fields are added.

### 2. Bluetooth & Webhook

#### BLE address

MAC address of the BLE OBD adapter.
You can type it manually or use the BLE scan results from the portal.

#### BLE display name

Human-readable label for the selected BLE device.
This is informational only and helps identify the adapter in the portal.

#### Webhook URL

Destination URL that receives the JSON payload.

This can be:

- `https://...`
- `http://...`

#### Bearer token

Optional bearer token added to the request as:

```http
Authorization: Bearer <token>
```

If left empty, the request is sent without the `Authorization` header.
To obtain your unique bearer token, you need to create a long-lasting token in your Home Assistant account settings.

## Built-in PIDs

Built-in PIDs are fixed in code and shown in the portal as defaults.

You can:
- enable them,
- disable them.
You cannot edit their parser, formula or command from the portal.

### Built-in PID list

| Label | Service | PID | Parser | Schedule | JSON key |
|---|---|---:|---|---|---|
| Vehicle VIN | `09` | `02` | `VIN` | `Once per session` | `vin` |
| Vehicle Speed | `01` | `0D` | `Formula: A` | `Fast` | `speed` |
| Odometer | `22` | `10E0` | `Formula: A*16777216+B*65536+C*256+D` | `Fast` | `odometer` |
| ECU Voltage | `AT` | `RV` | `ATRV voltage` | `Slow` | `ecu_voltage` |
| Fuel Level | `01` | `2F` | `Formula: (A*100)/255` | `Slow` | `fuel_level` |
| Engine Runtime | `01` | `1F` | `Formula: (A*256+B)/60` | `Slow` | `engine_runtime_min` |

### VIN and webhook state

When built-in VIN is enabled and successfully read, the webhook top-level `state` is set to the VIN.

If VIN is disabled or not available:

- the webhook `state` falls back to `unknown`

## Custom PIDs

The portal supports up to `10` custom PID rows.

Each row can be:

- left empty and unused,
- filled in and enabled,
- filled in and disabled for temporary testing.

### Custom PID fields

#### Enabled

Turns the custom PID on or off.

If disabled:

- it stays saved in configuration,
- but it is not polled.

#### Label

Human-readable name shown in the portal.

Examples:

- `Coolant Temperature`
- `Transmission Oil Temp`
- `Battery SOC`

#### JSON key

Key used in the webhook payload under `attributes`.

Example:

- label: `Coolant Temperature`
- key: `coolant_temp`

Resulting payload field:

```json
"coolant_temp": 92
```

If left empty, the sketch generates a normalized key from the label.

#### Service

The OBD service / mode.

Examples:

- `01`
- `09`
- `22`
- `AT`

#### PID

PID value for the chosen service.

Examples:

- `0C`
- `05`
- `10E0`
- `RV` for `ATRV`

#### Parser

Selects how the response should be interpreted.

Available parsers:

- `Formula`
- `VIN`
- `ATRV voltage`
- `ASCII text`
- `Unsigned integer`
- `Raw hex`

#### Formula

Used only by the `Formula` parser.

The sketch maps response bytes to variables:

- `A`
- `B`
- `C`
- `D`
- `E`
- `F`
- `G`
- `H`

Supported operators:

- `+`
- `-`
- `*`
- `/`
- parentheses `()`

Examples:

- `A`
- `A-40`
- `(A*256+B)/4`
- `(A*256+B)/60`
- `A*16777216+B*65536+C*256+D`

#### Schedule

Controls when the PID is polled.

Available values:

- `Fast`
- `Slow`
- `Once per session`

Meaning:

- `Fast` = every main loop
- `Slow` = every N loops, based on `Run slow PIDs every N loops`
- `Once per session` = read once after BLE session starts, then stop until reconnect or reboot

#### Request header

Optional per-PID request header.

If set, it overrides the `Default OBD request header`.

Examples:

- `7E0`
- `7E1`
- `7E2`

Use this when:
- a PID belongs to another ECU,
- the default header is not the correct request target.

#### Resp CAN

Optional expected response CAN ID filter.

Example:

- request header `7E0` usually implies response `7E8`
- request header `7E1` usually implies response `7E9`

If this field is left empty, the sketch derives it automatically from the request header by adding `8` in the usual OBD style.

Use this field when:
- the responding ECU does not follow the default `request + 8` pattern,
- you want to force the parser to accept only a specific response CAN ID.

#### Frames

Expected number of frames for the response.

Usage:

- `0` = auto
- `1` = single-frame response expected
- `2+` = require at least that many frames

This is useful for:

- multi-frame text/VIN-like data,
- long UDS / mode `22` responses,
- debugging custom manufacturer-specific PIDs.

#### Decimals

Number of decimal places used when numeric results are formatted.

Examples:

- `0` for integer values
- `1` for values like `12.3`
- `2` for values like `13.82`

## Parser reference

### Formula

Best for most numeric PIDs.

The sketch:
- removes transport framing,
- removes the positive response prefix,
- exposes remaining data bytes as `A..H`,
- evaluates the formula.

Examples:

- RPM: service `01`, pid `0C`, formula `(A*256+B)/4`
- Coolant temperature: service `01`, pid `05`, formula `A-40`
- Fuel pressure: service `01`, pid `0A`, formula `A*3`

### VIN

Special parser for VIN-style data.

Use for:

- service `09`
- pid `02`

This parser expects multi-frame VIN data and rebuilds the 17-character VIN.

### ATRV voltage

Special parser for adapter voltage response.

Use with:
- service `AT`
- pid `RV`

This parser is different from normal OBD frame parsing and reads the adapter's textual voltage output.

### ASCII text

Converts response bytes after the positive response prefix into readable ASCII text.

Use when:
- the ECU returns printable text,
- raw numeric formulas are not suitable.

### Unsigned integer

Interprets the data bytes after the positive response prefix as one big unsigned integer.

Useful for:
- counters,
- identifiers,
- raw numeric values when you do not want to write a formula.

### Raw hex

Returns the remaining response bytes as a hex string.

This is very useful for:
- reverse engineering new PIDs,
- comparing responses with Car Scanner,
- checking multi-frame payload structure before building a formula.

## How custom PID parsing works

For normal OBD services like `01`, `09`, `22`:

1. the sketch sends `ATSH<header>` when needed,
2. it configures `ATCRA<resp_can>` when needed,
3. it sends the actual OBD command,
4. it rebuilds the payload from single or multi-frame ELM output,
5. it removes the positive response prefix,
6. it applies the selected parser.

For example:

- request: service `01`, pid `0C`
- command sent: `010C`
- positive response prefix expected: `410C`
- formula parser then sees the remaining data bytes as `A`, `B`, etc.

## Example custom PID entries

### Example 1: Standard RPM

| Field | Value |
|---|---|
| Enabled | on |
| Label | `Engine RPM` |
| JSON key | `engine_rpm` |
| Service | `01` |
| PID | `0C` |
| Parser | `Formula` |
| Formula | `(A*256+B)/4` |
| Schedule | `Fast` |
| Request header | *(blank)* |
| Resp CAN | *(blank)* |
| Frames | `1` |
| Decimals | `0` |

### Example 2: Coolant temperature

| Field | Value |
|---|---|
| Enabled | on |
| Label | `Coolant Temperature` |
| JSON key | `coolant_temp` |
| Service | `01` |
| PID | `05` |
| Parser | `Formula` |
| Formula | `A-40` |
| Schedule | `Slow` |
| Request header | *(blank)* |
| Resp CAN | *(blank)* |
| Frames | `1` |
| Decimals | `0` |

### Example 3: Raw debug PID

| Field | Value |
|---|---|
| Enabled | on |
| Label | `Debug 22 F190` |
| JSON key | `debug_f190` |
| Service | `22` |
| PID | `F190` |
| Parser | `Raw hex` |
| Formula | *(blank)* |
| Schedule | `Once per session` |
| Request header | `7E0` |
| Resp CAN | `7E8` |
| Frames | `0` |
| Decimals | `0` |

## Webhook payload

The sketch sends a merged JSON payload.

Example:

```json
{
  "state": "WAUZZZ...",
  "attributes": {
    "wifi_ssid": "MyWiFi",
    "bt_address": "41:42:86:99:67:84",
    "gps_enabled": true,
    "default_request_header": "7E0",
    "gps_sats": 7,
    "latitude": 50.123456,
    "longitude": 19.123456,
    "active_pids": [
      "vin",
      "speed",
      "odometer",
      "fuel",
      "runtime",
      "coolant_temp",
      "engine_rpm"
    ],
    "vin": "WAUZZZ...",
    "speed": 0,
    "odometer": 182345,
    "fuel_level": 63.9,
    "engine_runtime_min": 12.4,
    "coolant_temp": 91,
    "engine_rpm": 842
  }
}
```

## Home Assistant example

If your webhook target is Home Assistant, it is often useful to add a proxy template sensor in `configuration.yaml`.

This helps expose the latest webhook-backed values in one stable entity and is convenient for dashboards, automations and map cards.

Example:

```yaml
template:
  sensor:
    - name: "Audi car proxy"
      state: >-
          {% set s = states('sensor.audi_car') %}
          {% if s not in ['unknown', 'unavailable', 'none', ''] %}
            {{ s }}
          {% else %}
            {{ this.state if this.state is defined else 'unknown' }}
          {% endif %}
        attributes:
          odometer: >
            {% set v = state_attr('sensor.audi_car', 'odometer') %}
            {{ v if v is not none else this.attributes.get('odometer', 0) }}
          fuel_level: >
            {% set v = state_attr('sensor.audi_car', 'fuel_level') %}
            {{ v if v is not none else this.attributes.get('fuel_level', 0) }}
          latitude: >
            {% set v = state_attr('sensor.audi_car', 'latitude') %}
            {{ v if v is not none else this.attributes.get('latitude', 0) }}
          longitude: >
            {% set v = state_attr('sensor.audi_car', 'longitude') %}
            {{ v if v is not none else this.attributes.get('longitude', 0) }}
          speed: >
            {% set v = state_attr('sensor.audi_car', 'speed') %}
            {{ v if v is not none else this.attributes.get('speed', 0) }}
          ecu_voltage: >
            {% set v = state_attr('sensor.audi_car', 'ecu_voltage') %}
            {{ v if v is not none else this.attributes.get('ecu_voltage', 0) }}
          engine_runtime_min: >
            {% set v = state_attr('sensor.audi_car', 'engine_runtime_min') %}
            {{ v if v is not none else this.attributes.get('engine_runtime_min', 0) }}
          fuel_liters: >
            {% set v = states('sensor.obd2bridge_fuel_liters') %}
            {{ v if v not in ['unknown', 'unavailable', 'none', ''] else this.attributes.get('fuel_liters', 0) }}
```

### Adapting the Home Assistant template

The exact attribute list should match what your ESP32 actually sends.

Use the example above as a starting point, then adjust it to your own captive portal setup:

- keep only the built-in PIDs you have enabled,
- add any custom PIDs by their `JSON key` from the portal,
- remove attributes you do not send,
- add derived helper sensors such as `fuel_liters` if you use them elsewhere in HA.

For example, if you create a custom PID with:

- label: `Coolant Temperature`
- JSON key: `coolant_temp`

then the HA proxy can expose it like this:

```yaml
        coolant_temp: "{{ state_attr('sensor.audi_car', 'coolant_temp') | float(0) }}"
```

This proxy pattern is especially handy when:
- you want one friendly entity for Lovelace,
- you want to keep selected attributes grouped together,
- you prefer referencing a stable helper sensor in automations instead of reading many raw webhook attributes directly.

## Runtime behavior

- GPS is sampled at the beginning of the loop
- the main loop runs approximately every `20 seconds`
- BLE routes are changed on demand when a PID uses a different header / response CAN
- `Once per session` values reset after:
  - reboot,
  - BLE reconnect,
  - manual restart

## Notes about Car Scanner discovered PIDs

This sketch is intentionally suited to the common workflow:

1. discover a PID in Car Scanner or another diagnostic tool,
2. inspect:
   - service / mode,
   - PID,
   - header,
   - number of bytes or frames,
   - conversion formula,
3. copy those values into the portal,
4. start with `Raw hex` if you are unsure,
5. switch to `Formula`, `ASCII text`, or another parser once the response is understood.

Recommended reverse-engineering workflow:

- first test with `Raw hex`
- confirm request header and response CAN
- confirm whether the response is single-frame or multi-frame
- only then write the final formula

## Limitations

- custom formula parsing currently supports bytes `A..H`
- maximum custom PID slots: `10`
- complex manufacturer-specific text parsers beyond the included modes may still need code changes
- this sketch assumes ELM327-style BLE behavior and OBD framing

## Security notes

- Wi-Fi credentials, BLE target, and webhook token are stored in NVS (`Preferences`)
- the setup AP password is fixed in the sketch source
- HTTPS webhook mode currently uses `setInsecure()`


## Troubleshooting

### Portal does not appear

- power-cycle the board with the configured Wi-Fi unavailable
- look for `OBD2-Setup-XXXX`
- open `192.168.4.1` manually

### GPS never gets a fix

- verify `GPS TX -> ESP GPIO5`
- confirm the module outputs NMEA at `9600 baud`
- test outdoors with a clear sky view
- check that `Enable GPS reading` is turned on

### BLE connects but a PID stays empty

- confirm the service and PID values
- verify the request header
- try setting the response CAN manually
- switch parser to `Raw hex` first
- if multi-frame is expected, set `Frames` to `0` or a higher value

### Formula gives wrong value

- verify byte order
- confirm the positive response prefix is being removed as expected
- compare the raw bytes with Car Scanner
- test with `Raw hex`, then rebuild the formula

## Build status

This sketch was tested on Audi A3 8V 2018 and Audi Q2 2019 with vGate iCar Pro 2S BLE Scanner.
If it works for you with different ELM327 BLE scanner please let me know so I can start creating compatibility list.
