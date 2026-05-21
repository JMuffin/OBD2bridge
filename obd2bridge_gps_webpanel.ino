#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>

#include <algorithm>
#include <ctype.h>
#include <math.h>
#include <vector>

constexpr char kPrefsNamespace[] = "obd2cfg";
constexpr char kDefaultWifiSsid[] = "";
constexpr char kDefaultWifiPassword[] = "";
constexpr char kDefaultWebhookUrl[] = "";
constexpr char kDefaultWebhookToken[] = "";
constexpr char kDefaultBtAddress[] = "";
constexpr char kDefaultBtName[] = "ELM327";
constexpr char kDefaultRequestHeader[] = "7E0";
constexpr char kPortalPassword[] = "9876543210";

// Wait this long on boot before falling back to the setup portal.
constexpr uint32_t kWifiStartupConnectTimeoutMs = 60000;
// While the setup portal is active, keep retrying the saved Wi-Fi in the background.
constexpr uint32_t kWifiPortalRetryIntervalMs = 15000;
// Normal reconnect interval after the device is already running without the portal.
constexpr uint32_t kWifiReconnectIntervalMs = 15000;
constexpr uint32_t kGpsReadWindowMs = 3000;
constexpr uint32_t kLoopIntervalMs = 20000;
constexpr uint32_t kBleScanSeconds = 5;

constexpr int RXPin = 5;
constexpr int TXPin = 6;
constexpr uint32_t GPSBaud = 9600;

constexpr size_t BUILTIN_PID_COUNT = 6;
constexpr size_t MAX_CUSTOM_PIDS = 10;
constexpr size_t MAX_PAYLOAD_BYTES = 48;
constexpr size_t MAX_FORMULA_BYTES = 8;

enum TaskSchedule : uint8_t {
  SCHEDULE_FAST = 0,
  SCHEDULE_SLOW = 1,
  SCHEDULE_ONCE = 2,
};

enum ParserKind : uint8_t {
  PARSER_FORMULA = 0,
  PARSER_ATRV = 1,
  PARSER_VIN = 2,
  PARSER_ASCII = 3,
  PARSER_HEX_UINT = 4,
  PARSER_RAW_HEX = 5,
};

struct TelemetryValue {
  bool valid = false;
  bool numeric = false;
  double number = 0.0;
  String text;
  uint8_t frames = 0;
  String rawHex;
};

struct BuiltinPidDefinition {
  const char* id;
  const char* label;
  const char* key;
  const char* service;
  const char* pid;
  const char* requestHeader;
  TaskSchedule schedule;
  ParserKind parser;
  const char* formula;
  uint8_t decimals;
  uint8_t expectedFrames;
  bool enabledByDefault;
  bool useAsState;
};

struct BuiltinPidRuntime {
  bool enabled = true;
  bool onceDone = false;
  TelemetryValue value;
};

struct CustomPidConfig {
  bool enabled = false;
  String label;
  String key;
  String service;
  String pid;
  String requestHeader;
  String responseFilter;
  uint8_t schedule = SCHEDULE_FAST;
  uint8_t parser = PARSER_FORMULA;
  String formula;
  uint8_t decimals = 1;
  uint8_t expectedFrames = 1;
  bool onceDone = false;
  TelemetryValue value;
};

struct DeviceConfig {
  String wifiSsid;
  String wifiPassword;
  String btAddress;
  String btName;
  String webhookUrl;
  String webhookToken;
  String defaultRequestHeader;
  bool gpsEnabled = true;
  uint8_t slowEvery = 3;
  BuiltinPidRuntime builtin[BUILTIN_PID_COUNT];
  std::vector<CustomPidConfig> customPids;
};

struct WifiScanResult {
  String ssid;
  int32_t rssi = 0;
  bool secure = false;
};

struct BleScanResult {
  String name;
  String address;
  int32_t rssi = 0;
};

struct GpsState {
  double lat = 0.0;
  double lon = 0.0;
  int sats = 0;
};

struct QuerySpec {
  String label;
  String key;
  String service;
  String pid;
  String requestHeader;
  String responseFilter;
  String formula;
  uint8_t schedule = SCHEDULE_FAST;
  uint8_t parser = PARSER_FORMULA;
  uint8_t decimals = 1;
  uint8_t expectedFrames = 1;
  bool useAsState = false;
};

struct ObdPayload {
  bool valid = false;
  uint8_t frames = 0;
  size_t length = 0;
  uint8_t bytes[MAX_PAYLOAD_BYTES] = {0};
  String cleaned;
};

const BuiltinPidDefinition kBuiltinPidDefs[BUILTIN_PID_COUNT] = {
    {"vin", "Vehicle VIN", "vin", "09", "02", "", SCHEDULE_ONCE, PARSER_VIN, "", 0, 3, true, true},
    {"speed", "Vehicle Speed", "speed", "01", "0D", "", SCHEDULE_FAST, PARSER_FORMULA, "A", 0, 1, true, false},
    {"odometer", "Odometer", "odometer", "22", "10E0", "", SCHEDULE_FAST, PARSER_FORMULA, "A*16777216+B*65536+C*256+D", 0, 1, true, false},
    {"voltage", "ECU Voltage", "ecu_voltage", "AT", "RV", "", SCHEDULE_SLOW, PARSER_ATRV, "", 1, 0, true, false},
    {"fuel", "Fuel Level", "fuel_level", "01", "2F", "", SCHEDULE_SLOW, PARSER_FORMULA, "(A*100)/255", 1, 1, true, false},
    {"runtime", "Engine Runtime", "engine_runtime_min", "01", "1F", "", SCHEDULE_SLOW, PARSER_FORMULA, "(A*256+B)/60", 1, 1, true, false},
};

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
Preferences preferences;
WebServer webServer(80);
DNSServer dnsServer;

DeviceConfig config;
GpsState gpsState;
std::vector<WifiScanResult> scannedWifi;
std::vector<BleScanResult> scannedBle;

NimBLEClient* pClient = nullptr;
BLERemoteCharacteristic* pWriteChar = nullptr;

String lastResponse;
String portalApSsid;
String portalReason;
String activeRequestHeader;
String activeResponseFilter;
bool responseReceived = false;
bool configPortalActive = false;
bool bleReady = false;
unsigned long cycleCounter = 0;
unsigned long lastWifiReconnectAttempt = 0;

String trimCopy(String value);
String toUpperTrimmed(String value);
String normalizeHex(String value);
String normalizeKey(String value);
String htmlEscape(const String& value);
String checkedAttr(bool enabled);
String selectedAttr(bool selected);
String scheduleToValue(uint8_t schedule);
String scheduleLabel(uint8_t schedule);
uint8_t scheduleFromValue(const String& value);
String parserToValue(uint8_t parser);
String parserLabel(uint8_t parser);
uint8_t parserFromValue(const String& value);
String renderScheduleOptions(uint8_t selected);
String renderParserOptions(uint8_t selected);
String buildConfigPage();
String buildCommand(const String& service, const String& pid);
String deriveResponsePrefix(const String& service, const String& pid);
String deriveResponseFilter(const String& requestHeader);
double roundToDecimals(double value, uint8_t decimals);
bool isHexString(const String& value);
bool parserNeedsRouting(uint8_t parser);
bool parserNeedsFormula(uint8_t parser);
void applyDefaultConfig();
void resetRuntimeState();
void loadConfig();
void saveConfig();
void ensureBleReady();
bool connectToConfiguredWifi(uint32_t timeoutMs);
void maintainWiFi();
void startConfigPortal(const String& reason);
void stopConfigPortal();
void handleRoot();
void handleSave();
void handleWifiScanApi();
void handleBleScanApi();
void scanWifiNetworks();
void scanBleDevices();
void disconnectBleSession();
bool ensureBleConnection();
bool ensurePidRoute(const QuerySpec& query);
void readGpsWindow(uint32_t timeoutMs);
void processGpsBackground();
bool shouldRunSchedule(uint8_t schedule, bool onceDone, bool runSlowCycle);
bool runTelemetryCycle();
bool executeQuery(const QuerySpec& query, TelemetryValue& value);
bool parseValueFromResponse(const QuerySpec& query, const String& rawResponse, TelemetryValue& value);
ObdPayload extractObdPayload(const String& rawResponse, const String& responseFilter, uint8_t expectedFrames);
bool extractDataBytesAfterPrefix(const ObdPayload& payload, const String& prefix, uint8_t* dataBytes, size_t& dataLength);
bool parseAtrvValue(const String& rawResponse, TelemetryValue& value);
bool parseVinValue(const QuerySpec& query, const String& rawResponse, TelemetryValue& value);
bool parseFormulaValue(const QuerySpec& query, const String& rawResponse, TelemetryValue& value);
bool parseAsciiValue(const QuerySpec& query, const String& rawResponse, TelemetryValue& value);
bool parseHexUIntValue(const QuerySpec& query, const String& rawResponse, TelemetryValue& value);
bool parseRawHexValue(const QuerySpec& query, const String& rawResponse, TelemetryValue& value);
bool pushToWebhook();
void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
bool sendCommand(const String& cmd, int timeout = 2500);
void cleanResponse(String& response);

class FormulaEvaluator {
 public:
  FormulaEvaluator(const String& expression, const uint8_t* dataBytes, size_t dataLength)
      : expr_(expression), data_(dataBytes), length_(dataLength), pos_(0), ok_(true) {}

  bool evaluate(double& outValue) {
    pos_ = 0;
    ok_ = true;
    double value = parseExpression();
    skipWhitespace();
    if (!ok_ || pos_ != expr_.length()) {
      return false;
    }
    outValue = value;
    return true;
  }

 private:
  const String& expr_;
  const uint8_t* data_;
  size_t length_;
  size_t pos_;
  bool ok_;

  void skipWhitespace() {
    while (pos_ < expr_.length() && isspace(static_cast<unsigned char>(expr_.charAt(pos_)))) {
      pos_++;
    }
  }

  double parseExpression() {
    double value = parseTerm();
    while (ok_) {
      skipWhitespace();
      if (pos_ >= expr_.length()) {
        break;
      }
      char op = expr_.charAt(pos_);
      if (op != '+' && op != '-') {
        break;
      }
      pos_++;
      double rhs = parseTerm();
      if (!ok_) {
        return 0.0;
      }
      value = (op == '+') ? (value + rhs) : (value - rhs);
    }
    return value;
  }

  double parseTerm() {
    double value = parseFactor();
    while (ok_) {
      skipWhitespace();
      if (pos_ >= expr_.length()) {
        break;
      }
      char op = expr_.charAt(pos_);
      if (op != '*' && op != '/') {
        break;
      }
      pos_++;
      double rhs = parseFactor();
      if (!ok_) {
        return 0.0;
      }
      if (op == '*') {
        value *= rhs;
      } else {
        if (fabs(rhs) < 0.0000001) {
          ok_ = false;
          return 0.0;
        }
        value /= rhs;
      }
    }
    return value;
  }

  double parseFactor() {
    skipWhitespace();
    if (pos_ >= expr_.length()) {
      ok_ = false;
      return 0.0;
    }

    char ch = expr_.charAt(pos_);
    if (ch == '+') {
      pos_++;
      return parseFactor();
    }
    if (ch == '-') {
      pos_++;
      return -parseFactor();
    }
    if (ch == '(') {
      pos_++;
      double value = parseExpression();
      skipWhitespace();
      if (pos_ >= expr_.length() || expr_.charAt(pos_) != ')') {
        ok_ = false;
        return 0.0;
      }
      pos_++;
      return value;
    }

    if (isalpha(static_cast<unsigned char>(ch))) {
      char upper = static_cast<char>(toupper(static_cast<unsigned char>(ch)));
      if (upper < 'A' || upper > 'H') {
        ok_ = false;
        return 0.0;
      }
      size_t index = static_cast<size_t>(upper - 'A');
      if (index >= length_) {
        ok_ = false;
        return 0.0;
      }
      pos_++;
      return static_cast<double>(data_[index]);
    }

    return parseNumber();
  }

  double parseNumber() {
    skipWhitespace();
    size_t start = pos_;
    bool hasDot = false;
    while (pos_ < expr_.length()) {
      char ch = expr_.charAt(pos_);
      if (isdigit(static_cast<unsigned char>(ch))) {
        pos_++;
        continue;
      }
      if (ch == '.' && !hasDot) {
        hasDot = true;
        pos_++;
        continue;
      }
      break;
    }
    if (start == pos_) {
      ok_ = false;
      return 0.0;
    }
    return expr_.substring(start, pos_).toFloat();
  }
};

String trimCopy(String value) {
  value.trim();
  return value;
}

String toUpperTrimmed(String value) {
  value.trim();
  value.toUpperCase();
  return value;
}

String normalizeHex(String value) {
  String out;
  out.reserve(value.length());
  for (size_t i = 0; i < value.length(); ++i) {
    char ch = value.charAt(i);
    if (isxdigit(static_cast<unsigned char>(ch))) {
      out += static_cast<char>(toupper(static_cast<unsigned char>(ch)));
    }
  }
  return out;
}

String normalizeKey(String value) {
  value.trim();
  String out;
  out.reserve(value.length());
  bool lastUnderscore = false;
  for (size_t i = 0; i < value.length(); ++i) {
    char ch = value.charAt(i);
    if (isalnum(static_cast<unsigned char>(ch))) {
      out += static_cast<char>(tolower(static_cast<unsigned char>(ch)));
      lastUnderscore = false;
    } else if (!lastUnderscore) {
      out += '_';
      lastUnderscore = true;
    }
  }
  while (out.startsWith("_")) {
    out.remove(0, 1);
  }
  while (out.endsWith("_")) {
    out.remove(out.length() - 1);
  }
  return out;
}

String htmlEscape(const String& value) {
  String out = value;
  out.replace("&", "&amp;");
  out.replace("<", "&lt;");
  out.replace(">", "&gt;");
  out.replace("\"", "&quot;");
  out.replace("'", "&#39;");
  return out;
}

String checkedAttr(bool enabled) {
  return enabled ? " checked" : "";
}

String selectedAttr(bool selected) {
  return selected ? " selected" : "";
}

String scheduleToValue(uint8_t schedule) {
  switch (schedule) {
    case SCHEDULE_SLOW:
      return "slow";
    case SCHEDULE_ONCE:
      return "once";
    case SCHEDULE_FAST:
    default:
      return "fast";
  }
}

String scheduleLabel(uint8_t schedule) {
  switch (schedule) {
    case SCHEDULE_SLOW:
      return "Slow";
    case SCHEDULE_ONCE:
      return "Once per session";
    case SCHEDULE_FAST:
    default:
      return "Fast";
  }
}

uint8_t scheduleFromValue(const String& value) {
  if (value == "slow") {
    return SCHEDULE_SLOW;
  }
  if (value == "once") {
    return SCHEDULE_ONCE;
  }
  return SCHEDULE_FAST;
}

String parserToValue(uint8_t parser) {
  switch (parser) {
    case PARSER_ATRV:
      return "atrv";
    case PARSER_VIN:
      return "vin";
    case PARSER_ASCII:
      return "ascii";
    case PARSER_HEX_UINT:
      return "hex_uint";
    case PARSER_RAW_HEX:
      return "raw_hex";
    case PARSER_FORMULA:
    default:
      return "formula";
  }
}

String parserLabel(uint8_t parser) {
  switch (parser) {
    case PARSER_ATRV:
      return "ATRV voltage";
    case PARSER_VIN:
      return "VIN";
    case PARSER_ASCII:
      return "ASCII text";
    case PARSER_HEX_UINT:
      return "Unsigned integer";
    case PARSER_RAW_HEX:
      return "Raw hex";
    case PARSER_FORMULA:
    default:
      return "Formula";
  }
}

uint8_t parserFromValue(const String& value) {
  if (value == "atrv") {
    return PARSER_ATRV;
  }
  if (value == "vin") {
    return PARSER_VIN;
  }
  if (value == "ascii") {
    return PARSER_ASCII;
  }
  if (value == "hex_uint") {
    return PARSER_HEX_UINT;
  }
  if (value == "raw_hex") {
    return PARSER_RAW_HEX;
  }
  return PARSER_FORMULA;
}

String renderScheduleOptions(uint8_t selected) {
  String html;
  html.reserve(160);
  html += "<option value='fast'";
  html += selectedAttr(selected == SCHEDULE_FAST);
  html += ">Fast</option>";
  html += "<option value='slow'";
  html += selectedAttr(selected == SCHEDULE_SLOW);
  html += ">Slow</option>";
  html += "<option value='once'";
  html += selectedAttr(selected == SCHEDULE_ONCE);
  html += ">Once per session</option>";
  return html;
}

String renderParserOptions(uint8_t selected) {
  String html;
  html.reserve(320);
  const uint8_t parsers[] = {PARSER_FORMULA, PARSER_VIN, PARSER_ATRV, PARSER_ASCII, PARSER_HEX_UINT, PARSER_RAW_HEX};
  for (uint8_t parser : parsers) {
    html += "<option value='";
    html += parserToValue(parser);
    html += "'";
    html += selectedAttr(selected == parser);
    html += ">";
    html += parserLabel(parser);
    html += "</option>";
  }
  return html;
}

String buildCommand(const String& service, const String& pid) {
  String mode = toUpperTrimmed(service);
  String suffix = toUpperTrimmed(pid);
  suffix.replace(" ", "");
  if (mode.isEmpty()) {
    return suffix;
  }
  mode.replace(" ", "");
  if (mode == "AT") {
    return mode + suffix;
  }
  return normalizeHex(mode) + normalizeHex(suffix);
}

String deriveResponsePrefix(const String& service, const String& pid) {
  String mode = normalizeHex(service);
  String identifier = normalizeHex(pid);
  if (mode.length() != 2) {
    return "";
  }
  char* endPtr = nullptr;
  long value = strtol(mode.c_str(), &endPtr, 16);
  if (endPtr == nullptr || *endPtr != '\0') {
    return "";
  }
  char buffer[3];
  snprintf(buffer, sizeof(buffer), "%02lX", (value + 0x40) & 0xFF);
  return String(buffer) + identifier;
}

String deriveResponseFilter(const String& requestHeader) {
  String header = normalizeHex(requestHeader);
  if (header.length() == 3) {
    char* endPtr = nullptr;
    long value = strtol(header.c_str(), &endPtr, 16);
    if (endPtr != nullptr && *endPtr == '\0' && value >= 0x700 && value <= 0x7F7) {
      char buffer[4];
      snprintf(buffer, sizeof(buffer), "%03lX", value + 0x8);
      return String(buffer);
    }
  }
  return "";
}

double roundToDecimals(double value, uint8_t decimals) {
  double scale = 1.0;
  for (uint8_t i = 0; i < decimals; ++i) {
    scale *= 10.0;
  }
  return round(value * scale) / scale;
}

bool isHexString(const String& value) {
  if (value.isEmpty()) {
    return false;
  }
  for (size_t i = 0; i < value.length(); ++i) {
    if (!isxdigit(static_cast<unsigned char>(value.charAt(i)))) {
      return false;
    }
  }
  return true;
}

bool parserNeedsRouting(uint8_t parser) {
  return parser != PARSER_ATRV;
}

bool parserNeedsFormula(uint8_t parser) {
  return parser == PARSER_FORMULA;
}

void applyDefaultConfig() {
  config.wifiSsid = kDefaultWifiSsid;
  config.wifiPassword = kDefaultWifiPassword;
  config.btAddress = kDefaultBtAddress;
  config.btName = kDefaultBtName;
  config.webhookUrl = kDefaultWebhookUrl;
  config.webhookToken = kDefaultWebhookToken;
  config.defaultRequestHeader = kDefaultRequestHeader;
  config.gpsEnabled = true;
  config.slowEvery = 3;
  config.customPids.clear();

  for (size_t i = 0; i < BUILTIN_PID_COUNT; ++i) {
    config.builtin[i].enabled = kBuiltinPidDefs[i].enabledByDefault;
    config.builtin[i].onceDone = false;
    config.builtin[i].value = TelemetryValue();
  }
}

void resetRuntimeState() {
  gpsState.sats = 0;
  activeRequestHeader = "";
  activeResponseFilter = "";
  cycleCounter = 0;

  for (size_t i = 0; i < BUILTIN_PID_COUNT; ++i) {
    config.builtin[i].onceDone = false;
    config.builtin[i].value = TelemetryValue();
  }

  for (CustomPidConfig& pid : config.customPids) {
    pid.onceDone = false;
    pid.value = TelemetryValue();
  }
}

void loadConfig() {
  applyDefaultConfig();

  preferences.begin(kPrefsNamespace, true);
  config.wifiSsid = preferences.getString("wifi_ssid", config.wifiSsid);
  config.wifiPassword = preferences.getString("wifi_pass", config.wifiPassword);
  config.btAddress = preferences.getString("bt_addr", config.btAddress);
  config.btName = preferences.getString("bt_name", config.btName);
  config.webhookUrl = preferences.getString("webhook_url", config.webhookUrl);
  config.webhookToken = preferences.getString("webhook_token", config.webhookToken);
  config.defaultRequestHeader = preferences.getString("req_header", config.defaultRequestHeader);
  config.gpsEnabled = preferences.getBool("gps_enabled", config.gpsEnabled);
  config.slowEvery = preferences.getUChar("slow_every", config.slowEvery);
  String builtinJson = preferences.getString("builtin_cfg", "");
  String customJson = preferences.getString("custom_cfg", "");
  preferences.end();

  config.defaultRequestHeader = normalizeHex(config.defaultRequestHeader);
  if (config.defaultRequestHeader.isEmpty()) {
    config.defaultRequestHeader = kDefaultRequestHeader;
  }
  if (config.slowEvery < 1) {
    config.slowEvery = 1;
  }

  if (!builtinJson.isEmpty()) {
    DynamicJsonDocument doc(2048);
    if (!deserializeJson(doc, builtinJson)) {
      JsonObject root = doc.as<JsonObject>();
      for (size_t i = 0; i < BUILTIN_PID_COUNT; ++i) {
        config.builtin[i].enabled = root[kBuiltinPidDefs[i].id] | config.builtin[i].enabled;
      }
    }
  }

  if (!customJson.isEmpty()) {
    DynamicJsonDocument doc(16384);
    if (!deserializeJson(doc, customJson)) {
      JsonArray array = doc.as<JsonArray>();
      for (JsonObject item : array) {
        if (config.customPids.size() >= MAX_CUSTOM_PIDS) {
          break;
        }
        CustomPidConfig pid;
        pid.enabled = item["enabled"] | true;
        pid.label = trimCopy(item["label"] | "");
        pid.key = normalizeKey(item["key"] | pid.label);
        pid.service = toUpperTrimmed(item["service"] | "");
        pid.pid = toUpperTrimmed(item["pid"] | "");
        pid.requestHeader = normalizeHex(item["request_header"] | "");
        pid.responseFilter = normalizeHex(item["response_filter"] | "");
        pid.schedule = item["schedule"] | static_cast<uint8_t>(SCHEDULE_FAST);
        pid.parser = item["parser"] | static_cast<uint8_t>(PARSER_FORMULA);
        pid.formula = trimCopy(item["formula"] | "");
        pid.decimals = item["decimals"] | static_cast<uint8_t>(1);
        pid.expectedFrames = item["expected_frames"] | static_cast<uint8_t>(1);

        if (pid.label.isEmpty()) {
          pid.label = "Custom PID";
        }
        if (pid.key.isEmpty()) {
          pid.key = normalizeKey(pid.label);
        }
        if (pid.requestHeader.isEmpty()) {
          pid.requestHeader = "";
        }
        if (pid.responseFilter.isEmpty() && parserNeedsRouting(pid.parser)) {
          String effectiveHeader = pid.requestHeader.isEmpty() ? config.defaultRequestHeader : pid.requestHeader;
          pid.responseFilter = deriveResponseFilter(effectiveHeader);
        }
        if (pid.expectedFrames > 15) {
          pid.expectedFrames = 15;
        }
        config.customPids.push_back(pid);
      }
    }
  }

  resetRuntimeState();
}

void saveConfig() {
  DynamicJsonDocument builtinDoc(2048);
  JsonObject builtinRoot = builtinDoc.to<JsonObject>();
  for (size_t i = 0; i < BUILTIN_PID_COUNT; ++i) {
    builtinRoot[kBuiltinPidDefs[i].id] = config.builtin[i].enabled;
  }

  DynamicJsonDocument customDoc(16384);
  JsonArray customArray = customDoc.to<JsonArray>();
  for (const CustomPidConfig& pid : config.customPids) {
    JsonObject item = customArray.createNestedObject();
    item["enabled"] = pid.enabled;
    item["label"] = pid.label;
    item["key"] = pid.key;
    item["service"] = pid.service;
    item["pid"] = pid.pid;
    item["request_header"] = pid.requestHeader;
    item["response_filter"] = pid.responseFilter;
    item["schedule"] = pid.schedule;
    item["parser"] = pid.parser;
    item["formula"] = pid.formula;
    item["decimals"] = pid.decimals;
    item["expected_frames"] = pid.expectedFrames;
  }

  String builtinJson;
  String customJson;
  serializeJson(builtinDoc, builtinJson);
  serializeJson(customDoc, customJson);

  preferences.begin(kPrefsNamespace, false);
  preferences.putString("wifi_ssid", config.wifiSsid);
  preferences.putString("wifi_pass", config.wifiPassword);
  preferences.putString("bt_addr", config.btAddress);
  preferences.putString("bt_name", config.btName);
  preferences.putString("webhook_url", config.webhookUrl);
  preferences.putString("webhook_token", config.webhookToken);
  preferences.putString("req_header", config.defaultRequestHeader);
  preferences.putBool("gps_enabled", config.gpsEnabled);
  preferences.putUChar("slow_every", config.slowEvery);
  preferences.putString("builtin_cfg", builtinJson);
  preferences.putString("custom_cfg", customJson);
  preferences.end();
}

String buildConfigPage() {
  String html;
  html.reserve(32000);

  html += F(R"HTML(
<!doctype html>
<html>
<head>
  <meta charset='utf-8'>
  <meta name='viewport' content='width=device-width,initial-scale=1'>
  <title>OBD2 Bridge Setup</title>
  <style>
    :root {
      --bg: #f5f2eb;
      --card: #fffdf9;
      --ink: #1f2933;
      --muted: #61707d;
      --line: #d7d0c4;
      --brand: #0f766e;
      --accent: #9a5b12;
      --soft: #edf7f6;
      --warn: #fff3de;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Segoe UI", Arial, sans-serif;
      color: var(--ink);
      background: linear-gradient(180deg, #f5f2eb 0%, #ece3d5 100%);
    }
    .wrap {
      max-width: 1280px;
      margin: 0 auto;
      padding: 24px 16px 40px;
    }
    .hero {
      background: linear-gradient(135deg, #17384a 0%, #0f766e 100%);
      color: #fff;
      border-radius: 22px;
      padding: 24px;
      box-shadow: 0 16px 40px rgba(20, 45, 60, 0.24);
    }
    .hero h1 { margin: 0 0 8px; font-size: 30px; }
    .hero p { margin: 6px 0 0; opacity: 0.95; line-height: 1.5; }
    .chips {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 14px;
    }
    .chip {
      padding: 6px 10px;
      border-radius: 999px;
      background: rgba(255, 255, 255, 0.18);
      font-size: 12px;
      font-weight: 700;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 16px;
      margin-top: 18px;
    }
    .card {
      background: var(--card);
      border: 1px solid rgba(0, 0, 0, 0.08);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 8px 28px rgba(0, 0, 0, 0.06);
    }
    .card.full {
      grid-column: 1 / -1;
    }
    .card h2 {
      margin: 0 0 10px;
      font-size: 20px;
    }
    .muted {
      color: var(--muted);
      font-size: 14px;
      line-height: 1.5;
    }
    label {
      display: block;
      margin: 12px 0 6px;
      font-size: 13px;
      font-weight: 700;
      letter-spacing: 0.01em;
    }
    input[type=text],
    input[type=password],
    input[type=number],
    select {
      width: 100%;
      border: 1px solid var(--line);
      border-radius: 12px;
      padding: 11px 12px;
      background: #fff;
      font-size: 14px;
    }
    input[type=checkbox] {
      transform: scale(1.1);
    }
    .checkline {
      display: flex;
      align-items: center;
      gap: 10px;
      margin-top: 12px;
      font-size: 14px;
      color: var(--muted);
    }
    .row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
    }
    .actions {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-top: 12px;
    }
    button {
      border: 0;
      border-radius: 12px;
      padding: 11px 16px;
      font-size: 14px;
      font-weight: 700;
      cursor: pointer;
    }
    .btn-main { background: var(--brand); color: #fff; }
    .btn-alt { background: #e6eef0; color: #17384a; }
    .btn-pick { background: #fde8d5; color: #8a4b08; }
    .results {
      margin-top: 12px;
      display: grid;
      gap: 8px;
    }
    .result {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      padding: 10px 12px;
      border: 1px solid var(--line);
      border-radius: 12px;
      background: #fff;
    }
    .table-wrap {
      overflow-x: auto;
      margin-top: 12px;
    }
    table {
      width: 100%;
      min-width: 980px;
      border-collapse: collapse;
    }
    th, td {
      padding: 12px 10px;
      border-bottom: 1px solid #ece6dc;
      text-align: left;
      vertical-align: top;
      font-size: 14px;
    }
    th {
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--muted);
    }
    td input[type=text],
    td input[type=password],
    td input[type=number],
    td select {
      min-width: 120px;
    }
    code {
      white-space: nowrap;
      font-family: Consolas, monospace;
      font-size: 12px;
    }
    .hint {
      margin-top: 14px;
      padding: 12px 14px;
      border-radius: 14px;
      border: 1px solid #f0d2a7;
      background: var(--warn);
      color: #7a4b0b;
      font-size: 14px;
      line-height: 1.5;
    }
    .footer {
      margin-top: 18px;
      display: flex;
      justify-content: space-between;
      gap: 12px;
      align-items: center;
      flex-wrap: wrap;
    }
    @media (max-width: 720px) {
      .row { grid-template-columns: 1fr; }
      .hero h1 { font-size: 26px; }
    }
  </style>
</head>
<body>
<div class='wrap'>
)HTML");

  html += "<section class='hero'><h1>OBD2 Bridge Setup</h1>";
  html += "<p>Configuration mode started because the device could not join its saved Wi-Fi network during boot.</p>";
  if (!portalReason.isEmpty()) {
    html += "<p><strong>Reason:</strong> " + htmlEscape(portalReason) + "</p>";
  }
  html += "<div class='chips'>";
  html += "<span class='chip'>AP: " + htmlEscape(portalApSsid) + "</span>";
  html += "<span class='chip'>IP: " + WiFi.softAPIP().toString() + "</span>";
  html += "<span class='chip'>BLE ready: " + String(bleReady ? "yes" : "no") + "</span>";
  html += "<span class='chip'>Default header: " + htmlEscape(config.defaultRequestHeader) + "</span>";
  html += "</div></section>";

  html += F("<form method='post' action='/save'><div class='grid'>");

  html += F(R"HTML(
    <section class='card'>
      <h2>Wi-Fi & GPS</h2>
      <p class='muted'>Choose the startup Wi-Fi network. If connection fails on boot, this setup portal becomes available again.</p>
      <label for='wifi_ssid'>Wi-Fi SSID</label>
      <input id='wifi_ssid' name='wifi_ssid' type='text' list='wifi-list' value=')HTML");
  html += htmlEscape(config.wifiSsid);
  html += F(R"HTML('>
      <datalist id='wifi-list'></datalist>
      <label for='wifi_password'>Wi-Fi password</label>
      <input id='wifi_password' name='wifi_password' type='password' value=')HTML");
  html += "'" + htmlEscape(config.wifiPassword) + "'";
  html += F(R"HTML(>
      <div class='actions'>
        <button class='btn-alt' type='button' onclick='scanWifi()'>Scan Wi-Fi</button>
      </div>
      <div id='wifi-results' class='results'></div>

      <label for='default_request_header'>Default OBD request header</label>
      <input id='default_request_header' name='default_request_header' type='text' value=')HTML");
  html += "'" + htmlEscape(config.defaultRequestHeader) + "'";
  html += F(R"HTML(>

      <label for='slow_every'>Run slow PIDs every N loops</label>
      <input id='slow_every' name='slow_every' type='number' min='1' max='20' value=')HTML");
  html += "'" + String(config.slowEvery) + "'";
  html += F(R"HTML(>

      <label class='checkline'>
        <input id='gps_enabled' name='gps_enabled' type='checkbox')HTML");
  html += checkedAttr(config.gpsEnabled);
  html += F(R"HTML(>
        <span>Enable GPS reading</span>
      </label>
    </section>

    <section class='card'>
      <h2>Bluetooth & Webhook</h2>
      <p class='muted'>Pick the BLE OBD adapter and the webhook endpoint that receives the merged telemetry payload.</p>
      <label for='bt_address'>BLE address</label>
      <input id='bt_address' name='bt_address' type='text' list='ble-list' value=')HTML");
  html += "'" + htmlEscape(config.btAddress) + "'";
  html += F(R"HTML(>
      <datalist id='ble-list'></datalist>
      <label for='bt_name'>BLE display name</label>
      <input id='bt_name' name='bt_name' type='text' value=')HTML");
  html += "'" + htmlEscape(config.btName) + "'";
  html += F(R"HTML(>
      <div class='actions'>
        <button class='btn-alt' type='button' onclick='scanBle()'>Scan BLE</button>
      </div>
      <div id='ble-results' class='results'></div>

      <label for='webhook_url'>Webhook URL</label>
      <input id='webhook_url' name='webhook_url' type='text' value=')HTML");
  html += "'" + htmlEscape(config.webhookUrl) + "'";
  html += F(R"HTML(>

      <label for='webhook_token'>Bearer token</label>
      <input id='webhook_token' name='webhook_token' type='password' value=')HTML");
  html += "'" + htmlEscape(config.webhookToken) + "'";
  html += F(R"HTML(>
    </section>
)HTML");

  html += F(R"HTML(
    <section class='card full'>
      <h2>Built-in PIDs</h2>
      <p class='muted'>These are the default built-in queries. They stay fixed and can only be enabled or disabled.</p>
      <div class='table-wrap'>
        <table>
          <thead>
            <tr>
              <th>Enabled</th>
              <th>Label</th>
              <th>Key</th>
              <th>Command</th>
              <th>Parser</th>
              <th>Schedule</th>
            </tr>
          </thead>
          <tbody>
)HTML");

  for (size_t i = 0; i < BUILTIN_PID_COUNT; ++i) {
    const BuiltinPidDefinition& def = kBuiltinPidDefs[i];
    html += "<tr><td><input type='checkbox' name='builtin_" + String(def.id) + "_enabled'";
    html += checkedAttr(config.builtin[i].enabled);
    html += "></td><td><strong>" + htmlEscape(def.label) + "</strong></td>";
    html += "<td><code>" + htmlEscape(def.key) + "</code></td>";
    html += "<td><code>" + htmlEscape(buildCommand(def.service, def.pid)) + "</code></td>";
    html += "<td>" + parserLabel(def.parser) + "</td>";
    html += "<td>" + scheduleLabel(def.schedule) + "</td></tr>";
  }

  html += F(R"HTML(
          </tbody>
        </table>
      </div>
      <div class='hint'>Built-in VIN remains the default state source when enabled. If you disable it, the webhook state falls back to <code>unknown</code>.</div>
    </section>

    <section class='card full'>
      <h2>Custom PIDs</h2>
      <p class='muted'>Fill any row below to add a custom PID. Leave a row empty to keep it unused. Clear an existing row to remove it.</p>
      <div class='table-wrap'>
        <table>
          <thead>
            <tr>
              <th>Enabled</th>
              <th>Label</th>
              <th>JSON key</th>
              <th>Service</th>
              <th>PID</th>
              <th>Parser</th>
              <th>Formula</th>
              <th>Schedule</th>
              <th>Request header</th>
              <th>Resp CAN</th>
              <th>Frames</th>
              <th>Decimals</th>
            </tr>
          </thead>
          <tbody>
)HTML");

  for (size_t i = 0; i < MAX_CUSTOM_PIDS; ++i) {
    const CustomPidConfig* pid = (i < config.customPids.size()) ? &config.customPids[i] : nullptr;
    html += "<tr>";
    html += "<td><input type='checkbox' name='custom_" + String(i) + "_enabled'";
    html += checkedAttr(pid != nullptr && pid->enabled);
    html += "></td>";
    html += "<td><input type='text' name='custom_" + String(i) + "_label' value='" + htmlEscape(pid != nullptr ? pid->label : "") + "'></td>";
    html += "<td><input type='text' name='custom_" + String(i) + "_key' value='" + htmlEscape(pid != nullptr ? pid->key : "") + "'></td>";
    html += "<td><input type='text' name='custom_" + String(i) + "_service' value='" + htmlEscape(pid != nullptr ? pid->service : "") + "'></td>";
    html += "<td><input type='text' name='custom_" + String(i) + "_pid' value='" + htmlEscape(pid != nullptr ? pid->pid : "") + "'></td>";
    html += "<td><select name='custom_" + String(i) + "_parser'>" + renderParserOptions(pid != nullptr ? pid->parser : PARSER_FORMULA) + "</select></td>";
    html += "<td><input type='text' name='custom_" + String(i) + "_formula' value='" + htmlEscape(pid != nullptr ? pid->formula : "") + "'></td>";
    html += "<td><select name='custom_" + String(i) + "_schedule'>" + renderScheduleOptions(pid != nullptr ? pid->schedule : SCHEDULE_FAST) + "</select></td>";
    html += "<td><input type='text' name='custom_" + String(i) + "_header' value='" + htmlEscape(pid != nullptr ? pid->requestHeader : "") + "'></td>";
    html += "<td><input type='text' name='custom_" + String(i) + "_response_filter' value='" + htmlEscape(pid != nullptr ? pid->responseFilter : "") + "'></td>";
    html += "<td><input type='number' name='custom_" + String(i) + "_frames' min='0' max='15' value='" + String(pid != nullptr ? pid->expectedFrames : 1) + "'></td>";
    html += "<td><input type='number' name='custom_" + String(i) + "_decimals' min='0' max='6' value='" + String(pid != nullptr ? pid->decimals : 1) + "'></td>";
    html += "</tr>";
  }

  html += F(R"HTML(
          </tbody>
        </table>
      </div>
      <div class='hint'>
        Parser guide:
        <strong>Formula</strong> uses bytes <code>A..H</code>, for example <code>(A*256+B)/4</code>.
        <strong>Resp CAN</strong> is optional; if left empty, the code derives it from the request header.
        <strong>Frames</strong> accepts <code>0</code> for auto or a higher number to require multi-frame responses.
      </div>
      <div class='footer'>
        <div class='muted'>Target environment: ESP32 4MB, Huge App (3MB app + 1MB SPIFFS).</div>
        <button class='btn-main' type='submit'>Save and restart</button>
      </div>
    </section>
  </div>
  </form>
)HTML");

  html += F(R"HTML(
<script>
const esc = (v) => String(v ?? '').replace(/[&<>"']/g, (ch) => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));

async function scanWifi() {
  const box = document.getElementById('wifi-results');
  box.innerHTML = '<div class="result"><span>Scanning Wi-Fi...</span></div>';
  try {
    const res = await fetch('/api/scan/wifi', { cache: 'no-store' });
    const data = await res.json();
    const items = data.wifi || [];
    const list = document.getElementById('wifi-list');
    list.innerHTML = '';
    if (!items.length) {
      box.innerHTML = '<div class="result"><span>No Wi-Fi networks found.</span></div>';
      return;
    }
    box.innerHTML = '';
    items.forEach((item) => {
      const opt = document.createElement('option');
      opt.value = item.ssid;
      opt.label = `${item.rssi} dBm`;
      list.appendChild(opt);
      const row = document.createElement('div');
      row.className = 'result';
      row.innerHTML = `<div><strong>${esc(item.ssid)}</strong><div class="muted">${item.rssi} dBm${item.secure ? ' | secured' : ' | open'}</div></div>`;
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'btn-pick';
      btn.textContent = 'Use';
      btn.onclick = () => { document.getElementById('wifi_ssid').value = item.ssid; };
      row.appendChild(btn);
      box.appendChild(row);
    });
  } catch (err) {
    box.innerHTML = `<div class="result"><span>Wi-Fi scan failed: ${esc(err.message)}</span></div>`;
  }
}

async function scanBle() {
  const box = document.getElementById('ble-results');
  box.innerHTML = '<div class="result"><span>Scanning BLE...</span></div>';
  try {
    const res = await fetch('/api/scan/ble', { cache: 'no-store' });
    const data = await res.json();
    const items = data.ble || [];
    const list = document.getElementById('ble-list');
    list.innerHTML = '';
    if (!items.length) {
      box.innerHTML = '<div class="result"><span>No BLE devices found.</span></div>';
      return;
    }
    box.innerHTML = '';
    items.forEach((item) => {
      const opt = document.createElement('option');
      opt.value = item.address;
      opt.label = item.name || item.address;
      list.appendChild(opt);
      const row = document.createElement('div');
      row.className = 'result';
      row.innerHTML = `<div><strong>${esc(item.name || 'Unnamed device')}</strong><div class="muted">${esc(item.address)} | ${item.rssi} dBm</div></div>`;
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'btn-pick';
      btn.textContent = 'Use';
      btn.onclick = () => {
        document.getElementById('bt_address').value = item.address;
        document.getElementById('bt_name').value = item.name || '';
      };
      row.appendChild(btn);
      box.appendChild(row);
    });
  } catch (err) {
    box.innerHTML = `<div class="result"><span>BLE scan failed: ${esc(err.message)}</span></div>`;
  }
}
</script>
</div>
</body>
</html>
)HTML");

  return html;
}

void ensureBleReady() {
  if (bleReady) {
    return;
  }

  NimBLEDevice::init("");
  bleReady = true;
}

bool connectToConfiguredWifi(uint32_t timeoutMs) {
  if (config.wifiSsid.isEmpty()) {
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
  lastWifiReconnectAttempt = millis();

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
  }

  return WiFi.status() == WL_CONNECTED;
}

void maintainWiFi() {
  if (configPortalActive || config.wifiSsid.isEmpty()) {
    return;
  }
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  if (millis() - lastWifiReconnectAttempt < kWifiReconnectIntervalMs) {
    return;
  }

  Serial.println("Wi-Fi disconnected, retrying...");
  WiFi.disconnect(false, false);
  WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
  lastWifiReconnectAttempt = millis();
}

void startConfigPortal(const String& reason) {
  portalReason = reason;
  configPortalActive = true;

  uint64_t chipId = ESP.getEfuseMac();
  char ssidBuffer[32];
  snprintf(ssidBuffer, sizeof(ssidBuffer), "OBD2-Setup-%04X", static_cast<unsigned>(chipId & 0xFFFF));
  portalApSsid = ssidBuffer;

  WiFi.disconnect(false, false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(portalApSsid.c_str(), kPortalPassword);

  dnsServer.start(53, "*", WiFi.softAPIP());

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/save", HTTP_POST, handleSave);
  webServer.on("/api/scan/wifi", HTTP_GET, handleWifiScanApi);
  webServer.on("/api/scan/ble", HTTP_GET, handleBleScanApi);
  webServer.onNotFound([]() {
    webServer.sendHeader("Location", "/", true);
    webServer.send(302, "text/plain", "");
  });
  webServer.begin();

  if (!config.wifiSsid.isEmpty()) {
    WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
    lastWifiReconnectAttempt = millis();
    Serial.printf("Config portal active, background Wi-Fi retry enabled for SSID: %s\n", config.wifiSsid.c_str());
  }

  Serial.printf("Config portal ready: SSID=%s, IP=%s\n", portalApSsid.c_str(), WiFi.softAPIP().toString().c_str());
}

void stopConfigPortal() {
  if (!configPortalActive) {
    return;
  }

  dnsServer.stop();
  webServer.stop();
  WiFi.softAPdisconnect(true);
  configPortalActive = false;
  portalReason = "";

  Serial.printf("Config portal stopped, Wi-Fi connected: %s\n", WiFi.localIP().toString().c_str());
}

void handleRoot() {
  webServer.send(200, "text/html; charset=utf-8", buildConfigPage());
}

void handleSave() {
  config.wifiSsid = trimCopy(webServer.arg("wifi_ssid"));
  config.wifiPassword = webServer.arg("wifi_password");
  config.btAddress = trimCopy(webServer.arg("bt_address"));
  config.btName = trimCopy(webServer.arg("bt_name"));
  config.webhookUrl = trimCopy(webServer.arg("webhook_url"));
  config.webhookToken = trimCopy(webServer.arg("webhook_token"));
  config.defaultRequestHeader = normalizeHex(webServer.arg("default_request_header"));
  if (config.defaultRequestHeader.isEmpty()) {
    config.defaultRequestHeader = kDefaultRequestHeader;
  }
  config.gpsEnabled = webServer.hasArg("gps_enabled");

  int slowEvery = webServer.arg("slow_every").toInt();
  if (slowEvery < 1) {
    slowEvery = 1;
  }
  if (slowEvery > 20) {
    slowEvery = 20;
  }
  config.slowEvery = static_cast<uint8_t>(slowEvery);

  for (size_t i = 0; i < BUILTIN_PID_COUNT; ++i) {
    config.builtin[i].enabled = webServer.hasArg("builtin_" + String(kBuiltinPidDefs[i].id) + "_enabled");
  }

  std::vector<CustomPidConfig> customPids;
  customPids.reserve(MAX_CUSTOM_PIDS);
  for (size_t i = 0; i < MAX_CUSTOM_PIDS; ++i) {
    CustomPidConfig pid;
    pid.enabled = webServer.hasArg("custom_" + String(i) + "_enabled");
    pid.label = trimCopy(webServer.arg("custom_" + String(i) + "_label"));
    pid.key = normalizeKey(webServer.arg("custom_" + String(i) + "_key"));
    pid.service = toUpperTrimmed(webServer.arg("custom_" + String(i) + "_service"));
    pid.pid = toUpperTrimmed(webServer.arg("custom_" + String(i) + "_pid"));
    pid.requestHeader = normalizeHex(webServer.arg("custom_" + String(i) + "_header"));
    pid.responseFilter = normalizeHex(webServer.arg("custom_" + String(i) + "_response_filter"));
    pid.schedule = scheduleFromValue(webServer.arg("custom_" + String(i) + "_schedule"));
    pid.parser = parserFromValue(webServer.arg("custom_" + String(i) + "_parser"));
    pid.formula = trimCopy(webServer.arg("custom_" + String(i) + "_formula"));
    int decimals = webServer.arg("custom_" + String(i) + "_decimals").toInt();
    if (decimals < 0) {
      decimals = 0;
    }
    if (decimals > 6) {
      decimals = 6;
    }
    pid.decimals = static_cast<uint8_t>(decimals);
    int frames = webServer.arg("custom_" + String(i) + "_frames").toInt();
    if (frames < 0) {
      frames = 0;
    }
    if (frames > 15) {
      frames = 15;
    }
    pid.expectedFrames = static_cast<uint8_t>(frames);

    if (pid.label.isEmpty() && pid.key.isEmpty() && pid.service.isEmpty() && pid.pid.isEmpty() && pid.formula.isEmpty()) {
      continue;
    }

    if (pid.label.isEmpty()) {
      pid.label = "Custom PID " + String(customPids.size() + 1);
    }
    if (pid.key.isEmpty()) {
      pid.key = normalizeKey(pid.label);
    }
    if (pid.parser == PARSER_ATRV && pid.service.isEmpty()) {
      pid.service = "AT";
      pid.pid = "RV";
    }
    if (pid.responseFilter.isEmpty() && parserNeedsRouting(pid.parser)) {
      String effectiveHeader = pid.requestHeader.isEmpty() ? config.defaultRequestHeader : pid.requestHeader;
      pid.responseFilter = deriveResponseFilter(effectiveHeader);
    }
    customPids.push_back(pid);
  }

  config.customPids = customPids;
  resetRuntimeState();
  saveConfig();

  webServer.send(
      200,
      "text/html; charset=utf-8",
      "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>body{font-family:Segoe UI,Arial,sans-serif;background:#f5f2eb;color:#1f2933;padding:24px}div{max-width:560px;margin:48px auto;background:#fff;border-radius:16px;padding:24px;border:1px solid #ddd}</style>"
      "</head><body><div><h1>Configuration saved</h1><p>The ESP32 will restart in a moment and try the new settings.</p></div></body></html>");
  delay(1200);
  ESP.restart();
}

void handleWifiScanApi() {
  scanWifiNetworks();

  DynamicJsonDocument doc(2048 + scannedWifi.size() * 96);
  JsonArray arr = doc.createNestedArray("wifi");
  for (const WifiScanResult& item : scannedWifi) {
    JsonObject obj = arr.createNestedObject();
    obj["ssid"] = item.ssid;
    obj["rssi"] = item.rssi;
    obj["secure"] = item.secure;
  }

  String payload;
  serializeJson(doc, payload);
  webServer.send(200, "application/json", payload);
}

void handleBleScanApi() {
  scanBleDevices();

  DynamicJsonDocument doc(2048 + scannedBle.size() * 96);
  JsonArray arr = doc.createNestedArray("ble");
  for (const BleScanResult& item : scannedBle) {
    JsonObject obj = arr.createNestedObject();
    obj["name"] = item.name;
    obj["address"] = item.address;
    obj["rssi"] = item.rssi;
  }

  String payload;
  serializeJson(doc, payload);
  webServer.send(200, "application/json", payload);
}

void scanWifiNetworks() {
  scannedWifi.clear();

  int found = WiFi.scanNetworks(false, true);
  if (found <= 0) {
    WiFi.scanDelete();
    return;
  }

  for (int i = 0; i < found; ++i) {
    String ssid = WiFi.SSID(i);
    if (ssid.isEmpty()) {
      continue;
    }

    int existing = -1;
    for (size_t j = 0; j < scannedWifi.size(); ++j) {
      if (scannedWifi[j].ssid == ssid) {
        existing = static_cast<int>(j);
        break;
      }
    }

    WifiScanResult item;
    item.ssid = ssid;
    item.rssi = WiFi.RSSI(i);
    item.secure = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;

    if (existing >= 0) {
      if (item.rssi > scannedWifi[existing].rssi) {
        scannedWifi[existing] = item;
      }
    } else {
      scannedWifi.push_back(item);
    }
  }

  std::sort(scannedWifi.begin(), scannedWifi.end(), [](const WifiScanResult& a, const WifiScanResult& b) {
    return a.rssi > b.rssi;
  });

  WiFi.scanDelete();
}

void scanBleDevices() {
  scannedBle.clear();
  ensureBleReady();

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(15);
  scan->clearResults();

  NimBLEScanResults results = scan->getResults(kBleScanSeconds, false);
  for (int i = 0; i < results.getCount(); ++i) {
    const NimBLEAdvertisedDevice* device = results.getDevice(i);
    if (device == nullptr) {
      continue;
    }

    BleScanResult item;
    item.name = String(device->getName().c_str());
    item.address = String(device->getAddress().toString().c_str());
    item.rssi = device->getRSSI();
    scannedBle.push_back(item);
  }

  std::sort(scannedBle.begin(), scannedBle.end(), [](const BleScanResult& a, const BleScanResult& b) {
    return a.rssi > b.rssi;
  });

  scan->clearResults();
}

void disconnectBleSession() {
  pWriteChar = nullptr;
  if (pClient != nullptr && pClient->isConnected()) {
    pClient->disconnect();
  }
  resetRuntimeState();
}

bool ensureBleConnection() {
  if (config.btAddress.isEmpty()) {
    Serial.println("No BLE address configured.");
    return false;
  }

  ensureBleReady();

  if (pClient == nullptr) {
    pClient = NimBLEDevice::createClient();
  }
  if (pClient->isConnected() && pWriteChar != nullptr) {
    return true;
  }

  Serial.printf("Connecting BLE to %s...\n", config.btAddress.c_str());
  NimBLEAddress address(std::string(config.btAddress.c_str()), 0);
  if (!pClient->connect(address)) {
    Serial.println("BLE connect failed.");
    pWriteChar = nullptr;
    return false;
  }

  auto* pService = pClient->getService("18F0");
  if (pService == nullptr) {
    Serial.println("BLE service 18F0 not found.");
    disconnectBleSession();
    return false;
  }

  pWriteChar = pService->getCharacteristic("2AF1");
  auto* pNotify = pService->getCharacteristic("2AF0");
  if (pWriteChar == nullptr || pNotify == nullptr) {
    Serial.println("BLE characteristics not found.");
    disconnectBleSession();
    return false;
  }

  if (!pNotify->subscribe(true, notifyCallback)) {
    Serial.println("BLE notify subscribe failed.");
    disconnectBleSession();
    return false;
  }

  const char* initCommands[] = {"ATZ", "ATE0", "ATL0", "ATS0", "ATH1", "ATSP6"};
  for (const char* command : initCommands) {
    if (!sendCommand(command, 3000)) {
      Serial.printf("Init command failed: %s\n", command);
    }
    delay(150);
  }

  resetRuntimeState();
  Serial.println("BLE connected and initialized.");
  return true;
}

bool ensurePidRoute(const QuerySpec& query) {
  if (!parserNeedsRouting(query.parser)) {
    return true;
  }

  String requestHeader = normalizeHex(query.requestHeader);
  if (requestHeader.isEmpty()) {
    requestHeader = config.defaultRequestHeader;
  }
  String responseFilter = normalizeHex(query.responseFilter);
  if (responseFilter.isEmpty()) {
    responseFilter = deriveResponseFilter(requestHeader);
  }

  if (!requestHeader.isEmpty() && activeRequestHeader != requestHeader) {
    if (!sendCommand("ATSH" + requestHeader, 2500)) {
      return false;
    }
    activeRequestHeader = requestHeader;
  }

  if (!responseFilter.isEmpty() && activeResponseFilter != responseFilter) {
    if (!sendCommand("ATCRA" + responseFilter, 2500)) {
      return false;
    }
    activeResponseFilter = responseFilter;
  }

  return true;
}

void readGpsWindow(uint32_t timeoutMs) {
  if (!config.gpsEnabled) {
    gpsState.sats = 0;
    return;
  }

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (SerialGPS.available() > 0) {
      gps.encode(SerialGPS.read());
    }
  }

  if (gps.location.isValid()) {
    gpsState.lat = gps.location.lat();
    gpsState.lon = gps.location.lng();
    gpsState.sats = gps.satellites.value();
  } else {
    gpsState.sats = 0;
  }
}

void processGpsBackground() {
  if (!config.gpsEnabled) {
    return;
  }
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
}

bool shouldRunSchedule(uint8_t schedule, bool onceDone, bool runSlowCycle) {
  switch (schedule) {
    case SCHEDULE_SLOW:
      return runSlowCycle;
    case SCHEDULE_ONCE:
      return !onceDone;
    case SCHEDULE_FAST:
    default:
      return true;
  }
}

bool runTelemetryCycle() {
  bool attempted = false;
  bool anySuccess = false;
  bool runSlowCycle = (cycleCounter % static_cast<unsigned long>(std::max<uint8_t>(1, config.slowEvery)) == 0);

  for (size_t i = 0; i < BUILTIN_PID_COUNT; ++i) {
    if (!config.builtin[i].enabled) {
      continue;
    }
    if (!shouldRunSchedule(kBuiltinPidDefs[i].schedule, config.builtin[i].onceDone, runSlowCycle)) {
      continue;
    }

    QuerySpec query;
    query.label = kBuiltinPidDefs[i].label;
    query.key = kBuiltinPidDefs[i].key;
    query.service = kBuiltinPidDefs[i].service;
    query.pid = kBuiltinPidDefs[i].pid;
    query.requestHeader = kBuiltinPidDefs[i].requestHeader;
    query.responseFilter = "";
    query.formula = kBuiltinPidDefs[i].formula;
    query.schedule = kBuiltinPidDefs[i].schedule;
    query.parser = kBuiltinPidDefs[i].parser;
    query.decimals = kBuiltinPidDefs[i].decimals;
    query.expectedFrames = kBuiltinPidDefs[i].expectedFrames;
    query.useAsState = kBuiltinPidDefs[i].useAsState;

    attempted = true;
    if (executeQuery(query, config.builtin[i].value)) {
      anySuccess = true;
      if (query.schedule == SCHEDULE_ONCE) {
        config.builtin[i].onceDone = true;
      }
    } else {
      config.builtin[i].value = TelemetryValue();
      if (query.schedule == SCHEDULE_ONCE) {
        config.builtin[i].onceDone = false;
      }
    }
  }

  for (CustomPidConfig& pid : config.customPids) {
    if (!pid.enabled) {
      continue;
    }
    if (!shouldRunSchedule(pid.schedule, pid.onceDone, runSlowCycle)) {
      continue;
    }

    QuerySpec query;
    query.label = pid.label;
    query.key = pid.key;
    query.service = pid.service;
    query.pid = pid.pid;
    query.requestHeader = pid.requestHeader;
    query.responseFilter = pid.responseFilter;
    query.formula = pid.formula;
    query.schedule = pid.schedule;
    query.parser = pid.parser;
    query.decimals = pid.decimals;
    query.expectedFrames = pid.expectedFrames;

    attempted = true;
    if (executeQuery(query, pid.value)) {
      anySuccess = true;
      if (query.schedule == SCHEDULE_ONCE) {
        pid.onceDone = true;
      }
    } else {
      pid.value = TelemetryValue();
      if (query.schedule == SCHEDULE_ONCE) {
        pid.onceDone = false;
      }
    }
  }

  if (!attempted) {
    return true;
  }
  return anySuccess;
}

bool executeQuery(const QuerySpec& query, TelemetryValue& value) {
  value = TelemetryValue();

  if (pClient == nullptr || !pClient->isConnected()) {
    return false;
  }

  if (!ensurePidRoute(query)) {
    Serial.printf("Routing failed for %s\n", query.label.c_str());
    return false;
  }

  String command = buildCommand(query.service, query.pid);
  if (command.isEmpty()) {
    return false;
  }

  int timeout = (query.parser == PARSER_VIN || query.expectedFrames > 1) ? 5000 : 3000;
  if (!sendCommand(command, timeout)) {
    return false;
  }

  return parseValueFromResponse(query, lastResponse, value);
}

bool parseValueFromResponse(const QuerySpec& query, const String& rawResponse, TelemetryValue& value) {
  switch (query.parser) {
    case PARSER_ATRV:
      return parseAtrvValue(rawResponse, value);
    case PARSER_VIN:
      return parseVinValue(query, rawResponse, value);
    case PARSER_ASCII:
      return parseAsciiValue(query, rawResponse, value);
    case PARSER_HEX_UINT:
      return parseHexUIntValue(query, rawResponse, value);
    case PARSER_RAW_HEX:
      return parseRawHexValue(query, rawResponse, value);
    case PARSER_FORMULA:
    default:
      return parseFormulaValue(query, rawResponse, value);
  }
}

ObdPayload extractObdPayload(const String& rawResponse, const String& responseFilter, uint8_t expectedFrames) {
  ObdPayload payload;
  payload.cleaned = rawResponse;
  cleanResponse(payload.cleaned);

  String header = normalizeHex(responseFilter);
  if (header.isEmpty()) {
    return payload;
  }

  std::vector<int> frameStarts;
  int searchPos = 0;
  while (true) {
    int idx = payload.cleaned.indexOf(header, searchPos);
    if (idx == -1) {
      break;
    }
    int pciPos = idx + static_cast<int>(header.length());
    if (pciPos + 1 < payload.cleaned.length()) {
      char typeNibble = payload.cleaned.charAt(pciPos);
      if ((typeNibble == '0' || typeNibble == '1' || typeNibble == '2') &&
          isxdigit(static_cast<unsigned char>(payload.cleaned.charAt(pciPos + 1)))) {
        frameStarts.push_back(idx);
      }
    }
    searchPos = idx + 1;
  }

  uint16_t totalLength = 0;
  for (size_t i = 0; i < frameStarts.size(); ++i) {
    int frameStart = frameStarts[i];
    int frameEnd = (i + 1 < frameStarts.size()) ? frameStarts[i + 1] : payload.cleaned.length();
    int pciPos = frameStart + static_cast<int>(header.length());
    if (pciPos + 1 >= frameEnd) {
      continue;
    }

    char typeNibble = payload.cleaned.charAt(pciPos);
    int dataStart = 0;
    int dataBytes = 0;

    if (typeNibble == '0') {
      dataStart = pciPos + 2;
      int available = std::max(0, (frameEnd - dataStart) / 2);
      dataBytes = strtol(payload.cleaned.substring(pciPos, pciPos + 2).c_str(), nullptr, 16);
      dataBytes = std::min(dataBytes, available);
    } else if (typeNibble == '1') {
      dataStart = pciPos + 4;
      int available = std::max(0, (frameEnd - dataStart) / 2);
      totalLength = strtol(payload.cleaned.substring(pciPos, pciPos + 4).c_str(), nullptr, 16) & 0x0FFF;
      dataBytes = available;
    } else if (typeNibble == '2') {
      dataStart = pciPos + 2;
      dataBytes = std::max(0, (frameEnd - dataStart) / 2);
    } else {
      continue;
    }

    payload.frames++;
    for (int byteIndex = 0; byteIndex < dataBytes && payload.length < MAX_PAYLOAD_BYTES; ++byteIndex) {
      int pos = dataStart + byteIndex * 2;
      payload.bytes[payload.length++] = static_cast<uint8_t>(strtol(payload.cleaned.substring(pos, pos + 2).c_str(), nullptr, 16));
    }

    if (totalLength > 0 && payload.length >= totalLength) {
      payload.length = std::min(static_cast<size_t>(totalLength), static_cast<size_t>(MAX_PAYLOAD_BYTES));
      break;
    }
  }

  payload.valid = payload.length > 0 && (expectedFrames == 0 || payload.frames >= expectedFrames);
  return payload;
}

bool extractDataBytesAfterPrefix(const ObdPayload& payload, const String& prefix, uint8_t* dataBytes, size_t& dataLength) {
  dataLength = 0;
  if (!payload.valid) {
    return false;
  }

  String responsePrefix = normalizeHex(prefix);
  size_t prefixBytes = responsePrefix.length() / 2;
  if (responsePrefix.length() % 2 != 0 || prefixBytes > payload.length) {
    return false;
  }

  for (size_t i = 0; i < prefixBytes; ++i) {
    uint8_t expected = static_cast<uint8_t>(strtol(responsePrefix.substring(i * 2, i * 2 + 2).c_str(), nullptr, 16));
    if (payload.bytes[i] != expected) {
      return false;
    }
  }

  size_t remaining = payload.length - prefixBytes;
  for (size_t i = 0; i < remaining; ++i) {
    dataBytes[i] = payload.bytes[prefixBytes + i];
  }
  dataLength = remaining;
  return true;
}

bool parseAtrvValue(const String& rawResponse, TelemetryValue& value) {
  String cleaned = rawResponse;
  cleaned.replace("\r", "");
  cleaned.replace("\n", "");
  cleaned.replace(">", "");
  cleaned.trim();
  cleaned.replace("V", "");
  cleaned.trim();

  float volts = cleaned.toFloat();
  if (volts <= 0.0f) {
    return false;
  }
  value.valid = true;
  value.numeric = true;
  value.number = volts;
  value.text = String(volts, 2);
  value.rawHex = cleaned;
  return true;
}

bool parseVinValue(const QuerySpec& query, const String& rawResponse, TelemetryValue& value) {
  String requestHeader = normalizeHex(query.requestHeader);
  if (requestHeader.isEmpty()) {
    requestHeader = config.defaultRequestHeader;
  }
  String responseFilter = normalizeHex(query.responseFilter);
  if (responseFilter.isEmpty()) {
    responseFilter = deriveResponseFilter(requestHeader);
  }

  ObdPayload payload = extractObdPayload(rawResponse, responseFilter, query.expectedFrames);
  uint8_t dataBytes[MAX_PAYLOAD_BYTES] = {0};
  size_t dataLength = 0;
  if (!extractDataBytesAfterPrefix(payload, "4902", dataBytes, dataLength)) {
    return false;
  }
  if (dataLength < 4) {
    return false;
  }

  size_t startIndex = 0;
  if (dataBytes[0] <= 0x20 && dataLength >= 18) {
    startIndex = 1;
  }

  String vin;
  vin.reserve(17);
  for (size_t i = startIndex; i < dataLength && vin.length() < 17; ++i) {
    char ch = static_cast<char>(dataBytes[i]);
    if (isalnum(static_cast<unsigned char>(ch))) {
      vin += ch;
    }
  }

  if (vin.length() != 17) {
    return false;
  }

  value.valid = true;
  value.numeric = false;
  value.text = vin;
  value.frames = payload.frames;
  return true;
}

bool parseFormulaValue(const QuerySpec& query, const String& rawResponse, TelemetryValue& value) {
  String requestHeader = normalizeHex(query.requestHeader);
  if (requestHeader.isEmpty()) {
    requestHeader = config.defaultRequestHeader;
  }
  String responseFilter = normalizeHex(query.responseFilter);
  if (responseFilter.isEmpty()) {
    responseFilter = deriveResponseFilter(requestHeader);
  }

  ObdPayload payload = extractObdPayload(rawResponse, responseFilter, query.expectedFrames);
  uint8_t dataBytes[MAX_PAYLOAD_BYTES] = {0};
  size_t dataLength = 0;
  if (!extractDataBytesAfterPrefix(payload, deriveResponsePrefix(query.service, query.pid), dataBytes, dataLength)) {
    return false;
  }
  if (dataLength == 0) {
    return false;
  }

  String formula = trimCopy(query.formula);
  if (formula.isEmpty()) {
    return false;
  }

  FormulaEvaluator evaluator(formula, dataBytes, std::min(dataLength, static_cast<size_t>(MAX_FORMULA_BYTES)));
  double result = 0.0;
  if (!evaluator.evaluate(result)) {
    return false;
  }

  value.valid = true;
  value.numeric = true;
  value.number = roundToDecimals(result, query.decimals);
  value.text = String(static_cast<double>(value.number), static_cast<unsigned int>(query.decimals));
  value.frames = payload.frames;
  return true;
}

bool parseAsciiValue(const QuerySpec& query, const String& rawResponse, TelemetryValue& value) {
  String requestHeader = normalizeHex(query.requestHeader);
  if (requestHeader.isEmpty()) {
    requestHeader = config.defaultRequestHeader;
  }
  String responseFilter = normalizeHex(query.responseFilter);
  if (responseFilter.isEmpty()) {
    responseFilter = deriveResponseFilter(requestHeader);
  }

  ObdPayload payload = extractObdPayload(rawResponse, responseFilter, query.expectedFrames);
  uint8_t dataBytes[MAX_PAYLOAD_BYTES] = {0};
  size_t dataLength = 0;
  if (!extractDataBytesAfterPrefix(payload, deriveResponsePrefix(query.service, query.pid), dataBytes, dataLength)) {
    return false;
  }

  String text;
  text.reserve(dataLength);
  for (size_t i = 0; i < dataLength; ++i) {
    char ch = static_cast<char>(dataBytes[i]);
    if (ch >= 32 && ch <= 126) {
      text += ch;
    }
  }
  text.trim();
  if (text.isEmpty()) {
    return false;
  }

  value.valid = true;
  value.numeric = false;
  value.text = text;
  value.frames = payload.frames;
  return true;
}

bool parseHexUIntValue(const QuerySpec& query, const String& rawResponse, TelemetryValue& value) {
  String requestHeader = normalizeHex(query.requestHeader);
  if (requestHeader.isEmpty()) {
    requestHeader = config.defaultRequestHeader;
  }
  String responseFilter = normalizeHex(query.responseFilter);
  if (responseFilter.isEmpty()) {
    responseFilter = deriveResponseFilter(requestHeader);
  }

  ObdPayload payload = extractObdPayload(rawResponse, responseFilter, query.expectedFrames);
  uint8_t dataBytes[MAX_PAYLOAD_BYTES] = {0};
  size_t dataLength = 0;
  if (!extractDataBytesAfterPrefix(payload, deriveResponsePrefix(query.service, query.pid), dataBytes, dataLength)) {
    return false;
  }
  if (dataLength == 0) {
    return false;
  }

  uint64_t number = 0;
  for (size_t i = 0; i < dataLength && i < 8; ++i) {
    number = (number << 8) | dataBytes[i];
  }

  value.valid = true;
  value.numeric = true;
  value.number = roundToDecimals(static_cast<double>(number), query.decimals);
  value.text = String(static_cast<double>(value.number), static_cast<unsigned int>(query.decimals));
  value.frames = payload.frames;
  return true;
}

bool parseRawHexValue(const QuerySpec& query, const String& rawResponse, TelemetryValue& value) {
  String requestHeader = normalizeHex(query.requestHeader);
  if (requestHeader.isEmpty()) {
    requestHeader = config.defaultRequestHeader;
  }
  String responseFilter = normalizeHex(query.responseFilter);
  if (responseFilter.isEmpty()) {
    responseFilter = deriveResponseFilter(requestHeader);
  }

  ObdPayload payload = extractObdPayload(rawResponse, responseFilter, query.expectedFrames);
  uint8_t dataBytes[MAX_PAYLOAD_BYTES] = {0};
  size_t dataLength = 0;
  if (!extractDataBytesAfterPrefix(payload, deriveResponsePrefix(query.service, query.pid), dataBytes, dataLength)) {
    return false;
  }
  if (dataLength == 0) {
    return false;
  }

  String hex;
  for (size_t i = 0; i < dataLength; ++i) {
    if (i > 0) {
      hex += ' ';
    }
    char buffer[3];
    snprintf(buffer, sizeof(buffer), "%02X", dataBytes[i]);
    hex += buffer;
  }

  value.valid = true;
  value.numeric = false;
  value.text = hex;
  value.rawHex = hex;
  value.frames = payload.frames;
  return true;
}

bool pushToWebhook() {
  if (WiFi.status() != WL_CONNECTED || config.webhookUrl.isEmpty()) {
    return false;
  }

  DynamicJsonDocument doc(12288);
  String stateValue = "unknown";

  for (size_t i = 0; i < BUILTIN_PID_COUNT; ++i) {
    if (kBuiltinPidDefs[i].useAsState && config.builtin[i].enabled && config.builtin[i].value.valid) {
      stateValue = config.builtin[i].value.text;
      break;
    }
  }
  doc["state"] = stateValue;

  JsonObject attr = doc.createNestedObject("attributes");
  attr["wifi_ssid"] = config.wifiSsid;
  attr["bt_address"] = config.btAddress;
  attr["gps_enabled"] = config.gpsEnabled;
  attr["default_request_header"] = config.defaultRequestHeader;
  attr["gps_sats"] = gpsState.sats;
  if (config.gpsEnabled && gpsState.sats > 0) {
    attr["latitude"] = gpsState.lat;
    attr["longitude"] = gpsState.lon;
  }

  JsonArray activePids = attr.createNestedArray("active_pids");
  for (size_t i = 0; i < BUILTIN_PID_COUNT; ++i) {
    if (config.builtin[i].enabled) {
      activePids.add(kBuiltinPidDefs[i].id);
    }
  }
  for (const CustomPidConfig& pid : config.customPids) {
    if (pid.enabled) {
      activePids.add(pid.key);
    }
  }

  for (size_t i = 0; i < BUILTIN_PID_COUNT; ++i) {
    const BuiltinPidDefinition& def = kBuiltinPidDefs[i];
    const TelemetryValue& value = config.builtin[i].value;
    if (!config.builtin[i].enabled || !value.valid) {
      continue;
    }
    if (value.numeric) {
      attr[def.key] = value.number;
    } else {
      attr[def.key] = value.text;
    }
  }

  for (const CustomPidConfig& pid : config.customPids) {
    if (!pid.enabled || !pid.value.valid || pid.key.isEmpty()) {
      continue;
    }
    if (pid.value.numeric) {
      attr[pid.key] = pid.value.number;
    } else {
      attr[pid.key] = pid.value.text;
    }
  }

  String payload;
  serializeJson(doc, payload);

  HTTPClient http;
  int code = -1;

  if (config.webhookUrl.startsWith("https://")) {
    WiFiClientSecure client;
    client.setInsecure();
    if (!http.begin(client, config.webhookUrl)) {
      return false;
    }
    if (!config.webhookToken.isEmpty()) {
      http.addHeader("Authorization", "Bearer " + config.webhookToken);
    }
    http.addHeader("Content-Type", "application/json");
    code = http.POST(payload);
    http.end();
  } else {
    WiFiClient client;
    if (!http.begin(client, config.webhookUrl)) {
      return false;
    }
    if (!config.webhookToken.isEmpty()) {
      http.addHeader("Authorization", "Bearer " + config.webhookToken);
    }
    http.addHeader("Content-Type", "application/json");
    code = http.POST(payload);
    http.end();
  }

  return code >= 200 && code < 300;
}

void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  (void) pChar;
  (void) isNotify;
  for (size_t i = 0; i < length; ++i) {
    lastResponse += static_cast<char>(pData[i]);
  }
  if (lastResponse.indexOf('>') != -1) {
    responseReceived = true;
  }
}

bool sendCommand(const String& cmd, int timeout) {
  if (pWriteChar == nullptr) {
    return false;
  }

  lastResponse = "";
  responseReceived = false;
  pWriteChar->writeValue(cmd + "\r", true);

  unsigned long start = millis();
  while (millis() - start < static_cast<unsigned long>(timeout)) {
    if (responseReceived) {
      return true;
    }
    delay(10);
  }
  return false;
}

void cleanResponse(String& response) {
  response.replace(" ", "");
  response.replace("\n", "");
  response.replace("\r", "");
  response.replace(">", "");
  response.toUpperCase();
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println();
  Serial.println("Booting OBD2 bridge...");

  loadConfig();

  if (config.gpsEnabled) {
    SerialGPS.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  }

  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);

  ensureBleReady();

  if (!connectToConfiguredWifi(kWifiStartupConnectTimeoutMs)) {
    String reason = config.wifiSsid.isEmpty()
                        ? "No Wi-Fi SSID is saved."
                        : "Could not connect to Wi-Fi '" + config.wifiSsid + "' within " + String(kWifiStartupConnectTimeoutMs / 1000) + " seconds.";
    startConfigPortal(reason);
    return;
  }

  Serial.printf("Wi-Fi connected: %s\n", WiFi.localIP().toString().c_str());
}

void loop() {
  if (configPortalActive) {
    dnsServer.processNextRequest();
    webServer.handleClient();

    if (!config.wifiSsid.isEmpty() && WiFi.status() != WL_CONNECTED &&
        millis() - lastWifiReconnectAttempt >= kWifiPortalRetryIntervalMs) {
      Serial.printf("Config portal mode: retrying Wi-Fi connection to %s...\n", config.wifiSsid.c_str());
      WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
      lastWifiReconnectAttempt = millis();
    }

    if (WiFi.status() == WL_CONNECTED) {
      stopConfigPortal();
    }

    delay(5);
    return;
  }

  unsigned long loopStart = millis();

  readGpsWindow(kGpsReadWindowMs);
  maintainWiFi();

  if (ensureBleConnection()) {
    if (!runTelemetryCycle()) {
      Serial.println("Telemetry cycle failed, disconnecting BLE session.");
      disconnectBleSession();
    } else if (WiFi.status() == WL_CONNECTED) {
      if (!pushToWebhook()) {
        Serial.println("Webhook push failed.");
      }
    }
  }

  cycleCounter++;

  while (millis() - loopStart < kLoopIntervalMs) {
    processGpsBackground();
    maintainWiFi();
    delay(200);
  }
}
