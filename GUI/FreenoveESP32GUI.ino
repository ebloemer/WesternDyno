#include "display.h"
#include <lvgl.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>

#include "dyno_protocol.h"

Display screen;

// =====================================================
// Screen size
// =====================================================
static const int SCREEN_W = 480; 
static const int SCREEN_H = 320;
static const int TOPBAR_H = 56;

// =====================================================
// GUI + BLE state
// =====================================================
static CommandPacket g_cmd;
static TelemetryPacket g_tel{};
static bool g_tel_valid = false;
static bool g_tel_dirty = false;
static bool g_connected = false;
static bool g_connecting = false;
static bool g_scan_active = false;
static uint32_t g_last_cmd_ms = 0;
static uint32_t g_last_scan_ms = 0;
static bool g_cmd_dirty = true;

static BLEAdvertisedDevice* g_found_device = nullptr;
static BLEClient* g_client = nullptr;
static BLERemoteCharacteristic* g_cmd_char = nullptr;
static BLERemoteCharacteristic* g_tel_char = nullptr;
static BLEUUID g_service_uuid(DYNO_SERVICE_UUID);
static BLEUUID g_cmd_uuid(DYNO_CMD_UUID);
static BLEUUID g_tel_uuid(DYNO_TEL_UUID);

// =====================================================
// Screens
// =====================================================
static lv_obj_t* scr_home = nullptr;
static lv_obj_t* scr_mode_select = nullptr;
static lv_obj_t* scr_manual = nullptr;
static lv_obj_t* scr_torque = nullptr;
static lv_obj_t* scr_set_torque = nullptr;
static lv_obj_t* scr_rpm = nullptr;
static lv_obj_t* scr_set_rpm = nullptr;
static lv_obj_t* scr_variable_ratio = nullptr;
static lv_obj_t* scr_live_menu = nullptr;
static lv_obj_t* scr_live_static = nullptr;
static lv_obj_t* scr_live_dynamic = nullptr;
static lv_obj_t* scr_live_chart = nullptr;

// =====================================================
// Widgets
// =====================================================
static lv_obj_t* rpm_ta = nullptr;
static lv_obj_t* rpm_kb = nullptr;
static lv_obj_t* tq_ta = nullptr;
static lv_obj_t* tq_kb = nullptr;

static lv_obj_t* manual_flow_slider = nullptr;
static lv_obj_t* manual_pressure_slider = nullptr;

static lv_obj_t* chart = nullptr;
static lv_chart_series_t* series_a = nullptr;
static lv_chart_series_t* series_b = nullptr;

static lv_obj_t* lbl_link_home = nullptr;
static lv_obj_t* lbl_link_mode = nullptr;
static lv_obj_t* lbl_link_manual = nullptr;
static lv_obj_t* lbl_link_torque = nullptr;
static lv_obj_t* lbl_link_rpm = nullptr;
static lv_obj_t* lbl_link_variable = nullptr;
static lv_obj_t* lbl_link_live = nullptr;
static lv_obj_t* lbl_link_live_static = nullptr;
static lv_obj_t* lbl_link_live_dynamic = nullptr;
static lv_obj_t* lbl_link_chart = nullptr;

static lv_obj_t* lbl_manual_engine = nullptr;
static lv_obj_t* lbl_manual_pump = nullptr;
static lv_obj_t* lbl_manual_flow = nullptr;
static lv_obj_t* lbl_manual_pressure = nullptr;
static lv_obj_t* lbl_manual_throttle = nullptr;

static lv_obj_t* lbl_torque_actual = nullptr;
static lv_obj_t* lbl_torque_target = nullptr;
static lv_obj_t* lbl_torque_engine = nullptr;
static lv_obj_t* lbl_torque_pressure = nullptr;
static lv_obj_t* lbl_torque_scale = nullptr;

static lv_obj_t* lbl_rpm_actual = nullptr;
static lv_obj_t* lbl_rpm_target = nullptr;
static lv_obj_t* lbl_rpm_engine = nullptr;
static lv_obj_t* lbl_rpm_flow = nullptr;
static lv_obj_t* lbl_rpm_throttle = nullptr;

static lv_obj_t* lbl_variable_status = nullptr;

static lv_obj_t* lbl_live_mode = nullptr;
static lv_obj_t* lbl_live_engine = nullptr;
static lv_obj_t* lbl_live_pump = nullptr;
static lv_obj_t* lbl_live_torque = nullptr;
static lv_obj_t* lbl_live_power = nullptr;
static lv_obj_t* lbl_live_flow = nullptr;
static lv_obj_t* lbl_live_pressure = nullptr;
static lv_obj_t* lbl_live_throttle = nullptr;

static lv_obj_t* lbl_chart_title = nullptr;
static lv_obj_t* lbl_chart_subtitle = nullptr;

// =====================================================
// Chart mode
// =====================================================
enum ChartView {
  CHART_ENGINE_RPM = 0,
  CHART_PUMP_RPM,
  CHART_TORQUE,
  CHART_POWER
};

static ChartView g_chart_view = CHART_ENGINE_RPM;

// =====================================================
// Helpers
// =====================================================
static void go_to(lv_obj_t* s) {
  if (s) lv_scr_load(s);
}

static const char* dyno_mode_str(uint8_t mode) {
  switch (mode) {
    case MODE_MANUAL: return "MANUAL";
    case MODE_TORQUE: return "TORQUE";
    case MODE_RPM:    return "RPM";
    case MODE_CVT:    return "VARIABLE RATIO";
    default:          return "UNKNOWN";
  }
}

static lv_obj_t* make_root(lv_obj_t* parent) {
  lv_obj_set_style_bg_color(parent, lv_color_hex(0x101418), 0);
  lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_all(parent, 0, 0);
  return parent;
}

static lv_obj_t* make_topbar(lv_obj_t* root, bool show_back, lv_event_cb_t back_cb, const char* title, lv_event_cb_t right_cb, const char* right_text) {
  lv_obj_t* top = lv_obj_create(root);
  lv_obj_set_pos(top, 0, 0);
  lv_obj_set_size(top, SCREEN_W, TOPBAR_H);
  lv_obj_set_style_radius(top, 0, 0);
  lv_obj_set_style_border_width(top, 0, 0);
  lv_obj_set_style_pad_all(top, 8, 0);
  lv_obj_set_style_bg_color(top, lv_color_hex(0x1E293B), 0);
  lv_obj_clear_flag(top, LV_OBJ_FLAG_SCROLLABLE);

  if (show_back) {
    lv_obj_t* btn = lv_btn_create(top);
    lv_obj_set_size(btn, 70, 36);
    lv_obj_align(btn, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_event_cb(btn, back_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "Back");
    lv_obj_center(lbl);
  }

  lv_obj_t* btn = lv_btn_create(top);
  lv_obj_set_size(btn, 90, 36);
  lv_obj_align(btn, LV_ALIGN_RIGHT_MID, 0, 0);
  lv_obj_add_event_cb(btn, right_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl = lv_label_create(btn);
  lv_label_set_text(lbl, right_text);
  lv_obj_center(lbl);

  lv_obj_t* title_lbl = lv_label_create(top);
  lv_label_set_text(title_lbl, title);
  lv_obj_set_style_text_color(title_lbl, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(title_lbl, LV_ALIGN_CENTER, 0, 0);

  lv_obj_move_foreground(top);
  return top;
}

static lv_obj_t* make_body(lv_obj_t* root, bool scrollable = false) {
  lv_obj_t* body = lv_obj_create(root);
  lv_obj_set_pos(body, 0, TOPBAR_H);
  lv_obj_set_size(body, SCREEN_W, SCREEN_H - TOPBAR_H);

  lv_obj_set_style_pad_top(body, 12, 0);
  lv_obj_set_style_pad_bottom(body, 12, 0);
  lv_obj_set_style_pad_left(body, 12, 0);
  lv_obj_set_style_pad_right(body, 12, 0);
  lv_obj_set_style_border_width(body, 0, 0);
  lv_obj_set_style_radius(body, 0, 0);
  lv_obj_set_style_bg_color(body, lv_color_hex(0x101418), 0);

  if (scrollable) {
    lv_obj_set_scroll_dir(body, LV_DIR_VER);
  } else {
    lv_obj_clear_flag(body, LV_OBJ_FLAG_SCROLLABLE);
  }

  lv_obj_set_flex_flow(body, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(body, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_set_style_pad_row(body, 10, 0);

  return body;
}

static lv_obj_t* make_button_list(lv_obj_t* parent) {
  lv_obj_t* list = lv_obj_create(parent);
  lv_obj_set_width(list, lv_pct(100));
  lv_obj_set_flex_grow(list, 1);
  lv_obj_set_style_border_width(list, 0, 0);
  lv_obj_set_style_bg_color(list, lv_color_hex(0x101418), 0);
  lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_gap(list, 10, 0);
  return list;
}

static lv_obj_t* make_value_label(lv_obj_t* parent, const char* text) {
  lv_obj_t* lbl = lv_label_create(parent);
  lv_label_set_text(lbl, text);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0xE5E7EB), 0);
  return lbl;
}

static void set_link_label(lv_obj_t* lbl) {
  if (!lbl) return;

  char buf[96];
  if (g_connected) {
    snprintf(buf, sizeof(buf), "Link: Connected | Mode: %s", g_tel_valid ? dyno_mode_str(g_tel.mode) : "--");
  } else if (g_connecting) {
    snprintf(buf, sizeof(buf), "Link: Connecting...");
  } else {
    snprintf(buf, sizeof(buf), "Link: Scanning...");
  }
  lv_label_set_text(lbl, buf);
}

static void set_all_link_labels() {
  set_link_label(lbl_link_home);
  set_link_label(lbl_link_mode);
  set_link_label(lbl_link_manual);
  set_link_label(lbl_link_torque);
  set_link_label(lbl_link_rpm);
  set_link_label(lbl_link_variable);
  set_link_label(lbl_link_live);
  set_link_label(lbl_link_live_static);
  set_link_label(lbl_link_live_dynamic);
  set_link_label(lbl_link_chart);
}

static void dyno_set_mode(uint8_t mode) {
  g_cmd.mode = mode;
  g_cmd_dirty = true;
}

static void dyno_set_desired_rpm(float rpm) {
  g_cmd.targetRpm = rpm;
  g_cmd_dirty = true;
}

static void dyno_set_desired_torque(float tq) {
  g_cmd.targetTorque = tq;
  g_cmd_dirty = true;
}

static void dyno_set_manual_flow(uint16_t pct) {
  g_cmd.manualFlowPct = constrain((int)pct, 0, 100);
  g_cmd_dirty = true;
}

static void dyno_set_manual_pressure(uint16_t pct) {
  g_cmd.manualPressurePct = constrain((int)pct, 0, 100);
  g_cmd_dirty = true;
}

// =====================================================
// BLE callbacks
// =====================================================
class DynoClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* client) override {
    g_connected = true;
    g_connecting = false;
    set_all_link_labels();
  }

  void onDisconnect(BLEClient* client) override {
    g_connected = false;
    g_connecting = false;
    g_cmd_char = nullptr;
    g_tel_char = nullptr;
    set_all_link_labels();
  }
};

class DynoAdvertisedCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) override {
    if (!device.haveServiceUUID()) return;
    if (!device.isAdvertisingService(g_service_uuid)) return;

    if (g_found_device) {
      delete g_found_device;
      g_found_device = nullptr;
    }
    g_found_device = new BLEAdvertisedDevice(device);
    BLEDevice::getScan()->stop();
    g_scan_active = false;
  }
};

static void telemetry_notify_cb(BLERemoteCharacteristic* c, uint8_t* data, size_t length, bool isNotify) {
  if (length != sizeof(TelemetryPacket)) return;

  TelemetryPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (!dynoTelemetryPacketValid(pkt)) return;

  g_tel = pkt;
  g_tel_valid = true;
  g_tel_dirty = true;
}

static bool dyno_connect_to_found_device() {
  if (!g_found_device) return false;

  g_connecting = true;
  set_all_link_labels();

  if (g_client) {
    g_client->disconnect();
    delete g_client;
    g_client = nullptr;
  }

  g_client = BLEDevice::createClient();
  g_client->setClientCallbacks(new DynoClientCallbacks());

  if (!g_client->connect(g_found_device)) {
    g_connecting = false;
    set_all_link_labels();
    delete g_found_device;
    g_found_device = nullptr;
    return false;
  }

  BLERemoteService* svc = g_client->getService(g_service_uuid);
  if (!svc) {
    g_client->disconnect();
    g_connecting = false;
    set_all_link_labels();
    delete g_found_device;
    g_found_device = nullptr;
    return false;
  }

  g_cmd_char = svc->getCharacteristic(g_cmd_uuid);
  g_tel_char = svc->getCharacteristic(g_tel_uuid);

  if (!g_cmd_char || !g_tel_char) {
    g_client->disconnect();
    g_connecting = false;
    set_all_link_labels();
    delete g_found_device;
    g_found_device = nullptr;
    return false;
  }

  if (g_tel_char->canNotify()) {
    g_tel_char->registerForNotify(telemetry_notify_cb);
  }

  g_connected = true;
  g_connecting = false;
  g_cmd_dirty = true;
  set_all_link_labels();

  delete g_found_device;
  g_found_device = nullptr;
  return true;
}

static void dyno_ble_init() {
  dynoInitCommandPacket(g_cmd);

  BLEDevice::init("DynoGUI");
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new DynoAdvertisedCallbacks(), false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(80);
}

static void dyno_send_command_packet(bool force) {
  if (!g_connected || !g_cmd_char) return;

  uint32_t now = millis();
  if (!force && !g_cmd_dirty && (now - g_last_cmd_ms < 100)) return;

  g_cmd.magic = DYNO_PACKET_MAGIC;
  g_cmd.version = DYNO_PACKET_VERSION;
  g_cmd_char->writeValue((uint8_t*)&g_cmd, sizeof(g_cmd), false);
  g_last_cmd_ms = now;
  g_cmd_dirty = false;
}

static void dyno_ble_task() {
  if (!g_connected && !g_connecting && !g_scan_active && !g_found_device && (millis() - g_last_scan_ms > 1000)) {
    BLEDevice::getScan()->start(3, false);
    g_scan_active = true;
    g_last_scan_ms = millis();
    set_all_link_labels();
  }

  if (g_found_device && !g_connected && !g_connecting) {
    dyno_connect_to_found_device();
  }

  if (g_connected && g_client && !g_client->isConnected()) {
    g_connected = false;
    g_cmd_char = nullptr;
    g_tel_char = nullptr;
    set_all_link_labels();
  }

  dyno_send_command_packet(false);
}

// =====================================================
// Navigation callbacks
// =====================================================
static void cb_estop(lv_event_t*) {
  g_cmd.emergency = g_cmd.emergency ? 0 : 1;
  g_cmd_dirty = true;
  dyno_send_command_packet(true);
}

static void cb_back_home(lv_event_t*)           { go_to(scr_home); }
static void cb_open_mode_select(lv_event_t*)    { go_to(scr_mode_select); }
static void cb_open_live_menu(lv_event_t*)      { go_to(scr_live_menu); }

static void cb_open_manual(lv_event_t*)         { dyno_set_mode(MODE_MANUAL); go_to(scr_manual); }
static void cb_open_torque(lv_event_t*)         { dyno_set_mode(MODE_TORQUE); go_to(scr_torque); }
static void cb_open_rpm(lv_event_t*)            { dyno_set_mode(MODE_RPM);    go_to(scr_rpm); }
static void cb_open_variable_ratio(lv_event_t*) { dyno_set_mode(MODE_CVT);    go_to(scr_variable_ratio); }

static void cb_back_mode_select(lv_event_t*)    { go_to(scr_mode_select); }
static void cb_back_torque(lv_event_t*)         { go_to(scr_torque); }
static void cb_back_rpm(lv_event_t*)            { go_to(scr_rpm); }
static void cb_back_live_menu(lv_event_t*)      { go_to(scr_live_menu); }
static void cb_back_live_dynamic(lv_event_t*)   { go_to(scr_live_dynamic); }

static void cb_open_live_static(lv_event_t*)    { go_to(scr_live_static); }
static void cb_open_live_dynamic(lv_event_t*)   { go_to(scr_live_dynamic); }

static void cb_open_set_torque(lv_event_t*) {
  dyno_set_mode(MODE_TORQUE);
  if (tq_ta) lv_textarea_set_text(tq_ta, "");
  go_to(scr_set_torque);
}

static void cb_open_set_rpm(lv_event_t*) {
  dyno_set_mode(MODE_RPM);
  if (rpm_ta) lv_textarea_set_text(rpm_ta, "");
  go_to(scr_set_rpm);
}

static void cb_tq_kb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_READY) {
    dyno_set_desired_torque((float)atof(lv_textarea_get_text(tq_ta)));
    dyno_set_mode(MODE_TORQUE);
    dyno_send_command_packet(true);
    go_to(scr_torque);
  } else if (code == LV_EVENT_CANCEL) {
    go_to(scr_torque);
  }
}

static void cb_rpm_kb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_READY) {
    dyno_set_desired_rpm((float)atof(lv_textarea_get_text(rpm_ta)));
    dyno_set_mode(MODE_RPM);
    dyno_send_command_packet(true);
    go_to(scr_rpm);
  } else if (code == LV_EVENT_CANCEL) {
    go_to(scr_rpm);
  }
}

static void cb_manual_flow_slider(lv_event_t* e) {
  lv_obj_t* slider = lv_event_get_target(e);
  dyno_set_mode(MODE_MANUAL);
  dyno_set_manual_flow((uint16_t)lv_slider_get_value(slider));
  dyno_send_command_packet(false);
}

static void cb_manual_pressure_slider(lv_event_t* e) {
  lv_obj_t* slider = lv_event_get_target(e);
  dyno_set_mode(MODE_MANUAL);
  dyno_set_manual_pressure((uint16_t)lv_slider_get_value(slider));
  dyno_send_command_packet(false);
}

static void set_chart_view(ChartView view) {
  g_chart_view = view;

  if (!lbl_chart_title || !lbl_chart_subtitle || !chart || !series_a || !series_b) return;

  lv_chart_set_all_value(chart, series_a, 0);
  lv_chart_set_all_value(chart, series_b, 0);

  switch (g_chart_view) {
    case CHART_ENGINE_RPM:
      lv_label_set_text(lbl_chart_title, "Engine RPM Over Time");
      lv_label_set_text(lbl_chart_subtitle, "Actual Engine RPM");
      lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 6000);
      break;

    case CHART_PUMP_RPM:
      lv_label_set_text(lbl_chart_title, "Pump RPM vs Target");
      lv_label_set_text(lbl_chart_subtitle, "Blue = Actual, Red = Target");
      lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 6000);
      break;

    case CHART_TORQUE:
      lv_label_set_text(lbl_chart_title, "Torque vs Target");
      lv_label_set_text(lbl_chart_subtitle, "Blue = Actual, Red = Target");
      lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 200);
      break;

    case CHART_POWER:
      lv_label_set_text(lbl_chart_title, "Power Chart");
      lv_label_set_text(lbl_chart_subtitle, "Power kW");
      lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 50);
      break;
  }

  lv_chart_refresh(chart);
}

static void cb_open_chart_engine(lv_event_t*) { set_chart_view(CHART_ENGINE_RPM); go_to(scr_live_chart); }
static void cb_open_chart_pump(lv_event_t*)   { set_chart_view(CHART_PUMP_RPM);   go_to(scr_live_chart); }
static void cb_open_chart_torque(lv_event_t*) { set_chart_view(CHART_TORQUE);     go_to(scr_live_chart); }
static void cb_open_chart_power(lv_event_t*)  { set_chart_view(CHART_POWER);      go_to(scr_live_chart); }

// =====================================================
// Build screens
// =====================================================
static void build_home() {
  scr_home = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_home);
  make_topbar(root, false, nullptr, "Dyno Home", cb_estop, "E-STOP");
  lv_obj_t* body = make_body(root);

  lbl_link_home = make_value_label(body, "Link: --");
  make_value_label(body, "Main Menu");

  lv_obj_t* list = make_button_list(body);

  lv_obj_t* btn1 = lv_btn_create(list);
  lv_obj_set_size(btn1, lv_pct(100), 60);
  lv_obj_add_event_cb(btn1, cb_open_mode_select, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl1 = lv_label_create(btn1);
  lv_label_set_text(lbl1, "Mode Select");
  lv_obj_center(lbl1);

  lv_obj_t* btn2 = lv_btn_create(list);
  lv_obj_set_size(btn2, lv_pct(100), 60);
  lv_obj_add_event_cb(btn2, cb_open_live_menu, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl2 = lv_label_create(btn2);
  lv_label_set_text(lbl2, "Live Data");
  lv_obj_center(lbl2);
}

static void build_mode_select() {
  scr_mode_select = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_mode_select);
  make_topbar(root, true, cb_back_home, "Mode Select", cb_estop, "E-STOP");
  lv_obj_t* body = make_body(root);

  lbl_link_mode = make_value_label(body, "Link: --");
  make_value_label(body, "Choose control mode:");

  lv_obj_t* list = make_button_list(body);

  const char* names[] = {
    "Manual Control",
    "Torque Control",
    "RPM Control",
    "Variable Ratio"
  };
  lv_event_cb_t callbacks[] = {
    cb_open_manual,
    cb_open_torque,
    cb_open_rpm,
    cb_open_variable_ratio
  };

  for (int i = 0; i < 4; i++) {
    lv_obj_t* btn = lv_btn_create(list);
    lv_obj_set_size(btn, lv_pct(100), 56);
    lv_obj_add_event_cb(btn, callbacks[i], LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, names[i]);
    lv_obj_center(lbl);
  }
}

static void build_manual() {
  scr_manual = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_manual);
  make_topbar(root, true, cb_back_mode_select, "Manual Control", cb_estop, "E-STOP");
  lv_obj_t* body = make_body(root, true);

  lbl_link_manual = make_value_label(body, "Link: --");

  lbl_manual_engine = make_value_label(body, "Engine RPM: --");
  lbl_manual_pump = make_value_label(body, "Pump RPM: --");
  lbl_manual_flow = make_value_label(body, "Flow Valve Cmd: 0 %");
  lbl_manual_pressure = make_value_label(body, "Pressure Valve Cmd: 0 %");
  lbl_manual_throttle = make_value_label(body, "Throttle: -- %");

  make_value_label(body, "Manual Flow");
  manual_flow_slider = lv_slider_create(body);
  lv_obj_set_width(manual_flow_slider, lv_pct(100));
  lv_slider_set_range(manual_flow_slider, 0, 100);
  lv_slider_set_value(manual_flow_slider, 0, LV_ANIM_OFF);
  lv_obj_add_event_cb(manual_flow_slider, cb_manual_flow_slider, LV_EVENT_VALUE_CHANGED, nullptr);

  make_value_label(body, "Manual Pressure");
  manual_pressure_slider = lv_slider_create(body);
  lv_obj_set_width(manual_pressure_slider, lv_pct(100));
  lv_slider_set_range(manual_pressure_slider, 0, 100);
  lv_slider_set_value(manual_pressure_slider, 0, LV_ANIM_OFF);
  lv_obj_add_event_cb(manual_pressure_slider, cb_manual_pressure_slider, LV_EVENT_VALUE_CHANGED, nullptr);
}

static void build_torque() {
  scr_torque = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_torque);
  make_topbar(root, true, cb_back_mode_select, "Torque Control", cb_estop, "E-STOP");
  lv_obj_t* body = make_body(root);

  lbl_link_torque = make_value_label(body, "Link: --");
  lbl_torque_actual = make_value_label(body, "Actual Torque: -- Nm");
  lbl_torque_target = make_value_label(body, "Target Torque: -- Nm");
  lbl_torque_engine = make_value_label(body, "Engine RPM: -- / --");
  lbl_torque_pressure = make_value_label(body, "Pressure Valve: -- %");
  lbl_torque_scale = make_value_label(body, "Scale: --");

  lv_obj_t* btn = lv_btn_create(body);
  lv_obj_set_size(btn, lv_pct(100), 60);
  lv_obj_add_event_cb(btn, cb_open_set_torque, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "Set Desired Torque");
  lv_obj_center(lbl);
}

static void build_set_torque() {
  scr_set_torque = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_set_torque);
  make_topbar(root, true, cb_back_torque, "Set Torque", cb_estop, "E-STOP");
  lv_obj_t* body = make_body(root);

  make_value_label(body, "Enter desired torque:");

  tq_ta = lv_textarea_create(body);
  lv_obj_set_width(tq_ta, lv_pct(100));
  lv_textarea_set_one_line(tq_ta, true);
  lv_textarea_set_placeholder_text(tq_ta, "e.g. 12.5");
  lv_textarea_set_accepted_chars(tq_ta, "0123456789.");

  tq_kb = lv_keyboard_create(body);
  lv_obj_set_size(tq_kb, lv_pct(100), 180);
  lv_keyboard_set_mode(tq_kb, LV_KEYBOARD_MODE_NUMBER);
  lv_keyboard_set_textarea(tq_kb, tq_ta);
  lv_obj_add_event_cb(tq_kb, cb_tq_kb, LV_EVENT_ALL, nullptr);
}

static void build_rpm() {
  scr_rpm = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_rpm);
  make_topbar(root, true, cb_back_mode_select, "RPM Control", cb_estop, "E-STOP");
  lv_obj_t* body = make_body(root);

  lbl_link_rpm = make_value_label(body, "Link: --");
  lbl_rpm_actual = make_value_label(body, "Actual Pump RPM: --");
  lbl_rpm_target = make_value_label(body, "Target Pump RPM: --");
  lbl_rpm_engine = make_value_label(body, "Engine RPM: -- / --");
  lbl_rpm_flow = make_value_label(body, "Flow Valve: -- %");
  lbl_rpm_throttle = make_value_label(body, "Throttle: -- %");

  lv_obj_t* btn = lv_btn_create(body);
  lv_obj_set_size(btn, lv_pct(100), 60);
  lv_obj_add_event_cb(btn, cb_open_set_rpm, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "Set Desired RPM");
  lv_obj_center(lbl);
}

static void build_set_rpm() {
  scr_set_rpm = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_set_rpm);
  make_topbar(root, true, cb_back_rpm, "Set RPM", cb_estop, "E-STOP");
  lv_obj_t* body = make_body(root);

  make_value_label(body, "Enter desired RPM:");

  rpm_ta = lv_textarea_create(body);
  lv_obj_set_width(rpm_ta, lv_pct(100));
  lv_textarea_set_one_line(rpm_ta, true);
  lv_textarea_set_placeholder_text(rpm_ta, "e.g. 2500");
  lv_textarea_set_accepted_chars(rpm_ta, "0123456789");

  rpm_kb = lv_keyboard_create(body);
  lv_obj_set_size(rpm_kb, lv_pct(100), 180);
  lv_keyboard_set_mode(rpm_kb, LV_KEYBOARD_MODE_NUMBER);
  lv_keyboard_set_textarea(rpm_kb, rpm_ta);
  lv_obj_add_event_cb(rpm_kb, cb_rpm_kb, LV_EVENT_ALL, nullptr);
}

static void build_variable_ratio() {
  scr_variable_ratio = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_variable_ratio);
  make_topbar(root, true, cb_back_mode_select, "Variable Ratio", cb_estop, "E-STOP");
  lv_obj_t* body = make_body(root);

  lbl_link_variable = make_value_label(body, "Link: --");
  lbl_variable_status = make_value_label(body, "Under construction");
}

static void build_live_menu() {
  scr_live_menu = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_live_menu);
  make_topbar(root, true, cb_back_home, "Live Data", cb_estop, "E-STOP");
  lv_obj_t* body = make_body(root);

  lbl_link_live = make_value_label(body, "Link: --");
  make_value_label(body, "Choose data view:");

  lv_obj_t* list = make_button_list(body);

  lv_obj_t* btn1 = lv_btn_create(list);
  lv_obj_set_size(btn1, lv_pct(100), 60);
  lv_obj_add_event_cb(btn1, cb_open_live_static, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl1 = lv_label_create(btn1);
  lv_label_set_text(lbl1, "Static Data");
  lv_obj_center(lbl1);

  lv_obj_t* btn2 = lv_btn_create(list);
  lv_obj_set_size(btn2, lv_pct(100), 60);
  lv_obj_add_event_cb(btn2, cb_open_live_dynamic, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl2 = lv_label_create(btn2);
  lv_label_set_text(lbl2, "Dynamic Data");
  lv_obj_center(lbl2);
}

static void build_live_static() {
  scr_live_static = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_live_static);
  make_topbar(root, true, cb_back_live_menu, "Static Data", cb_estop, "E-STOP");
  lv_obj_t* body = make_body(root, true);

  lbl_link_live_static = make_value_label(body, "Link: --");
  lbl_live_mode = make_value_label(body, "Mode: --");
  lbl_live_engine = make_value_label(body, "Engine RPM: --");
  lbl_live_pump = make_value_label(body, "Pump RPM: -- / --");
  lbl_live_torque = make_value_label(body, "Torque: -- / --");
  lbl_live_power = make_value_label(body, "Power: -- kW");
  lbl_live_flow = make_value_label(body, "Flow Valve: -- %");
  lbl_live_pressure = make_value_label(body, "Pressure Valve: -- %");
  lbl_live_throttle = make_value_label(body, "Throttle: -- %");
}

static void build_live_dynamic() {
  scr_live_dynamic = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_live_dynamic);
  make_topbar(root, true, cb_back_live_menu, "Dynamic Data", cb_estop, "E-STOP");
  lv_obj_t* body = make_body(root, true);

  lbl_link_live_dynamic = make_value_label(body, "Link: --");
  make_value_label(body, "Charts");

  lv_obj_t* list = make_button_list(body);

  struct Item { const char* name; lv_event_cb_t cb; } items[] = {
    {"Engine RPM Chart", cb_open_chart_engine},
    {"Pump RPM vs Target", cb_open_chart_pump},
    {"Torque vs Target", cb_open_chart_torque},
    {"Power Chart", cb_open_chart_power}
  };

  for (auto& it : items) {
    lv_obj_t* btn = lv_btn_create(list);
    lv_obj_set_size(btn, lv_pct(100), 56);
    lv_obj_add_event_cb(btn, it.cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, it.name);
    lv_obj_center(lbl);
  }
}

static void build_live_chart() {
  scr_live_chart = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_live_chart);
  make_topbar(root, true, cb_back_live_dynamic, "Chart", cb_estop, "E-STOP");
  lv_obj_t* body = make_body(root,true);

  lbl_link_chart = make_value_label(body, "Link: --");
  lbl_chart_title = make_value_label(body, "Engine RPM Over Time");
  lbl_chart_subtitle = make_value_label(body, "Actual Engine RPM");

  chart = lv_chart_create(body);
  lv_obj_set_size(chart, lv_pct(100), 200);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(chart, 100);
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 6000);

  lv_obj_clear_flag(chart, LV_OBJ_FLAG_SCROLLABLE);

  series_a = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
  series_b = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);

  lv_chart_set_all_value(chart, series_a, 0);
  lv_chart_set_all_value(chart, series_b, 0);
}

// =====================================================
// Telemetry refresh
// =====================================================
static void chart_push_from_telemetry() {
  if (!chart || !series_a || !series_b || !g_tel_valid) return;

  int v1 = 0;
  int v2 = 0;

  switch (g_chart_view) {
    case CHART_ENGINE_RPM:
      v1 = (int)g_tel.engineRpm;
      v2 = 0;
      break;

    case CHART_PUMP_RPM:
      v1 = (int)g_tel.pumpRpm;
      v2 = (int)g_tel.targetPumpRpm;
      break;

    case CHART_TORQUE:
      v1 = (int)g_tel.torque;
      v2 = (int)g_tel.targetTorque;
      break;

    case CHART_POWER:
      v1 = (int)g_tel.powerKw;
      v2 = 0;
      break;
  }

  lv_chart_set_next_value(chart, series_a, v1);
  lv_chart_set_next_value(chart, series_b, v2);
  lv_chart_refresh(chart);
}

static void gui_refresh_from_telemetry() {
  if (!g_tel_dirty) {
    set_all_link_labels();
    return;
  }
  g_tel_dirty = false;

  set_all_link_labels();

  if (manual_flow_slider && lv_slider_get_value(manual_flow_slider) != (int)g_cmd.manualFlowPct) {
    lv_slider_set_value(manual_flow_slider, g_cmd.manualFlowPct, LV_ANIM_OFF);
  }
  if (manual_pressure_slider && lv_slider_get_value(manual_pressure_slider) != (int)g_cmd.manualPressurePct) {
    lv_slider_set_value(manual_pressure_slider, g_cmd.manualPressurePct, LV_ANIM_OFF);
  }

  if (lbl_manual_engine) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Engine RPM: %.0f / %.0f", (double)g_tel.engineRpm, (double)g_tel.targetEngineRpm);
    lv_label_set_text(lbl_manual_engine, buf);
  }
  if (lbl_manual_pump) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Pump RPM: %.0f / %.0f", (double)g_tel.pumpRpm, (double)g_tel.targetPumpRpm);
    lv_label_set_text(lbl_manual_pump, buf);
  }
  if (lbl_manual_flow) {
    char buf[96];
    snprintf(buf, sizeof(buf), "Flow Valve Cmd: %u %% | Actual: %.0f %%", g_cmd.manualFlowPct, (double)g_tel.flowValvePercent);
    lv_label_set_text(lbl_manual_flow, buf);
  }
  if (lbl_manual_pressure) {
    char buf[96];
    snprintf(buf, sizeof(buf), "Pressure Valve Cmd: %u %% | Actual: %.0f %%", g_cmd.manualPressurePct, (double)g_tel.pressureValvePercent);
    lv_label_set_text(lbl_manual_pressure, buf);
  }
  if (lbl_manual_throttle) {
    char buf[96];
    snprintf(buf, sizeof(buf), "Throttle: %.0f %% | Emergency: %s", (double)g_tel.throttlePercent, g_tel.emergency ? "YES" : "NO");
    lv_label_set_text(lbl_manual_throttle, buf);
  }

  if (lbl_torque_actual) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Actual Torque: %.2f Nm", (double)g_tel.torque);
    lv_label_set_text(lbl_torque_actual, buf);
  }
  if (lbl_torque_target) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Target Torque: %.2f Nm", (double)g_tel.targetTorque);
    lv_label_set_text(lbl_torque_target, buf);
  }
  if (lbl_torque_engine) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Engine RPM: %.0f / %.0f", (double)g_tel.engineRpm, (double)g_tel.targetEngineRpm);
    lv_label_set_text(lbl_torque_engine, buf);
  }
  if (lbl_torque_pressure) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Pressure Valve: %.0f %%", (double)g_tel.pressureValvePercent);
    lv_label_set_text(lbl_torque_pressure, buf);
  }
  if (lbl_torque_scale) {
    char buf[80];
    snprintf(buf, sizeof(buf), "Scale: %s | Emergency: %s", g_tel.scaleConnected ? "CONNECTED" : "MISSING", g_tel.emergency ? "YES" : "NO");
    lv_label_set_text(lbl_torque_scale, buf);
  }

  if (lbl_rpm_actual) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Actual Pump RPM: %.0f", (double)g_tel.pumpRpm);
    lv_label_set_text(lbl_rpm_actual, buf);
  }
  if (lbl_rpm_target) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Target Pump RPM: %.0f", (double)g_tel.targetPumpRpm);
    lv_label_set_text(lbl_rpm_target, buf);
  }
  if (lbl_rpm_engine) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Engine RPM: %.0f / %.0f", (double)g_tel.engineRpm, (double)g_tel.targetEngineRpm);
    lv_label_set_text(lbl_rpm_engine, buf);
  }
  if (lbl_rpm_flow) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Flow Valve: %.0f %%", (double)g_tel.flowValvePercent);
    lv_label_set_text(lbl_rpm_flow, buf);
  }
  if (lbl_rpm_throttle) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Throttle: %.0f %%", (double)g_tel.throttlePercent);
    lv_label_set_text(lbl_rpm_throttle, buf);
  }

  if (lbl_live_mode) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Mode: %s", dyno_mode_str(g_tel.mode));
    lv_label_set_text(lbl_live_mode, buf);
  }
  if (lbl_live_engine) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Engine RPM: %.0f / %.0f", (double)g_tel.engineRpm, (double)g_tel.targetEngineRpm);
    lv_label_set_text(lbl_live_engine, buf);
  }
  if (lbl_live_pump) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Pump RPM: %.0f / %.0f", (double)g_tel.pumpRpm, (double)g_tel.targetPumpRpm);
    lv_label_set_text(lbl_live_pump, buf);
  }
  if (lbl_live_torque) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Torque: %.2f / %.2f", (double)g_tel.torque, (double)g_tel.targetTorque);
    lv_label_set_text(lbl_live_torque, buf);
  }
  if (lbl_live_power) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Power: %.2f kW", (double)g_tel.powerKw);
    lv_label_set_text(lbl_live_power, buf);
  }
  if (lbl_live_flow) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Flow Valve: %.0f %%", (double)g_tel.flowValvePercent);
    lv_label_set_text(lbl_live_flow, buf);
  }
  if (lbl_live_pressure) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Pressure Valve: %.0f %%", (double)g_tel.pressureValvePercent);
    lv_label_set_text(lbl_live_pressure, buf);
  }
  if (lbl_live_throttle) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Throttle: %.0f %%", (double)g_tel.throttlePercent);
    lv_label_set_text(lbl_live_throttle, buf);
  }

  chart_push_from_telemetry();
}

// =====================================================
// setup / loop
// =====================================================
void setup() {
  Serial.begin(115200);

  screen.init();
  dyno_ble_init();

  build_home();
  build_mode_select();
  build_manual();
  build_torque();
  build_set_torque();
  build_rpm();
  build_set_rpm();
  build_variable_ratio();
  build_live_menu();
  build_live_static();
  build_live_dynamic();
  build_live_chart();

  set_chart_view(CHART_ENGINE_RPM);

  lv_scr_load(scr_home);
  set_all_link_labels();

  Serial.println("Dyno GUI ready");
}

void loop() {
  screen.routine();
  dyno_ble_task();
  gui_refresh_from_telemetry();
  delay(5);
}