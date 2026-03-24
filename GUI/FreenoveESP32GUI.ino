#include "display.h"
#include <lvgl.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLERemoteCharacteristic.h>

#include "dyno_protocol.h"

Display screen;

// =====================================================
// GUI state
// =====================================================
enum class CourseMode { RPM, TORQUE };
static int g_course = 1;
static CourseMode g_course_mode = CourseMode::RPM;

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

// =====================================================
// BLE objects
// =====================================================
static BLEAdvertisedDevice* g_found_device = nullptr;
static BLEClient* g_client = nullptr;
static BLERemoteCharacteristic* g_cmd_char = nullptr;
static BLERemoteCharacteristic* g_tel_char = nullptr;
static BLEUUID g_service_uuid(DYNO_SERVICE_UUID);
static BLEUUID g_cmd_uuid(DYNO_CMD_UUID);
static BLEUUID g_tel_uuid(DYNO_TEL_UUID);

// =====================================================
// Screens + widgets
// =====================================================
static lv_obj_t* scr_main = nullptr;
static lv_obj_t* scr_manual = nullptr;
static lv_obj_t* scr_rpm_menu = nullptr;
static lv_obj_t* scr_set_rpm = nullptr;
static lv_obj_t* scr_torque_menu = nullptr;
static lv_obj_t* scr_set_torque = nullptr;
static lv_obj_t* scr_course_pick = nullptr;
static lv_obj_t* scr_course_mode = nullptr;
static lv_obj_t* scr_course_confirm = nullptr;
static lv_obj_t* scr_course_menu = nullptr;
static lv_obj_t* scr_course_chart = nullptr;

static lv_obj_t* rpm_ta = nullptr;
static lv_obj_t* rpm_kb = nullptr;
static lv_obj_t* tq_ta = nullptr;
static lv_obj_t* tq_kb = nullptr;
static lv_obj_t* chart = nullptr;
static lv_chart_series_t* series_main = nullptr;

static lv_obj_t* lbl_link_main = nullptr;
static lv_obj_t* lbl_link_manual = nullptr;
static lv_obj_t* lbl_link_rpm = nullptr;
static lv_obj_t* lbl_link_torque = nullptr;
static lv_obj_t* lbl_link_course = nullptr;

static lv_obj_t* lbl_manual_engine = nullptr;
static lv_obj_t* lbl_manual_pump = nullptr;
static lv_obj_t* lbl_manual_flow = nullptr;
static lv_obj_t* lbl_manual_pressure = nullptr;
static lv_obj_t* lbl_manual_throttle = nullptr;

static lv_obj_t* lbl_rpm_actual = nullptr;
static lv_obj_t* lbl_rpm_target = nullptr;
static lv_obj_t* lbl_rpm_engine = nullptr;
static lv_obj_t* lbl_rpm_flow = nullptr;
static lv_obj_t* lbl_rpm_throttle = nullptr;

static lv_obj_t* lbl_torque_actual = nullptr;
static lv_obj_t* lbl_torque_target = nullptr;
static lv_obj_t* lbl_torque_engine = nullptr;
static lv_obj_t* lbl_torque_pressure = nullptr;
static lv_obj_t* lbl_torque_scale = nullptr;

static lv_obj_t* lbl_course_mode_title = nullptr;
static lv_obj_t* lbl_confirm_course = nullptr;
static lv_obj_t* lbl_confirm_course_mode = nullptr;
static lv_obj_t* lbl_course_summary = nullptr;
static lv_obj_t* lbl_chart_title = nullptr;

static const char* g_chart_name = "Actual RPM";

// =====================================================
// Forward declarations
// =====================================================
static void go_to(lv_obj_t* s);
static void gui_refresh_from_telemetry();
static void dyno_ble_task();
static void dyno_set_mode(uint8_t mode);
static void dyno_set_desired_rpm(float rpm);
static void dyno_set_desired_torque(float tq);
static void dyno_set_manual_flow(uint16_t pct);
static void dyno_set_manual_pressure(uint16_t pct);

static const char* course_mode_str(CourseMode m) { return (m == CourseMode::RPM) ? "RPM" : "Torque"; }

static const char* dyno_mode_str(uint8_t mode) {
  switch (mode) {
    case MODE_MANUAL: return "MANUAL";
    case MODE_TORQUE: return "TORQUE";
    case MODE_RPM:    return "RPM";
    case MODE_CVT:    return "CVT";
    default:          return "UNKNOWN";
  }
}

// =====================================================
// LVGL helper builders
// =====================================================
static lv_obj_t* make_root(lv_obj_t* parent) {
  lv_obj_set_style_bg_color(parent, lv_color_hex(0x101418), 0);
  lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_all(parent, 0, 0);
  return parent;
}

static lv_obj_t* make_topbar(lv_obj_t* root,
                             bool show_back, lv_event_cb_t back_cb,
                             bool show_right, lv_event_cb_t right_cb,
                             const char* title) {
  lv_obj_t* top = lv_obj_create(root);
  lv_obj_set_size(top, lv_pct(100), 48);
  lv_obj_align(top, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_radius(top, 0, 0);
  lv_obj_set_style_border_width(top, 0, 0);
  lv_obj_set_style_pad_all(top, 8, 0);
  lv_obj_set_style_bg_color(top, lv_color_hex(0x1E293B), 0);
  lv_obj_clear_flag(top, LV_OBJ_FLAG_SCROLLABLE);

  if (show_back) {
    lv_obj_t* btn = lv_btn_create(top);
    lv_obj_set_size(btn, 56, 30);
    lv_obj_align(btn, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_event_cb(btn, back_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "Back");
    lv_obj_center(lbl);
  }

  if (show_right) {
    lv_obj_t* btn = lv_btn_create(top);
    lv_obj_set_size(btn, 56, 30);
    lv_obj_align(btn, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(btn, right_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "END");
    lv_obj_center(lbl);
  }

  lv_obj_t* lbl = lv_label_create(top);
  lv_label_set_text(lbl, title);
  lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);
  return top;
}

static lv_obj_t* make_body(lv_obj_t* root) {
  lv_obj_t* body = lv_obj_create(root);
  lv_obj_set_size(body, lv_pct(100), lv_pct(100));
  lv_obj_set_style_pad_top(body, 56, 0);
  lv_obj_set_style_pad_bottom(body, 8, 0);
  lv_obj_set_style_pad_left(body, 8, 0);
  lv_obj_set_style_pad_right(body, 8, 0);
  lv_obj_set_style_border_width(body, 0, 0);
  lv_obj_set_style_bg_color(body, lv_color_hex(0x101418), 0);
  lv_obj_clear_flag(body, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(body, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(body, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_set_style_pad_row(body, 8, 0);
  return body;
}

static lv_obj_t* make_button_list(lv_obj_t* parent) {
  lv_obj_t* list = lv_obj_create(parent);
  lv_obj_set_width(list, lv_pct(100));
  lv_obj_set_flex_grow(list, 1);
  lv_obj_set_style_border_width(list, 0, 0);
  lv_obj_set_style_bg_color(list, lv_color_hex(0x101418), 0);
  lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_gap(list, 8, 0);
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

  char buf[64];
  if (g_connected) {
    snprintf(buf, sizeof(buf), "Link: Connected | Mode: %s", g_tel_valid ? dyno_mode_str(g_tel.mode) : "--");
  } else if (g_connecting) {
    snprintf(buf, sizeof(buf), "Link: Connecting...");
  } else {
    snprintf(buf, sizeof(buf), "Link: Scanning for controller...");
  }
  lv_label_set_text(lbl, buf);
}

static void set_all_link_labels() {
  set_link_label(lbl_link_main);
  set_link_label(lbl_link_manual);
  set_link_label(lbl_link_rpm);
  set_link_label(lbl_link_torque);
  set_link_label(lbl_link_course);
}

static void chart_set_view(const char* name) {
  g_chart_name = name;
  if (lbl_chart_title) lv_label_set_text(lbl_chart_title, name);
  if (chart && series_main) {
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 5000);
    lv_chart_set_all_value(chart, series_main, 0);
    lv_chart_refresh(chart);
  }
}

static void go_to(lv_obj_t* s) {
  if (s) lv_scr_load(s);
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
  g_client->setClientCallbacks(new DynoClientCallbacks(), false);

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

  const uint32_t now = millis();
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
// GUI -> command helpers
// =====================================================
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
// Navigation callbacks
// =====================================================
static void cb_go_main(lv_event_t*) { dyno_set_mode(MODE_MANUAL); go_to(scr_main); }
static void cb_open_manual(lv_event_t*) { dyno_set_mode(MODE_MANUAL); go_to(scr_manual); }
static void cb_open_rpm(lv_event_t*)    { dyno_set_mode(MODE_RPM);    go_to(scr_rpm_menu); }
static void cb_open_torque(lv_event_t*) { dyno_set_mode(MODE_TORQUE); go_to(scr_torque_menu); }
static void cb_open_course(lv_event_t*) { go_to(scr_course_pick); }

static void cb_open_set_rpm(lv_event_t*) {
  dyno_set_mode(MODE_RPM);
  if (rpm_ta) lv_textarea_set_text(rpm_ta, "");
  go_to(scr_set_rpm);
}

static void cb_open_set_torque(lv_event_t*) {
  dyno_set_mode(MODE_TORQUE);
  if (tq_ta) lv_textarea_set_text(tq_ta, "");
  go_to(scr_set_torque);
}

static void cb_rpm_kb(lv_event_t* e) {
  const lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_READY) {
    dyno_set_desired_rpm((float)atof(lv_textarea_get_text(rpm_ta)));
    dyno_set_mode(MODE_RPM);
    dyno_send_command_packet(true);
    go_to(scr_rpm_menu);
  } else if (code == LV_EVENT_CANCEL) {
    go_to(scr_rpm_menu);
  }
}

static void cb_tq_kb(lv_event_t* e) {
  const lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_READY) {
    dyno_set_desired_torque((float)atof(lv_textarea_get_text(tq_ta)));
    dyno_set_mode(MODE_TORQUE);
    dyno_send_command_packet(true);
    go_to(scr_torque_menu);
  } else if (code == LV_EVENT_CANCEL) {
    go_to(scr_torque_menu);
  }
}

static void cb_manual_flow_up(lv_event_t*)    { dyno_set_mode(MODE_MANUAL); dyno_set_manual_flow(g_cmd.manualFlowPct + 5); dyno_send_command_packet(true); }
static void cb_manual_flow_down(lv_event_t*)  { dyno_set_mode(MODE_MANUAL); dyno_set_manual_flow(g_cmd.manualFlowPct >= 5 ? g_cmd.manualFlowPct - 5 : 0); dyno_send_command_packet(true); }
static void cb_manual_pressure_up(lv_event_t*)   { dyno_set_mode(MODE_MANUAL); dyno_set_manual_pressure(g_cmd.manualPressurePct + 5); dyno_send_command_packet(true); }
static void cb_manual_pressure_down(lv_event_t*) { dyno_set_mode(MODE_MANUAL); dyno_set_manual_pressure(g_cmd.manualPressurePct >= 5 ? g_cmd.manualPressurePct - 5 : 0); dyno_send_command_packet(true); }

static void cb_emergency_toggle(lv_event_t*) {
  g_cmd.emergency = g_cmd.emergency ? 0 : 1;
  g_cmd_dirty = true;
  dyno_send_command_packet(true);
}

static void cb_course_selected(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  g_course = (int)(intptr_t)lv_obj_get_user_data(btn);
  if (lbl_course_mode_title) {
    String s = "Course " + String(g_course) + " - choose mode";
    lv_label_set_text(lbl_course_mode_title, s.c_str());
  }
  go_to(scr_course_mode);
}

static void cb_course_mode_selected(lv_event_t* e) {
  const char* tag = (const char*)lv_obj_get_user_data(lv_event_get_target(e));
  g_course_mode = (String(tag) == "RPM") ? CourseMode::RPM : CourseMode::TORQUE;

  if (lbl_confirm_course) {
    String s = "Course: " + String(g_course);
    lv_label_set_text(lbl_confirm_course, s.c_str());
  }
  if (lbl_confirm_course_mode) {
    String s = String("Control Mode: ") + course_mode_str(g_course_mode);
    lv_label_set_text(lbl_confirm_course_mode, s.c_str());
  }
  go_to(scr_course_confirm);
}

static void cb_course_start(lv_event_t*) {
  dyno_set_mode((g_course_mode == CourseMode::RPM) ? MODE_RPM : MODE_TORQUE);
  dyno_send_command_packet(true);
  go_to(scr_course_menu);
}

static void cb_course_end(lv_event_t*) {
  go_to(scr_course_pick);
}

static void cb_course_menu_item(lv_event_t* e) {
  chart_set_view((const char*)lv_obj_get_user_data(lv_event_get_target(e)));
  go_to(scr_course_chart);
}

static void cb_back_to_course_menu(lv_event_t*) {
  go_to(scr_course_menu);
}

// =====================================================
// Screen builders
// =====================================================
static void build_main_menu() {
  scr_main = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_main);
  make_topbar(root, false, nullptr, false, nullptr, "Dyno Main Menu");
  lv_obj_t* body = make_body(root);

  lbl_link_main = make_value_label(body, "Link: --");
  lv_obj_t* title = make_value_label(body, "Select Mode:");
  (void)title;

  lv_obj_t* list = make_button_list(body);

  struct Item { const char* name; lv_event_cb_t cb; } items[] = {
    {"Manual Mode", cb_open_manual},
    {"RPM Mode", cb_open_rpm},
    {"Torque Mode", cb_open_torque},
    {"Course Simulation", cb_open_course},
  };

  for (auto& it : items) {
    lv_obj_t* btn = lv_btn_create(list);
    lv_obj_set_size(btn, lv_pct(100), 48);
    lv_obj_add_event_cb(btn, it.cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, it.name);
    lv_obj_center(lbl);
  }
}

static void build_manual_screen() {
  scr_manual = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_manual);
  make_topbar(root, true, cb_go_main, false, nullptr, "Manual");
  lv_obj_t* body = make_body(root);

  lbl_link_manual = make_value_label(body, "Link: --");
  lbl_manual_engine = make_value_label(body, "Engine RPM: --");
  lbl_manual_pump = make_value_label(body, "Pump RPM: --");
  lbl_manual_flow = make_value_label(body, "Flow Valve Cmd: 0 %");
  lbl_manual_pressure = make_value_label(body, "Pressure Valve Cmd: 0 %");
  lbl_manual_throttle = make_value_label(body, "Throttle: -- %");

  lv_obj_t* list = make_button_list(body);

  lv_obj_t* btn1 = lv_btn_create(list);
  lv_obj_set_size(btn1, lv_pct(100), 46);
  lv_obj_add_event_cb(btn1, cb_manual_flow_down, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl1 = lv_label_create(btn1);
  lv_label_set_text(lbl1, "Flow -5%");
  lv_obj_center(lbl1);

  lv_obj_t* btn2 = lv_btn_create(list);
  lv_obj_set_size(btn2, lv_pct(100), 46);
  lv_obj_add_event_cb(btn2, cb_manual_flow_up, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl2 = lv_label_create(btn2);
  lv_label_set_text(lbl2, "Flow +5%");
  lv_obj_center(lbl2);

  lv_obj_t* btn3 = lv_btn_create(list);
  lv_obj_set_size(btn3, lv_pct(100), 46);
  lv_obj_add_event_cb(btn3, cb_manual_pressure_down, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl3 = lv_label_create(btn3);
  lv_label_set_text(lbl3, "Pressure -5%");
  lv_obj_center(lbl3);

  lv_obj_t* btn4 = lv_btn_create(list);
  lv_obj_set_size(btn4, lv_pct(100), 46);
  lv_obj_add_event_cb(btn4, cb_manual_pressure_up, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl4 = lv_label_create(btn4);
  lv_label_set_text(lbl4, "Pressure +5%");
  lv_obj_center(lbl4);

  lv_obj_t* btn5 = lv_btn_create(list);
  lv_obj_set_size(btn5, lv_pct(100), 46);
  lv_obj_add_event_cb(btn5, cb_emergency_toggle, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl5 = lv_label_create(btn5);
  lv_label_set_text(lbl5, "Toggle Emergency");
  lv_obj_center(lbl5);
}

static void build_rpm_menu() {
  scr_rpm_menu = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_rpm_menu);
  make_topbar(root, true, cb_go_main, false, nullptr, "RPM Mode");
  lv_obj_t* body = make_body(root);

  lbl_link_rpm = make_value_label(body, "Link: --");
  lbl_rpm_actual = make_value_label(body, "Actual Pump RPM: --");
  lbl_rpm_target = make_value_label(body, "Target Pump RPM: --");
  lbl_rpm_engine = make_value_label(body, "Engine RPM: -- / --");
  lbl_rpm_flow = make_value_label(body, "Flow Valve: -- %");
  lbl_rpm_throttle = make_value_label(body, "Throttle: -- %");

  lv_obj_t* list = make_button_list(body);

  lv_obj_t* b1 = lv_btn_create(list);
  lv_obj_set_size(b1, lv_pct(100), 52);
  lv_obj_add_event_cb(b1, cb_open_set_rpm, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* l1 = lv_label_create(b1);
  lv_label_set_text(l1, "Set RPM");
  lv_obj_center(l1);
}

static void build_set_rpm() {
  scr_set_rpm = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_set_rpm);
  make_topbar(root, true, cb_open_rpm, false, nullptr, "Set RPM");
  lv_obj_t* body = make_body(root);

  make_value_label(body, "Enter target RPM:");

  rpm_ta = lv_textarea_create(body);
  lv_obj_set_width(rpm_ta, lv_pct(100));
  lv_textarea_set_one_line(rpm_ta, true);
  lv_textarea_set_placeholder_text(rpm_ta, "e.g. 2500");
  lv_textarea_set_accepted_chars(rpm_ta, "0123456789");

  rpm_kb = lv_keyboard_create(body);
  lv_keyboard_set_mode(rpm_kb, LV_KEYBOARD_MODE_NUMBER);
  lv_keyboard_set_textarea(rpm_kb, rpm_ta);
  lv_obj_add_event_cb(rpm_kb, cb_rpm_kb, LV_EVENT_ALL, nullptr);
}

static void build_torque_menu() {
  scr_torque_menu = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_torque_menu);
  make_topbar(root, true, cb_go_main, false, nullptr, "Torque Mode");
  lv_obj_t* body = make_body(root);

  lbl_link_torque = make_value_label(body, "Link: --");
  lbl_torque_actual = make_value_label(body, "Actual Torque: -- Nm");
  lbl_torque_target = make_value_label(body, "Target Torque: -- Nm");
  lbl_torque_engine = make_value_label(body, "Engine RPM: -- / --");
  lbl_torque_pressure = make_value_label(body, "Pressure Valve: -- %");
  lbl_torque_scale = make_value_label(body, "Scale: --");

  lv_obj_t* list = make_button_list(body);

  lv_obj_t* b1 = lv_btn_create(list);
  lv_obj_set_size(b1, lv_pct(100), 52);
  lv_obj_add_event_cb(b1, cb_open_set_torque, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* l1 = lv_label_create(b1);
  lv_label_set_text(l1, "Set Torque");
  lv_obj_center(l1);
}

static void build_set_torque() {
  scr_set_torque = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_set_torque);
  make_topbar(root, true, cb_open_torque, false, nullptr, "Set Torque");
  lv_obj_t* body = make_body(root);

  make_value_label(body, "Enter target torque (Nm):");

  tq_ta = lv_textarea_create(body);
  lv_obj_set_width(tq_ta, lv_pct(100));
  lv_textarea_set_one_line(tq_ta, true);
  lv_textarea_set_placeholder_text(tq_ta, "e.g. 12.5");
  lv_textarea_set_accepted_chars(tq_ta, "0123456789.");

  tq_kb = lv_keyboard_create(body);
  lv_keyboard_set_mode(tq_kb, LV_KEYBOARD_MODE_NUMBER);
  lv_keyboard_set_textarea(tq_kb, tq_ta);
  lv_obj_add_event_cb(tq_kb, cb_tq_kb, LV_EVENT_ALL, nullptr);
}

static void build_course_pick() {
  scr_course_pick = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_course_pick);
  make_topbar(root, true, cb_go_main, false, nullptr, "Course Sim");
  lv_obj_t* body = make_body(root);

  lbl_link_course = make_value_label(body, "Link: --");
  make_value_label(body, "Select course 1-4:");

  lv_obj_t* grid = lv_obj_create(body);
  lv_obj_set_size(grid, lv_pct(100), 120);
  lv_obj_set_style_pad_gap(grid, 10, 0);
  lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER);

  for (int i = 1; i <= 4; i++) {
    lv_obj_t* btn = lv_btn_create(grid);
    lv_obj_set_size(btn, lv_pct(46), 46);
    lv_obj_set_user_data(btn, (void*)(intptr_t)i);
    lv_obj_add_event_cb(btn, cb_course_selected, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lbl = lv_label_create(btn);
    String txt = "Course " + String(i);
    lv_label_set_text(lbl, txt.c_str());
    lv_obj_center(lbl);
  }
}

static void build_course_mode() {
  scr_course_mode = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_course_mode);
  make_topbar(root, true, cb_open_course, false, nullptr, "Course Sim");
  lv_obj_t* body = make_body(root);

  lbl_course_mode_title = make_value_label(body, "Course 1 - choose mode");

  lv_obj_t* list = make_button_list(body);
  lv_obj_t* bt = lv_btn_create(list);
  lv_obj_set_size(bt, lv_pct(100), 55);
  lv_obj_set_user_data(bt, (void*)"TORQUE");
  lv_obj_add_event_cb(bt, cb_course_mode_selected, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lt = lv_label_create(bt);
  lv_label_set_text(lt, "Torque");
  lv_obj_center(lt);

  lv_obj_t* br = lv_btn_create(list);
  lv_obj_set_size(br, lv_pct(100), 55);
  lv_obj_set_user_data(br, (void*)"RPM");
  lv_obj_add_event_cb(br, cb_course_mode_selected, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lr = lv_label_create(br);
  lv_label_set_text(lr, "RPM");
  lv_obj_center(lr);
}

static void build_course_confirm() {
  scr_course_confirm = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_course_confirm);
  make_topbar(root, true, cb_open_course, false, nullptr, "Confirm");
  lv_obj_t* body = make_body(root);

  make_value_label(body, "Selected settings:");
  lbl_confirm_course = make_value_label(body, "Course: --");
  lbl_confirm_course_mode = make_value_label(body, "Control Mode: --");

  lv_obj_t* btn = lv_btn_create(body);
  lv_obj_set_size(btn, lv_pct(100), 60);
  lv_obj_add_event_cb(btn, cb_course_start, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "START");
  lv_obj_center(lbl);
}

static void build_course_menu() {
  scr_course_menu = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_course_menu);
  make_topbar(root, false, nullptr, true, cb_course_end, "Course Active");
  lv_obj_t* body = make_body(root);

  lbl_course_summary = make_value_label(body, "Waiting for telemetry...");
  lv_obj_t* list = make_button_list(body);

  const char* items[] = {
    "Actual RPM",
    "Torque",
    "Throttle",
    "Flow Valve",
    "Pressure Valve",
    "Pump RPM"
  };

  for (size_t i = 0; i < sizeof(items) / sizeof(items[0]); i++) {
    lv_obj_t* btn = lv_btn_create(list);
    lv_obj_set_size(btn, lv_pct(100), 48);
    lv_obj_set_user_data(btn, (void*)items[i]);
    lv_obj_add_event_cb(btn, cb_course_menu_item, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, items[i]);
    lv_obj_center(lbl);
  }
}

static void build_course_chart() {
  scr_course_chart = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_course_chart);
  make_topbar(root, true, cb_back_to_course_menu, true, cb_course_end, "Chart");
  lv_obj_t* body = make_body(root);

  lbl_chart_title = make_value_label(body, "Actual RPM");

  chart = lv_chart_create(body);
  lv_obj_set_size(chart, lv_pct(100), 180);
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(chart, 80);
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 5000);
  series_main = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_all_value(chart, series_main, 0);
  chart_set_view("Actual RPM");
}

// =====================================================
// Telemetry -> labels
// =====================================================
static void chart_push_from_telemetry() {
  if (!chart || !series_main || !g_tel_valid) return;

  int value = 0;
  int ymax = 100;

  if (strcmp(g_chart_name, "Actual RPM") == 0 || strcmp(g_chart_name, "Pump RPM") == 0) {
    value = (int)g_tel.pumpRpm;
    ymax = 5000;
  } else if (strcmp(g_chart_name, "Torque") == 0) {
    value = (int)g_tel.torque;
    ymax = 200;
  } else if (strcmp(g_chart_name, "Throttle") == 0) {
    value = (int)g_tel.throttlePercent;
    ymax = 100;
  } else if (strcmp(g_chart_name, "Flow Valve") == 0) {
    value = (int)g_tel.flowValvePercent;
    ymax = 100;
  } else if (strcmp(g_chart_name, "Pressure Valve") == 0) {
    value = (int)g_tel.pressureValvePercent;
    ymax = 100;
  }

  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, ymax);
  lv_chart_set_next_value(chart, series_main, value);
  lv_chart_refresh(chart);
}

static void gui_refresh_from_telemetry() {
  if (!g_tel_dirty) {
    set_all_link_labels();
    return;
  }
  g_tel_dirty = false;

  set_all_link_labels();

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
    char buf[64];
    snprintf(buf, sizeof(buf), "Flow Valve Cmd: %u %% | Actual: %.0f %%", g_cmd.manualFlowPct, (double)g_tel.flowValvePercent);
    lv_label_set_text(lbl_manual_flow, buf);
  }
  if (lbl_manual_pressure) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Pressure Valve Cmd: %u %% | Actual: %.0f %%", g_cmd.manualPressurePct, (double)g_tel.pressureValvePercent);
    lv_label_set_text(lbl_manual_pressure, buf);
  }
  if (lbl_manual_throttle) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Throttle: %.0f %% | Emergency: %s", (double)g_tel.throttlePercent, g_tel.emergency ? "YES" : "NO");
    lv_label_set_text(lbl_manual_throttle, buf);
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
    char buf[64];
    snprintf(buf, sizeof(buf), "Scale: %s | Emergency: %s", g_tel.scaleConnected ? "CONNECTED" : "MISSING", g_tel.emergency ? "YES" : "NO");
    lv_label_set_text(lbl_torque_scale, buf);
  }

  if (lbl_course_summary) {
    char buf[128];
    snprintf(buf, sizeof(buf), "Mode: %s | Pump %.0f / %.0f RPM | Torque %.2f Nm | Power %.2f kW",
             dyno_mode_str(g_tel.mode),
             (double)g_tel.pumpRpm,
             (double)g_tel.targetPumpRpm,
             (double)g_tel.torque,
             (double)g_tel.powerKw);
    lv_label_set_text(lbl_course_summary, buf);
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

  build_main_menu();
  build_manual_screen();
  build_rpm_menu();
  build_set_rpm();
  build_torque_menu();
  build_set_torque();
  build_course_pick();
  build_course_mode();
  build_course_confirm();
  build_course_menu();
  build_course_chart();

  lv_scr_load(scr_main);
  set_all_link_labels();
  Serial.println("Dyno GUI BLE build ready");
}

void loop() {
  screen.routine();
  dyno_ble_task();
  gui_refresh_from_telemetry();
  delay(5);
}
