/*
  DynoGUI_v4_UART_ALL_DEAD_CHARTS.ino

  - Main Menu: Manual / RPM / Torque / Course Simulation
  - Course Simulation Mode:
      Course 1-4 -> Mode RPM/Torque -> Confirm -> Active Menu (6 items)
      Each menu item opens the SAME "dead" chart screen (blank graph), with a title matching the item.
  - No fake graph data anywhere.
  - UART at bottom: sends MODE + SETRPM + SETTQ at 20 Hz, receives TEL,ACT,rpm,tq to update labels.

  IMPORTANT:
    - Charts stay "dead" (all zeros) until you later add real UART plotting.
*/

#include "display.h"
#include <lvgl.h>
#include <HardwareSerial.h>

Display screen;

// ==============================
// UART forward declarations 
// ==============================
void dyno_uart_init();
void dyno_uart_task();
void dyno_set_mode(const char* mode);
void dyno_set_desired_rpm(float rpm);
void dyno_set_desired_torque(float tq);

// ==============================
// Global state
// ==============================
enum class CourseMode { RPM, TORQUE };
static int        g_course = 1;
static CourseMode g_course_mode = CourseMode::RPM;

// Screens
static lv_obj_t* scr_main          = nullptr;

// Manual
static lv_obj_t* scr_manual        = nullptr;

// RPM mode screens
static lv_obj_t* scr_rpm_menu      = nullptr;
static lv_obj_t* scr_set_rpm       = nullptr;
static lv_obj_t* scr_power_test    = nullptr;

// Torque mode screens
static lv_obj_t* scr_torque_menu   = nullptr;
static lv_obj_t* scr_set_torque    = nullptr;

// Course sim screens
static lv_obj_t* scr_course_pick    = nullptr;
static lv_obj_t* scr_course_mode    = nullptr;
static lv_obj_t* scr_course_confirm = nullptr;
static lv_obj_t* scr_course_menu    = nullptr;
static lv_obj_t* scr_course_chart   = nullptr;

// Labels (course sim)
static lv_obj_t* lbl_course_mode_title   = nullptr;
static lv_obj_t* lbl_confirm_course      = nullptr;
static lv_obj_t* lbl_confirm_course_mode = nullptr;

// Telemetry labels (UART updates these)
static lv_obj_t* lbl_act_rpm    = nullptr;
static lv_obj_t* lbl_act_torque = nullptr;

// Chart screen widgets
static lv_obj_t*          lbl_chart_title = nullptr;
static lv_obj_t*          chart = nullptr;
static lv_chart_series_t* series_main = nullptr;

// Currently selected chart name (for later routing)
static const char* g_chart_name = "Target RPM";

// RPM input widgets
static lv_obj_t* rpm_ta = nullptr;
static lv_obj_t* rpm_kb = nullptr;

// Torque input widgets
static lv_obj_t* tq_ta = nullptr;
static lv_obj_t* tq_kb = nullptr;

// ==============================
// Helpers
// ==============================
static void serial_send(const String& s) { Serial.println(s); }

static void go_to(lv_obj_t* scr) {
  if (scr) lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_MOVE_LEFT, 160, 0, false);
}

static const char* course_mode_str(CourseMode m) {
  return (m == CourseMode::RPM) ? "RPM" : "Torque";
}

// Dead chart reset (no fake data)
static void chart_dead_reset() {
  if (chart && series_main) {
    lv_chart_set_all_value(chart, series_main, 0);
    lv_chart_refresh(chart);
  }
}

// Set chart title + (optional) reasonable Y-range placeholder
static void chart_set_view(const char* name) {
  g_chart_name = name;

  if (lbl_chart_title) {
    lv_label_set_text(lbl_chart_title, name);
  }

  // Optional: set a sensible Y range per chart (still dead, all zeros)
  if (chart) {
    if (strstr(name, "RPM") != nullptr) {
      lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 7000);
    } else if (strstr(name, "HP") != nullptr) {
      lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 50);
    } else if (strstr(name, "Accuracy") != nullptr) {
      lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    } else if (strstr(name, "Pump Data") != nullptr) {
      lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    } else {
      lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    }
  }

  chart_dead_reset();
}

// ==============================
// Layout helpers
// ==============================
static lv_obj_t* make_root(lv_obj_t* scr) {
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* root = lv_obj_create(scr);
  lv_obj_set_size(root, lv_pct(100), lv_pct(100));
  lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_all(root, 10, 0);
  lv_obj_set_style_pad_row(root, 10, 0);
  lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);
  return root;
}

static lv_obj_t* make_topbar(lv_obj_t* root,
                            bool show_back, lv_event_cb_t back_cb,
                            bool show_end,  lv_event_cb_t end_cb,
                            const char* center_text)
{
  lv_obj_t* bar = lv_obj_create(root);
  lv_obj_set_size(bar, lv_pct(100), 44);
  lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(bar, 6, 0);
  lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* left  = lv_obj_create(bar);
  lv_obj_t* mid   = lv_obj_create(bar);
  lv_obj_t* right = lv_obj_create(bar);

  for (auto* c : {left, mid, right}) {
    lv_obj_set_size(c, lv_pct(33), lv_pct(100));
    lv_obj_set_style_bg_opa(c, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(c, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
  }

  if (show_back) {
    lv_obj_t* btn = lv_btn_create(left);
    lv_obj_set_size(btn, 80, 32);
    lv_obj_align(btn, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_event_cb(btn, back_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "Back");
    lv_obj_center(lbl);
  }

  lv_obj_t* lblc = lv_label_create(mid);
  lv_label_set_text(lblc, center_text ? center_text : "");
  lv_obj_center(lblc);

  if (show_end) {
    lv_obj_t* btn = lv_btn_create(right);
    lv_obj_set_size(btn, 80, 32);
    lv_obj_align(btn, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(btn, end_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "END");
    lv_obj_center(lbl);
  }

  return bar;
}

static lv_obj_t* make_body(lv_obj_t* root) {
  lv_obj_t* body = lv_obj_create(root);
  lv_obj_set_size(body, lv_pct(100), lv_pct(100));
  lv_obj_set_flex_grow(body, 1);
  lv_obj_set_style_pad_all(body, 10, 0);
  lv_obj_set_style_pad_row(body, 10, 0);
  lv_obj_clear_flag(body, LV_OBJ_FLAG_SCROLLABLE);
  return body;
}

static lv_obj_t* make_button_list(lv_obj_t* parent) {
  lv_obj_t* list = lv_obj_create(parent);
  lv_obj_set_size(list, lv_pct(100), lv_pct(100));
  lv_obj_set_flex_grow(list, 1);
  lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(list, 8, 0);
  lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_opa(list, LV_OPA_TRANSP, 0);
  lv_obj_set_scroll_dir(list, LV_DIR_VER);
  return list;
}

// ==============================
// Forward declarations
// ==============================
static void build_main_menu();
static void build_manual_screen();
static void build_rpm_menu();
static void build_set_rpm();
static void build_power_test();
static void build_torque_menu();
static void build_set_torque();
static void build_course_pick();
static void build_course_mode();
static void build_course_confirm();
static void build_course_menu();
static void build_course_chart();
static void update_course_mode_title();
static void update_course_confirm_labels();

// ==============================
// Navigation callbacks
// ==============================
static void cb_go_main(lv_event_t*) { dyno_set_mode("MANUAL"); go_to(scr_main); }

// Main menu
static void cb_open_manual(lv_event_t*) { dyno_set_mode("MANUAL"); go_to(scr_manual); }
static void cb_open_rpm(lv_event_t*)    { dyno_set_mode("RPM");    go_to(scr_rpm_menu); }
static void cb_open_torque(lv_event_t*) { dyno_set_mode("TORQUE"); go_to(scr_torque_menu); }
static void cb_open_course(lv_event_t*) { dyno_set_mode("COURSE_SIM"); go_to(scr_course_pick); }

// RPM menu
static void cb_open_set_rpm(lv_event_t*) { dyno_set_mode("RPM"); if (rpm_ta) lv_textarea_set_text(rpm_ta, ""); go_to(scr_set_rpm); }
static void cb_open_power_test(lv_event_t*) { dyno_set_mode("RPM_POWERTEST"); go_to(scr_power_test); }

// Torque menu
static void cb_open_set_torque(lv_event_t*) { dyno_set_mode("TORQUE"); if (tq_ta) lv_textarea_set_text(tq_ta, ""); go_to(scr_set_torque); }

// RPM keyboard
static void cb_rpm_kb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_READY) {
    const char* txt = lv_textarea_get_text(rpm_ta);
    dyno_set_desired_rpm((float)atof(txt));
    dyno_set_mode("RPM");
    serial_send(String("RPM_SET:") + txt);
    go_to(scr_rpm_menu);
  } else if (code == LV_EVENT_CANCEL) {
    go_to(scr_rpm_menu);
  }
}

// Torque keyboard
static void cb_tq_kb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_READY) {
    const char* txt = lv_textarea_get_text(tq_ta);
    dyno_set_desired_torque((float)atof(txt));
    dyno_set_mode("TORQUE");
    serial_send(String("TORQUE_SET:") + txt);
    go_to(scr_torque_menu);
  } else if (code == LV_EVENT_CANCEL) {
    go_to(scr_torque_menu);
  }
}

// Course sim
static void cb_course_selected(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  g_course = (int)(intptr_t)lv_obj_get_user_data(btn);
  serial_send("COURSE:" + String(g_course));
  update_course_mode_title();
  go_to(scr_course_mode);
}

static void cb_course_mode_selected(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  const char* tag = (const char*)lv_obj_get_user_data(btn);
  g_course_mode = (String(tag) == "RPM") ? CourseMode::RPM : CourseMode::TORQUE;

  serial_send(String("COURSE_MODE:") + course_mode_str(g_course_mode));
  update_course_confirm_labels();
  go_to(scr_course_confirm);
}

static void cb_course_start(lv_event_t*) { dyno_set_mode("COURSE_SIM"); go_to(scr_course_menu); }
static void cb_course_end(lv_event_t*)   { dyno_set_mode("COURSE_SIM"); go_to(scr_course_pick); }

static void cb_course_menu_item(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  const char* item = (const char*)lv_obj_get_user_data(btn);

  // Every item opens the SAME dead chart screen
  chart_set_view(item);
  go_to(scr_course_chart);
}

static void cb_back_to_course_menu(lv_event_t*) { go_to(scr_course_menu); }

// ==============================
// Screen builders
// ==============================
static void build_main_menu() {
  scr_main = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_main);
  make_topbar(root, false, nullptr, false, nullptr, "Dyno Main Menu");
  lv_obj_t* body = make_body(root);

  lv_obj_t* title = lv_label_create(body);
  lv_label_set_text(title, "Select Mode:");

  lv_obj_t* list = make_button_list(body);

  struct Item { const char* name; lv_event_cb_t cb; };
  Item items[] = {
    {"Manual Mode (Hand Valves)", cb_open_manual},
    {"RPM Mode",                  cb_open_rpm},
    {"Torque Mode",               cb_open_torque},
    {"Course Simulation Mode",    cb_open_course},
  };

  for (auto &it : items) {
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

  lv_obj_t* lbl = lv_label_create(body);
  lv_label_set_text(lbl, "Manual Mode\n(placeholder)");
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 0, 0);
}

static void build_rpm_menu() {
  scr_rpm_menu = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_rpm_menu);
  make_topbar(root, true, cb_go_main, false, nullptr, "RPM Mode");
  lv_obj_t* body = make_body(root);

  lbl_act_rpm = lv_label_create(body);
  lv_label_set_text(lbl_act_rpm, "Actual RPM: --");

  //lbl_act_torque = lv_label_create(body);
  //lv_label_set_text(lbl_act_torque, "Actual Torque: -- Nm");

  lv_obj_t* list = make_button_list(body);

  lv_obj_t* b1 = lv_btn_create(list);
  lv_obj_set_size(b1, lv_pct(100), 52);
  lv_obj_add_event_cb(b1, cb_open_set_rpm, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* l1 = lv_label_create(b1);
  lv_label_set_text(l1, "Set RPM");
  lv_obj_center(l1);

  lv_obj_t* b2 = lv_btn_create(list);
  lv_obj_set_size(b2, lv_pct(100), 52);
  lv_obj_add_event_cb(b2, cb_open_power_test, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* l2 = lv_label_create(b2);
  lv_label_set_text(l2, "Power Test (Max -> Stall)");
  lv_obj_center(l2);
}

static void build_set_rpm() {
  scr_set_rpm = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_set_rpm);
  make_topbar(root, true, cb_open_rpm, false, nullptr, "Set RPM");
  lv_obj_t* body = make_body(root);

  lv_obj_t* title = lv_label_create(body);
  lv_label_set_text(title, "Enter RPM:");

  rpm_ta = lv_textarea_create(body);
  lv_obj_set_width(rpm_ta, lv_pct(100));
  lv_textarea_set_one_line(rpm_ta, true);
  lv_textarea_set_placeholder_text(rpm_ta, "e.g., 2500");
  lv_textarea_set_accepted_chars(rpm_ta, "0123456789");

  rpm_kb = lv_keyboard_create(body);
  lv_keyboard_set_mode(rpm_kb, LV_KEYBOARD_MODE_NUMBER);
  lv_keyboard_set_textarea(rpm_kb, rpm_ta);
  lv_obj_add_event_cb(rpm_kb, cb_rpm_kb, LV_EVENT_ALL, nullptr);
}

static void build_power_test() {
  scr_power_test = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_power_test);
  make_topbar(root, true, cb_open_rpm, false, nullptr, "Power Test");
  lv_obj_t* body = make_body(root);

  lv_obj_t* lbl = lv_label_create(body);
  lv_label_set_text(lbl, "Power Test\n(placeholder - no fake data)");
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 0, 0);
}

static void build_torque_menu() {
  scr_torque_menu = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_torque_menu);
  make_topbar(root, true, cb_go_main, false, nullptr, "Torque Mode");
  lv_obj_t* body = make_body(root);

  //lbl_act_rpm = lv_label_create(body);
  //lv_label_set_text(lbl_act_rpm, "Actual RPM: --");

  lbl_act_torque = lv_label_create(body);
  lv_label_set_text(lbl_act_torque, "Actual Torque: -- Nm");

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

  lv_obj_t* title = lv_label_create(body);
  lv_label_set_text(title, "Enter torque (N·m):");

  tq_ta = lv_textarea_create(body);
  lv_obj_set_width(tq_ta, lv_pct(100));
  lv_textarea_set_one_line(tq_ta, true);
  lv_textarea_set_placeholder_text(tq_ta, "e.g., 12.5");
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

  lv_obj_t* title = lv_label_create(body);
  lv_label_set_text(title, "Select a course (1-4):");

  lv_obj_t* grid = lv_obj_create(body);
  lv_obj_set_size(grid, lv_pct(100), lv_pct(100));
  lv_obj_set_flex_grow(grid, 1);
  lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_gap(grid, 10, 0);

  for (int i=1; i<=4; i++) {
    lv_obj_t* btn = lv_btn_create(grid);
    lv_obj_set_size(btn, lv_pct(46), 60);
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

  lbl_course_mode_title = lv_label_create(body);
  lv_label_set_text(lbl_course_mode_title, "Course 1 - choose mode");

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

static void update_course_mode_title() {
  if (!lbl_course_mode_title) return;
  String s = "Course " + String(g_course) + " - choose mode";
  lv_label_set_text(lbl_course_mode_title, s.c_str());
}

static void build_course_confirm() {
  scr_course_confirm = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_course_confirm);
  make_topbar(root, true, cb_open_course, false, nullptr, "Confirm");
  lv_obj_t* body = make_body(root);

  lv_obj_t* title = lv_label_create(body);
  lv_label_set_text(title, "Selected Settings:");

  lbl_confirm_course = lv_label_create(body);
  lbl_confirm_course_mode = lv_label_create(body);

  lv_obj_t* btn = lv_btn_create(body);
  lv_obj_set_size(btn, lv_pct(100), 60);
  lv_obj_add_event_cb(btn, cb_course_start, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "START");
  lv_obj_center(lbl);

  update_course_confirm_labels();
}

static void update_course_confirm_labels() {
  if (lbl_confirm_course) {
    String s = "Course: " + String(g_course);
    lv_label_set_text(lbl_confirm_course, s.c_str());
  }
  if (lbl_confirm_course_mode) {
    String s = String("Mode: ") + course_mode_str(g_course_mode);
    lv_label_set_text(lbl_confirm_course_mode, s.c_str());
  }
}

static void build_course_menu() {
  scr_course_menu = lv_obj_create(nullptr);
  lv_obj_t* root = make_root(scr_course_menu);
  make_topbar(root, false, nullptr, true, cb_course_end, "Active");
  lv_obj_t* body = make_body(root);

  //lbl_act_rpm = lv_label_create(body);
  //lv_label_set_text(lbl_act_rpm, "Actual RPM: --");

  //lbl_act_torque = lv_label_create(body);
  //lv_label_set_text(lbl_act_torque, "Actual Torque: -- N·m");

  lv_obj_t* title = lv_label_create(body);
  lv_label_set_text(title, "Select a view:");

  lv_obj_t* list = make_button_list(body);

  const char* items[6] = {
    "Accuracy",
    "Target RPM",
    "Actual RPM",
    "Engine HP",
    "Pump HP",
    "Pump Data"
  };

  for (int i=0; i<6; i++) {
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

  lbl_chart_title = lv_label_create(body);
  lv_label_set_text(lbl_chart_title, "Target RPM");

  chart = lv_chart_create(body);
  lv_obj_set_size(chart, lv_pct(100), lv_pct(100));
  lv_obj_set_flex_grow(chart, 1);

  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(chart, 120);

  series_main = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);

  // Initialize as dead/blank
  chart_set_view("Target RPM");
}

// ==============================
// setup / loop
// ==============================
void setup() {
  Serial.begin(115200);
  screen.init();

  // Build screens
  build_main_menu();
  build_manual_screen();

  build_rpm_menu();
  build_set_rpm();
  build_power_test();

  build_torque_menu();
  build_set_torque();

  build_course_pick();
  build_course_mode();
  build_course_confirm();
  build_course_menu();
  build_course_chart();

  // Start at main menu
  lv_scr_load(scr_main);

  // UART init
  dyno_uart_init();

  Serial.println("DynoGUI v4 (all dead charts) ready");
}

void loop() {
  screen.routine();
 // dyno_uart_task();
  delay(5);
}

/* =======================================================================
   =========================== UART MODULE (BOTTOM) =======================
   ======================================================================= */

/*******************************************************
 * UART LINK (GUI ESP32 <-> Control ESP32-S3)
 *
 * Protocol (newline terminated CSV):
 *   GUI -> S3:
 *     CMD,MODE,RPM
 *     CMD,MODE,TORQUE
 *     CMD,MODE,MANUAL
 *     CMD,MODE,COURSE_SIM
 *     CMD,SETRPM,2500
 *     CMD,SETTQ,12.50
 *     CMD,HEARTBEAT,123456
 *
 *   S3 -> GUI:
 *     TEL,ACT,2450,11.80
 *******************************************************/

static HardwareSerial& DYNO_UART = Serial2;

// CHANGE these pins to match your wiring
static const int DYNO_UART_RX_PIN = 16;  // GUI RX  <- S3 TX
static const int DYNO_UART_TX_PIN = 17;  // GUI TX  -> S3 RX
static const uint32_t DYNO_UART_BAUD = 115200;

// Desired setpoints (GUI -> S3)
static volatile float g_des_rpm    = 0.0f;
static volatile float g_des_torque = 0.0f;
static String g_mode_str = "MANUAL";

// Actual telemetry (S3 -> GUI)
static volatile float g_act_rpm    = 0.0f;
static volatile float g_act_torque = 0.0f;

// Send rate
static uint32_t g_last_send_ms = 0;
static const uint32_t SEND_PERIOD_MS = 50; // 20 Hz

// RX line buffer
static char rx_line[128];
static size_t rx_len = 0;

static void uart_send_line(const String& s) {
  DYNO_UART.print(s);
  DYNO_UART.print('\n');
}

void dyno_uart_init() {
  DYNO_UART.begin(DYNO_UART_BAUD, SERIAL_8N1, DYNO_UART_RX_PIN, DYNO_UART_TX_PIN);
  delay(50);
  uart_send_line("CMD,HELLO,GUI");
}

static void parse_line(const char* line) {
  // Expect: TEL,ACT,<rpm>,<torque>
  if (strncmp(line, "TEL,", 4) != 0) return;

  char buf[128];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  char* tok = strtok(buf, ","); // TEL
  if (!tok) return;

  tok = strtok(nullptr, ",");   // ACT
  if (!tok) return;

  if (strcmp(tok, "ACT") == 0) {
    char* rpm_s = strtok(nullptr, ",");
    char* tq_s  = strtok(nullptr, ",");

    if (rpm_s && tq_s) {
      g_act_rpm    = atof(rpm_s);
      g_act_torque = atof(tq_s);

      // Update labels wherever they currently point
      if (lbl_act_rpm) {
        char tmp[48];
        snprintf(tmp, sizeof(tmp), "Actual RPM: %.0f", (double)g_act_rpm);
        lv_label_set_text(lbl_act_rpm, tmp);
      }
      if (lbl_act_torque) {
        char tmp[64];
        snprintf(tmp, sizeof(tmp), "Actual Torque: %.2f N·m", (double)g_act_torque);
        lv_label_set_text(lbl_act_torque, tmp);
      }

      // NOTE: No graph updates here (dead charts).
      // Later, when you want REAL plotting, decide based on g_chart_name:
      // if (chart && series_main && strcmp(g_chart_name, "Actual RPM")==0) lv_chart_set_next_value(chart, series_main, (int)g_act_rpm);
      // etc.
    }
  }
}

void dyno_uart_task() {
  // 1) Read incoming UART and parse lines
  while (DYNO_UART.available() > 0) {
    char c = (char)DYNO_UART.read();
    if (c == '\r') continue;

    if (c == '\n') {
      rx_line[rx_len] = '\0';
      if (rx_len > 0) parse_line(rx_line);
      rx_len = 0;
      continue;
    }

    if (rx_len < sizeof(rx_line) - 1) rx_line[rx_len++] = c;
    else rx_len = 0; // overflow reset
  }

  // 2) Periodically send mode + setpoints
  uint32_t now = millis();
  if (now - g_last_send_ms >= SEND_PERIOD_MS) {
    g_last_send_ms = now;

    uart_send_line(String("CMD,MODE,") + g_mode_str);
    uart_send_line(String("CMD,SETRPM,") + String(g_des_rpm, 0));
    uart_send_line(String("CMD,SETTQ,")  + String(g_des_torque, 2));
    uart_send_line(String("CMD,HEARTBEAT,") + String(now));
  }
}

void dyno_set_mode(const char* mode) { g_mode_str = mode; }
void dyno_set_desired_rpm(float rpm) { g_des_rpm = rpm; }
void dyno_set_desired_torque(float tq) { g_des_torque = tq; }