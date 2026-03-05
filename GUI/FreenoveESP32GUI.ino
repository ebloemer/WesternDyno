/*
  DynoGUI_Freenove_LVGL.ino

  Freenove ESP32 Display + LVGL + display.h wrapper

  Screens:
   - Start Screen (Course 1-4)
   - Mode Screen (Torque / RPM)
   - Confirm Screen (selected course + mode + START)
   - Menu Screen (Active + END + 6 selections)
   - Chart Screen (Active + END + Back + live Target RPM chart)

  Serial messages:
   COURSE:<n>
   MODE:RPM
   MODE:TORQUE
   START
   END
   VIEW:<item>
*/

#include "display.h"
#include <lvgl.h>

Display screen;

// -------------------- App state --------------------
enum class Mode { RPM, TORQUE };

static int  g_course = 1;
static Mode g_mode   = Mode::RPM;
static bool g_sim_active = false;

// Screens
static lv_obj_t* scr_start   = nullptr;
static lv_obj_t* scr_mode    = nullptr;
static lv_obj_t* scr_confirm = nullptr;
static lv_obj_t* scr_menu    = nullptr;
static lv_obj_t* scr_chart   = nullptr;

// Labels we update
static lv_obj_t* lbl_mode_title      = nullptr;
static lv_obj_t* lbl_confirm_course  = nullptr;
static lv_obj_t* lbl_confirm_mode    = nullptr;

// Chart stuff
static lv_obj_t* chart = nullptr;
static lv_chart_series_t* series_target = nullptr;
static lv_timer_t* chart_timer = nullptr;

static uint32_t t_idx = 0;
static int32_t  target_rpm_value = 1500;

// -------------------- Helpers --------------------
static const char* mode_to_str(Mode m) {
  return (m == Mode::RPM) ? "RPM" : "Torque";
}

static void serial_send(const String& s) {
  Serial.println(s);
}

static void stop_chart_timer() {
  if (chart_timer != nullptr) {
    lv_timer_del(chart_timer);
    chart_timer = nullptr;
  }
}

static void start_chart_timer() {
  stop_chart_timer();
  chart_timer = lv_timer_create(
    [](lv_timer_t* timer) {
      (void)timer;

      // Demo data: rising ramp that wraps
      t_idx++;
      target_rpm_value = 1500 + (int32_t)(t_idx * 10);
      if (target_rpm_value > 6000) target_rpm_value = 1500;

      if (chart != nullptr && series_target != nullptr) {
        lv_chart_set_next_value(chart, series_target, target_rpm_value);
      }
    },
    200,   // 200 ms
    nullptr
  );
}

static void reset_chart_data() {
  t_idx = 0;
  target_rpm_value = 1500;

  if (chart != nullptr && series_target != nullptr) {
    lv_chart_set_all_value(chart, series_target, 0);
    lv_chart_refresh(chart);
  }
}

static void go_to(lv_obj_t* scr) {
  if (scr != nullptr) {
    lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
  }
}

static lv_obj_t* make_top_bar(lv_obj_t* parent,
                              bool show_back,
                              lv_event_cb_t back_cb,
                              bool show_end,
                              lv_event_cb_t end_cb,
                              bool show_active_label) {
  lv_obj_t* bar = lv_obj_create(parent);
  lv_obj_set_size(bar, 240, 40);
  lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

  // Back button
  if (show_back) {
    lv_obj_t* btn_back = lv_btn_create(bar);
    lv_obj_set_size(btn_back, 60, 30);
    lv_obj_align(btn_back, LV_ALIGN_LEFT_MID, 5, 0);
    lv_obj_add_event_cb(btn_back, back_cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t* lbl = lv_label_create(btn_back);
    lv_label_set_text(lbl, "Back");
    lv_obj_center(lbl);
  }

  // Active label
  if (show_active_label) {
    lv_obj_t* lbl_active = lv_label_create(bar);
    lv_label_set_text(lbl_active, "Active: Dyno Sim ON");
    lv_obj_align(lbl_active, LV_ALIGN_CENTER, 0, 0);
  }

  // END button
  if (show_end) {
    lv_obj_t* btn_end = lv_btn_create(bar);
    lv_obj_set_size(btn_end, 60, 30);
    lv_obj_align(btn_end, LV_ALIGN_RIGHT_MID, -5, 0);
    lv_obj_add_event_cb(btn_end, end_cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t* lbl = lv_label_create(btn_end);
    lv_label_set_text(lbl, "END");
    lv_obj_center(lbl);
  }

  return bar;
}

// -------------------- Forward declarations --------------------
static void build_start_screen();
static void build_mode_screen();
static void build_confirm_screen();
static void build_menu_screen();
static void build_chart_screen();

static void update_mode_title();
static void update_confirm_labels();

// -------------------- Event callbacks --------------------
static void on_end_clicked(lv_event_t* e) {
  (void)e;
  g_sim_active = false;
  stop_chart_timer();
  serial_send("END");
  go_to(scr_start);
}

static void on_back_to_start(lv_event_t* e) {
  (void)e;
  stop_chart_timer();
  go_to(scr_start);
}

static void on_back_to_mode(lv_event_t* e) {
  (void)e;
  stop_chart_timer();
  go_to(scr_mode);
}

static void on_back_to_menu(lv_event_t* e) {
  (void)e;
  stop_chart_timer();
  go_to(scr_menu);
}

static void on_course_btn(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  int course = (int)(intptr_t)lv_obj_get_user_data(btn);

  g_course = course;
  serial_send("COURSE:" + String(g_course));

  update_mode_title();
  go_to(scr_mode);
}

static void on_mode_select(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  const char* tag = (const char*)lv_obj_get_user_data(btn);

  if (String(tag) == "RPM") {
    g_mode = Mode::RPM;
  } else {
    g_mode = Mode::TORQUE;
  }

  serial_send("MODE:" + String(mode_to_str(g_mode)));

  update_confirm_labels();
  go_to(scr_confirm);
}

static void on_start_pressed(lv_event_t* e) {
  (void)e;
  g_sim_active = true;
  serial_send("START");
  go_to(scr_menu);
}

static void on_menu_item(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  const char* item = (const char*)lv_obj_get_user_data(btn);

  serial_send("VIEW:" + String(item));

  if (String(item) == "Target RPM") {
    reset_chart_data();
    start_chart_timer();
    go_to(scr_chart);
  } else {
    lv_obj_t* mbox = lv_msgbox_create(
      nullptr,
      "Not implemented yet",
      "This is a placeholder.\nOnly Target RPM chart is implemented.",
      nullptr,
      true
    );
    lv_obj_center(mbox);
  }
}

// -------------------- UI update helpers --------------------
static void update_mode_title() {
  if (lbl_mode_title != nullptr) {
    String s = "Course " + String(g_course) + " - choose mode";
    lv_label_set_text(lbl_mode_title, s.c_str());
  }
}

static void update_confirm_labels() {
  if (lbl_confirm_course != nullptr) {
    String s = "Selected Course: Course " + String(g_course);
    lv_label_set_text(lbl_confirm_course, s.c_str());
  }

  if (lbl_confirm_mode != nullptr) {
    String s = "Selected Option: " + String(mode_to_str(g_mode));
    lv_label_set_text(lbl_confirm_mode, s.c_str());
  }
}

// -------------------- Screen builders --------------------
static void build_start_screen() {
  scr_start = lv_obj_create(nullptr);

  lv_obj_t* title = lv_label_create(scr_start);
  lv_label_set_text(title, "Start Screen");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t* hint = lv_label_create(scr_start);
  lv_label_set_text(hint, "Select a course:");
  lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 35);

  const int btn_w = 100;
  const int btn_h = 50;
  const int x1 = -55;
  const int x2 = 55;
  const int y1 = -10;
  const int y2 = 55;

  for (int i = 1; i <= 4; i++) {
    lv_obj_t* btn = lv_btn_create(scr_start);
    lv_obj_set_size(btn, btn_w, btn_h);

    int x = (i % 2 == 1) ? x1 : x2;
    int y = (i <= 2) ? y1 : y2;
    lv_obj_align(btn, LV_ALIGN_CENTER, x, y);

    lv_obj_set_user_data(btn, (void*)(intptr_t)i);
    lv_obj_add_event_cb(btn, on_course_btn, LV_EVENT_CLICKED, nullptr);

    lv_obj_t* lbl = lv_label_create(btn);
    String txt = "Course " + String(i);
    lv_label_set_text(lbl, txt.c_str());
    lv_obj_center(lbl);
  }
}

static void build_mode_screen() {
  scr_mode = lv_obj_create(nullptr);

  make_top_bar(scr_mode, true, on_back_to_start, false, nullptr, false);

  lbl_mode_title = lv_label_create(scr_mode);
  lv_label_set_text(lbl_mode_title, "Course 1 - choose mode");
  lv_obj_align(lbl_mode_title, LV_ALIGN_TOP_MID, 0, 50);

  lv_obj_t* subtitle = lv_label_create(scr_mode);
  lv_label_set_text(subtitle, "Torque or RPM?");
  lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 75);

  lv_obj_t* btn_torque = lv_btn_create(scr_mode);
  lv_obj_set_size(btn_torque, 200, 50);
  lv_obj_align(btn_torque, LV_ALIGN_CENTER, 0, -5);
  lv_obj_set_user_data(btn_torque, (void*)"TORQUE");
  lv_obj_add_event_cb(btn_torque, on_mode_select, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* lbl_t = lv_label_create(btn_torque);
  lv_label_set_text(lbl_t, "Torque");
  lv_obj_center(lbl_t);

  lv_obj_t* btn_rpm = lv_btn_create(scr_mode);
  lv_obj_set_size(btn_rpm, 200, 50);
  lv_obj_align(btn_rpm, LV_ALIGN_CENTER, 0, 60);
  lv_obj_set_user_data(btn_rpm, (void*)"RPM");
  lv_obj_add_event_cb(btn_rpm, on_mode_select, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* lbl_r = lv_label_create(btn_rpm);
  lv_label_set_text(lbl_r, "RPM");
  lv_obj_center(lbl_r);
}

static void build_confirm_screen() {
  scr_confirm = lv_obj_create(nullptr);

  make_top_bar(scr_confirm, true, on_back_to_mode, false, nullptr, false);

  lv_obj_t* title = lv_label_create(scr_confirm);
  lv_label_set_text(title, "Selected Settings");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 50);

  lbl_confirm_course = lv_label_create(scr_confirm);
  lv_obj_align(lbl_confirm_course, LV_ALIGN_TOP_MID, 0, 80);

  lbl_confirm_mode = lv_label_create(scr_confirm);
  lv_obj_align(lbl_confirm_mode, LV_ALIGN_TOP_MID, 0, 105);

  lv_obj_t* btn_start = lv_btn_create(scr_confirm);
  lv_obj_set_size(btn_start, 200, 55);
  lv_obj_align(btn_start, LV_ALIGN_CENTER, 0, 40);
  lv_obj_add_event_cb(btn_start, on_start_pressed, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* lbl = lv_label_create(btn_start);
  lv_label_set_text(lbl, "START");
  lv_obj_center(lbl);

  update_confirm_labels();
}

static void build_menu_screen() {
  scr_menu = lv_obj_create(nullptr);

  make_top_bar(scr_menu, false, nullptr, true, on_end_clicked, true);

  lv_obj_t* title = lv_label_create(scr_menu);
  lv_label_set_text(title, "Select a view:");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 50);

  const char* items[6] = {
    "Accuracy",
    "Target RPM",
    "Actual RPM",
    "Engine HP",
    "Pump HP",
    "Pump Torque / Pressure / RPM / Flow"
  };

  lv_obj_t* cont = lv_obj_create(scr_menu);
  lv_obj_set_size(cont, 230, 200);
  lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(cont, 6, 0);

  for (int i = 0; i < 6; i++) {
    lv_obj_t* btn = lv_btn_create(cont);
    lv_obj_set_width(btn, 220);
    lv_obj_set_user_data(btn, (void*)items[i]);
    lv_obj_add_event_cb(btn, on_menu_item, LV_EVENT_CLICKED, nullptr);

    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, items[i]);
    lv_obj_center(lbl);
  }
}

static void build_chart_screen() {
  scr_chart = lv_obj_create(nullptr);

  make_top_bar(scr_chart, true, on_back_to_menu, true, on_end_clicked, true);

  lv_obj_t* title = lv_label_create(scr_chart);
  lv_label_set_text(title, "Target RPM vs Time (live)");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 45);

  chart = lv_chart_create(scr_chart);
  lv_obj_set_size(chart, 230, 250);
  lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 0, -10);

  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(chart, 60);
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 7000);

  series_target = lv_chart_add_series(
    chart,
    lv_palette_main(LV_PALETTE_BLUE),
    LV_CHART_AXIS_PRIMARY_Y
  );

  reset_chart_data();
}

// -------------------- Arduino setup / loop --------------------
void setup() {
  Serial.begin(115200);

  // Freenove wrapper: initializes LVGL + display + touch
  screen.init();

  build_start_screen();
  build_mode_screen();
  build_confirm_screen();
  build_menu_screen();
  build_chart_screen();

  update_mode_title();
  update_confirm_labels();

  lv_scr_load(scr_start);

  Serial.println("DynoGUI ready");
}

void loop() {
  screen.routine();   // let LVGL run
  delay(5);
}