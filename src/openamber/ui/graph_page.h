#pragma once

// History graph page: one sample per minute for 24 hours, kept in RAM only.
// Nothing is written to flash, so the graph starts empty after a restart.
// The page only reads sensor values; it never touches the control loop.

#include <cmath>
#include <cstdio>
#include "constants.h"

namespace openamber_graph {

enum SampleState : uint8_t { STATE_IDLE, STATE_RUN, STATE_SOFTSTART, STATE_DHW, STATE_DEFROST };

struct Sample {
  int16_t tc, sp, tuo, tui;  // 0.1 °C, MISSING when the sensor had no value
  uint8_t hz;
  uint8_t state;
};

static constexpr int16_t MISSING = INT16_MIN;
static constexpr int CAPACITY = 24 * 60;
static constexpr int PLOT_W = 708;
static constexpr int PLOT_H = 274;
static constexpr int LANE_W = 708;
static constexpr int LANE_H = 64;
static constexpr int LANE_MAX_HZ = 70;
static constexpr int MAX_POINTS = 240;
static constexpr int BAND_COLUMNS = 60;
static constexpr int MAX_TICKS = 7;
static constexpr int MAX_HOUR_LINES = 8;

class History {
 public:
  void push(const Sample &s) {
    if (this->data_ == nullptr) {
      esphome::RAMAllocator<Sample> allocator;
      this->data_ = allocator.allocate(CAPACITY);
      if (this->data_ == nullptr) {
        ESP_LOGE("graph", "Could not allocate graph history");
        return;
      }
    }
    this->data_[this->head_] = s;
    this->head_ = (this->head_ + 1) % CAPACITY;
    if (this->size_ < CAPACITY)
      this->size_++;
  }
  int size() const { return this->size_; }
  // age 0 is the newest sample
  const Sample &at_age(int age) const { return this->data_[(this->head_ - 1 - age + 2 * CAPACITY) % CAPACITY]; }

 private:
  Sample *data_{nullptr};
  int head_{0};
  int size_{0};
};

inline int16_t to_tenths(float value) {
  return std::isnan(value) ? MISSING : static_cast<int16_t>(std::lround(value * 10.0f));
}

inline uint8_t read_state() {
  const std::string &dhw = id(state_machine_state_dhw).state;
  const std::string &hc = id(state_machine_state_heat_cool).state;
  if (dhw.rfind("Compressor", 0) == 0)
    return STATE_DHW;
  if (hc == "Defrosting")
    return STATE_DEFROST;
  if (hc == "Compressor softstart" || hc == "Wait compressor running")
    return STATE_SOFTSTART;
  if (hc == "Compressor running")
    return STATE_RUN;
  return STATE_IDLE;
}

inline uint32_t state_color(uint8_t state) {
  switch (state) {
    case STATE_RUN: return 0x2E8B57;
    case STATE_SOFTSTART: return 0xE09F1F;
    case STATE_DHW: return 0xB5306A;
    case STATE_DEFROST: return 0x2B8AD9;
    default: return 0xE5E7EB;
  }
}

class Page {
 public:
  History &history() { return this->history_; }

  void on_interval() {
    Sample s;
    s.tc = to_tenths(id(heat_cool_temperature_tc).state);
    s.sp = to_tenths(id(current_setpoint).state);
    s.tuo = to_tenths(id(outlet_temperature_tuo).state);
    s.tui = to_tenths(id(inlet_temperature_tui).state);
    float hz = id(current_compressor_frequency).state;
    s.hz = std::isnan(hz) ? 0 : static_cast<uint8_t>(std::lround(std::min(std::max(hz, 0.0f), 255.0f)));
    s.state = read_state();
    this->history_.push(s);
    if (this->cursor_age_ >= 0)
      this->cursor_age_++;
    this->refresh();
  }

  void set_hours(int hours) {
    this->hours_ = hours;
    this->cursor_age_ = -1;
    this->refresh();
  }

  void toggle_pipes() {
    this->show_pipes_ = !this->show_pipes_;
    this->refresh();
  }

  void on_opened() {
    this->cursor_age_ = -1;
    this->refresh();
  }

  void set_cursor_age(int age) {
    int span = this->hours_ * 60;
    this->cursor_age_ = std::max(0, std::min(age, std::min(span, this->history_.size() - 1)));
    this->refresh();
  }

  void on_plot_click() {
    lv_indev_t *indev = lv_indev_active();
    if (indev == nullptr || this->history_.size() == 0)
      return;
    lv_point_t point;
    lv_indev_get_point(indev, &point);
    lv_area_t area;
    lv_obj_get_coords(&id(graph_plot), &area);
    int span = this->hours_ * 60;
    this->set_cursor_age(static_cast<int>(std::lround(static_cast<float>(area.x2 - point.x) / (PLOT_W - 1) * span)));
  }

  void refresh() {
    if (lv_obj_has_flag(&id(page_graph), LV_OBJ_FLAG_HIDDEN))
      return;
    uint32_t started = esphome::millis();
    this->create_widgets_();
    this->update_buttons_();
    this->update_values_();
    this->update_plot_();
    this->update_lane_();
    ESP_LOGD("graph", "Redraw prepared in %u ms", static_cast<unsigned>(esphome::millis() - started));
  }

 private:
  lv_obj_t *make_line_(lv_obj_t *parent, uint32_t color, int width) {
    lv_obj_t *line = lv_line_create(parent);
    lv_obj_set_pos(line, 0, 0);
    lv_obj_set_style_line_color(line, lv_color_hex(color), 0);
    lv_obj_set_style_line_width(line, width, 0);
    lv_obj_remove_flag(line, LV_OBJ_FLAG_CLICKABLE);
    return line;
  }

  lv_obj_t *make_label_(lv_obj_t *parent, uint32_t color) {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, id(font_text_20).get_lv_font(), 0);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    lv_obj_add_flag(label, LV_OBJ_FLAG_HIDDEN);
    return label;
  }

  void create_widgets_() {
    if (this->created_)
      return;
    lv_obj_t *plot = &id(graph_plot);
    lv_obj_t *card = &id(graph_card);
    esphome::RAMAllocator<lv_point_precise_t> allocator;
    lv_point_precise_t *points = allocator.allocate(4 * MAX_POINTS);
    if (points == nullptr) {
      ESP_LOGE("graph", "Could not allocate graph points");
      return;
    }
    this->sp_points_ = points;
    this->tui_points_ = this->sp_points_ + MAX_POINTS;
    this->tuo_points_ = this->tui_points_ + MAX_POINTS;
    this->tc_points_ = this->tuo_points_ + MAX_POINTS;

    // The band is a row of plain rectangles: LVGL draws a thick line per segment, which leaves wedge-shaped gaps.
    for (int i = 0; i < BAND_COLUMNS; i++) {
      lv_obj_t *column = lv_obj_create(plot);
      lv_obj_remove_style_all(column);
      lv_obj_remove_flag(column, LV_OBJ_FLAG_CLICKABLE);
      lv_obj_set_style_bg_color(column, lv_color_hex(0xCBDDF1), 0);
      lv_obj_set_style_bg_opa(column, LV_OPA_COVER, 0);
      lv_obj_add_flag(column, LV_OBJ_FLAG_HIDDEN);
      this->band_columns_[i] = column;
    }
    for (int i = 0; i < MAX_TICKS; i++) {
      this->tick_lines_[i] = make_line_(plot, 0xDDE3EA, 1);
      this->tick_labels_[i] = make_label_(card, 0x5B6878);
      lv_obj_set_width(this->tick_labels_[i], 36);
      lv_obj_set_style_text_align(this->tick_labels_[i], LV_TEXT_ALIGN_RIGHT, 0);
    }
    for (int i = 0; i < MAX_HOUR_LINES; i++) {
      this->hour_lines_[i] = make_line_(plot, 0xE7ECF2, 1);
      this->hour_labels_[i] = make_label_(plot, 0x5B6878);
    }
    this->sp_line_ = make_line_(plot, 0x3F76B3, 2);
    this->tui_line_ = make_line_(plot, 0x0A8176, 2);
    this->tuo_line_ = make_line_(plot, 0xC8480C, 2);
    this->tc_line_ = make_line_(plot, 0x0F3564, 4);
    lv_obj_set_style_line_rounded(this->tc_line_, true, 0);
    this->cursor_line_ = make_line_(plot, 0x1F2933, 2);
    lv_obj_add_flag(this->cursor_line_, LV_OBJ_FLAG_HIDDEN);
    this->created_ = true;
  }

  void style_button_(lv_obj_t *button, lv_obj_t *label, bool active) {
    lv_obj_set_style_bg_color(button, lv_color_hex(active ? 0x1F3B68 : 0xE5E7EB), 0);
    lv_obj_set_style_text_color(label, lv_color_hex(active ? 0xFFFFFF : 0x1F2933), 0);
  }

  void update_buttons_() {
    style_button_(&id(graph_btn_1h), &id(graph_btn_1h_label), this->hours_ == 1);
    style_button_(&id(graph_btn_6h), &id(graph_btn_6h_label), this->hours_ == 6);
    style_button_(&id(graph_btn_24h), &id(graph_btn_24h_label), this->hours_ == 24);
    style_button_(&id(graph_btn_pipes), &id(graph_btn_pipes_label), this->show_pipes_);
  }

  // Clock text for a sample, or the time ago when the clock has no valid time yet.
  void format_time_(char *out, size_t size, int age) {
    auto now = id(my_time).now();
    if (now.is_valid()) {
      int minutes = ((now.hour * 60 + now.minute - age) % 1440 + 1440) % 1440;
      snprintf(out, size, "%02d:%02d", minutes / 60, minutes % 60);
    } else if (age >= 60) {
      snprintf(out, size, "-%d u", age / 60);
    } else {
      snprintf(out, size, "-%d min", age);
    }
  }

  void set_temp_(lv_obj_t *label, const char *prefix, int16_t tenths, const char *suffix) {
    char text[32];
    if (tenths == MISSING)
      snprintf(text, sizeof(text), "%s-", prefix);
    else
      snprintf(text, sizeof(text), "%s%.1f%s", prefix, tenths / 10.0f, suffix);
    lv_label_set_text(label, text);
  }

  void set_visible_(lv_obj_t *obj, bool visible) {
    if (visible)
      lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
  }

  bool is_cooling_() { return id(working_mode_switch).active_index().value() == WORKING_MODE_COOLING; }

  void update_values_() {
    lv_obj_t *chip = &id(graph_state_chip);
    lv_obj_t *state = &id(graph_state_value);
    lv_obj_t *tuo = &id(graph_tuo_value);
    lv_obj_t *tui = &id(graph_tui_value);
    bool has_data = this->history_.size() > 0;
    set_visible_(tuo, has_data && this->show_pipes_);
    set_visible_(tui, has_data && this->show_pipes_);
    if (!has_data) {
      lv_label_set_text(&id(graph_tc_caption), "Tc nu");
      lv_label_set_text(&id(graph_tc_value), "-");
      lv_label_set_text(&id(graph_sp_value), "");
      lv_label_set_text(&id(graph_hz_value), "-");
      lv_label_set_text(state, "");
      lv_obj_add_flag(chip, LV_OBJ_FLAG_HIDDEN);
      return;
    }
    bool has_cursor = this->cursor_age_ >= 0 && this->cursor_age_ < this->history_.size();
    const Sample &s = this->history_.at_age(has_cursor ? this->cursor_age_ : 0);
    if (has_cursor) {
      char time_text[16], caption[24];
      format_time_(time_text, sizeof(time_text), this->cursor_age_);
      snprintf(caption, sizeof(caption), "Tc om %s", time_text);
      lv_label_set_text(&id(graph_tc_caption), caption);
    } else {
      lv_label_set_text(&id(graph_tc_caption), "Tc nu");
    }
    set_temp_(&id(graph_tc_value), "", s.tc, " °C");
    set_temp_(&id(graph_sp_value), "Setpoint ", s.sp, "");
    char hz_text[16];
    snprintf(hz_text, sizeof(hz_text), "%u Hz", s.hz);
    lv_label_set_text(&id(graph_hz_value), hz_text);

    const char *state_text = "Staat stil";
    if (s.state == STATE_RUN)
      state_text = this->is_cooling_() ? "Koelen" : "Verwarmen";
    else if (s.state == STATE_SOFTSTART)
      state_text = "Softstart";
    else if (s.state == STATE_DHW)
      state_text = "Tapwater";
    else if (s.state == STATE_DEFROST)
      state_text = "Ontdooien";
    lv_label_set_text(state, state_text);
    set_visible_(chip, s.state != STATE_IDLE);
    lv_obj_set_x(state, s.state == STATE_IDLE ? 20 : 40);
    if (s.state != STATE_IDLE)
      lv_obj_set_style_bg_color(chip, lv_color_hex(state_color(s.state)), 0);

    if (this->show_pipes_) {
      set_temp_(tuo, "Tuo ", s.tuo, "");
      set_temp_(tui, "Tui ", s.tui, "");
    }
  }

  static int age_x_(int age, int span) { return PLOT_W - 1 - (age * (PLOT_W - 1)) / span; }

  // Averages buckets of samples into polyline points. Empty buckets are skipped.
  int build_line_(lv_point_precise_t *points, int max_points, int16_t Sample::*field, float offset, int span, int visible,
                  float low, float px_per_degree) {
    int count = 0;
    int bucket = std::max(1, (span + max_points - 1) / max_points);
    for (int age = visible - 1; age >= 0 && count < max_points; age -= bucket) {
      int sum = 0, n = 0;
      for (int a = age; a > age - bucket && a >= 0; a--) {
        int16_t value = this->history_.at_age(a).*field;
        if (value != MISSING) {
          sum += value;
          n++;
        }
      }
      if (n == 0)
        continue;
      points[count].x = static_cast<lv_value_precise_t>(age_x_(age, span));
      points[count].y = static_cast<lv_value_precise_t>(PLOT_H - (sum / 10.0f / n + offset - low) * px_per_degree);
      count++;
    }
    return count;
  }

  void set_line_(lv_obj_t *line, lv_point_precise_t *points, int16_t Sample::*field, int span, int visible, float low, float px,
                 bool shown) {
    int count = shown ? build_line_(points, MAX_POINTS, field, 0, span, visible, low, px) : 0;
    set_visible_(line, count >= 2);
    if (count >= 2)
      lv_line_set_points(line, points, count);
  }

  void update_plot_() {
    if (!this->created_)
      return;
    int span = this->hours_ * 60;
    int visible = std::min(this->history_.size(), span + 1);
    set_visible_(&id(graph_empty_label), visible == 0);

    bool cooling = this->is_cooling_();
    float above = cooling ? id(compressor_start_delta_cooling).state : id(compressor_stop_delta_heating).state;
    float below = cooling ? id(compressor_stop_delta_cooling).state : id(compressor_start_delta_heating).state;

    // The scale follows Tc and the band. Tuo and Tui may clip at the top during DHW.
    float low = 1000, high = -1000;
    for (int age = 0; age < visible; age++) {
      const Sample &s = this->history_.at_age(age);
      if (s.tc != MISSING) {
        low = std::min(low, s.tc / 10.0f);
        high = std::max(high, s.tc / 10.0f);
      }
      if (s.sp != MISSING) {
        low = std::min(low, s.sp / 10.0f - below);
        high = std::max(high, s.sp / 10.0f + above);
      }
    }
    if (low > high) {
      low = 20;
      high = 40;
    }
    int tick_step = (high - low) > 30 ? 10 : 5;
    int scale_low = static_cast<int>(std::floor(low / tick_step)) * tick_step;
    int scale_high = static_cast<int>(std::ceil(high / tick_step)) * tick_step;
    if (scale_high <= scale_low)
      scale_high = scale_low + tick_step;
    float px = static_cast<float>(PLOT_H) / (scale_high - scale_low);

    this->update_band_(span, visible, above, below, scale_low, px);

    for (int i = 0; i < MAX_TICKS; i++) {
      int value = scale_low + i * tick_step;
      bool used = value <= scale_high;
      set_visible_(this->tick_lines_[i], used);
      set_visible_(this->tick_labels_[i], used);
      if (!used)
        continue;
      int y = PLOT_H - static_cast<int>((value - scale_low) * px);
      this->tick_points_[i][0] = {0, static_cast<lv_value_precise_t>(y)};
      this->tick_points_[i][1] = {PLOT_W, static_cast<lv_value_precise_t>(y)};
      lv_line_set_points(this->tick_lines_[i], this->tick_points_[i], 2);
      lv_label_set_text_fmt(this->tick_labels_[i], "%d", value);
      lv_obj_set_pos(this->tick_labels_[i], 206, 16 + y - 12);
    }

    int every = this->hours_ == 1 ? 15 : (this->hours_ == 6 ? 60 : 240);
    auto now = id(my_time).now();
    int now_minutes = now.is_valid() ? now.hour * 60 + now.minute : 0;
    int used_hours = 0;
    for (int age = 0; age <= span && used_hours < MAX_HOUR_LINES; age++) {
      if (((now_minutes - age) % every + every) % every != 0)
        continue;
      int x = age_x_(age, span);
      this->hour_points_[used_hours][0] = {static_cast<lv_value_precise_t>(x), 0};
      this->hour_points_[used_hours][1] = {static_cast<lv_value_precise_t>(x), 352};
      lv_line_set_points(this->hour_lines_[used_hours], this->hour_points_[used_hours], 2);
      lv_obj_remove_flag(this->hour_lines_[used_hours], LV_OBJ_FLAG_HIDDEN);
      lv_obj_t *label = this->hour_labels_[used_hours];
      bool show_label = x > 28 && x < PLOT_W - 28;
      set_visible_(label, show_label);
      if (show_label) {
        char text[16];
        format_time_(text, sizeof(text), age);
        lv_label_set_text(label, text);
        lv_obj_set_pos(label, x - 27, 358);
      }
      used_hours++;
    }
    for (int i = used_hours; i < MAX_HOUR_LINES; i++) {
      lv_obj_add_flag(this->hour_lines_[i], LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(this->hour_labels_[i], LV_OBJ_FLAG_HIDDEN);
    }

    set_line_(this->sp_line_, this->sp_points_, &Sample::sp, span, visible, scale_low, px, true);
    set_line_(this->tui_line_, this->tui_points_, &Sample::tui, span, visible, scale_low, px, this->show_pipes_);
    set_line_(this->tuo_line_, this->tuo_points_, &Sample::tuo, span, visible, scale_low, px, this->show_pipes_);
    set_line_(this->tc_line_, this->tc_points_, &Sample::tc, span, visible, scale_low, px, true);

    bool has_cursor = this->cursor_age_ >= 0 && this->cursor_age_ < visible;
    set_visible_(this->cursor_line_, has_cursor);
    if (has_cursor) {
      int x = age_x_(this->cursor_age_, span);
      this->cursor_points_[0] = {static_cast<lv_value_precise_t>(x), 0};
      this->cursor_points_[1] = {static_cast<lv_value_precise_t>(x), 352};
      lv_line_set_points(this->cursor_line_, this->cursor_points_, 2);
    }
  }

  void update_band_(int span, int visible, float above, float below, float low, float px) {
    int height = static_cast<int>(std::lround((above + below) * px));
    for (int i = 0; i < BAND_COLUMNS; i++) {
      int x0 = i * PLOT_W / BAND_COLUMNS;
      int x1 = (i + 1) * PLOT_W / BAND_COLUMNS;
      int newest = std::max(0, (PLOT_W - x1) * span / PLOT_W);
      int oldest = std::min((PLOT_W - x0) * span / PLOT_W, visible - 1);
      int sum = 0, n = 0;
      for (int age = newest; age <= oldest; age++) {
        int16_t value = this->history_.at_age(age).sp;
        if (value != MISSING) {
          sum += value;
          n++;
        }
      }
      set_visible_(this->band_columns_[i], n > 0);
      if (n == 0)
        continue;
      lv_obj_set_pos(this->band_columns_[i], x0, static_cast<int>(PLOT_H - (sum / 10.0f / n + above - low) * px));
      lv_obj_set_size(this->band_columns_[i], x1 - x0, height);
    }
  }

  // Frequency sets the bar height, the control loop state sets the colour.
  void update_lane_() {
    lv_obj_t *canvas = &id(graph_lane);
    lv_canvas_fill_bg(canvas, lv_color_hex(0xE5E7EB), LV_OPA_COVER);
    int span = this->hours_ * 60;
    int visible = std::min(this->history_.size(), span + 1);
    for (int x = 0; x < LANE_W; x += 3) {
      int age = static_cast<int>(std::lround(static_cast<float>(LANE_W - 1 - x) / (LANE_W - 1) * span));
      if (age >= visible)
        continue;
      const Sample &s = this->history_.at_age(age);
      if (s.hz == 0)
        continue;
      int height = std::min<int>(s.hz, LANE_MAX_HZ) * LANE_H / LANE_MAX_HZ;
      lv_color_t color = lv_color_hex(state_color(s.state == STATE_IDLE ? STATE_RUN : s.state));
      for (int dx = 0; dx < 3 && x + dx < LANE_W; dx++)
        for (int y = LANE_H - height; y < LANE_H; y++)
          lv_canvas_set_px(canvas, x + dx, y, color, LV_OPA_COVER);
    }
    lv_obj_invalidate(canvas);
  }

  History history_;
  int hours_{6};
  int cursor_age_{-1};
  bool show_pipes_{false};
  bool created_{false};
  lv_obj_t *band_columns_[BAND_COLUMNS]{};
  lv_obj_t *sp_line_{nullptr}, *tui_line_{nullptr}, *tuo_line_{nullptr}, *tc_line_{nullptr}, *cursor_line_{nullptr};
  lv_obj_t *tick_lines_[MAX_TICKS]{}, *tick_labels_[MAX_TICKS]{};
  lv_obj_t *hour_lines_[MAX_HOUR_LINES]{}, *hour_labels_[MAX_HOUR_LINES]{};
  lv_point_precise_t *sp_points_{nullptr}, *tui_points_{nullptr}, *tuo_points_{nullptr}, *tc_points_{nullptr};
  lv_point_precise_t tick_points_[MAX_TICKS][2]{}, hour_points_[MAX_HOUR_LINES][2]{}, cursor_points_[2]{};
};

inline Page &page() {
  static Page instance;
  return instance;
}

}  // namespace openamber_graph
