#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include <functional>
#include <vector>

namespace esphome {
namespace teleruptor {

enum class TeleruptorMode : uint8_t {
  PULSE_FEEDBACK = 0,
  PULSE_OPTIMISTIC = 1,
  RELAY_LATCH = 2,
};

class Teleruptor : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void set_mode(uint8_t mode) { this->mode_ = static_cast<TeleruptorMode>(mode); }
  void set_default_state(bool default_state) { this->default_state_ = default_state; }

  void set_detection(binary_sensor::BinarySensor *detection) { this->detection_ = detection; }
  void set_detection_pin(GPIOPin *pin) { this->detection_pin_ = pin; }
  void set_detection_debounce(uint32_t debounce) { this->detection_debounce_ = debounce; }

  void set_input(binary_sensor::BinarySensor *input) { this->input_ = input; }
  void set_input_pin(GPIOPin *pin) { this->input_pin_ = pin; }
  void set_input_debounce(uint32_t debounce) { this->input_debounce_ = debounce; }

  void set_pulse_output(output::BinaryOutput *output) { this->pulse_output_ = output; }
  void set_pulse_pin(GPIOPin *pin) { this->pulse_pin_ = pin; }

  void set_relay_output(output::BinaryOutput *output) { this->relay_output_ = output; }
  void set_relay_pin(GPIOPin *pin) { this->relay_pin_ = pin; }

  void set_pulse_duration(uint32_t duration) { this->pulse_duration_ = duration; }
  void set_command_cooldown(uint32_t cooldown) { this->command_cooldown_ = cooldown; }
  void set_feedback_timeout(uint32_t timeout) { this->feedback_timeout_ = timeout; }
  void set_state_inverted(bool inverted) { this->state_inverted_ = inverted; }
  void set_sync_on_boot(bool sync_on_boot) { this->sync_on_boot_ = sync_on_boot; }

  void set_allow_pulse_without_feedback(bool allow) { this->allow_pulse_without_feedback_ = allow; }
  void set_block_after_feedback_timeout(bool block) { this->block_after_feedback_timeout_ = block; }
  void set_unstable_detection_enabled(bool enabled) { this->unstable_detection_enabled_ = enabled; }
  void set_unstable_detection_window(uint32_t window) { this->unstable_detection_window_ = window; }
  void set_unstable_detection_max_changes(uint32_t max_changes) { this->unstable_detection_max_changes_ = max_changes; }
  void set_unstable_detection_block_commands(bool block) { this->unstable_detection_block_commands_ = block; }

  bool has_state() const { return this->has_state_; }
  bool state() const { return this->state_; }
  bool command_in_progress() const { return this->command_in_progress_; }
  bool pulse_active() const { return this->pulse_active_; }
  bool feedback_unstable() const { return this->feedback_unstable_; }
  bool feedback_timeout_fault() const { return this->feedback_timeout_fault_; }
  TeleruptorMode mode() const { return this->mode_; }

  void add_state_callback(std::function<void(bool)> &&callback);

  /// Request a logical ON/OFF state from Home Assistant.
  void request_state(bool target_state);

  /// Software button action.
  /// In pulse modes: sends one pulse.
  /// In relay_latch mode: toggles the maintained relay state.
  void request_toggle_pulse();

 protected:
  TeleruptorMode mode_{TeleruptorMode::PULSE_FEEDBACK};

  binary_sensor::BinarySensor *detection_{nullptr};
  GPIOPin *detection_pin_{nullptr};
  binary_sensor::BinarySensor *input_{nullptr};
  GPIOPin *input_pin_{nullptr};
  output::BinaryOutput *pulse_output_{nullptr};
  GPIOPin *pulse_pin_{nullptr};
  output::BinaryOutput *relay_output_{nullptr};
  GPIOPin *relay_pin_{nullptr};

  uint32_t detection_debounce_{100};
  uint32_t input_debounce_{50};
  uint32_t pulse_duration_{200};
  uint32_t command_cooldown_{1000};
  uint32_t feedback_timeout_{3000};
  bool state_inverted_{false};
  bool sync_on_boot_{true};
  bool default_state_{false};

  bool allow_pulse_without_feedback_{false};
  bool block_after_feedback_timeout_{false};
  bool unstable_detection_enabled_{true};
  uint32_t unstable_detection_window_{10000};
  uint32_t unstable_detection_max_changes_{10};
  bool unstable_detection_block_commands_{false};

  bool has_state_{false};
  bool state_{false};
  bool desired_state_{false};

  bool command_in_progress_{false};
  uint32_t command_started_ms_{0};

  bool pulse_active_{false};
  uint32_t pulse_started_ms_{0};
  uint32_t last_pulse_ms_{0};

  bool has_detection_raw_state_{false};
  bool last_detection_raw_state_{false};
  uint32_t detection_raw_changed_ms_{0};
  bool detection_debounced_state_{false};

  bool has_input_raw_state_{false};
  bool last_input_raw_state_{false};
  uint32_t input_raw_changed_ms_{0};
  bool input_debounced_state_{false};

  bool feedback_unstable_{false};
  bool feedback_timeout_fault_{false};
  uint32_t unstable_window_started_ms_{0};
  uint32_t unstable_change_count_{0};
  uint32_t last_unstable_log_ms_{0};

  std::vector<std::function<void(bool)>> state_callbacks_;

  const char *mode_name_() const;
  bool has_detection_source_() const;
  bool has_input_source_() const;
  bool has_pulse_target_() const;
  bool has_relay_target_() const;

  void setup_detection_();
  void setup_input_();
  void setup_pulse_target_();
  void setup_relay_target_();

  void read_detection_pin_();
  void read_input_pin_();
  void accept_raw_feedback_(bool raw_state);
  void accept_input_state_(bool active);

  void set_state_(bool new_state, bool publish = true);
  void publish_state_();
  void record_feedback_change_();
  void update_unstable_window_();

  bool can_start_pulse_() const;
  void start_pulse_();
  void stop_pulse_();
  void write_pulse_target_(bool state);

  void apply_relay_state_(bool target_state);
  void toggle_relay_state_();
  void write_relay_target_(bool state);
};

}  // namespace teleruptor
}  // namespace esphome

// Include facade entity declarations when the core component is used with nested entities.
#include "teleruptor_light.h"
#include "teleruptor_switch.h"
#include "teleruptor_button.h"
#include "teleruptor_binary_sensor.h"
