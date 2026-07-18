#include "teleruptor.h"

#include "esphome/core/log.h"

#include <cinttypes>
#include <utility>

namespace esphome {
namespace teleruptor {

static const char *const TAG = "teleruptor";

void Teleruptor::setup() {
  ESP_LOGD(TAG, "Setting up teleruptor in %s mode", this->mode_name_());

  this->setup_detection_();
  this->setup_input_();
  this->setup_pulse_target_();
  this->setup_relay_target_();

  if (this->mode_ == TeleruptorMode::PULSE_FEEDBACK) {
    if (!this->has_detection_source_()) {
      ESP_LOGE(TAG, "No detection source configured in pulse_feedback mode");
      this->mark_failed();
    }
    if (!this->has_pulse_target_()) {
      ESP_LOGE(TAG, "No pulse output configured in pulse_feedback mode");
      this->mark_failed();
    }
  } else if (this->mode_ == TeleruptorMode::PULSE_OPTIMISTIC) {
    if (!this->has_pulse_target_()) {
      ESP_LOGE(TAG, "No pulse output configured in pulse_optimistic mode");
      this->mark_failed();
    }
    if (!this->has_state_) {
      ESP_LOGW(TAG, "pulse_optimistic mode uses an assumed state. Physical wall button presses can desynchronize it.");
      this->set_state_(this->default_state_);
    }
  } else if (this->mode_ == TeleruptorMode::RELAY_LATCH) {
    if (!this->has_input_source_()) {
      ESP_LOGE(TAG, "No input configured in relay_latch mode");
      this->mark_failed();
    }
    if (!this->has_relay_target_()) {
      ESP_LOGE(TAG, "No relay output configured in relay_latch mode");
      this->mark_failed();
    }
    this->set_state_(this->default_state_, false);
    this->write_relay_target_(this->state_);
    this->publish_state_();
  }
}

void Teleruptor::loop() {
  const uint32_t now = millis();

  if (this->detection_pin_ != nullptr && this->detection_ == nullptr) {
    this->read_detection_pin_();
  }

  if (this->input_pin_ != nullptr && this->input_ == nullptr) {
    this->read_input_pin_();
  }

  if (this->pulse_active_ && now - this->pulse_started_ms_ >= this->pulse_duration_) {
    this->stop_pulse_();
  }

  if (this->mode_ == TeleruptorMode::PULSE_FEEDBACK && this->command_in_progress_ && !this->pulse_active_ &&
      now - this->command_started_ms_ >= this->feedback_timeout_) {
    ESP_LOGW(TAG, "Feedback timeout after command toward %s. Keeping current state: %s", ONOFF(this->desired_state_),
             this->has_state_ ? ONOFF(this->state_) : "unknown");
    this->command_in_progress_ = false;
    this->feedback_timeout_fault_ = true;
    if (this->has_state_) {
      this->publish_state_();
    }
  }

  this->update_unstable_window_();
}

void Teleruptor::dump_config() {
  ESP_LOGCONFIG(TAG, "Teleruptor:");
  ESP_LOGCONFIG(TAG, "  Mode: %s", this->mode_name_());
  ESP_LOGCONFIG(TAG, "  Default state: %s", ONOFF(this->default_state_));

  if (this->detection_ != nullptr) {
    LOG_BINARY_SENSOR("  Detection: ", "physical feedback", this->detection_);
  } else if (this->detection_pin_ != nullptr) {
    LOG_PIN("  Detection Pin: ", this->detection_pin_);
    ESP_LOGCONFIG(TAG, "  Detection debounce: %" PRIu32 " ms", this->detection_debounce_);
  } else {
    ESP_LOGCONFIG(TAG, "  Detection: not configured");
  }

  if (this->input_ != nullptr) {
    LOG_BINARY_SENSOR("  Input: ", "physical button input", this->input_);
  } else if (this->input_pin_ != nullptr) {
    LOG_PIN("  Input Pin: ", this->input_pin_);
    ESP_LOGCONFIG(TAG, "  Input debounce: %" PRIu32 " ms", this->input_debounce_);
  } else {
    ESP_LOGCONFIG(TAG, "  Input: not configured");
  }

  if (this->pulse_output_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Pulse output: output component");
  } else if (this->pulse_pin_ != nullptr) {
    LOG_PIN("  Pulse Pin: ", this->pulse_pin_);
  } else {
    ESP_LOGCONFIG(TAG, "  Pulse output: not configured");
  }

  if (this->relay_output_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Relay output: output component");
  } else if (this->relay_pin_ != nullptr) {
    LOG_PIN("  Relay Pin: ", this->relay_pin_);
  } else {
    ESP_LOGCONFIG(TAG, "  Relay output: not configured");
  }

  ESP_LOGCONFIG(TAG, "  Pulse duration: %" PRIu32 " ms", this->pulse_duration_);
  ESP_LOGCONFIG(TAG, "  Command cooldown: %" PRIu32 " ms", this->command_cooldown_);
  ESP_LOGCONFIG(TAG, "  Feedback timeout: %" PRIu32 " ms", this->feedback_timeout_);
  ESP_LOGCONFIG(TAG, "  State inverted: %s", YESNO(this->state_inverted_));
  ESP_LOGCONFIG(TAG, "  Sync on boot: %s", YESNO(this->sync_on_boot_));
  ESP_LOGCONFIG(TAG, "  Allow pulse without feedback: %s", YESNO(this->allow_pulse_without_feedback_));
  ESP_LOGCONFIG(TAG, "  Block after feedback timeout: %s", YESNO(this->block_after_feedback_timeout_));
  ESP_LOGCONFIG(TAG, "  Unstable detection: %s", YESNO(this->unstable_detection_enabled_));
  if (this->unstable_detection_enabled_) {
    ESP_LOGCONFIG(TAG, "    Window: %" PRIu32 " ms", this->unstable_detection_window_);
    ESP_LOGCONFIG(TAG, "    Max changes: %" PRIu32, this->unstable_detection_max_changes_);
    ESP_LOGCONFIG(TAG, "    Block commands: %s", YESNO(this->unstable_detection_block_commands_));
  }
}

const char *Teleruptor::mode_name_() const {
  switch (this->mode_) {
    case TeleruptorMode::PULSE_FEEDBACK:
      return "pulse_feedback";
    case TeleruptorMode::PULSE_OPTIMISTIC:
      return "pulse_optimistic";
    case TeleruptorMode::RELAY_LATCH:
      return "relay_latch";
    default:
      return "unknown";
  }
}

bool Teleruptor::has_detection_source_() const { return this->detection_ != nullptr || this->detection_pin_ != nullptr; }

bool Teleruptor::has_input_source_() const { return this->input_ != nullptr || this->input_pin_ != nullptr; }

bool Teleruptor::has_pulse_target_() const { return this->pulse_output_ != nullptr || this->pulse_pin_ != nullptr; }

bool Teleruptor::has_relay_target_() const { return this->relay_output_ != nullptr || this->relay_pin_ != nullptr; }

void Teleruptor::setup_detection_() {
  if (this->detection_pin_ != nullptr && this->detection_ == nullptr) {
    this->detection_pin_->setup();
    const bool raw_state = this->detection_pin_->digital_read();
    this->last_detection_raw_state_ = raw_state;
    this->detection_debounced_state_ = raw_state;
    this->has_detection_raw_state_ = true;
    this->detection_raw_changed_ms_ = millis();
    if (this->sync_on_boot_) {
      this->accept_raw_feedback_(raw_state);
    }
  }

  if (this->detection_ != nullptr) {
    this->detection_->add_on_state_callback([this](bool state) { this->accept_raw_feedback_(state); });
    if (this->sync_on_boot_ && this->detection_->has_state()) {
      this->accept_raw_feedback_(this->detection_->state);
    }
  }
}

void Teleruptor::setup_input_() {
  if (this->input_pin_ != nullptr && this->input_ == nullptr) {
    this->input_pin_->setup();
    const bool raw_state = this->input_pin_->digital_read();
    this->last_input_raw_state_ = raw_state;
    this->input_debounced_state_ = raw_state;
    this->has_input_raw_state_ = true;
    this->input_raw_changed_ms_ = millis();
  }

  if (this->input_ != nullptr) {
    this->input_->add_on_state_callback([this](bool state) { this->accept_input_state_(state); });
  }
}

void Teleruptor::setup_pulse_target_() {
  if (this->pulse_pin_ != nullptr && this->pulse_output_ == nullptr) {
    this->pulse_pin_->setup();
  }
  this->write_pulse_target_(false);
}

void Teleruptor::setup_relay_target_() {
  if (this->relay_pin_ != nullptr && this->relay_output_ == nullptr) {
    this->relay_pin_->setup();
  }
}

void Teleruptor::add_state_callback(std::function<void(bool)> &&callback) {
  this->state_callbacks_.push_back(std::move(callback));
  if (this->has_state_) {
    this->state_callbacks_.back()(this->state_);
  }
}

void Teleruptor::request_state(bool target_state) {
  if (this->mode_ == TeleruptorMode::RELAY_LATCH) {
    this->apply_relay_state_(target_state);
    return;
  }

  if (!this->has_pulse_target_()) {
    ESP_LOGE(TAG, "Command ignored: no pulse output configured");
    return;
  }

  if (!this->has_state_) {
    ESP_LOGW(TAG, "Command ignored: feedback/state is still unknown");
    return;
  }

  if (target_state == this->state_) {
    ESP_LOGD(TAG, "Command ignored: state is already %s", ONOFF(target_state));
    this->publish_state_();
    return;
  }

  if (!this->can_start_pulse_()) {
    this->publish_state_();
    return;
  }

  if (this->mode_ == TeleruptorMode::PULSE_FEEDBACK) {
    this->desired_state_ = target_state;
    this->command_in_progress_ = true;
    this->command_started_ms_ = millis();
    this->start_pulse_();
    return;
  }

  // pulse_optimistic: the new state is assumed immediately because there is no required feedback.
  this->start_pulse_();
  this->set_state_(target_state);
}

void Teleruptor::request_toggle_pulse() {
  if (this->mode_ == TeleruptorMode::RELAY_LATCH) {
    this->toggle_relay_state_();
    return;
  }

  if (!this->has_pulse_target_()) {
    ESP_LOGE(TAG, "Pulse ignored: no pulse output configured");
    return;
  }

  if (!this->has_state_ && !this->allow_pulse_without_feedback_ && this->mode_ == TeleruptorMode::PULSE_FEEDBACK) {
    ESP_LOGW(TAG, "Pulse ignored: feedback is unknown and allow_pulse_without_feedback is false");
    return;
  }

  if (!this->can_start_pulse_()) {
    if (this->has_state_) {
      this->publish_state_();
    }
    return;
  }

  if (this->mode_ == TeleruptorMode::PULSE_FEEDBACK) {
    if (this->has_state_) {
      this->desired_state_ = !this->state_;
      this->command_in_progress_ = true;
      this->command_started_ms_ = millis();
    } else {
      ESP_LOGW(TAG, "Direct pulse sent while feedback is unknown");
      this->command_in_progress_ = false;
    }
    this->start_pulse_();
    return;
  }

  // pulse_optimistic
  const bool next_state = this->has_state_ ? !this->state_ : !this->default_state_;
  this->start_pulse_();
  this->set_state_(next_state);
}

void Teleruptor::read_detection_pin_() {
  const uint32_t now = millis();
  const bool raw_state = this->detection_pin_->digital_read();

  if (!this->has_detection_raw_state_) {
    this->last_detection_raw_state_ = raw_state;
    this->detection_debounced_state_ = raw_state;
    this->detection_raw_changed_ms_ = now;
    this->has_detection_raw_state_ = true;
    this->accept_raw_feedback_(raw_state);
    return;
  }

  if (raw_state != this->last_detection_raw_state_) {
    this->last_detection_raw_state_ = raw_state;
    this->detection_raw_changed_ms_ = now;
    return;
  }

  if (raw_state != this->detection_debounced_state_ && now - this->detection_raw_changed_ms_ >= this->detection_debounce_) {
    this->detection_debounced_state_ = raw_state;
    this->accept_raw_feedback_(raw_state);
  }
}

void Teleruptor::read_input_pin_() {
  const uint32_t now = millis();
  const bool raw_state = this->input_pin_->digital_read();

  if (!this->has_input_raw_state_) {
    this->last_input_raw_state_ = raw_state;
    this->input_debounced_state_ = raw_state;
    this->input_raw_changed_ms_ = now;
    this->has_input_raw_state_ = true;
    return;
  }

  if (raw_state != this->last_input_raw_state_) {
    this->last_input_raw_state_ = raw_state;
    this->input_raw_changed_ms_ = now;
    return;
  }

  if (raw_state != this->input_debounced_state_ && now - this->input_raw_changed_ms_ >= this->input_debounce_) {
    this->input_debounced_state_ = raw_state;
    this->accept_input_state_(raw_state);
  }
}

void Teleruptor::accept_raw_feedback_(bool raw_state) {
  const bool new_state = this->state_inverted_ ? !raw_state : raw_state;
  const bool changed = !this->has_state_ || new_state != this->state_;

  this->set_state_(new_state, false);

  if (changed) {
    ESP_LOGD(TAG, "Feedback state changed: %s", ONOFF(this->state_));
    this->record_feedback_change_();
    this->feedback_timeout_fault_ = false;
  }

  if (this->mode_ == TeleruptorMode::PULSE_FEEDBACK && this->command_in_progress_ && this->state_ == this->desired_state_) {
    ESP_LOGD(TAG, "Command confirmed by feedback: %s", ONOFF(this->state_));
    this->command_in_progress_ = false;
  }

  this->publish_state_();
}

void Teleruptor::accept_input_state_(bool active) {
  if (this->mode_ != TeleruptorMode::RELAY_LATCH) {
    return;
  }

  // Only the active edge toggles the latch. Release must not toggle again.
  if (active) {
    this->toggle_relay_state_();
  }
}

void Teleruptor::set_state_(bool new_state, bool publish) {
  this->state_ = new_state;
  this->has_state_ = true;
  if (publish) {
    this->publish_state_();
  }
}

void Teleruptor::publish_state_() {
  if (!this->has_state_) {
    return;
  }
  for (auto &callback : this->state_callbacks_) {
    callback(this->state_);
  }
}

void Teleruptor::record_feedback_change_() {
  if (!this->unstable_detection_enabled_ || this->unstable_detection_window_ == 0 ||
      this->unstable_detection_max_changes_ == 0) {
    return;
  }

  const uint32_t now = millis();
  if (this->unstable_window_started_ms_ == 0 || now - this->unstable_window_started_ms_ > this->unstable_detection_window_) {
    this->unstable_window_started_ms_ = now;
    this->unstable_change_count_ = 1;
    this->feedback_unstable_ = false;
    return;
  }

  this->unstable_change_count_++;
  if (this->unstable_change_count_ > this->unstable_detection_max_changes_) {
    this->feedback_unstable_ = true;
    if (this->last_unstable_log_ms_ == 0 || now - this->last_unstable_log_ms_ > this->unstable_detection_window_) {
      ESP_LOGW(TAG, "Unstable feedback detected: %" PRIu32 " changes in %" PRIu32 " ms",
               this->unstable_change_count_, this->unstable_detection_window_);
      this->last_unstable_log_ms_ = now;
    }
  }
}

void Teleruptor::update_unstable_window_() {
  if (!this->feedback_unstable_ || this->unstable_window_started_ms_ == 0) {
    return;
  }

  const uint32_t now = millis();
  if (now - this->unstable_window_started_ms_ > this->unstable_detection_window_) {
    this->feedback_unstable_ = false;
    this->unstable_change_count_ = 0;
    this->unstable_window_started_ms_ = now;
    ESP_LOGI(TAG, "Feedback is no longer considered unstable");
  }
}

bool Teleruptor::can_start_pulse_() const {
  const uint32_t now = millis();

  if (this->pulse_active_) {
    ESP_LOGW(TAG, "Command ignored: pulse already active");
    return false;
  }

  if (this->command_in_progress_) {
    ESP_LOGW(TAG, "Command ignored: waiting for previous feedback");
    return false;
  }

  if (this->feedback_unstable_ && this->unstable_detection_block_commands_) {
    ESP_LOGW(TAG, "Command ignored: feedback is unstable and unstable_detection.block_commands is true");
    return false;
  }

  if (this->feedback_timeout_fault_ && this->block_after_feedback_timeout_) {
    ESP_LOGW(TAG, "Command ignored: previous feedback timeout fault and block_after_feedback_timeout is true");
    return false;
  }

  if (this->last_pulse_ms_ != 0 && now - this->last_pulse_ms_ < this->command_cooldown_) {
    ESP_LOGW(TAG, "Command ignored: anti-double-pulse cooldown active");
    return false;
  }

  return true;
}

void Teleruptor::start_pulse_() {
  this->pulse_active_ = true;
  this->pulse_started_ms_ = millis();
  this->last_pulse_ms_ = this->pulse_started_ms_;

  ESP_LOGD(TAG, "Pulse ON");
  this->write_pulse_target_(true);
}

void Teleruptor::stop_pulse_() {
  ESP_LOGD(TAG, "Pulse OFF");
  this->write_pulse_target_(false);
  this->pulse_active_ = false;
}

void Teleruptor::write_pulse_target_(bool state) {
  if (this->pulse_output_ != nullptr) {
    this->pulse_output_->set_state(state);
  } else if (this->pulse_pin_ != nullptr) {
    this->pulse_pin_->digital_write(state);
  }
}

void Teleruptor::apply_relay_state_(bool target_state) {
  if (!this->has_relay_target_()) {
    ESP_LOGE(TAG, "Relay command ignored: no relay output configured");
    return;
  }

  if (this->has_state_ && target_state == this->state_) {
    ESP_LOGD(TAG, "Relay state already %s", ONOFF(target_state));
    this->publish_state_();
    return;
  }

  this->set_state_(target_state, false);
  this->write_relay_target_(this->state_);
  this->publish_state_();
}

void Teleruptor::toggle_relay_state_() {
  const bool next_state = this->has_state_ ? !this->state_ : !this->default_state_;
  this->apply_relay_state_(next_state);
}

void Teleruptor::write_relay_target_(bool state) {
  if (this->relay_output_ != nullptr) {
    this->relay_output_->set_state(state);
  } else if (this->relay_pin_ != nullptr) {
    this->relay_pin_->digital_write(state);
  }
}

}  // namespace teleruptor
}  // namespace esphome
