#pragma once

#include "esphome/components/sensor/sensor.h"
//#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"

namespace esphome {
namespace pylontech {

enum ENUMPollingCommand {
  POLLING_pwrsys = 0,
  POLLING_pwr = 1,
  POLLING_stat = 2,
  POLLING_getpwr = 3,
  POLLING_bat = 4,
  POLLING_info = 5,
};
struct PollingCommand {
  uint8_t *command;
  uint8_t length = 0;
  uint8_t errors;
  ENUMPollingCommand identifier;
};

#define PYLONTECH_VALUED_ENTITY_(type, name, polling_command, value_type) \
 protected: \
  value_type value_##name##_; \
  PYLONTECH_ENTITY_(type, name, polling_command)

#define PYLONTECH_ENTITY_(type, name, polling_command) \
 protected: \
  type *name##_{}; /* NOLINT */ \
\
 public: \
  void set_##name(type *name) { /* NOLINT */ \
    this->name##_ = name; \
    this->add_polling_command_(#polling_command, POLLING_##polling_command); \
  }

#define PYLONTECH_SENSOR(name, polling_command, value_type) \
  PYLONTECH_VALUED_ENTITY_(sensor::Sensor, name, polling_command, value_type)

#define PYLONTECH_VALUED_TEXT_SENSOR(name, polling_command, value_type) \
  PYLONTECH_VALUED_ENTITY_(text_sensor::TextSensor, name, polling_command, value_type)
#define PYLONTECH_TEXT_SENSOR(name, polling_command) PYLONTECH_ENTITY_(text_sensor::TextSensor, name, polling_command)

static const uint8_t NUM_BUFFERS = 20;
static const uint8_t TEXT_SENSOR_MAX_LEN = 8;

class PylontechComponent : public PollingComponent, public uart::UARTDevice {
  
  PYLONTECH_SENSOR(systeme_is, pwrsys, float)
  PYLONTECH_TEXT_SENSOR(last_pwrsys, pwrsys)

 public:
  PylontechComponent();

  void switch_command(const std::string &command);
  /// Setup the sensor and test for a connection.
  void setup() override;
  /// Read data once available
  void loop() override;
  void dump_config() override;
  /// Schedule data readings.
  void update() override;

  float get_setup_priority() const override; //possible delete

  void register_listener(PylontechListener *listener) { this->listeners_.push_back(listener); }

 protected:
  void process_line_(std::string &buffer);

  // ring buffer
  std::string buffer_[NUM_BUFFERS];
  int buffer_index_write_ = 0;
  int buffer_index_read_ = 0;

  std::vector<PylontechListener *> listeners_{};

  //Pipsolar Import
  static const size_t PYLONTECH_READ_BUFFER_LENGTH = 110;  // maximum supported answer length
  static const size_t COMMAND_QUEUE_LENGTH = 10;
  static const size_t COMMAND_TIMEOUT = 5000;
  uint32_t last_poll_ = 0;
  void add_polling_command_(const char *command, ENUMPollingCommand polling_command);
  void empty_uart_buffer_();
  uint8_t check_incoming_crc_();
  uint8_t check_incoming_length_(uint8_t length);
  uint16_t pylontech_crc_(uint8_t *msg, uint8_t len); //utile ?
  uint8_t send_next_command_();
  void send_next_poll_();
  void queue_command_(const char *command, uint8_t length);
  std::string command_queue_[COMMAND_QUEUE_LENGTH];
  uint8_t command_queue_position_ = 0;
  uint8_t read_buffer_[PYLONTECH_READ_BUFFER_LENGTH];
  size_t read_pos_{0};

  uint32_t command_start_millis_ = 0;
  uint8_t state_;
  enum State {
    STATE_IDLE = 0,
    STATE_POLL = 1,
    STATE_COMMAND = 2,
    STATE_POLL_COMPLETE = 3,
    STATE_COMMAND_COMPLETE = 4,
    STATE_POLL_CHECKED = 5,
    STATE_POLL_DECODED = 6,
  };

  uint8_t last_polling_command_ = 0;
  PollingCommand used_polling_commands_[15];
};


class PylontechListener {
 public:
  struct LineContents {
    int bat_num = 0, volt, curr, tempr, tlow, thigh, vlow, vhigh, coulomb, mostempr;
    char base_st[TEXT_SENSOR_MAX_LEN], volt_st[TEXT_SENSOR_MAX_LEN], curr_st[TEXT_SENSOR_MAX_LEN],
        temp_st[TEXT_SENSOR_MAX_LEN];
  };

  virtual void on_line_read(LineContents *line);
  virtual void dump_config();
};


}  // namespace pylontech
}  // namespace esphome
