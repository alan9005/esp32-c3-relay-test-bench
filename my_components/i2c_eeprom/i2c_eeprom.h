#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace i2c_eeprom {

class I2CEEPROMComponent : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_memory_size(uint32_t size) { this->memory_size_ = size; }

  /// Read `length` bytes starting at EEPROM address `addr` into `buffer`.
  /// Returns true on success.
  bool get(uint16_t addr, uint8_t *buffer, uint16_t length);

  /// Write `length` bytes from `buffer` into EEPROM starting at address `addr`.
  /// Handles page-boundary splitting internally.
  /// Returns true on success.
  bool set(uint16_t addr, const uint8_t *buffer, uint16_t length);

  Trigger<> *get_on_setup_trigger() { return &this->on_setup_trigger_; }

 protected:
  /// Write a single page-aligned chunk (must not cross a page boundary).
  bool write_page_(uint16_t addr, const uint8_t *data, uint8_t length);

  /// Determine the EEPROM page size based on the configured memory size.
  uint8_t get_page_size_() const;

  uint32_t memory_size_{32768};  // default 32KB (AT24C256)
  Trigger<> on_setup_trigger_;
};

}  // namespace i2c_eeprom
}  // namespace esphome
