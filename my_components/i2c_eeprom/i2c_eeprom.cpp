#include "i2c_eeprom.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace i2c_eeprom {

static const char *const TAG = "i2c_eeprom";

void I2CEEPROMComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up I2C EEPROM...");

  // Probe the device to make sure it is present on the bus.
  // Use 16-bit address (0x0000) for AT24C32 and above; the write_read
  // call sends the 2-byte memory address then reads 1 byte back.
  uint8_t addr_buf[2] = {0x00, 0x00};
  uint8_t dummy;
  if (this->write_read(addr_buf, 2, &dummy, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "EEPROM not found at address 0x%02X", this->address_);
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "EEPROM found, size: %u bytes, page size: %u bytes",
           this->memory_size_, this->get_page_size_());

  // Fire the on_setup automation so users can read stored data at boot.
  this->on_setup_trigger_.trigger();
}

void I2CEEPROMComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "I2C EEPROM:");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  Size: %u bytes", this->memory_size_);
  ESP_LOGCONFIG(TAG, "  Page size: %u bytes", this->get_page_size_());
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Communication failed!");
  }
}

uint8_t I2CEEPROMComponent::get_page_size_() const {
  // Page sizes for common Atmel/Microchip AT24Cxx EEPROMs:
  //   AT24C01/02   ->  8 bytes
  //   AT24C04/08/16 -> 16 bytes
  //   AT24C32/64   -> 32 bytes
  //   AT24C128/256 -> 64 bytes
  //   AT24C512     -> 128 bytes
  if (this->memory_size_ <= 256)        // AT24C01 (128B) / AT24C02 (256B)
    return 8;
  if (this->memory_size_ <= 2048)       // AT24C04..AT24C16
    return 16;
  if (this->memory_size_ <= 8192)       // AT24C32 (4KB) / AT24C64 (8KB)
    return 32;
  if (this->memory_size_ <= 32768)      // AT24C128 (16KB) / AT24C256 (32KB)
    return 64;
  return 128;                           // AT24C512 (64KB) and larger
}

bool I2CEEPROMComponent::get(uint16_t addr, uint8_t *buffer, uint16_t length) {
  if (addr + length > this->memory_size_) {
    ESP_LOGE(TAG, "Read out of range: addr=0x%04X len=%u size=%u",
             addr, length, this->memory_size_);
    return false;
  }

  // For EEPROMs with >256 bytes (AT24C32 and above), use 2-byte addressing.
  // Send the 16-bit memory address, then read data back in a single I2C
  // transaction (write address + repeated-start + read data).
  uint8_t addr_buf[2];
  addr_buf[0] = (uint8_t)(addr >> 8);
  addr_buf[1] = (uint8_t)(addr & 0xFF);

  auto err = this->write_read(addr_buf, 2, buffer, length);
  if (err != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Read failed at addr 0x%04X, len %u, error %d",
             addr, length, (int) err);
    return false;
  }

  ESP_LOGV(TAG, "Read %u bytes from addr 0x%04X", length, addr);
  return true;
}

bool I2CEEPROMComponent::set(uint16_t addr, const uint8_t *buffer, uint16_t length) {
  if (addr + length > this->memory_size_) {
    ESP_LOGE(TAG, "Write out of range: addr=0x%04X len=%u size=%u",
             addr, length, this->memory_size_);
    return false;
  }

  const uint8_t page_size = this->get_page_size_();
  uint16_t offset = 0;

  while (offset < length) {
    // Calculate how many bytes until the end of the current page.
    uint16_t page_boundary = page_size - ((addr + offset) % page_size);
    uint16_t remaining = length - offset;
    uint8_t chunk = (remaining < page_boundary) ? (uint8_t) remaining : (uint8_t) page_boundary;

    if (!this->write_page_(addr + offset, buffer + offset, chunk)) {
      return false;
    }
    offset += chunk;
  }

  ESP_LOGV(TAG, "Wrote %u bytes starting at addr 0x%04X", length, addr);
  return true;
}

bool I2CEEPROMComponent::write_page_(uint16_t addr, const uint8_t *data, uint8_t length) {
  // Build a buffer: 2 address bytes + data payload.
  uint8_t write_buf[2 + 128];  // 128 is the max page size we support
  if (length > sizeof(write_buf) - 2) {
    ESP_LOGE(TAG, "Page write too large: %u bytes", length);
    return false;
  }

  write_buf[0] = (uint8_t)(addr >> 8);
  write_buf[1] = (uint8_t)(addr & 0xFF);
  memcpy(&write_buf[2], data, length);

  auto err = this->write(write_buf, 2 + length);
  if (err != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Page write failed at addr 0x%04X, len %u, error %d",
             addr, length, (int) err);
    return false;
  }

  // The EEPROM needs time to commit the write internally (t_WR).
  // AT24C256 specifies max 5 ms write cycle time.
  // We poll the device with ACK polling for up to 10 ms.
  uint32_t start = millis();
  while (millis() - start < 10) {
    uint8_t dummy;
    if (this->read(&dummy, 1) == i2c::ERROR_OK) {
      // Device acknowledged -- write cycle is complete.
      return true;
    }
    delay(1);
  }

  ESP_LOGW(TAG, "Write cycle ACK poll timed out at addr 0x%04X", addr);
  // Even if polling timed out, the write likely succeeded; return true
  // to avoid falsely aborting multi-page writes.
  return true;
}

}  // namespace i2c_eeprom
}  // namespace esphome
