#include "mijia_light_bar.h"

#include <cinttypes>
#include <cstdint>
#include <vector>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mijia_light_bar {

static const char *const TAG = "mijia_light_bar";
// 0x533914DD1C493412
static const uint8_t PREAMBLE[8] = {0x53, 0x39, 0x14, 0xDD, 0x1C, 0x49, 0x34, 0x12};

void MijiaLightBarComponent::setup() {
  ESP_LOGD(TAG, "Setting up Mijia Light Bar");

  remote_id_preference_ = global_preferences->make_preference<uint32_t>(fnv1_hash("mijiao_lightbar_remote_id"));
  if (remote_id_ == 0) {
    remote_id_preference_.load(&remote_id_);
    if (remote_id_ != 0) {
      ESP_LOGI(TAG, "Loaded saved remote ID: 0x%06X", remote_id_);
    } else {
      ESP_LOGW(TAG, "No saved remote ID found, use paring mode to capture one");
    }
  }

  // Call parent setup first to initialize nRF24
  nrf24::NRF24Device::setup_nrf24();
  if (is_failed()) {
    ESP_LOGE(TAG, "Failed to initialize nRF24 radio");
    return;
  }

  radio_->disableDynamicPayloads();
  radio_->stopListening();
  radio_->powerDown();

  ESP_LOGI(TAG, "Mijia Light Bar initialized");
}

void MijiaLightBarComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Mijia Light Bar:");
  ESP_LOGCONFIG(TAG, "  Remote ID: 0x%08X", remote_id_);
  ESP_LOGCONFIG(TAG, "  Repetitions: %d", repetitions_);
  ESP_LOGCONFIG(TAG, "  Delay: %d ms", delay_ms_);
  nrf24::NRF24Device::dump_config();
}

bool MijiaLightBarComponent::queue_command(uint8_t cmd, uint8_t value) {
  if (command_queue_.size() >= MAX_QUEUE_SIZE) {
    ESP_LOGW(TAG, "Command queue full, dropping command 0x%02X", cmd);
    return false;
  }

  Command cmd_entry;
  init_packet(cmd_entry.packet, cmd, value, ++counter_);
  cmd_entry.remaining_repetitions = repetitions_;
  cmd_entry.last_sent = 0;

  command_queue_.push(cmd_entry);
  ESP_LOGD(TAG, "Queued command 0x%02X%02X (counter: %d, queue: %d/%d)", cmd, value, counter_, command_queue_.size(),
           MAX_QUEUE_SIZE);
  return true;
}

bool MijiaLightBarComponent::process_next_command() {
  if (command_queue_.empty()) {
    return false;
  }

  Command &cmd = command_queue_.front();
  uint32_t now = millis();

  // Check if it's time to send the next repetition
  if (now - cmd.last_sent >= delay_ms_) {
    if (cmd.remaining_repetitions == repetitions_) {
      ESP_LOGD(TAG, "Powering up radio for new command");
      radio_->powerUp();
    }

    ESP_LOGD(TAG, "Sending command 0x%02X%02X (repetition %d/%d)", cmd.packet.cmd, cmd.packet.value,
             repetitions_ - cmd.remaining_repetitions + 1, repetitions_);
    ESP_LOGV(TAG, "Packet %s",
             format_hex_pretty(reinterpret_cast<const uint8_t *>(&cmd.packet), sizeof(Packet)).c_str());

    nrf24::NRF24Device::write(reinterpret_cast<const uint8_t *>(&cmd.packet), sizeof(Packet));

    cmd.last_sent = now;
    cmd.remaining_repetitions--;

    if (cmd.remaining_repetitions == 0) {
      // Command completed, remove from queue
      command_queue_.pop();

      if (command_queue_.empty()) {
        ESP_LOGD(TAG, "No more commands, powering down radio");
        radio_->powerDown();
      }
      return true;
    }
  }

  return false;
}

void MijiaLightBarComponent::loop() {
  // Handle pairing mode if active
  if (pairing_mode_) {
    handle_pairing_mode();
    return;
  }

  // Process commands in the queue
  while (process_next_command()) {
    ESP_LOGV(TAG, "Processed command, queue size: %d", command_queue_.size());
  }
}

void MijiaLightBarComponent::start_pairing() {
  if (pairing_mode_) {
    ESP_LOGI(TAG, "Already in pairing mode");
    return;
  }

  pairing_mode_ = true;
  pairing_start_time_ = millis();
  // Power up radio and start listening
  radio_->powerUp();
  radio_->openReadingPipe(0, 0xAAAAAAAAAAAA);
  radio_->startListening();
  ESP_LOGI(TAG, "Entering pairing mode for 2 minutes");
}

void MijiaLightBarComponent::exit_pairing_mode() {
  radio_->stopListening();
  radio_->closeReadingPipe(0);
  radio_->powerDown();
  pairing_mode_ = false;
}

void MijiaLightBarComponent::handle_pairing_mode() {
  // Check if pairing mode has timed out
  if (millis() - pairing_start_time_ >= PAIRING_TIMEOUT_MS) {
    exit_pairing_mode();
    ESP_LOGI(TAG, "Pairing mode timed out");
    return;
  }

  // Check for received packets
  if (radio_->available()) {
    uint8_t raw_data[18];
    radio_->read(raw_data, sizeof(raw_data));
    if (check_pairing_packet(raw_data)) {
      exit_pairing_mode();
    }
  }
}

bool MijiaLightBarComponent::check_pairing_packet(const uint8_t *raw_data) {
  byte data[17] = {0x5};
  for (int i = 0; i < 17; i++) {
    if (i == 0) {
      data[i] = 0x50 | raw_data[i] >> 5;
    } else {
      data[i] = ((raw_data[i - 1] >> 1) & 0x0F) << 4 | ((raw_data[i - 1] & 0x01) << 3) | raw_data[i] >> 5;
    }
  }
  ESP_LOGV(TAG, "Raw data: %s", format_hex_pretty(raw_data, 17).c_str());
  ESP_LOGV(TAG, "Data: %s", format_hex_pretty(data, 17).c_str());
  // Check preamble
  if (memcmp(data, PREAMBLE, sizeof(PREAMBLE)) != 0) {
    ESP_LOGV(TAG, "Invalid preamble, expected: %s, got: %s", format_hex_pretty(PREAMBLE, sizeof(PREAMBLE)).c_str(),
             format_hex_pretty(data, sizeof(PREAMBLE)).c_str());
    return false;
  }

  // Check CRC
  uint16_t received_crc = (data[15] << 8) | data[16];
  uint16_t calculated_crc = calculate_crc(data, 15);

  if (received_crc == calculated_crc) {
    // Extract remote ID from packet
    uint32_t remote_id = (data[8] << 16) | (data[9] << 8) | data[10];
    ESP_LOGI(TAG, "Received pairing packet, saving remote ID to preferences: 0x%06X", remote_id);
    // Save the remote ID
    set_remote_id(remote_id);
    // Save to preferences
    remote_id_preference_.save(&remote_id_);
    return true;
  }
  ESP_LOGV(TAG, "Invalid CRC");
  return false;
}

void MijiaLightBarComponent::write_state(light::LightState *state) {
  bool is_on;
  state->current_values_as_binary(&is_on);

  if (is_on != last_state_.is_on) {
    ESP_LOGD(TAG, "State change: %d -> %d", last_state_.is_on, is_on);
    toggle();
    last_state_.is_on = is_on;
  }

  if (is_on) {
    float brightness;
    float color_temp;
    state->current_values_as_ct(&color_temp, &brightness);

    // Convert values to device levels
    uint8_t brightness_level = brightness_to_level(brightness);
    uint8_t color_temp_level = color_temp_to_level(color_temp);

    ESP_LOGD(TAG, "Brightness: %.2f -> %d, Color Temp: %.2f -> %d", brightness, brightness_level, color_temp,
             color_temp_level);

    if (brightness_level != last_state_.brightness) {
      ESP_LOGD(TAG, "Brightness change: %d -> %d", last_state_.brightness, brightness_level);
      set_brightness(brightness_level);
      last_state_.brightness = brightness_level;
    }

    if (color_temp_level != last_state_.color_temp) {
      ESP_LOGD(TAG, "Color temp change: %d -> %d", last_state_.color_temp, color_temp_level);
      set_color_temp(color_temp_level);
      last_state_.color_temp = color_temp_level;
    }
  }
}

bool MijiaLightBarComponent::toggle() {
  ESP_LOGD(TAG, "Toggling");
  return queue_command(CMD_TOGGLE);
}

bool MijiaLightBarComponent::reset() {
  ESP_LOGD(TAG, "Resetting");
  return queue_command(CMD_RESET);
}

bool MijiaLightBarComponent::cooler() {
  ESP_LOGD(TAG, "Cooler");
  return queue_command(CMD_COOLER);
}

bool MijiaLightBarComponent::warmer() {
  ESP_LOGD(TAG, "Warmer");
  return queue_command(CMD_WARMER);
}

bool MijiaLightBarComponent::brighter() {
  ESP_LOGD(TAG, "Brighter");
  return queue_command(CMD_BRIGHTER);
}

bool MijiaLightBarComponent::dimmer() {
  ESP_LOGD(TAG, "Dimmer");
  return queue_command(CMD_DIMMER);
}

bool MijiaLightBarComponent::set_brightness(uint8_t brightness) {
  ESP_LOGD(TAG, "Setting brightness: %d", brightness);
  return queue_command(CMD_DIMMER, 0xF0) && queue_command(CMD_BRIGHTER, brightness);
}

bool MijiaLightBarComponent::set_color_temp(uint8_t color_temp) {
  ESP_LOGD(TAG, "Setting color temperature: %d", color_temp);
  return queue_command(CMD_WARMER, 0xF0) && queue_command(CMD_COOLER, color_temp);
}

void MijiaLightBarComponent::init_packet(Packet &packet, uint8_t command, uint8_t value, uint8_t counter) {
  ESP_LOGV(TAG, "Initializing packet - Command: 0x%02X, Value: 0x%02X, Counter: %d", command, value, counter);

  // Initialize static parts
  memcpy(packet.preamble, PREAMBLE, sizeof(PREAMBLE));
  packet.set_remote_id(remote_id_);
  packet.reserved = 0xFF;

  // Set command data
  packet.counter = counter;
  packet.cmd = command;
  packet.value = value;

  // Calculate and set CRC
  uint16_t crc = calculate_crc(reinterpret_cast<const uint8_t *>(&packet), 15);
  packet.crc[0] = (crc & 0xFF00) >> 8;
  packet.crc[1] = crc & 0x00FF;

  ESP_LOGV(TAG, "Packet initialized with CRC: 0x%04X", crc);
}

uint16_t MijiaLightBarComponent::calculate_crc(const uint8_t *data, size_t length) {
  uint16_t crc = crc16be(data, length, 0xFFFE, 0x1021, false, false);
  return crc;
}

}  // namespace mijia_light_bar
}  // namespace esphome
