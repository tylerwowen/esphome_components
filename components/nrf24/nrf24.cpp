#include "nrf24.h"
#include "esphome/core/log.h"

namespace esphome {
using namespace spi;

namespace nrf24 {

static const char *const TAG = "nrf24";

void NRF24Device::setup_nrf24() {
  ESP_LOGD(TAG, "Setting up SPIDevice...");
  spi_setup();

  int i = 0;
  while (i++ < 10 && !spi_is_ready()) {
    ESP_LOGD(TAG, "SPI is not ready, wait... %d/10", i);
    delay(50);
  }

  if (spi_is_ready()) {
    ESP_LOGD(TAG, "SPIDevice set up!");
  } else {
    ESP_LOGE(TAG, "SPI hardware not ready!");
    mark_nrf24_failed();
    return;
  }

  ESP_LOGV(TAG, "Setting up nRF24 with CE pin %d, CS pin %d, DR: %d", Utility::get_pin_no(ce_pin_),
           Utility::get_pin_no(cs_), data_rate_);
  radio_ = new RF24(Utility::get_pin_no(ce_pin_), Utility::get_pin_no(cs_), data_rate_);

  i = 0;
  while (i++ < 10 && !radio_->begin()) {
    ESP_LOGD(TAG, "Radio is not ready, wait...%d/10", i);
    delay(50);
  }

  if (radio_->begin()) {
    ESP_LOGD(TAG, "Radio began!");
  } else {
    ESP_LOGE(TAG, "Radio hardware not responding, check wiring!");
    mark_nrf24_failed();
    return;
  }

  // Configure radio
  radio_->setChannel(channel_);
  radio_->setPALevel(static_cast<rf24_pa_dbm_e>(pa_level_));
  radio_->setDataRate(static_cast<rf24_datarate_e>(rf_data_rate_));
  radio_->setCRCLength(static_cast<rf24_crclength_e>(crc_length_));
  radio_->setPayloadSize(payload_size_);
  radio_->setRetries(retry_delay_, retry_count_);
  radio_->setAutoAck(auto_ack_);

  // Configure CRC
  if (crc_length_ == RF24_CRC_DISABLED) {
    radio_->disableCRC();
  }
  // Configure pipes
  if (write_address_ != 0) {
    radio_->openWritingPipe(write_address_);
  }

  for (const auto &pipe : pipes_) {
    radio_->openReadingPipe(pipe.pipe_num, pipe.address);
  }

  ESP_LOGI(TAG, "nRF24 setup completed");
}

void NRF24Device::dump_config() {
  ESP_LOGCONFIG(TAG, "nRF24:");
  LOG_PIN("  CE Pin: ", ce_pin_);
  LOG_PIN("  CS Pin: ", cs_);
  ESP_LOGCONFIG(TAG, "  Channel: %d", channel_);
  ESP_LOGCONFIG(TAG, "  PA Level: %d", pa_level_);
  ESP_LOGCONFIG(TAG, "  SPI Data Rate: %d", data_rate_);
  ESP_LOGCONFIG(TAG, "  RF Data Rate: %d [0: 1M, 1: 2M, 2: 250K]", rf_data_rate_);
  ESP_LOGCONFIG(TAG, "  Payload Size: %d", payload_size_);
  ESP_LOGCONFIG(TAG, "  CRC Length: %d", crc_length_);
  ESP_LOGCONFIG(TAG, "  Auto ACK: %s", YESNO(auto_ack_));
  ESP_LOGCONFIG(TAG, "  Retry Delay: %d", retry_delay_);
  ESP_LOGCONFIG(TAG, "  Retry Count: %d", retry_count_);
  ESP_LOGCONFIG(TAG, "  Write Address: 0x%llX", write_address_);

  for (const auto &pipe : pipes_) {
    ESP_LOGCONFIG(TAG, "  Read pipe %d Address: 0x%llX", pipe.pipe_num, pipe.address);
  }
}

bool NRF24Device::write(const void *buf, uint8_t len) { return radio_->write(buf, len); }

void NRF24Device::read(void *buf, uint8_t len) { radio_->read(buf, len); }

bool NRF24Device::available() { return radio_->available(); }

void NRF24Device::start_listening() { radio_->startListening(); }

void NRF24Device::stop_listening() { radio_->stopListening(); }

}  // namespace nrf24
}  // namespace esphome