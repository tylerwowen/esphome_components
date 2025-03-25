#pragma once

#include "esphome/components/light/light_output.h"
#include "esphome/components/light/light_traits.h"
#include "esphome/components/nrf24/nrf24.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include <queue>
#include "esphome/core/preferences.h"

namespace esphome {
namespace mijia_light_bar {

/** @brief Command codes for the Mijia Light Bar */
enum MijiaLightBarCommand {
  CMD_TOGGLE = 0x01,    ///< Toggle light on/off
  CMD_COOLER = 0x02,    ///< Decrease color temperature
  CMD_WARMER = 0x03,    ///< Increase color temperature
  CMD_BRIGHTER = 0x04,  ///< Increase brightness
  CMD_DIMMER = 0x05,    ///< Decrease brightness
  CMD_RESET = 0x06,     ///< Reset to default state
};

/** @brief Component for controlling Mijia Light Bar via nRF24L01+ radio
 *
 * This component implements the ESPHome light interface to control a Mijia Light Bar
 * using an nRF24L01+ radio module. It supports:
 * - On/off control
 * - Brightness control (1-15 levels)
 * - Color temperature control (2700K-6500K)
 * - Command queuing with configurable repetitions
 */
class MijiaLightBarComponent : public Component, public nrf24::NRF24Device, public light::LightOutput {
 public:
  /** @brief Initialize the component
   *
   * Sets up the nRF24 radio and configures it for the Mijia Light Bar protocol.
   * Must be called before using any other methods.
   */
  void setup() override;

  /** @brief Process queued commands
   *
   * Called periodically to process commands in the queue. Handles command
   * timing, repetitions, and radio power management.
   */
  void loop() override;

  /** @brief Print component configuration
   *
   * Logs the current configuration including remote ID, repetitions, and delay.
   */
  void dump_config() override;

  /** @brief Get setup priority
   * @return Priority level for component setup
   */
  float get_setup_priority() const { return setup_priority::IO; }

  /** @brief Get supported light traits
   * @return Light traits supported by this component
   */
  light::LightTraits get_traits() override {
    auto traits = light::LightTraits();
    traits.set_supported_color_modes({light::ColorMode::COLOR_TEMPERATURE});
    traits.set_min_mireds(153);  // ~6500K
    traits.set_max_mireds(370);  // ~2700K
    return traits;
  }

  /** @brief Write new light state
   * @param state New light state to apply
   */
  void write_state(light::LightState *state) override;

  /** @brief Mark nRF24 initialization as failed
   *
   * Called when nRF24 initialization fails to mark the component as failed.
   */
  void mark_nrf24_failed() override { mark_failed(); }

  /** @brief Set the remote ID for the light bar
   * @param id 24-bit remote ID
   */
  void set_remote_id(uint32_t id) { remote_id_ = id; }

  /** @brief Set number of command repetitions
   * @param repetitions Number of times to repeat each command
   */
  void set_repetitions(uint8_t repetitions) { repetitions_ = repetitions; }

  /** @brief Set delay between command repetitions
   * @param delay_ms Delay in milliseconds
   */
  void set_delay_ms(uint8_t delay_ms) { delay_ms_ = delay_ms; }

  /** @brief Toggle light on/off */
  bool toggle();

  /** @brief Decrease color temperature */
  bool cooler();

  /** @brief Increase color temperature */
  bool warmer();

  /** @brief Increase brightness */
  bool brighter();

  /** @brief Decrease brightness */
  bool dimmer();

  /** @brief Reset light to default state */
  bool reset();

  /** @brief Set absolute brightness level
   * @param brightness Brightness level (1-15)
   */
  bool set_brightness(uint8_t brightness);

  /** @brief Set absolute color temperature level
   * @param color_temp Color temperature level (1-15)
   */
  bool set_color_temp(uint8_t color_temp);

  /** @brief Start pairing mode
   *
   * Enters pairing mode for 2 minutes. During this time, the model will try
   * to capature commands from the remote.
   */
  void start_pairing();

  /** @brief Exit pairing mode
   *
   * Exits pairing mode and restores normal operation.
   */
  void exit_pairing_mode();

 protected:
  /** @brief Create a packet for sending
   * @param data Buffer to store packet data
   * @param size Size of the buffer
   * @param command Command to send
   * @param value Optional command value
   */
  void create_packet(uint8_t *data, uint8_t size, uint8_t command, uint8_t value = 0);

  /** @brief Send a command to the light bar
   * @param command Command to send
   * @param value Optional command value
   */
  void send_command(uint8_t command, uint8_t value = 0);

  /** @brief Convert brightness to device level
   * @param brightness Brightness value (0.0-1.0)
   * @return Device brightness level (1-15)
   */
  uint8_t brightness_to_level(float brightness) { return static_cast<uint8_t>(brightness * 14.0f) + 1; }

  /** @brief Convert color temperature to device level
   * @param color_temp Color temperature value (0.0-1.0)
   * @return Device color temperature level (1-15)
   */
  uint8_t color_temp_to_level(float color_temp) { return static_cast<uint8_t>((1.0f - color_temp) * 14.0f + 1.0f); }

  uint32_t remote_id_{0};
  uint8_t repetitions_{20};
  uint8_t delay_ms_{10};
  uint8_t counter_{0};
  bool pairing_mode_{false};
  uint32_t pairing_start_time_{0};
  static constexpr uint32_t PAIRING_TIMEOUT_MS = 120000;  // 2 minutes

  /** @brief Packet structure for Mijia Light Bar protocol */
  struct Packet {
    uint8_t preamble[8];   // 0-7: Fixed preamble
    uint8_t remote_id[3];  // 8-10: Remote ID bytes
    uint8_t reserved;      // 11: Reserved (0xFF)
    uint8_t counter;       // 12: Command counter
    uint8_t cmd;           // 13: Command
    uint8_t value;         // 14: Command value
    uint8_t crc[2];        // 15-16: CRC bytes

    /** @brief Set remote ID in packet
     * @param id 24-bit remote ID
     */
    void set_remote_id(uint32_t id) {
      remote_id[0] = (id & 0xFF0000) >> 16;
      remote_id[1] = (id & 0x00FF00) >> 8;
      remote_id[2] = id & 0x0000FF;
    }
  };

  /** @brief Command structure for queuing */
  struct Command {
    Packet packet;
    uint8_t remaining_repetitions;
    uint32_t last_sent;
  };

  static constexpr size_t MAX_QUEUE_SIZE = 5;
  std::queue<Command> command_queue_;

  /** @brief Light bar state structure */
  struct LightBarState {
    bool is_on{false};
    uint8_t brightness{8};
    uint8_t color_temp{8};
  };
  LightBarState last_state_;

  /** @brief Queue a command for sending
   * @param cmd Command to queue
   * @param value Optional command value
   * @return true if command was queued, false if queue is full
   */
  bool queue_command(uint8_t cmd, uint8_t value = 0);

  /** @brief Process the next command in the queue
   * @return true if a command was processed, false if queue is empty
   */
  bool process_next_command();

  /** @brief Initialize a packet for sending
   * @param packet Packet to initialize
   * @param command Command to send
   * @param value Command value
   * @param counter Command counter
   */
  void init_packet(Packet &packet, uint8_t command, uint8_t value, uint8_t counter);

  /** @brief Calculate CRC for packet data
   * @param data Data to calculate CRC for
   * @param length Length of data
   * @return Calculated CRC value
   */
  uint16_t calculate_crc(const uint8_t *data, size_t length);

  /** @brief Handle pairing mode
   *
   * Called periodically to check if the light bar is in pairing mode and
   * respond to commands from the remote.
   */
  void handle_pairing_mode();

  /** @brief Check if a received packet is a pairing packet
   * @param data Data to check
   * @return true if the packet is a pairing packet, false otherwise
   */
  bool check_pairing_packet(const uint8_t *data);

  ESPPreferenceObject remote_id_preference_;
};
}  // namespace mijia_light_bar
}  // namespace esphome
