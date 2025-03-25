from esphome import pins
import esphome.codegen as cg
from esphome.components import spi
import esphome.config_validation as cv
from esphome.const import CONF_CHANNEL

DEPENDENCIES = ["spi"]
CODEOWNERS = ["@tylerwowen"]

# Configuration keys
CONF_CE_PIN = "ce_pin"
CONF_PA_LEVEL = "pa_level"
CONF_PAYLOAD_SIZE = "payload_size"
CONF_CRC = "crc"
CONF_RETRIES = "retries"
CONF_RETRY_DELAY = "delay"
CONF_RETRY_COUNT = "count"
CONF_ADDRESS = "address"
CONF_AUTO_ACK = "auto_ack"
CONF_READ_PIPES = "read_pipes"
CONF_PIPE_NUMBER = "pipe"
CONF_WRITE_ADDRESS = "write_address"
CONF_RF_DATA_RATE = "rf_data_rate"

# Create namespace and component class
nrf24_ns = cg.esphome_ns.namespace("nrf24")
NRF24Device = nrf24_ns.class_("NRF24Device", cg.Component, spi.SPIDevice)

NRF24DeviceCompmonent = nrf24_ns.class_(
    "NRF24DeviceComponent", cg.Component, spi.SPIDevice
)

# Enums
NRF24PALevel = nrf24_ns.enum("NRF24PALevel")
NRF24DataRate = nrf24_ns.enum("NRF24DataRate")
NRF24CRCLength = nrf24_ns.enum("NRF24CRCLength")

RETRY_RANGE = cv.int_range(min=0, max=15)

# Enum mappings
PA_LEVELS = {
    "min": NRF24PALevel.RF24_PA_MIN,
    "low": NRF24PALevel.RF24_PA_LOW,
    "high": NRF24PALevel.RF24_PA_HIGH,
    "max": NRF24PALevel.RF24_PA_MAX,
}

RF_DATA_RATES = {
    "250kbps": NRF24DataRate.RF24_250KBPS,
    "1mbps": NRF24DataRate.RF24_1MBPS,
    "2mbps": NRF24DataRate.RF24_2MBPS,
}

CRC_LENGTH = {
    "disabled": NRF24CRCLength.RF24_CRC_DISABLED,
    "8bit": NRF24CRCLength.RF24_CRC_8,
    "16bit": NRF24CRCLength.RF24_CRC_16,
}


def validate_address(value):
    """Validate a 5-byte address."""
    value = cv.hex_uint64_t(value)
    if value > 0xFFFFFFFFFF:
        raise cv.Invalid("Address must be a 5-byte address (max 0xFFFFFFFFFF)")
    return value


PIPE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_PIPE_NUMBER, default=1): cv.int_range(min=0, max=5),
        cv.Optional(CONF_ADDRESS, default=0xE8E8F0F0E1): validate_address,
    }
)

RETRY_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_RETRY_DELAY, default=15): RETRY_RANGE,
        cv.Optional(CONF_RETRY_COUNT, default=15): RETRY_RANGE,
    }
)

NRF24_DEVICE_SCHEMA = spi.spi_device_schema(True, "4MHz").extend(
    {
        cv.Required(CONF_CE_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_CHANNEL, default=76): cv.int_range(min=0, max=125),
        cv.Optional(CONF_PA_LEVEL, default="low"): cv.enum(PA_LEVELS, lower=True),
        cv.Optional(CONF_RF_DATA_RATE, default="2mbps"): cv.enum(
            RF_DATA_RATES, lower=True
        ),
        cv.Optional(CONF_PAYLOAD_SIZE, default=32): cv.int_range(min=1, max=32),
        cv.Optional(CONF_CRC, default="16bit"): cv.enum(CRC_LENGTH, lower=True),
        cv.Optional(CONF_AUTO_ACK, default=True): cv.boolean,
        cv.Optional(CONF_WRITE_ADDRESS, default=0xE8E8F0F0E1): validate_address,
        cv.Optional(
            CONF_RETRIES, default={CONF_RETRY_DELAY: 5, CONF_RETRY_COUNT: 15}
        ): RETRY_SCHEMA,
        cv.Optional(CONF_READ_PIPES): cv.ensure_list(PIPE_SCHEMA),
    }
)


async def register_nrf24_device(var, config):
    # Register SPI device
    await spi.register_spi_device(var, config)

    # Set CE pin
    ce_pin = await cg.gpio_pin_expression(config[CONF_CE_PIN])
    cg.add(var.set_ce_pin(ce_pin))

    cg.add(var.set_channel(config[CONF_CHANNEL]))
    cg.add(var.set_pa_level(config[CONF_PA_LEVEL]))
    cg.add(var.set_rf_data_rate(config[CONF_RF_DATA_RATE]))
    cg.add(var.set_payload_size(config[CONF_PAYLOAD_SIZE]))
    cg.add(var.set_crc_length(config[CONF_CRC]))
    cg.add(var.set_auto_ack(config[CONF_AUTO_ACK]))
    cg.add(var.set_write_address(config[CONF_WRITE_ADDRESS]))

    retries = config[CONF_RETRIES]
    cg.add(var.set_retries(retries[CONF_RETRY_DELAY], retries[CONF_RETRY_COUNT]))

    if CONF_READ_PIPES in config:
        for pipe in config[CONF_READ_PIPES]:
            cg.add(var.add_pipe(pipe[CONF_PIPE_NUMBER], pipe[CONF_ADDRESS]))
