from esphome import automation
import esphome.codegen as cg
from esphome.components import light, nrf24
import esphome.config_validation as cv
from esphome.const import CONF_LIGHT_ID

DEPENDENCIES = ["light"]
AUTO_LOAD = ["nrf24"]

mijia_light_bar_ns = cg.esphome_ns.namespace("mijia_light_bar")
MijiaLightBarComponent = mijia_light_bar_ns.class_(
    "MijiaLightBarComponent", nrf24.NRF24Device, light.LightOutput
)

# Actions
ToggleAction = mijia_light_bar_ns.class_("ToggleAction", automation.Action)

# Configuration constants
CONF_REMOTE_ID = "remote_id"
CONF_REPETITIONS = "repetitions"
CONF_DELAY_MS = "delay_ms"

# Base schema for Mijia-specific settings
CONFIG_SCHEMA = (
    nrf24.NRF24_DEVICE_SCHEMA.extend(
        {
            cv.GenerateID(CONF_LIGHT_ID): cv.declare_id(MijiaLightBarComponent),
            # Redefine defaults
            cv.Optional(nrf24.CONF_CHANNEL, default=68): cv.one_of(
                6, 7, 15, 16, 43, 44, 68, 69, int=True
            ),
            cv.Optional(
                nrf24.CONF_RETRIES,
                default={nrf24.CONF_RETRY_DELAY: 15, nrf24.CONF_RETRY_COUNT: 15},
            ): nrf24.RETRY_SCHEMA,
            # Fixed values
            cv.Optional(nrf24.CONF_RF_DATA_RATE, default="2mbps"): cv.enum(
                {"2mbps": nrf24.NRF24DataRate.RF24_2MBPS}, lower=True
            ),
            cv.Optional(nrf24.CONF_PAYLOAD_SIZE, default=17): cv.one_of(17, int=True),
            cv.Optional(nrf24.CONF_AUTO_ACK, default=False): cv.one_of(False),
            cv.Optional(nrf24.CONF_WRITE_ADDRESS, default=0x5555555555): cv.one_of(
                0x5555555555, int=True
            ),
            cv.Optional(nrf24.CONF_CRC, default="disabled"): cv.enum(
                {"disabled": nrf24.NRF24CRCLength.RF24_CRC_DISABLED}, lower=True
            ),
            # Mijia Light Bar specific
            cv.Optional(CONF_REMOTE_ID, default=0x00000000): cv.hex_uint32_t,
            cv.Optional(CONF_REPETITIONS, default=20): cv.int_range(min=1, max=30),
            cv.Optional(CONF_DELAY_MS, default=20): cv.int_range(min=1, max=20),
        }
    )
    .extend(light.LIGHT_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_LIGHT_ID])
    await cg.register_component(var, config)
    await nrf24.register_nrf24_device(var, config)

    # Add Mijia-specific settings
    cg.add(var.set_remote_id(config[CONF_REMOTE_ID]))
    cg.add(var.set_repetitions(config[CONF_REPETITIONS]))
    cg.add(var.set_delay_ms(config[CONF_DELAY_MS]))
    await light.register_light(var, config)
