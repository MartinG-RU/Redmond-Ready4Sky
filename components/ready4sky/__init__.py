import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import final_validate as fv
from esphome.components import time
from esphome.components.esp32 import (
  add_idf_sdkconfig_option, get_esp32_variant, request_bluetooth, request_software_coexistence,
)
from esphome.const import (
  CONF_ID,
  CONF_INTERVAL,
  CONF_DURATION,
  CONF_TIME_ID,
  PLATFORM_ESP32,
)
from esphome.core import CORE

CODEOWNERS = ["@KomX"]
CONFLICTS_WITH = ["esp32_ble", "esp32_ble_tracker", "esp32_ble_beacon"]
DEPENDENCIES = ["esp32"]
ESP_PLATFORMS = [PLATFORM_ESP32]


CONF_SCAN_PARAMETERS = "scan_parameters"
CONF_ACTIVE = "active"
CONF_WINDOW = "window"
CONF_MONITOR = "monitor"

CONF_R4S_ID = "r4s_id"

ready4sky_ns = cg.esphome_ns.namespace("ready4sky")
R4SEngine = ready4sky_ns.class_("R4SEngine", cg.Component)
R4SDriver = ready4sky_ns.class_("R4SDriver")

def validate_scan_parameters(config):
  duration = config[CONF_DURATION]
  interval = config[CONF_INTERVAL]
  window = config[CONF_WINDOW]
  if not 3 <= interval.total_milliseconds <= 10240:
    raise cv.Invalid("BLE scan interval must be between 3 ms and 10240 ms")
  if not 3 <= window.total_milliseconds <= 10240:
    raise cv.Invalid("BLE scan window must be between 3 ms and 10240 ms")
  if window > interval:
    raise cv.Invalid(
      f"Scan window ({window}) cannot be larger than scan interval ({interval})"
    )
  if not 1 <= duration.total_seconds <= 3600:
    raise cv.Invalid("Scan duration must be between 1 s and 3600 s")
  if interval.total_milliseconds * 3 > duration.total_milliseconds:
    raise cv.Invalid(
      "Scan duration needs to be at least three times the scan interval to"
      "cover all BLE channels."
    )
  return config

CONFIG_SCHEMA = cv.Schema(
  {
    cv.GenerateID(): cv.declare_id(R4SEngine),
    cv.GenerateID(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
    cv.Optional(CONF_SCAN_PARAMETERS, default={}): cv.All(
      cv.Schema(
        {
          cv.Optional(CONF_DURATION, default="30s"): cv.positive_time_period_seconds,
          cv.Optional(CONF_INTERVAL, default="150ms"): cv.positive_time_period_milliseconds,
          cv.Optional(CONF_WINDOW, default="50ms"): cv.positive_time_period_milliseconds,
          cv.Optional(CONF_ACTIVE, default=True): cv.boolean,
          cv.Optional(CONF_MONITOR, default=False): cv.boolean,
        }
      ),
      validate_scan_parameters,
    ),
  }
).extend(cv.COMPONENT_SCHEMA)

DRIVER_COMPONENTS = ("skybaker", "skycoffee", "skycooker", "skydew", "skyheat",
                     "skyiron", "skykettle", "skyplug", "skysmoke")

def validate_unique_addresses(config):
  full = fv.full_config.get()
  seen = {}
  variant = get_esp32_variant()
  if variant == "ESP32S2":
    raise cv.Invalid("ESP32-S2 has no Bluetooth controller and cannot run ready4sky")
  for component in DRIVER_COMPONENTS:
    for device in full.get(component, []):
      address = str(device["mac_address"])
      if address in seen:
        raise cv.Invalid(
          f"The same MAC address is assigned to both {seen[address]} and {component}; "
          "each Ready4Sky device must have a unique MAC address."
        )
      seen[address] = component
  limit = 2 if variant == "ESP32C2" else (3 if CORE.using_arduino else 9)
  if len(seen) > limit:
    raise cv.Invalid(f"This target/framework supports at most {limit} simultaneous Ready4Sky devices; "
                     f"configured {len(seen)}. Use another ESP32 gateway.")
  return config

FINAL_VALIDATE_SCHEMA = validate_unique_addresses

READY4SKY_DEVICE_SCHEMA = cv.Schema(
  {
    cv.GenerateID(CONF_R4S_ID): cv.use_id(R4SEngine),
  }
)


def as_hex(value):
  return cg.RawExpression(f"0x{value}ULL")

async def register_r4s_driver(var, config):
  paren = await cg.get_variable(config[CONF_R4S_ID])
  cg.add(paren.register_r4s_driver(var))
  return var

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    timevar = await cg.get_variable(config[CONF_TIME_ID])
    cg.add(var.set_time(timevar))
    params = config[CONF_SCAN_PARAMETERS]
    cg.add(var.set_scan_duration(params[CONF_DURATION]))
    cg.add(var.set_scan_active(params[CONF_ACTIVE]))
    cg.add(var.set_scan_parameters(int(params[CONF_INTERVAL].total_milliseconds/0.625), int(params[CONF_WINDOW].total_milliseconds/0.625)))
    cg.add(var.set_monitor(params[CONF_MONITOR]))
    
    if not CORE.using_arduino:
        request_bluetooth()
        request_software_coexistence()
        add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
        count = sum(len(CORE.config.get(component, [])) for component in DRIVER_COMPONENTS)
        variant = get_esp32_variant()
        acl_limit = 2 if variant == "ESP32C2" else max(4, count)
        add_idf_sdkconfig_option("CONFIG_BT_ACL_CONNECTIONS", acl_limit)
        if variant == "ESP32":
            add_idf_sdkconfig_option("CONFIG_BTDM_CTRL_BLE_MAX_CONN", max(3, count))
        elif variant in ("ESP32C3", "ESP32S3"):
            # One additional activity is needed for scanning.
            add_idf_sdkconfig_option("CONFIG_BT_CTRL_BLE_MAX_ACT", max(6, count + 1))
    
