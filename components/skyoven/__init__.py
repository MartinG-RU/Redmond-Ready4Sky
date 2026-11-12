"""SkyOven has no C++ driver in the upstream repository.

Fail during config validation rather than generating uncompilable code or inventing
commands for an oven. A protocol implementation needs verified device traces.
"""
import esphome.config_validation as cv

DEPENDENCIES = ["ready4sky"]
MULTI_CONF = True

CONFIG_SCHEMA = cv.invalid(
    "skyoven is unavailable: the upstream project does not include a C++ "
    "implementation or a verified protocol. Do not use it to control an oven."
)
