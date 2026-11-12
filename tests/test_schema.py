"""Configuration and code-generation regressions; no physical devices required."""
from __future__ import annotations

import ast
import copy
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
ESPHOME = os.environ.get("ESPHOME", shutil.which("esphome") or "esphome")


def models(component):
    tree = ast.parse((ROOT / "components" / component / "__init__.py").read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "MODEL_TYPE" for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f"No MODEL_TYPE in {component}")


def base():
    return {
        "esphome": {"name": "r4s-schema-test"},
        "esp32": {"board": "esp32dev", "toolchain": "platformio", "framework": {"type": "esp-idf"}},
        "logger": {},
        "wifi": {"ssid": "test-only", "password": "test-only-password"},
        "api": {},
        "time": [{"platform": "homeassistant", "id": "r4s_clock"}],
        "external_components": [{"source": {"type": "local", "path": str(ROOT / "components")}}],
        "ready4sky": {"time_id": "r4s_clock"},
    }


def item(component, model, serial=1):
    name = f"{component} {serial}"
    device = {"mac_address": f"00:00:00:00:{serial // 256:02X}:{serial % 256:02X}", "model": model}
    if component == "skysmoke":
        device["informing"] = {"smoke": {"name": name + " smoke"}}
        return device
    device["controlling"] = {"power": {"name": name + " power"}}
    if component == "skykettle":
        device["informing"] = {"temperature": {"name": name + " temperature"}}
    if component == "skydew":
        device["informing"] = {"humidity": {"name": name + " humidity"}}
    if component == "skyiron":
        device["informing"] = {}
    if component == "skybaker":
        device["controlling"]["baking_mode"] = {"name": name + " mode"}
    if component == "skycooker":
        device["controlling"]["cooking_mode"] = {"name": name + " mode"}
    return device


def run_config(tmp_path, config, generate=False):
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    command = [ESPHOME, "compile" if generate else "config", str(path)]
    if generate:
        command.append("--only-generate")
    return subprocess.run(command, capture_output=True, text=True, timeout=180)


COMPONENTS = ["skybaker", "skycoffee", "skycooker", "skydew", "skyheat", "skyiron", "skykettle", "skyplug", "skysmoke"]


@pytest.mark.parametrize("component", COMPONENTS)
def test_unknown_model_rejected_during_config(tmp_path, component):
    config = base()
    config[component] = [item(component, "NOT-A-REAL-MODEL")]
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "NOT-A-REAL-MODEL" in result.stdout + result.stderr


def max_entries(component):
    tree = ast.parse((ROOT / "components" / component / "__init__.py").read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "MULTI_CONF" for t in node.targets):
            return ast.literal_eval(node.value)
    return 1


def test_every_listed_model_generates_cpp(tmp_path):
    # Respect each upstream driver's per-configuration MULTI_CONF limit.
    groups = {component: [model for model, model_type in models(component).items()
                          if component != "skyheat" or model_type in (1, 2)]
              for component in COMPONENTS}
    # Nine different components per batch, within the ESP32 BLE connection limit.
    limits = {component: 1 for component in COMPONENTS}
    batches = max((len(groups[c]) + limits[c] - 1) // limits[c] for c in COMPONENTS)
    serial = 1
    seen = 0
    for batch in range(batches):
        directory = tmp_path / str(batch)
        directory.mkdir()
        config = base()
        for component in COMPONENTS:
            choices = groups[component][batch * limits[component]:(batch + 1) * limits[component]]
            if choices:
                config[component] = []
                for model in choices:
                    config[component].append(item(component, model, serial))
                    serial += 1
                    seen += 1
        result = run_config(directory, config, generate=True)
        assert result.returncode == 0, (result.stdout + result.stderr)[-8000:]
    assert seen > 50


def test_case_insensitive_model_is_rendered_as_string(tmp_path):
    config = base()
    config["skyplug"] = [item("skyplug", "rsp-100s")]
    result = run_config(tmp_path, config, generate=True)
    assert result.returncode == 0, result.stdout + result.stderr
    generated = (tmp_path / ".esphome" / "build" / "r4s-schema-test" / "src" / "main.cpp").read_text()
    assert 'set_model("RSP-100S")' in generated
    assert "set_type(1)" in generated


def test_old_kettle_rejects_unsupported_boil_adjustment(tmp_path):
    config = base()
    config["skykettle"] = [item("skykettle", "RK-M171S")]
    config["skykettle"][0]["controlling"]["boil_time_adjustment"] = {"name": "Unsupported adjustment"}
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "boil_time_adjustment" in result.stdout + result.stderr


def test_oven_fails_early_with_explanation(tmp_path):
    config = base()
    config["skyoven"] = [{"model": "RO-5706S", "mac_address": "00:00:00:00:00:01"}]
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "does not include a C++" in result.stdout + result.stderr


def test_duplicate_mac_rejected(tmp_path):
    config = base()
    config["skykettle"] = [item("skykettle", "RK-G200S")]
    config["skyplug"] = [item("skyplug", "RSP-100S")]
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "same MAC address" in result.stdout + result.stderr


def test_tracker_conflict_rejected(tmp_path):
    config = base()
    config["esp32_ble_tracker"] = {}
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "conflict" in (result.stdout + result.stderr).lower()


@pytest.mark.parametrize("parameters", [{"interval": "1ms", "window": "1ms"},
                                        {"interval": "10ms", "window": "20ms"},
                                        {"duration": "5000s"}])
def test_invalid_scan_parameters_rejected(tmp_path, parameters):
    config = base()
    config["ready4sky"]["scan_parameters"] = parameters
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "scan" in (result.stdout + result.stderr).lower()


def test_heater_does_not_pretend_to_support_temperature_control(tmp_path):
    config = base()
    config["skyheat"] = [item("skyheat", "RCH-7001S")]
    config["skyheat"][0]["controlling"]["target_temperature"] = {"name": "Unimplemented"}
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "not implemented" in (result.stdout + result.stderr).lower()


def test_single_timer_setting_is_not_silently_ignored(tmp_path):
    config = base()
    config["skycooker"] = [item("skycooker", "RMC-M222S")]
    config["skycooker"][0]["controlling"]["timer_hours_setting"] = {"name": "Hours"}
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "must be configured together" in (result.stdout + result.stderr).lower()


def test_too_many_devices_rejected(tmp_path):
    config = base()
    serial = 1
    for component in COMPONENTS:
        config[component] = [item(component, next(iter(models(component))), serial)]
        serial += 1
    config["skyplug"].append(item("skyplug", "RSP-103S", serial))
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "at most 9" in (result.stdout + result.stderr)


def test_esp32s2_rejected_early(tmp_path):
    config = base()
    config["esp32"]["board"] = "esp32-s2-saola-1"
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "has no Bluetooth" in (result.stdout + result.stderr)


@pytest.mark.parametrize("model", [name for name, kind in models("skyheat").items() if kind not in (1, 2)])
def test_unverified_heater_models_fail_safely(tmp_path, model):
    config = base()
    config["skyheat"] = [item("skyheat", model)]
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "not implemented safely" in result.stdout + result.stderr


def test_conflicting_esp32_ble_is_rejected(tmp_path):
    config = base()
    config["esp32_ble"] = {}
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "conflict" in (result.stdout + result.stderr).lower()


def test_c3_connection_capacity_is_generated(tmp_path):
    config = base()
    config["esp32"]["board"] = "esp32-c3-devkitm-1"
    for serial, component in enumerate(COMPONENTS, start=1):
        available = [name for name, kind in models(component).items()
                     if component != "skyheat" or kind in (1, 2)]
        config[component] = [item(component, available[0], serial)]
    result = run_config(tmp_path, config, generate=True)
    assert result.returncode == 0, result.stdout + result.stderr
    files = list((tmp_path / ".esphome" / "build").rglob("sdkconfig*"))
    text = "\n".join(f.read_text(errors="ignore") for f in files if f.is_file())
    assert "CONFIG_BT_CTRL_BLE_MAX_ACT=10" in text
    assert "CONFIG_BT_ACL_CONNECTIONS=9" in text


def test_example_yaml(tmp_path):
    import base64
    import shutil
    project = Path(__file__).resolve().parents[1]
    examples = tmp_path / "examples"
    examples.mkdir()
    (tmp_path / "components").symlink_to(project / "components", target_is_directory=True)
    shutil.copy(project / "examples" / "esp32-kettle-plug.yaml", examples)
    # A deterministic test-only key, never used by an actual device.
    key = base64.b64encode(b"x" * 32).decode()
    (examples / "secrets.yaml").write_text(
        f"wifi_ssid: test-only\nwifi_password: test-only-password\napi_encryption_key: {key}\n"
    )
    result = subprocess.run([ESPHOME, "config", str(examples / "esp32-kettle-plug.yaml")],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_coffee_example_at_esphome_root(tmp_path):
    import base64
    project_dir = tmp_path / "ESPHome-Ready4Sky"
    project_dir.mkdir()
    (project_dir / "components").symlink_to(ROOT / "components", target_is_directory=True)
    yaml_file = tmp_path / "coffee.yaml"
    shutil.copy(ROOT / "examples" / "coffee-in-esphome-root.yaml", yaml_file)
    key = base64.b64encode(b"x" * 32).decode()  # test-only, not a real secret
    (tmp_path / "secrets.yaml").write_text(
        f"wifi_ssid: test-only\nwifi_password: test-only-password\n"
        f"api_encryption_key: {key}\nota_password: test-only-password\n"
    )
    result = subprocess.run([ESPHOME, "config", str(yaml_file)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("model,group,feature", [
    ("RCM-M1508S", "controlling", "lock"),
    ("RCM-M1519S", "controlling", "buzzer"),
    ("RCM-M1509S", "informing", "energy"),
])
def test_unsupported_coffee_option_is_rejected(tmp_path, model, group, feature):
    config = base()
    coffee = item("skycoffee", model)
    coffee.setdefault(group, {})[feature] = {"name": "Unsupported"}
    config["skycoffee"] = [coffee]
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert feature in result.stdout + result.stderr


def test_coffee_1509_supported_options_generate_cpp(tmp_path):
    config = base()
    coffee = item("skycoffee", "RCM-M1509S")
    for feature in ("buzzer", "lock", "strength"):
        coffee["controlling"][feature] = {"name": feature}
    config["skycoffee"] = [coffee]
    result = run_config(tmp_path, config, generate=True)
    assert result.returncode == 0, result.stdout + result.stderr
    generated = (tmp_path / ".esphome/build/r4s-schema-test/src/main.cpp").read_text()
    for setter in ("set_buzzer", "set_lock", "set_strength"):
        assert setter in generated


def test_rhf_3320_rejects_unimplemented_warm_steam(tmp_path):
    config = base()
    dew = item("skydew", "RHF-3320S")
    dew["controlling"]["warm_steam"] = {"name": "Unsupported"}
    config["skydew"] = [dew]
    result = run_config(tmp_path, config)
    assert result.returncode != 0
    assert "warm_steam" in result.stdout + result.stderr
