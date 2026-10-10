"""Contract validation tests for entity schema synchronization across OpenAmber.

Verifies:
1. Zero entity ID collisions between Core logic (src/openamber/core/) and Simulator mocks (src/openamber/mock/mock_entities.yaml).
2. Zero entity ID collisions between Core logic (src/openamber/core/) and Modbus physical IO (src/openamber/modbus/).
3. Uniqueness of entity IDs within Core, Modbus, and Mock.
4. Parity: Hardware heat pump entities mocked in simulator correspond to physical Modbus registers.
5. UI Bindings integrity: All entities extended in src/openamber/ui/bindings/ exist in Core, Modbus, or Mock.
6. Package configuration integrity: Packages include required domain packages for production and simulator.
"""

import os
import glob
import pytest
import yaml

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CORE_DIR = os.path.join(PROJECT_ROOT, "src", "openamber", "core")
MODBUS_DIR = os.path.join(PROJECT_ROOT, "src", "openamber", "modbus")
MOCK_FILE = os.path.join(PROJECT_ROOT, "src", "openamber", "mock", "mock_entities.yaml")
UI_BINDINGS_DIR = os.path.join(PROJECT_ROOT, "src", "openamber", "ui", "bindings")
OPENAMBER_YAML = os.path.join(PROJECT_ROOT, "src", "openamber", "common", "openamber.yaml")
VIRTUAL_DISPLAY_YAML = os.path.join(PROJECT_ROOT, "src", "openamber-virtual-display.yaml")


class ESPHomeYamlLoader(yaml.SafeLoader):
    """Custom YAML loader that transparently handles ESPHome tags (!extend, !include, !lambda, etc.)."""
    pass


def _yaml_default_constructor(loader, tag_suffix, node):
    if isinstance(node, yaml.ScalarNode):
        return loader.construct_scalar(node)
    elif isinstance(node, yaml.SequenceNode):
        return loader.construct_sequence(node)
    elif isinstance(node, yaml.MappingNode):
        return loader.construct_mapping(node)
    return None


ESPHomeYamlLoader.add_multi_constructor("!", _yaml_default_constructor)


def extract_entity_ids(yaml_path):
    """
    Extract a dictionary of {entity_id: domain} from an ESPHome YAML file.
    Inspects standard domain lists (sensor, binary_sensor, number, switch, select, text_sensor, etc.).
    """
    if not os.path.exists(yaml_path):
        return {}

    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.load(f, Loader=ESPHomeYamlLoader)

    if not isinstance(data, dict):
        return {}

    entities = {}
    for domain, items in data.items():
        if isinstance(items, list):
            for item in items:
                if isinstance(item, dict) and "id" in item:
                    entity_id = item["id"]
                    # If ID is wrapped in an extend tag or object, get string
                    if isinstance(entity_id, str):
                        entities[entity_id] = domain
    return entities


def extract_all_ids_from_dir(directory):
    """Extract all entity IDs across all YAML files in a directory."""
    all_entities = {}
    duplicates = []
    for file_path in glob.glob(os.path.join(directory, "*.yaml")):
        if os.path.basename(file_path).endswith("_package.yaml"):
            continue
        file_entities = extract_entity_ids(file_path)
        for eid, domain in file_entities.items():
            if eid in all_entities:
                duplicates.append((eid, file_path, all_entities[eid]))
            all_entities[eid] = (domain, file_path)
    return all_entities, duplicates


def get_core_entities():
    entities, duplicates = extract_all_ids_from_dir(CORE_DIR)
    return entities, duplicates


def get_modbus_entities():
    entities, duplicates = extract_all_ids_from_dir(MODBUS_DIR)
    return entities, duplicates


def get_mock_entities():
    return extract_entity_ids(MOCK_FILE)


def test_no_entity_id_collisions_between_core_and_mock():
    """
    Verify ZERO entity ID collisions between Core logic (src/openamber/core/)
    and Mock entities (src/openamber/mock/mock_entities.yaml).
    
    Since openamber-virtual-display.yaml packages both core_package.yaml and mock_entities.yaml,
    any collision would indicate duplicate definitions and potential logic drift.
    """
    core_entities, _ = get_core_entities()
    mock_entities = get_mock_entities()

    collisions = set(core_entities.keys()) & set(mock_entities.keys())
    assert not collisions, (
        f"Found {len(collisions)} entity ID collision(s) between Core and Mock: {sorted(collisions)}. "
        "Core entities must not be redefined in mock_entities.yaml."
    )


def test_no_entity_id_collisions_between_core_and_modbus():
    """
    Verify ZERO entity ID collisions between Core logic (src/openamber/core/)
    and Modbus physical IO (src/openamber/modbus/).
    
    Since production openamber.yaml packages both core_package.yaml and modbus_package.yaml,
    any collision would result in invalid ESPHome duplicate ID definitions.
    """
    core_entities, _ = get_core_entities()
    modbus_entities, _ = get_modbus_entities()

    collisions = set(core_entities.keys()) & set(modbus_entities.keys())
    assert not collisions, (
        f"Found {len(collisions)} entity ID collision(s) between Core and Modbus: {sorted(collisions)}."
    )


def test_entity_ids_unique_within_core():
    """Verify that every entity ID defined within src/openamber/core/ is unique."""
    _, duplicates = get_core_entities()
    assert not duplicates, f"Duplicate entity IDs found within src/openamber/core/: {duplicates}"


def test_entity_ids_unique_within_modbus():
    """Verify that every entity ID defined within src/openamber/modbus/ is unique."""
    _, duplicates = get_modbus_entities()
    assert not duplicates, f"Duplicate entity IDs found within src/openamber/modbus/: {duplicates}"


def test_hardware_mocks_correspond_to_physical_modbus_or_subsystems():
    """
    Verify that mocked hardware signals correspond to actual Modbus entities or known subsystems.
    
    Simulator-specific infrastructure (mock_time, mock_update, analytics scripts, debug sensors)
    is explicitly exempted.
    """
    mock_entities = get_mock_entities()
    modbus_entities, _ = get_modbus_entities()
    core_entities, _ = get_core_entities()

    # Allowed simulator-only infrastructure entities
    simulator_infra = {
        "my_time",
        "firmware_update",
        "dump_eeprom_parameters",
        "dump_eeprom_parameters_ui_disable",
        "dump_eeprom_parameters_ui_enable",
        "fetch_analytics_device_id",
        "send_analytics",
        "system_debug_device_info",
        "system_debug_reset_reason",
        "system_heap_fragmentation",
        "system_heap_free",
        "system_heap_max_block",
        "system_psram_free",
        "calculated_heat_curve_setpoint",
        "current_cooling_temperature_setpoint",
        "current_heating_temperature_setpoint",
        "current_water_temperature_tc_sensor",
        "outdoor_temperature_tp_sensor",
        "water_flow_rate",
    }

    mock_ids = set(mock_entities.keys())
    hardware_mocks = mock_ids - simulator_infra

    # Every hardware mock MUST exist in Modbus
    missing_in_modbus = hardware_mocks - set(modbus_entities.keys())
    assert not missing_in_modbus, (
        f"The following mocked hardware entities do not exist in src/openamber/modbus/: {sorted(missing_in_modbus)}. "
        "Simulator hardware mocks must remain in sync with physical Modbus register entities."
    )


def test_ui_bindings_reference_valid_entities():
    """
    Verify that all entities extended or referenced in UI bindings exist in Core, Modbus, or Mock.
    """
    core_entities, _ = get_core_entities()
    modbus_entities, _ = get_modbus_entities()
    mock_entities = get_mock_entities()

    known_ids = set(core_entities.keys()) | set(modbus_entities.keys()) | set(mock_entities.keys())

    # Check all binding YAML files
    extended_ids = set()
    for binding_file in glob.glob(os.path.join(UI_BINDINGS_DIR, "*.yaml")):
        with open(binding_file, "r", encoding="utf-8") as f:
            for line in f:
                stripped = line.strip()
                if stripped.startswith("#"):
                    continue
                if "!extend" in stripped and "id:" in stripped:
                    parts = stripped.split("!extend")
                    if len(parts) > 1:
                        target_id = parts[1].strip().split()[0].strip("'\"")
                        if target_id:
                            extended_ids.add((target_id, os.path.basename(binding_file)))

    missing_bindings = []
    for eid, fname in extended_ids:
        if eid not in known_ids:
            missing_bindings.append((eid, fname))

    assert not missing_bindings, (
        f"UI bindings reference undefined entity IDs: {missing_bindings}"
    )


def test_package_configuration_integrity():
    """
    Verify that:
    1. src/openamber/common/openamber.yaml includes core_package.yaml and modbus_package.yaml.
    2. src/openamber-virtual-display.yaml includes core_package.yaml and mock_entities.yaml.
    """
    with open(OPENAMBER_YAML, "r", encoding="utf-8") as f:
        openamber_content = f.read()

    assert "core_package.yaml" in openamber_content, "openamber.yaml must package core_package.yaml"
    assert "modbus_package.yaml" in openamber_content, "openamber.yaml must package modbus_package.yaml"

    with open(VIRTUAL_DISPLAY_YAML, "r", encoding="utf-8") as f:
        virtual_content = f.read()

    assert "core_package.yaml" in virtual_content, "openamber-virtual-display.yaml must package core_package.yaml"
    assert "mock_entities.yaml" in virtual_content, "openamber-virtual-display.yaml must package mock_entities.yaml"
    assert "modbus_package.yaml" not in virtual_content, "openamber-virtual-display.yaml must NOT include physical Modbus IO"
