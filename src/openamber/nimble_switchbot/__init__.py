from esphome.components import esp32
import esphome.config_validation as cv

DEPENDENCIES = ["esp32", "wifi", "psram"]
CONFLICTS_WITH = ["esp32_ble", "esp32_ble_tracker", "bluetooth_proxy"]
CONFIG_SCHEMA = cv.Schema({})


async def to_code(config):
    esp32.request_bluetooth()
    esp32.request_software_coexistence()

    # Only the observer role is needed. Keep host allocations in PSRAM.
    for option, value in {
        "CONFIG_BT_ENABLED": True,
        "CONFIG_BT_NIMBLE_ENABLED": True,
        "CONFIG_BT_CTRL_PINNED_TO_CORE": 0,
        "CONFIG_BT_CTRL_BLE_MAX_ACT": 1,
        "CONFIG_BT_CTRL_SCAN_DUPL_CACHE_SIZE": 20,
        "CONFIG_BT_NIMBLE_ROLE_CENTRAL": False,
        "CONFIG_BT_NIMBLE_ROLE_PERIPHERAL": False,
        "CONFIG_BT_NIMBLE_ROLE_BROADCASTER": False,
        "CONFIG_BT_NIMBLE_ROLE_OBSERVER": True,
        "CONFIG_BT_NIMBLE_MAX_CONNECTIONS": 0,
        "CONFIG_BT_NIMBLE_SECURITY_ENABLE": False,
        "CONFIG_BT_NIMBLE_NVS_PERSIST": False,
        "CONFIG_BT_NIMBLE_MEM_ALLOC_MODE_EXTERNAL": True,
        "CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE": 4096,
        "CONFIG_BT_NIMBLE_TRANSPORT_ACL_FROM_LL_COUNT": 4,
        "CONFIG_BT_NIMBLE_TRANSPORT_EVT_COUNT": 6,
        "CONFIG_BT_NIMBLE_TRANSPORT_EVT_DISCARD_COUNT": 3,
        "CONFIG_BT_NIMBLE_50_FEATURE_SUPPORT": False,
        "CONFIG_BT_NIMBLE_GATT_SERVER": False,
    }.items():
        esp32.add_idf_sdkconfig_option(option, value)
