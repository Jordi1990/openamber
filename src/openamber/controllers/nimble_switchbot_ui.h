#pragma once

// Display and web UI use the same bounded snapshot; all calls run in ESPHome's
// main loop. The Bluetooth task only updates the fixed scanner cache.
static bool nimble_source_is_ble()
{
    return id(room_temperature_source_select).current_option() == "BLE";
}

static void nimble_update_source()
{
    nimble_update(nimble_source_is_ble());
}

static void nimble_format_mac(const ble_addr_t &address, char *text)
{
    snprintf(text, 18, "%02X:%02X:%02X:%02X:%02X:%02X", address.val[5], address.val[4], address.val[3],
             address.val[2], address.val[1], address.val[0]);
}

static bool nimble_device_fresh(const NimbleDevice &device)
{
    return device.model && device.reading.valid &&
           uint32_t(millis() - device.reading.received_ms) < NIMBLE_READING_TIMEOUT_MS;
}

static void nimble_choose_device(unsigned index)
{
    if (!nimble_source_is_ble() || index >= NIMBLE_DISCOVERY_LIMIT)
        return;
    NimbleDevice devices[NIMBLE_DISCOVERY_LIMIT];
    nimble_discovery_snapshot(devices);
    if (!nimble_device_fresh(devices[index]))
        return;
    char mac[18];
    nimble_format_mac(devices[index].address, mac);
    id(ble_device_mac).make_call().set_value(mac).perform();
}

static void nimble_set_label(lv_obj_t *label, const char *text)
{
    if (strcmp(lv_label_get_text(label), text) != 0)
        lv_label_set_text(label, text);
}

static void nimble_update_ui()
{
    if (!nimble_boot_ready)
        return;
    static bool placed = false;
    if (!placed)
    {
        lv_obj_move_to_index(&id(ble_source_row), 1);
        lv_obj_move_to_index(&id(ble_settings_panel), 2);
        placed = true;
    }
    bool enabled = nimble_source_is_ble();
    lv_dropdown_set_selected(id(ble_source_dropdown).obj, enabled ? 1 : 0);
    if (!enabled)
    {
        lv_obj_add_flag(&id(ble_settings_panel), LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(&id(ble_sensor_dialog), LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_clear_flag(&id(ble_settings_panel), LV_OBJ_FLAG_HIDDEN);
    lv_dropdown_set_selected(id(ble_timeout_dropdown).obj, nimble_reading_timeout_ms / 60000 - 1);
    char text[160];
    if (id(ble_device_mac).state.empty())
    {
        snprintf(text, sizeof(text), "Geen sensor gekozen");
    }
    else if (nimble_reading_fresh())
    {
        snprintf(text, sizeof(text),
                 "SwitchBot  %s\n%.1f °C   |   %.0f%% luchtvochtigheid   |   %lus geleden",
                 id(ble_device_mac).state.c_str(), nimble_reading.temperature, nimble_reading.humidity,
                 (unsigned long)(uint32_t(millis() - nimble_reading.received_ms) / 1000));
    }
    else
    {
        snprintf(text, sizeof(text), "SwitchBot  %s\nGeen actuele meting", id(ble_device_mac).state.c_str());
    }
    nimble_set_label(&id(ble_selected_label), text);
    if (lv_obj_has_flag(&id(ble_sensor_dialog), LV_OBJ_FLAG_HIDDEN))
        return;

    lv_obj_t *rows[] = {&id(ble_device_row_0), &id(ble_device_row_1), &id(ble_device_row_2),
                        &id(ble_device_row_3), &id(ble_device_row_4), &id(ble_device_row_5),
                        &id(ble_device_row_6), &id(ble_device_row_7)};
    lv_obj_t *labels[] = {&id(ble_device_label_0), &id(ble_device_label_1), &id(ble_device_label_2),
                          &id(ble_device_label_3), &id(ble_device_label_4), &id(ble_device_label_5),
                          &id(ble_device_label_6), &id(ble_device_label_7)};
    NimbleDevice devices[NIMBLE_DISCOVERY_LIMIT];
    nimble_discovery_snapshot(devices);
    unsigned count = 0;
    for (unsigned i = 0; i < NIMBLE_DISCOVERY_LIMIT; i++)
    {
        if (!nimble_device_fresh(devices[i]))
        {
            lv_obj_add_flag(rows[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        count++;
        char mac[18];
        nimble_format_mac(devices[i].address, mac);
        snprintf(text, sizeof(text),
                 "SwitchBot  %s\n%.1f °C   |   %.0f%% luchtvochtigheid   |   %lus geleden", mac,
                 devices[i].reading.temperature, devices[i].reading.humidity,
                 (unsigned long)(uint32_t(millis() - devices[i].reading.received_ms) / 1000));
        nimble_set_label(labels[i], text);
        lv_obj_clear_flag(rows[i], LV_OBJ_FLAG_HIDDEN);
    }
    if (nimble_is_discovering())
    {
        unsigned seconds = nimble_discovery_seconds();
        snprintf(text, sizeof(text), "Zoeken: %us resterend  |  %u sensoren gevonden", seconds, count);
    }
    else
    {
        snprintf(text, sizeof(text), "%u sensoren gevonden  |  Kies een sensor of zoek opnieuw", count);
    }
    nimble_set_label(&id(ble_discovery_status_label), text);
}

#ifdef USE_WEBSERVER
static std::string nimble_web_state()
{
    // No arbitrary advertisement strings enter JSON/HTML. MACs are normalized,
    // values are validated numbers, and the model label is fixed.
    std::string result;
    result.reserve(1900);
    char text[220], mac[18] = "";
    ble_addr_t address = {};
    if (nimble_parse_mac(id(ble_device_mac).state, address))
        nimble_format_mac(address, mac);
    bool enabled = nimble_source_is_ble();
    bool searching = nimble_is_discovering();
    unsigned remaining = nimble_discovery_seconds();
    snprintf(text, sizeof(text),
             "{\"source\":\"%s\",\"mac\":\"%s\",\"searching\":%s,\"remaining\":%u,\"reading\":",
             enabled ? "BLE" : "Tr", mac, searching ? "true" : "false", remaining);
    result += text;
    if (enabled && nimble_reading_fresh())
    {
        snprintf(text, sizeof(text), "{\"temperature\":%.1f,\"humidity\":%.0f,\"age\":%lu}",
                 nimble_reading.temperature, nimble_reading.humidity,
                 (unsigned long)(uint32_t(millis() - nimble_reading.received_ms) / 1000));
        result += text;
    }
    else
        result += "null";
    result += ",\"devices\":[";
    if (enabled)
    {
        NimbleDevice devices[NIMBLE_DISCOVERY_LIMIT];
        nimble_discovery_snapshot(devices);
        bool first = true;
        for (auto &device : devices)
        {
            if (!nimble_device_fresh(device))
                continue;
            nimble_format_mac(device.address, mac);
            snprintf(text, sizeof(text),
                     "%s{\"mac\":\"%s\",\"temperature\":%.1f,\"humidity\":%.0f,\"age\":%lu}",
                     first ? "" : ",", mac, device.reading.temperature, device.reading.humidity,
                     (unsigned long)(uint32_t(millis() - device.reading.received_ms) / 1000));
            result += text;
            first = false;
        }
    }
    snprintf(text, sizeof(text), "],\"timeout_minutes\":%u}", unsigned(nimble_reading_timeout_ms / 60000));
    result += text;
    return result;
}
#endif
