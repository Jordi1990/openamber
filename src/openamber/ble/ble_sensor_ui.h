#pragma once

// Display text translation, defined in ui/i18n/oa_i18n_<lang>.h (ui_language).
static const char *oa_tr(const char *text);

// All calls run in ESPHome's main loop, like the esp32_ble_tracker
// advertisement callback.

// room_temperature_source_select options; index 1..3 is paired slot 0..2.
static const char *const BLE_SENSOR_SOURCE_OPTIONS[1 + BLE_SENSOR_SLOTS] = {"Tr", "BLE 1", "BLE 2", "BLE 3"};

static esphome::text::Text *ble_sensor_slot_text(unsigned slot)
{
    switch (slot)
    {
    case 0:
        return &id(ble_device_mac);
    case 1:
        return &id(ble_device_mac_2);
    default:
        return &id(ble_device_mac_3);
    }
}

// Slot chosen in the source select (-1: Tr), regardless of the switch.
static int ble_sensor_selected_source_slot()
{
    auto idx = id(room_temperature_source_select).active_index();
    if (!idx.has_value() || idx.value() < 1 || idx.value() > BLE_SENSOR_SLOTS)
        return -1;
    return int(idx.value()) - 1;
}

static bool ble_sensor_source_is_ble()
{
    return id(ble_enabled_switch).state && ble_sensor_selected_source_slot() >= 0;
}

static void ble_sensor_set_source(int slot)
{
    const char *option = BLE_SENSOR_SOURCE_OPTIONS[slot >= 0 && slot < int(BLE_SENSOR_SLOTS) ? slot + 1 : 0];
    if (id(room_temperature_source_select).current_option() != option)
        id(room_temperature_source_select).make_call().set_option(option).perform();
}

static void ble_sensor_update_source()
{
    bool enabled = id(ble_enabled_switch).state;
    int source = ble_sensor_selected_source_slot();
    // Bluetooth off or an unpaired slot: fall back to the wired sensor.
    if (ble_sensor_boot_ready && source >= 0 && (!enabled || ble_sensor_slot_text(source)->state.empty()))
    {
        ble_sensor_set_source(-1);
        source = -1;
    }
    const std::string *macs[BLE_SENSOR_SLOTS];
    for (unsigned i = 0; i < BLE_SENSOR_SLOTS; i++)
        macs[i] = &ble_sensor_slot_text(i)->state;
    uint64_t *models[BLE_SENSOR_SLOTS] = {&id(ble_device_model), &id(ble_device_model_2), &id(ble_device_model_3)};
    ble_sensor_update(enabled, source, macs, models);
}

static bool ble_sensor_device_fresh(const BleSensorDevice &device)
{
    return device.model && device.reading.valid &&
           uint32_t(millis() - device.reading.received_ms) < BLE_SENSOR_READING_TIMEOUT_MS;
}

// Pair found sensor `index` into the first free slot.
static void ble_sensor_pair(unsigned index)
{
    if (!id(ble_enabled_switch).state || index >= BLE_SENSOR_DISCOVERY_LIMIT)
        return;
    BleSensorDevice devices[BLE_SENSOR_DISCOVERY_LIMIT];
    ble_sensor_discovery_snapshot(devices);
    if (!ble_sensor_device_fresh(devices[index]) || ble_sensor_slot_of(devices[index].address) >= 0)
        return;
    for (unsigned i = 0; i < BLE_SENSOR_SLOTS; i++)
    {
        if (!ble_sensor_slot_text(i)->state.empty())
            continue;
        char mac[18];
        ble_sensor_format_mac(devices[index].address, mac);
        ble_sensor_slot_text(i)->make_call().set_value(mac).perform();
        return;
    }
}

static void ble_sensor_unpair(unsigned slot)
{
    if (slot >= BLE_SENSOR_SLOTS)
        return;
    if (ble_sensor_selected_source_slot() == int(slot))
        ble_sensor_set_source(-1);
    // Move the sensor to the found list with its model and last reading, so it can
    // be paired again right away instead of waiting for a scan response.
    const BleSensorSlot &s = ble_sensor_slots[slot];
    if (s.valid && ble_sensor_model_known(s.model))
        ble_sensor_remember_device(s.address, s.model, s.reading);
    ble_sensor_slot_text(slot)->make_call().set_value("").perform();
    uint64_t *models[BLE_SENSOR_SLOTS] = {&id(ble_device_model), &id(ble_device_model_2), &id(ble_device_model_3)};
    *models[slot] = 0;
}

// "21.3 °C   |   45% luchtvochtigheid"; BTHome sensors may lack humidity.
static void ble_sensor_format_reading(const BleSensorReading &reading, char *text, size_t size)
{
    if (std::isnan(reading.humidity))
        snprintf(text, size, "%.1f °C", reading.temperature);
    else
        snprintf(text, size, oa_tr("%.1f °C   |   %.0f%% luchtvochtigheid"), reading.temperature, reading.humidity);
}

static void ble_sensor_set_label(lv_obj_t *label, const char *text)
{
    if (strcmp(lv_label_get_text(label), text) != 0)
        lv_label_set_text(label, text);
}

static void ble_sensor_set_hidden(lv_obj_t *obj, bool hidden)
{
    if (hidden != lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN))
    {
        if (hidden)
            lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
    }
}

// True while Settings -> Connectiviteit -> Bluetooth is the open menu page.
static bool ble_sensor_bluetooth_page_visible()
{
    return !lv_obj_has_flag(&id(page_settings), LV_OBJ_FLAG_HIDDEN) &&
           lv_menu_get_cur_main_page(&id(settings_menu)) == &id(panel_wifi_page_bluetooth);
}

// max_name > 0 shortens the name for narrow widgets (ble_sensor_format_short_title).
static void ble_sensor_slot_title(unsigned slot, char *text, size_t size, size_t max_name = 0)
{
    const BleSensorSlot &s = ble_sensor_slots[slot];
    BleSensorAddress address;
    const BleSensorAddress *shown = nullptr;
    uint8_t model = 0;
    if (s.valid)
    {
        shown = &s.address;
        model = s.model;
    }
    else if (ble_sensor_parse_mac(ble_sensor_slot_text(slot)->state, address))
        shown = &address;
    if (shown != nullptr && max_name > 0)
        ble_sensor_format_short_title(*shown, model, text, size, max_name);
    else if (shown != nullptr)
        ble_sensor_format_title(*shown, model, text, size);
    else
        snprintf(text, size, "%s", ble_sensor_slot_text(slot)->state.c_str());
}

// ── Room temperature source dropdown: "Tr (Bedraad)" plus paired sensors ──
static int ble_sensor_dropdown_slots[1 + BLE_SENSOR_SLOTS] = {-1, -1, -1, -1};

static void ble_sensor_update_source_dropdown()
{
    std::string options = oa_tr("Tr (Bedraad)");
    unsigned count = 1, selected = 0;
    int source = ble_sensor_selected_source_slot();
    for (unsigned i = 0; i < BLE_SENSOR_SLOTS; i++)
    {
        if (!id(ble_enabled_switch).state || ble_sensor_slot_text(i)->state.empty())
            continue;
        char title[64];
        // The 360 px dropdown fits about 34 characters next to the chevron.
        ble_sensor_slot_title(i, title, sizeof(title), 24);
        options += "\n";
        options += title;
        if (int(i) == source)
            selected = count;
        ble_sensor_dropdown_slots[count++] = i;
    }
    lv_obj_t *dd = id(ble_source_dropdown).obj;
    if (options != lv_dropdown_get_options(dd))
        lv_dropdown_set_options(dd, options.c_str());
    if (lv_dropdown_get_selected(dd) != selected)
        lv_dropdown_set_selected(dd, selected);
}

static void ble_sensor_source_dropdown_changed()
{
    unsigned selected = lv_dropdown_get_selected(id(ble_source_dropdown).obj);
    ble_sensor_set_source(selected <= BLE_SENSOR_SLOTS ? ble_sensor_dropdown_slots[selected] : -1);
}

// ── Settings → Connectiviteit → Bluetooth ──
static void ble_sensor_update_bluetooth_page()
{
    bool enabled = id(ble_enabled_switch).state;
    lv_obj_t *sw = &id(ble_enabled_switch_ui);
    if (enabled != lv_obj_has_state(sw, LV_STATE_CHECKED))
    {
        if (enabled)
            lv_obj_add_state(sw, LV_STATE_CHECKED);
        else
            lv_obj_clear_state(sw, LV_STATE_CHECKED);
    }
    char text[160];
    if (!enabled)
        snprintf(text, sizeof(text), "%s", oa_tr("Schakel Bluetooth in om sensoren te koppelen."));
    else if (ble_sensor_host_failed)
        snprintf(text, sizeof(text), "%s", oa_tr("Bluetooth kon niet starten. Herstart het display om het opnieuw te proberen."));
    else if (ble_sensor_restart_required)
        snprintf(text, sizeof(text), "%s", oa_tr("Bluetooth start na een herstart: te weinig vrij geheugen om nu te starten."));
    else
        text[0] = '\0';
    ble_sensor_set_label(&id(ble_hint_label), text);
    ble_sensor_set_hidden(&id(ble_hint_label), text[0] == '\0');
    ble_sensor_set_hidden(&id(ble_sensors_section), !enabled);
    if (!enabled)
        return;

    // Paired sensors: green check with a current reading, warning otherwise.
    lv_obj_t *rows[] = {&id(ble_paired_row_0), &id(ble_paired_row_1), &id(ble_paired_row_2)};
    lv_obj_t *icons[] = {&id(ble_paired_icon_0), &id(ble_paired_icon_1), &id(ble_paired_icon_2)};
    lv_obj_t *labels[] = {&id(ble_paired_label_0), &id(ble_paired_label_1), &id(ble_paired_label_2)};
    int source = ble_sensor_selected_source_slot();
    unsigned paired = 0;
    for (unsigned i = 0; i < BLE_SENSOR_SLOTS; i++)
    {
        bool used = !ble_sensor_slot_text(i)->state.empty();
        ble_sensor_set_hidden(rows[i], !used);
        if (!used)
            continue;
        paired++;
        char title[64];
        ble_sensor_slot_title(i, title, sizeof(title));
        const char *role = int(i) == source ? oa_tr("  |  ruimtetemperatuur") : "";
        const BleSensorSlot &slot = ble_sensor_slots[i];
        const char *icon = "\U000F0028";
        uint32_t color = 0xD97706;
        if (ble_sensor_slot_fresh(i))
        {
            char values[64];
            ble_sensor_format_reading(slot.reading, values, sizeof(values));
            snprintf(text, sizeof(text), oa_tr("%s%s\n%s   |   %lus geleden"), title, role, values,
                     (unsigned long)(uint32_t(millis() - slot.reading.received_ms) / 1000));
            icon = "\U000F05E0";
            color = 0x16A34A;
        }
        else
        {
            snprintf(text, sizeof(text), "%s%s\n%s", title, role,
                     slot.reading.received_ms ? oa_tr("Geen actuele meting") : oa_tr("Wacht op eerste meting"));
        }
        ble_sensor_set_label(labels[i], text);
        ble_sensor_set_label(icons[i], icon);
        lv_obj_set_style_text_color(icons[i], lv_color_hex(color), LV_PART_MAIN);
    }
    snprintf(text, sizeof(text), paired ? oa_tr("Gekoppelde sensoren (%u/%u)") : oa_tr("Nog geen sensoren gekoppeld (0/%u)"),
             paired ? paired : unsigned(BLE_SENSOR_SLOTS), unsigned(BLE_SENSOR_SLOTS));
    ble_sensor_set_label(&id(ble_paired_title), text);

    // Found sensors that are not paired yet.
    lv_obj_t *found[] = {&id(ble_device_row_0), &id(ble_device_row_1), &id(ble_device_row_2),
                         &id(ble_device_row_3), &id(ble_device_row_4), &id(ble_device_row_5),
                         &id(ble_device_row_6), &id(ble_device_row_7)};
    lv_obj_t *found_labels[] = {&id(ble_device_label_0), &id(ble_device_label_1), &id(ble_device_label_2),
                                &id(ble_device_label_3), &id(ble_device_label_4), &id(ble_device_label_5),
                                &id(ble_device_label_6), &id(ble_device_label_7)};
    lv_obj_t *pair_buttons[] = {&id(ble_device_pair_0), &id(ble_device_pair_1), &id(ble_device_pair_2),
                                &id(ble_device_pair_3), &id(ble_device_pair_4), &id(ble_device_pair_5),
                                &id(ble_device_pair_6), &id(ble_device_pair_7)};
    bool full = paired >= BLE_SENSOR_SLOTS;
    BleSensorDevice devices[BLE_SENSOR_DISCOVERY_LIMIT];
    ble_sensor_discovery_snapshot(devices);
    unsigned count = 0;
    for (unsigned i = 0; i < BLE_SENSOR_DISCOVERY_LIMIT; i++)
    {
        bool show = ble_sensor_device_fresh(devices[i]) && ble_sensor_slot_of(devices[i].address) < 0;
        ble_sensor_set_hidden(found[i], !show);
        if (!show)
            continue;
        count++;
        char mac[18], title[64], values[64];
        ble_sensor_format_mac(devices[i].address, mac);
        ble_sensor_format_title(devices[i].address, devices[i].model, title, sizeof(title));
        ble_sensor_format_reading(devices[i].reading, values, sizeof(values));
        snprintf(text, sizeof(text), "%s  %s\n%s", title, mac, values);
        ble_sensor_set_label(found_labels[i], text);
        if (full != lv_obj_has_state(pair_buttons[i], LV_STATE_DISABLED))
        {
            if (full)
                lv_obj_add_state(pair_buttons[i], LV_STATE_DISABLED);
            else
                lv_obj_clear_state(pair_buttons[i], LV_STATE_DISABLED);
            lv_obj_set_style_opa(pair_buttons[i], full ? LV_OPA_40 : LV_OPA_COVER, LV_PART_MAIN);
        }
    }
    if (ble_sensor_is_discovering())
        snprintf(text, sizeof(text), oa_tr("Gevonden sensoren  |  zoeken: %us  |  %u gevonden"), ble_sensor_discovery_seconds(),
                 count);
    else
        snprintf(text, sizeof(text), oa_tr("Gevonden sensoren  |  %u gevonden%s"), count,
                 full ? oa_tr("  |  ontkoppel eerst een sensor") : "");
    ble_sensor_set_label(&id(ble_discovery_status_label), text);
}

static void ble_sensor_update_ui()
{
    if (!ble_sensor_boot_ready)
        return;
    static bool placed = false;
    if (!placed)
    {
        lv_obj_move_to_index(&id(ble_source_row), 1);
        placed = true;
    }
    ble_sensor_update_source_dropdown();
    if (ble_sensor_bluetooth_page_visible())
        ble_sensor_update_bluetooth_page();
}

// Connectiviteit -> Bluetooth menu page was opened: start a search.
static void ble_sensor_bluetooth_page_opened()
{
    if (ble_sensor_boot_ready && id(ble_enabled_switch).state && !ble_sensor_is_discovering())
        ble_sensor_begin_discovery();
    ble_sensor_update_source();
    ble_sensor_update_ui();
}
