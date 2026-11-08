#pragma once

// All calls run in ESPHome's main loop, like the esp32_ble_tracker
// advertisement callback.

// room_temperature_source_select options; index 1..3 is paired slot 0..2.
static const char *const SWITCHBOT_SOURCE_OPTIONS[1 + SWITCHBOT_SLOTS] = {"Tr", "BLE 1", "BLE 2", "BLE 3"};

static esphome::text::Text *switchbot_slot_text(unsigned slot)
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
static int switchbot_selected_source_slot()
{
    auto idx = id(room_temperature_source_select).active_index();
    if (!idx.has_value() || idx.value() < 1 || idx.value() > SWITCHBOT_SLOTS)
        return -1;
    return int(idx.value()) - 1;
}

static bool switchbot_source_is_ble()
{
    return id(ble_enabled_switch).state && switchbot_selected_source_slot() >= 0;
}

static void switchbot_set_source(int slot)
{
    const char *option = SWITCHBOT_SOURCE_OPTIONS[slot >= 0 && slot < int(SWITCHBOT_SLOTS) ? slot + 1 : 0];
    if (id(room_temperature_source_select).current_option() != option)
        id(room_temperature_source_select).make_call().set_option(option).perform();
}

static void switchbot_update_source()
{
    bool enabled = id(ble_enabled_switch).state;
    int source = switchbot_selected_source_slot();
    // Bluetooth off or an unpaired slot: fall back to the wired sensor.
    if (switchbot_boot_ready && source >= 0 && (!enabled || switchbot_slot_text(source)->state.empty()))
    {
        switchbot_set_source(-1);
        source = -1;
    }
    const std::string *macs[SWITCHBOT_SLOTS];
    for (unsigned i = 0; i < SWITCHBOT_SLOTS; i++)
        macs[i] = &switchbot_slot_text(i)->state;
    switchbot_update(enabled, source, macs);
}

static bool switchbot_device_fresh(const SwitchbotDevice &device)
{
    return device.model && device.reading.valid &&
           uint32_t(millis() - device.reading.received_ms) < SWITCHBOT_READING_TIMEOUT_MS;
}

// Pair found sensor `index` into the first free slot.
static void switchbot_pair(unsigned index)
{
    if (!id(ble_enabled_switch).state || index >= SWITCHBOT_DISCOVERY_LIMIT)
        return;
    SwitchbotDevice devices[SWITCHBOT_DISCOVERY_LIMIT];
    switchbot_discovery_snapshot(devices);
    if (!switchbot_device_fresh(devices[index]) || switchbot_slot_of(devices[index].address) >= 0)
        return;
    for (unsigned i = 0; i < SWITCHBOT_SLOTS; i++)
    {
        if (!switchbot_slot_text(i)->state.empty())
            continue;
        char mac[18];
        switchbot_format_mac(devices[index].address, mac);
        switchbot_slot_text(i)->make_call().set_value(mac).perform();
        return;
    }
}

static void switchbot_unpair(unsigned slot)
{
    if (slot >= SWITCHBOT_SLOTS)
        return;
    if (switchbot_selected_source_slot() == int(slot))
        switchbot_set_source(-1);
    switchbot_slot_text(slot)->make_call().set_value("").perform();
}

static void switchbot_set_label(lv_obj_t *label, const char *text)
{
    if (strcmp(lv_label_get_text(label), text) != 0)
        lv_label_set_text(label, text);
}

static void switchbot_set_hidden(lv_obj_t *obj, bool hidden)
{
    if (hidden != lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN))
    {
        if (hidden)
            lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
    }
}

// max_name > 0 shortens the name for narrow widgets (switchbot_format_short_title).
static void switchbot_slot_title(unsigned slot, char *text, size_t size, size_t max_name = 0)
{
    const SwitchbotSlot &s = switchbot_slots[slot];
    SwitchbotAddress address;
    const SwitchbotAddress *shown = nullptr;
    uint8_t model = 0;
    if (s.valid)
    {
        shown = &s.address;
        model = s.model;
    }
    else if (switchbot_parse_mac(switchbot_slot_text(slot)->state, address))
        shown = &address;
    if (shown != nullptr && max_name > 0)
        switchbot_format_short_title(*shown, model, text, size, max_name);
    else if (shown != nullptr)
        switchbot_format_title(*shown, model, text, size);
    else
        snprintf(text, size, "%s", switchbot_slot_text(slot)->state.c_str());
}

// ── Room temperature source dropdown: "Tr (Bedraad)" plus paired sensors ──
static int switchbot_dropdown_slots[1 + SWITCHBOT_SLOTS] = {-1, -1, -1, -1};

static void switchbot_update_source_dropdown()
{
    std::string options = "Tr (Bedraad)";
    unsigned count = 1, selected = 0;
    int source = switchbot_selected_source_slot();
    for (unsigned i = 0; i < SWITCHBOT_SLOTS; i++)
    {
        if (!id(ble_enabled_switch).state || switchbot_slot_text(i)->state.empty())
            continue;
        char title[64];
        // The 360 px dropdown fits about 34 characters next to the chevron.
        switchbot_slot_title(i, title, sizeof(title), 24);
        options += "\n";
        options += title;
        if (int(i) == source)
            selected = count;
        switchbot_dropdown_slots[count++] = i;
    }
    lv_obj_t *dd = id(ble_source_dropdown).obj;
    if (options != lv_dropdown_get_options(dd))
        lv_dropdown_set_options(dd, options.c_str());
    if (lv_dropdown_get_selected(dd) != selected)
        lv_dropdown_set_selected(dd, selected);
}

static void switchbot_source_dropdown_changed()
{
    unsigned selected = lv_dropdown_get_selected(id(ble_source_dropdown).obj);
    switchbot_set_source(selected <= SWITCHBOT_SLOTS ? switchbot_dropdown_slots[selected] : -1);
}

// ── Settings → Connectiviteit → Bluetooth ──
static void switchbot_update_bluetooth_page()
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
        snprintf(text, sizeof(text), "Schakel Bluetooth in om SwitchBot-sensoren te koppelen.");
    else if (switchbot_host_failed)
        snprintf(text, sizeof(text), "Bluetooth kon niet starten. Herstart het display om het opnieuw te proberen.");
    else if (switchbot_restart_required)
        snprintf(text, sizeof(text), "Bluetooth start na een herstart: te weinig vrij geheugen om nu te starten.");
    else
        text[0] = '\0';
    switchbot_set_label(&id(ble_hint_label), text);
    switchbot_set_hidden(&id(ble_hint_label), text[0] == '\0');
    switchbot_set_hidden(&id(ble_sensors_section), !enabled);
    if (!enabled)
        return;

    // Paired sensors: green check with a current reading, warning otherwise.
    lv_obj_t *rows[] = {&id(ble_paired_row_0), &id(ble_paired_row_1), &id(ble_paired_row_2)};
    lv_obj_t *icons[] = {&id(ble_paired_icon_0), &id(ble_paired_icon_1), &id(ble_paired_icon_2)};
    lv_obj_t *labels[] = {&id(ble_paired_label_0), &id(ble_paired_label_1), &id(ble_paired_label_2)};
    int source = switchbot_selected_source_slot();
    unsigned paired = 0;
    for (unsigned i = 0; i < SWITCHBOT_SLOTS; i++)
    {
        bool used = !switchbot_slot_text(i)->state.empty();
        switchbot_set_hidden(rows[i], !used);
        if (!used)
            continue;
        paired++;
        char title[64];
        switchbot_slot_title(i, title, sizeof(title));
        const char *role = int(i) == source ? "  |  ruimtetemperatuur" : "";
        const SwitchbotSlot &slot = switchbot_slots[i];
        const char *icon = "\U000F0028";
        uint32_t color = 0xD97706;
        if (switchbot_slot_fresh(i))
        {
            snprintf(text, sizeof(text), "%s%s\n%.1f °C   |   %.0f%% luchtvochtigheid   |   %lus geleden", title, role,
                     slot.reading.temperature, slot.reading.humidity,
                     (unsigned long)(uint32_t(millis() - slot.reading.received_ms) / 1000));
            icon = "\U000F05E0";
            color = 0x16A34A;
        }
        else
        {
            snprintf(text, sizeof(text), "%s%s\n%s", title, role,
                     slot.reading.received_ms ? "Geen actuele meting" : "Wacht op eerste meting");
        }
        switchbot_set_label(labels[i], text);
        switchbot_set_label(icons[i], icon);
        lv_obj_set_style_text_color(icons[i], lv_color_hex(color), LV_PART_MAIN);
    }
    snprintf(text, sizeof(text), paired ? "Gekoppelde sensoren (%u/%u)" : "Nog geen sensoren gekoppeld (0/%u)",
             paired ? paired : unsigned(SWITCHBOT_SLOTS), unsigned(SWITCHBOT_SLOTS));
    switchbot_set_label(&id(ble_paired_title), text);

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
    bool full = paired >= SWITCHBOT_SLOTS;
    SwitchbotDevice devices[SWITCHBOT_DISCOVERY_LIMIT];
    switchbot_discovery_snapshot(devices);
    unsigned count = 0;
    for (unsigned i = 0; i < SWITCHBOT_DISCOVERY_LIMIT; i++)
    {
        bool show = switchbot_device_fresh(devices[i]) && switchbot_slot_of(devices[i].address) < 0;
        switchbot_set_hidden(found[i], !show);
        if (!show)
            continue;
        count++;
        char mac[18], title[64];
        switchbot_format_mac(devices[i].address, mac);
        switchbot_format_title(devices[i].address, devices[i].model, title, sizeof(title));
        snprintf(text, sizeof(text), "%s  %s\n%.1f °C   |   %.0f%% luchtvochtigheid", title, mac,
                 devices[i].reading.temperature, devices[i].reading.humidity);
        switchbot_set_label(found_labels[i], text);
        if (full != lv_obj_has_state(pair_buttons[i], LV_STATE_DISABLED))
        {
            if (full)
                lv_obj_add_state(pair_buttons[i], LV_STATE_DISABLED);
            else
                lv_obj_clear_state(pair_buttons[i], LV_STATE_DISABLED);
            lv_obj_set_style_opa(pair_buttons[i], full ? LV_OPA_40 : LV_OPA_COVER, LV_PART_MAIN);
        }
    }
    if (switchbot_is_discovering())
        snprintf(text, sizeof(text), "Gevonden sensoren  |  zoeken: %us  |  %u gevonden", switchbot_discovery_seconds(),
                 count);
    else
        snprintf(text, sizeof(text), "Gevonden sensoren  |  %u gevonden%s", count,
                 full ? "  |  ontkoppel eerst een sensor" : "");
    switchbot_set_label(&id(ble_discovery_status_label), text);
}

static void switchbot_update_ui()
{
    if (!switchbot_boot_ready)
        return;
    static bool placed = false;
    if (!placed)
    {
        lv_obj_move_to_index(&id(ble_source_row), 1);
        placed = true;
    }
    switchbot_update_source_dropdown();
    if (!lv_obj_has_flag(&id(panel_wifi_page_bluetooth), LV_OBJ_FLAG_HIDDEN))
        switchbot_update_bluetooth_page();
}

// Connectiviteit sub-tab: shows the Bluetooth page and starts a search.
static void switchbot_show_bluetooth_page(bool show)
{
    switchbot_set_hidden(&id(panel_wifi_page_bluetooth), !show);
    lv_obj_set_style_bg_color(&id(conn_stab_bluetooth), lv_color_hex(show ? 0x1F3B68 : 0xCBD5E1), LV_PART_MAIN);
    lv_obj_set_style_text_color(&id(conn_stab_bluetooth_lbl), lv_color_hex(show ? 0xFFFFFF : 0x4B5563),
                                LV_PART_MAIN);
    if (show && switchbot_boot_ready && id(ble_enabled_switch).state && !switchbot_is_discovering())
        switchbot_begin_discovery();
    if (show)
    {
        switchbot_update_source();
        switchbot_update_ui();
    }
}
