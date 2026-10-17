/*
 * Open Amber - native EEBus PoC
 *
 * mDNS advertisement of the EEBus SHIP service (_ship._tcp) so evcc can
 * discover and pair with the device on the local network.
 */

#pragma once

#include "esphome/core/component.h"
#include <string>

namespace esphome {
namespace eebus {

// Advertise the device as an EEBus node over mDNS (`_ship._tcp`) with the SHIP
// TXT record set that eebus-go/evcc/VenusOS expects (see ship-go/mdns + SHIP 7.3.2):
//   txtvers=1, id=<shipid>, path=/ship/, ski=<cert SKI>, register=<bool>,
//   brand=<brand>, model=<model>, type=<type>, cat=<category>
bool eebus_mdns_start(const std::string &device_sku, const std::string &brand,
                      const std::string &model, const std::string &device_category,
                      const std::string &device_type, const std::string &ski,
                      const std::string &ship_id, uint16_t ship_port);
void eebus_mdns_set_registered(bool registered);
void eebus_mdns_stop();

}  // namespace eebus
}  // namespace esphome
