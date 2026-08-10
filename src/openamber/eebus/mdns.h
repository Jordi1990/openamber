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
namespace openamber_eebus {

// Advertise the device as an EEBus HMS over mDNS (`_ship._tcp`) with the SHIP
// TXT record set that eebus-go/evcc expects (see ship-go/mdns + SHIP 7.3.2):
//   txtvers=1, id=<shipid>, path=/ship/, ski=<cert SKI>, register=<bool>
void eebus_mdns_start(const std::string &device_sku, const std::string &brand,
                      const std::string &model, const std::string &ski,
                      const std::string &ship_id, uint16_t ship_port);

}  // namespace openamber_eebus
}  // namespace esphome
