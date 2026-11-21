#pragma once

// Firmware version comparison for the update channels (Stabiel / Beta).
//
// Releases use semver: 0.10.0 for a stable release, 0.10.0-beta.1 for a beta. ESPHome's http_request update
// only checks whether the manifest version differs from the installed one, which also reports an older stable
// version as an update while a beta is installed. These helpers compare the versions properly, the same way
// Home Assistant does: 0.9.0 < 0.10.0-beta.1 < 0.10.0-beta.2 < 0.10.0.

#include <cstdlib>
#include <string>

namespace openamber_fw {

struct Version {
  bool valid{false};
  long major{0};
  long minor{0};
  long patch{0};
  std::string pre;  // pre-release part without the '-', empty for a stable release
};

inline bool parse_number_(const std::string &s, size_t &pos, long &out) {
  size_t start = pos;
  while (pos < s.size() && s[pos] >= '0' && s[pos] <= '9')
    pos++;
  if (pos == start)
    return false;
  out = std::strtol(s.substr(start, pos - start).c_str(), nullptr, 10);
  return true;
}

// Parses "1.2.3", "v1.2.3", "1.2.3-beta.1". A build suffix after '+' or '_' is ignored. Anything else, such as
// the "dev-20261121-0751" versions of development builds, is not valid.
inline Version parse(const std::string &text) {
  Version v;
  std::string s = text;
  size_t cut = s.find_first_of("+_");
  if (cut != std::string::npos)
    s.erase(cut);
  size_t pos = 0;
  if (pos < s.size() && (s[pos] == 'v' || s[pos] == 'V'))
    pos++;
  if (!parse_number_(s, pos, v.major) || pos >= s.size() || s[pos++] != '.')
    return v;
  if (!parse_number_(s, pos, v.minor) || pos >= s.size() || s[pos++] != '.')
    return v;
  if (!parse_number_(s, pos, v.patch))
    return v;
  if (pos < s.size()) {
    if (s[pos] != '-' || pos + 1 >= s.size())
      return v;
    v.pre = s.substr(pos + 1);
  }
  v.valid = true;
  return v;
}

inline bool is_numeric_(const std::string &s) {
  if (s.empty())
    return false;
  for (char c : s)
    if (c < '0' || c > '9')
      return false;
  return true;
}

// Semver pre-release precedence: dot separated identifiers, numbers compare numerically and sort before text,
// and a shorter list sorts first when all shared identifiers are equal.
inline int compare_pre_(const std::string &a, const std::string &b) {
  size_t ia = 0, ib = 0;
  while (true) {
    bool end_a = ia > a.size(), end_b = ib > b.size();
    if (end_a || end_b)
      return end_a == end_b ? 0 : (end_a ? -1 : 1);
    size_t na = a.find('.', ia), nb = b.find('.', ib);
    if (na == std::string::npos)
      na = a.size();
    if (nb == std::string::npos)
      nb = b.size();
    std::string pa = a.substr(ia, na - ia), pb = b.substr(ib, nb - ib);
    bool num_a = is_numeric_(pa), num_b = is_numeric_(pb);
    if (num_a && num_b) {
      long xa = std::strtol(pa.c_str(), nullptr, 10), xb = std::strtol(pb.c_str(), nullptr, 10);
      if (xa != xb)
        return xa < xb ? -1 : 1;
    } else if (num_a != num_b) {
      return num_a ? -1 : 1;
    } else if (pa != pb) {
      return pa < pb ? -1 : 1;
    }
    ia = na + 1;
    ib = nb + 1;
  }
}

// -1, 0 or 1. Both versions must be valid.
inline int compare(const Version &a, const Version &b) {
  if (a.major != b.major)
    return a.major < b.major ? -1 : 1;
  if (a.minor != b.minor)
    return a.minor < b.minor ? -1 : 1;
  if (a.patch != b.patch)
    return a.patch < b.patch ? -1 : 1;
  if (a.pre.empty() || b.pre.empty())
    return a.pre.empty() == b.pre.empty() ? 0 : (a.pre.empty() ? 1 : -1);  // a release outranks its betas
  return compare_pre_(a.pre, b.pre);
}

inline bool is_prerelease(const std::string &version) {
  Version v = parse(version);
  return v.valid && !v.pre.empty();
}

// True when `latest` is a real update over `current`. Versions that cannot be compared (development builds)
// fall back to ESPHome's own check: any different version is an update.
inline bool is_newer(const std::string &latest, const std::string &current) {
  if (latest.empty() || latest == current)
    return false;
  Version l = parse(latest), c = parse(current);
  if (!l.valid || !c.valid)
    return true;
  return compare(l, c) > 0;
}

enum UpdateOffer {
  OFFER_NONE = 0,
  OFFER_UPDATE = 1,    // a newer version is available
  OFFER_ROLLBACK = 2,  // a beta is installed and the user chose Stabiel: offer to go back to the stable version
};

inline UpdateOffer update_offer(const std::string &latest, const std::string &current, bool beta_channel) {
  // The Stabiel channel never installs a beta, also not one left over from a check on the beta channel before
  // the channel was switched (or one whose check is still in flight).
  if (!beta_channel && is_prerelease(latest))
    return OFFER_NONE;
  if (is_newer(latest, current))
    return OFFER_UPDATE;
  if (!beta_channel && is_prerelease(current) && parse(latest).valid && !is_prerelease(latest) && latest != current)
    return OFFER_ROLLBACK;
  return OFFER_NONE;
}

}  // namespace openamber_fw
