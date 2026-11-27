"""Compile the actual YAML calculations without a device or simulator."""
from pathlib import Path
import shutil
import subprocess

import yaml

from test_entity_schema_sync import ESPHomeYamlLoader


ROOT = Path(__file__).resolve().parents[1]


def sensors(path):
    return {
        sensor["id"]: sensor
        for sensor in yaml.load((ROOT / path).read_text(), Loader=ESPHomeYamlLoader)["sensor"]
        if "id" in sensor
    }


def test_experimental_flow(tmp_path):
    compiler = shutil.which("g++")
    assert compiler, "g++ is required for the production estimator regression test"
    flow = sensors("src/openamber/core/experimental_flow.yaml")
    source = tmp_path / "experimental_flow.cpp"
    harness = r"""
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

struct Sensor { float state = NAN; };
Sensor pump_feedback_1221_raw, outlet_temperature_tuo, inside_unit_status_1225_raw,
       flow_correction_3494_raw;
#define id(name) name
float feedback() { FEEDBACK_LAMBDA }
float high(float x) { HIGH_LAMBDA }
float medium_low(float x) { MEDIUM_LOW_LAMBDA }
float estimate(bool use_high) {
  return use_high ? high(feedback()) : medium_low(feedback());
}
void close(float actual, float expected) {
  assert(std::isfinite(actual) && std::fabs(actual - expected) < 0.00002f);
}
void reset() {
  pump_feedback_1221_raw.state = 250;
  outlet_temperature_tuo.state = 30;
  inside_unit_status_1225_raw.state = 0;
  flow_correction_3494_raw.state = 0;
}
int main() {
  assert(std::isnan(estimate(true)));
  assert(std::isnan(estimate(false)));
  reset();
  close(estimate(true), 24.486667f);
  close(estimate(false), 26.476667f);
  flow_correction_3494_raw.state = 60;
  close(estimate(true), 25.486667f);
  close(estimate(false), 27.476667f);
  flow_correction_3494_raw.state = -120;
  close(estimate(true), 22.486667f);
  close(estimate(false), 24.476667f);
  for (float raw : {0.0f, 49.0f}) {
    pump_feedback_1221_raw.state = raw;
    close(estimate(true), 0);
    close(estimate(false), 0);
  }
  reset();
  pump_feedback_1221_raw.state = 50;
  close(estimate(true), 3.1533337f);
  close(estimate(false), 5.1433334f);
  for (float raw : {700.0f, 701.0f, 1000.0f, 65535.0f}) {
    pump_feedback_1221_raw.state = raw;
    close(estimate(true), 72.486671f);
    close(estimate(false), 74.476669f);
  }
  // Correction is conditional on the base result, not on feedback or final flow.
  pump_feedback_1221_raw.state = 50;
  flow_correction_3494_raw.state = 600;
  for (bool high : {true, false}) {
    for (float base : {-1.0f, 0.09f, 0.11f}) {
      outlet_temperature_tuo.state = (base - 1.0666667222976685f * 5 +
                                    (high ? 4.88f : 1.69f)) / (high ? 0.09f : 0.05f);
      close(estimate(high), base + (base > 0.1f ? 10 : 0));
    }
  }
  reset();
  flow_correction_3494_raw.state = -3000;
  close(estimate(true), -25.513333f);  // No clamp after correction.
  close(estimate(false), -23.523333f);
  // Both status bits invalidate both estimates, even below the 5% threshold.
  for (float status : {32768.0f, 8.0f, 32776.0f, -1.0f, 65536.0f}) {
    reset();
    inside_unit_status_1225_raw.state = status;
    pump_feedback_1221_raw.state = 0;
    assert(std::isnan(estimate(true)));
    assert(std::isnan(estimate(false)));
  }
  reset();
  inside_unit_status_1225_raw.state = 4;  // Unrelated temperature fault.
  close(estimate(true), 24.486667f);
  // Missing (NaN) and non-finite inputs invalidate both outputs.
  for (auto *sensor : {&pump_feedback_1221_raw, &outlet_temperature_tuo,
                       &inside_unit_status_1225_raw, &flow_correction_3494_raw}) {
    reset();
    const float valid = sensor->state;
    for (float invalid : {NAN, std::numeric_limits<float>::infinity()}) {
      sensor->state = invalid;
      assert(std::isnan(estimate(true)));
      assert(std::isnan(estimate(false)));
      sensor->state = valid;
      close(estimate(true), 24.486667f);
      close(estimate(false), 26.476667f);
    }
  }
}
"""
    source.write_text(
        harness.replace("FEEDBACK_LAMBDA", flow["experimental_flow_feedback_percent"]["lambda"])
        .replace("HIGH_LAMBDA", flow["experimental_flow_high"]["filters"][0]["lambda"])
        .replace("MEDIUM_LOW_LAMBDA", flow["experimental_flow_medium_low"]["filters"][0]["lambda"])
    )
    binary = tmp_path / "experimental_flow"
    subprocess.run(
        [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
         "-fsanitize=address,undefined", str(source),
         "-o", str(binary)], check=True,
    )
    subprocess.run([str(binary)], check=True)
