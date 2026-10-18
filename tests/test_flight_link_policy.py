"""AP_FLAKE8_CLEAN: execute the production Rover scheduler for both feature flags."""
import pathlib
import shutil
import subprocess
import tempfile
import unittest

from test_usv_sampling_safety import function_body

ROOT = pathlib.Path(__file__).resolve().parents[1]


class FlightLinkPolicyTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'), 'requires host g++ (WSL/Linux)')
    def test_real_scheduler_filters_private_inactive_and_high_latency(self):
        signature = 'void Rover::usv_telemetry_send()'
        body = function_body((ROOT / 'Rover/sensors.cpp').read_text(), signature)
        for high_latency in (0, 1):
            with self.subTest(HAL_HIGH_LATENCY2_ENABLED=high_latency):
                source = r'''
#include <cassert>
#include <cstdint>
struct GCS_MAVLINK_Rover {
    bool active = true, private_link = false;
#if HAL_HIGH_LATENCY2_ENABLED
    bool is_high_latency_link = false;
#endif
    unsigned calls = 0;
    bool is_active() const { return active; }
    bool is_private() const { return private_link; }
    void send_usv_payload() { calls++; }
};
struct GCS {
    GCS_MAVLINK_Rover links[4];
    uint8_t num_gcs() const { return 5; }
    GCS_MAVLINK_Rover *chan(uint8_t i) { return i < 4 ? &links[i] : nullptr; }
};
struct Rover {
    GCS backend;
    GCS &gcs() { return backend; }
    void usv_telemetry_send();
};
'''
                source += signature + body
                source += r'''
int main() {
    Rover rover;
    rover.backend.links[1].private_link = true;
    rover.backend.links[2].active = false;
#if HAL_HIGH_LATENCY2_ENABLED
    rover.backend.links[3].is_high_latency_link = true;
#endif
    rover.usv_telemetry_send();
    assert(rover.backend.links[0].calls == 1);
    assert(rover.backend.links[1].calls == 0);
    assert(rover.backend.links[2].calls == 0);
#if HAL_HIGH_LATENCY2_ENABLED
    assert(rover.backend.links[3].calls == 0);
#else
    assert(rover.backend.links[3].calls == 1);
#endif
}
'''
                with tempfile.TemporaryDirectory() as directory:
                    binary = pathlib.Path(directory) / 'link_policy'
                    subprocess.run(['g++', '-x', 'c++', '-std=c++11', '-Wall', '-Werror',
                                    '-DHAL_HIGH_LATENCY2_ENABLED=%d' % high_latency, '-o', str(binary), '-'],
                                   input=source, text=True, check=True)
                    subprocess.run([str(binary)], cwd=directory, check=True)


if __name__ == '__main__':
    unittest.main()
