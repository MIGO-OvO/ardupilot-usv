"""AP_FLAKE8_CLEAN: run the production cache and link scheduler with bounded txspace."""
import pathlib
import shutil
import subprocess
import tempfile
import unittest

from test_usv_sampling_safety import function_body

ROOT = pathlib.Path(__file__).resolve().parents[1]


class PayloadCacheTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'), 'requires host g++ (WSL/Linux)')
    def test_real_cache_and_per_link_scheduler(self):
        source = (ROOT / 'Rover/GCS_MAVLink_Rover.cpp').read_text()
        harness = r'''
#include "Rover/USVPayload.h"
#include <cassert>
#include <set>
#include <string>
uint32_t now_ms = 100;
namespace AP_HAL { uint32_t millis() { return now_ms; } }
struct { USVPayload usv_payload; } rover;
#define HAVE_PAYLOAD_SPACE(chan, type) (space > 0)
struct GCS_MAVLINK_Rover {
    uint8_t usv_payload_cursor = 0;
    unsigned space = 0;
    std::set<std::string> sent;
    void send_named_float(const char *name, float) { assert(space); space--; sent.insert(name); }
    void send_usv_payload();
};
'''
        signature = 'void GCS_MAVLINK_Rover::send_usv_payload()'
        harness += signature + function_body(source, signature)
        harness += r'''
int main() {
    GCS_MAVLINK_Rover slow, fast;
    slow.space = 8;
    slow.send_usv_payload();
    assert(slow.sent.empty()); // No invented zero-value fields before first reception.
    for (uint8_t i = 0; i < USVPayload::FIELD_COUNT; i++) {
        assert(rover.usv_payload.update(USVPayload::name(i), float(i), now_ms));
    }
    for (unsigned cycle = 0; cycle < 4; cycle++) {
        slow.space = 8;
        slow.send_usv_payload();
        fast.space = 26;
        fast.send_usv_payload();
        now_ms += 500;
    }
    assert(slow.sent.size() == 26 && fast.sent.size() == 26);
    slow.space = 0;
    const uint8_t blocked_cursor = slow.usv_payload_cursor;
    slow.send_usv_payload();
    assert(slow.usv_payload_cursor == blocked_cursor);
    now_ms = 4000;
    assert(rover.usv_payload.update("PUMP_X", 42, now_ms));
    slow.sent.clear(); slow.space = 26; slow.send_usv_payload();
    assert(slow.sent == std::set<std::string>{"PUMP_X"});
    assert(!rover.usv_payload.update("USV_DONE", 1, 4500));
    assert(rover.usv_payload.last_update_ms == 4000);
    rover.usv_payload = {};
    assert(rover.usv_payload.update("USV_VLD", 1, UINT32_MAX - 100));
    assert(rover.usv_payload.fresh(18, 100)); // HAL clock rollover.
    assert(!rover.usv_payload.fresh(18, 4000));
}
'''
        with tempfile.TemporaryDirectory() as directory:
            binary = pathlib.Path(directory) / 'payload'
            subprocess.run(['g++', '-x', 'c++', '-std=c++11', '-Wall', '-Werror', '-I', str(ROOT),
                            '-o', str(binary), '-'], input=harness, text=True, check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
