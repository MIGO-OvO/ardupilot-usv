"""AP_FLAKE8_CLEAN: execute the actual script verification body with fake HAL/rover.

This is a host C++ regression, not SITL or a vehicle safety certification.
"""
import pathlib
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


def function_body(source, signature):
    start = source.index('{', source.index(signature))
    depth = 1
    pos = start + 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[start:pos]


class SamplingSafetyTests(unittest.TestCase):
    def run_mode_lifecycle(self, main):
        source = (ROOT / 'Rover/mode_auto.cpp').read_text()
        harness = r'''
#include <cstdint>
#include <cassert>
#include <cstring>
#include <initializer_list>
#define AP_SCRIPTING_ENABLED 1
#define GCS_SEND_TEXT(...) ((void)0)
uint32_t now_ms = 1;
uint32_t millis() { return now_ms; }
namespace AP_HAL { uint32_t millis() { return now_ms; } }
enum class ModeReason { FAILSAFE };
struct Mode {};
struct {
    Mode mode_hold, mode_manual, mode_rtl;
    Mode *control_mode = nullptr;
    bool held = false;
    struct { bool can_enter = true; bool enter() { return can_enter; } } mode_guided;
    struct { uint32_t last_update_ms = 1; } usv_payload;
    bool set_mode(Mode &mode, ModeReason) { control_mode = &mode; held = true; return true; }
} rover;
namespace AP_Mission {
struct Mission_Command { struct { struct {
    uint8_t command = 1, timeout_s = 5;
    struct { float get() const { return 0; } } arg1, arg2;
    int16_t arg3 = 0, arg4 = 0;
} nav_script_time; } content; };
}
unsigned fail_messages = 0;
uint16_t failed_id = 0;
struct ModeAuto : Mode {
    enum class SubMode { NavScriptTime, Stop };
    SubMode _submode = SubMode::Stop;
    struct { bool done = false; uint16_t id = 0; uint32_t start_ms = 0;
        uint8_t command = 0, timeout_s = 0; float arg1 = 0, arg2 = 0;
        int16_t arg3 = 0, arg4 = 0; } nav_scripting;
    struct GCS { void send_named_float(const char *name, float value) {
        if (strcmp(name, "USV_FAIL") == 0) { fail_messages++; failed_id = uint16_t(value); }
    } };
    GCS gcs() { return {}; }
    void start_stop();
    void do_nav_script_time(const AP_Mission::Mission_Command& cmd);
    void nav_script_time_done(uint16_t id);
    void usv_sampling_failed(uint16_t id);
    bool verify_nav_script_time();
};
'''
        for signature in (
            'void ModeAuto::start_stop()',
            'void ModeAuto::do_nav_script_time(const AP_Mission::Mission_Command& cmd)',
            'void ModeAuto::nav_script_time_done(uint16_t id)',
            'void ModeAuto::usv_sampling_failed(uint16_t id)',
            'bool ModeAuto::verify_nav_script_time()',
        ):
            harness += signature + function_body(source, signature) + '\n'
        harness += '\nint main() {\n' + main + '\n}\n'
        with tempfile.TemporaryDirectory() as directory:
            executable = pathlib.Path(directory) / 'lifecycle'
            subprocess.run(['g++', '-x', 'c++', '-std=c++11', '-Wall', '-Werror',
                            '-o', str(executable), '-'], input=harness, text=True, check=True)
            subprocess.run([str(executable)], cwd=directory, check=True)

    @unittest.skipUnless(shutil.which('g++'), 'requires host g++ (run in WSL/Linux)')
    def test_delayed_failure_preserves_manual_and_rtl_takeover(self):
        self.run_mode_lifecycle(r'''
    for (Mode *takeover : {&rover.mode_manual, &rover.mode_rtl}) {
        ModeAuto mode;
        rover.control_mode = &mode;
        rover.held = false;
        AP_Mission::Mission_Command sample;
        mode.do_nav_script_time(sample);
        rover.control_mode = takeover; // AUTO exit retains its script submode and ID.
        mode.usv_sampling_failed(mode.nav_scripting.id);
        assert(rover.control_mode == takeover);
        assert(!rover.held);
    }
''')

    @unittest.skipUnless(shutil.which('g++'), 'requires host g++ (run in WSL/Linux)')
    def test_non_usv_failed_entry_after_sampling_retains_upstream_completion(self):
        self.run_mode_lifecycle(r'''
    ModeAuto mode;
    rover.control_mode = &mode;
    AP_Mission::Mission_Command command;
    mode.do_nav_script_time(command);
    mode.nav_script_time_done(mode.nav_scripting.id);
    assert(mode.verify_nav_script_time());
    now_ms = 10001; // Intervening navigation outlasts the completed USV timeout.
    rover.usv_payload.last_update_ms = now_ms;
    rover.mode_guided.can_enter = false;
    command.content.nav_script_time.command = 2;
    mode.do_nav_script_time(command);
    assert(mode.verify_nav_script_time());
    assert(!rover.held);
    assert(fail_messages == 0);
''')

    @unittest.skipUnless(shutil.which('g++'), 'requires host g++ (run in WSL/Linux)')
    def test_failure_requires_current_usv_id_and_stops_it(self):
        self.run_mode_lifecycle(r'''
    ModeAuto mode;
    rover.control_mode = &mode;
    AP_Mission::Mission_Command command;
    mode.do_nav_script_time(command);
    const uint16_t sample_id = mode.nav_scripting.id;
    mode.usv_sampling_failed(sample_id + 1);
    assert(!rover.held);
    mode.usv_sampling_failed(sample_id);
    assert(rover.control_mode == &rover.mode_hold);
    assert(!mode.nav_scripting.done);
    assert(mode._submode == ModeAuto::SubMode::Stop);
    mode.nav_script_time_done(sample_id); // Delayed success cannot undo cancellation.
    assert(!mode.nav_scripting.done);
    rover.control_mode = &mode;
    rover.held = false;
    command.content.nav_script_time.command = 2;
    mode.do_nav_script_time(command);
    mode.usv_sampling_failed(mode.nav_scripting.id);
    assert(!rover.held);
''')

    @unittest.skipUnless(shutil.which('g++'), 'requires host g++ (run in WSL/Linux)')
    def test_actual_verifier_never_advances_failed_usv_sampling(self):
        body = function_body((ROOT / 'Rover/mode_auto.cpp').read_text(),
                             'bool ModeAuto::verify_nav_script_time()')
        harness = r'''
#include <cstdint>
#include <cassert>
#define MAV_SEVERITY_CRITICAL 2
#define GCS_SEND_TEXT(...) ((void)0)
uint32_t now_ms = 10001;
namespace AP_HAL { uint32_t millis() { return now_ms; } }
enum class ModeReason { FAILSAFE };
struct { int mode_hold = 4; bool held = false;
    struct { uint32_t last_update_ms = 10001; } usv_payload;
    bool set_mode(int, ModeReason) { held = true; return true; }
} rover;
struct ModeAuto {
    struct { bool done = false; uint16_t id = 1; uint32_t start_ms = 1;
        uint8_t command = 1; uint8_t timeout_s = 5; } nav_scripting;
    void start_stop() {}
    struct GCS { void send_named_float(const char*, float) {} };
    GCS gcs() { return {}; }
    bool verify_nav_script_time();
};
bool ModeAuto::verify_nav_script_time() BODY
int main() {
    ModeAuto mode;
    assert(!mode.verify_nav_script_time());
    assert(rover.held);
    rover.held = false;
    mode.nav_scripting.command = 2;
    assert(mode.verify_nav_script_time()); // unrelated Lua timeout unchanged
    assert(!rover.held);
    mode.nav_scripting.command = 1;
    mode.nav_scripting.start_ms = now_ms;
    mode.nav_scripting.done = true;
    assert(mode.verify_nav_script_time());
    mode.nav_scripting.done = false;
    assert(!mode.verify_nav_script_time());
    mode.nav_scripting.start_ms = 1;
    mode.nav_scripting.timeout_s = 255;
    rover.usv_payload.last_update_ms = 1;
    assert(!mode.verify_nav_script_time());
    assert(rover.held); // companion link loss is never an implicit SKIP
}
'''.replace('BODY', body)
        with tempfile.TemporaryDirectory() as directory:
            source = pathlib.Path(directory) / 'safety.cpp'
            executable = pathlib.Path(directory) / 'safety'
            source.write_text(harness)
            subprocess.run(['g++', '-std=c++11', '-Wall', '-Werror', str(source), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    unittest.main()
