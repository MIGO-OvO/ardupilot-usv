"""AP_FLAKE8_CLEAN: local-only smoke scenarios for a freshly built Rover SITL.

Never connects to an existing vehicle: starts its own simulator, private UDP
loopback port and temporary storage. Requires the existing pymavlink environment.
"""
import argparse
import json
import os
import pathlib
import socket
import subprocess
import tempfile
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil  # noqa: E402 (MAVLINK20 must be set before importing pymavlink)

SCENARIOS = ('success', 'cancel', 'timeout', 'link_loss', 'manual_takeover', 'rtl_takeover')


class Peer:
    def __init__(self, connection):
        self.connection = connection
        self.payload_enabled = True
        self.last_tx = 0
        self.max_seq = 0
        self.texts = []
        self.failures = []

    def named(self, name, value, component=191, system=1):
        mav = self.connection.mav
        mav.srcSystem, mav.srcComponent = system, component
        try:
            mav.named_value_float_send(0, name.encode(), float(value))
        finally:
            mav.srcSystem, mav.srcComponent = 255, 190

    def wait(self, predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if time.monotonic() - self.last_tx >= 0.25:
                self.connection.mav.heartbeat_send(6, 8, 0, 0, 4)
                if self.payload_enabled:
                    self.named('USV_PKT', 1)
                self.last_tx = time.monotonic()
            msg = self.connection.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                continue
            if msg.get_type() == 'MISSION_CURRENT':
                self.max_seq = max(self.max_seq, msg.seq)
            if msg.get_type() == 'STATUSTEXT':
                self.texts.append(str(msg.text))
                self.texts = self.texts[-12:]
            if (msg.get_type() == 'NAMED_VALUE_FLOAT' and msg.get_srcSystem() == 1 and
                    msg.get_srcComponent() == 1 and msg.name == 'USV_FAIL'):
                self.failures.append(msg)
            if predicate(msg):
                return msg
        raise AssertionError('SITL wait timeout; recent status: ' + repr(self.texts))

    def assert_mode_for(self, expected, duration=2):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            msg = self.wait(lambda m: m.get_type() == 'HEARTBEAT' and m.get_srcComponent() == 1, timeout=5)
            assert msg.custom_mode == expected, 'expected mode %s, got %s' % (expected, msg.custom_mode)

    def upload(self, timeout_s):
        mav = self.connection.mav
        mav.mission_count_send(1, 1, 3)
        while True:
            msg = self.wait(lambda m: m.get_type() in ('MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'))
            if msg.get_type() == 'MISSION_ACK':
                assert msg.type == 0, 'mission upload rejected: %s' % msg.type
                return
            seq = msg.seq
            assert seq in (0, 1, 2)
            command = 42702 if seq == 1 else 16
            p1, p2 = (1, timeout_s) if seq == 1 else (0, 0)
            lat, lon = ((0, 0) if seq == 1 else (-35.362938 + seq * 0.0001, 149.165085))
            frame = 2 if seq == 1 else 3
            send = mav.mission_item_int_send if msg.get_type() == 'MISSION_REQUEST_INT' else mav.mission_item_send
            if msg.get_type() == 'MISSION_REQUEST_INT':
                lat, lon = int(lat * 1e7), int(lon * 1e7)
            send(1, 1, seq, frame, command, 0, 1, p1, p2, 0, 0, lat, lon, 0)


def scenario(binary, name, instance=178):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    connection = mavutil.mavlink_connection(
        'udpin:127.0.0.1:%d' % port, source_system=255, source_component=190)
    with tempfile.TemporaryDirectory(prefix='usv-sitl-') as directory:
        with open(pathlib.Path(directory) / 'sitl.log', 'w') as output:
            process = subprocess.Popen([str(binary), '--model', 'rover', '--speedup', '1', '--instance', str(instance),
                                        '--home', '-35.362938,149.165085,584,0',
                                        '--serial0', 'udpclient:127.0.0.1:%d' % port],
                                       cwd=directory, stdout=output, stderr=subprocess.STDOUT)
            try:
                peer = Peer(connection)
                peer.wait(lambda m: m.get_type() == 'HEARTBEAT' and m.get_srcComponent() == 1)
                connection.mav.request_data_stream_send(1, 1, 0, 4, 1)
                connection.mav.param_set_send(1, 1, b'ARMING_CHECK', 0, 9)
                peer.wait(lambda m: m.get_type() == 'GPS_RAW_INT' and m.fix_type >= 3, timeout=60)
                peer.wait(lambda m: m.get_type() == 'EKF_STATUS_REPORT' and m.flags & 16, timeout=60)
                peer.upload(6 if name == 'timeout' else 30)
                connection.mav.command_long_send(1, 1, 400, 0, 1, 21196, 0, 0, 0, 0, 0)
                peer.wait(lambda m: m.get_type() == 'HEARTBEAT' and m.base_mode & 128)
                connection.mav.set_mode_send(1, 1, 10)
                trigger = peer.wait(lambda m: m.get_type() == 'NAMED_VALUE_FLOAT' and m.name == 'USV_SMPL', timeout=60)
                sample_id = int(trigger.value)
                peer.max_seq = 1  # ignore the pre-upload "no mission" index
                failure_elapsed_ms = None
                if name == 'success':
                    peer.named('USV_DONE', sample_id)
                    peer.wait(lambda m: m.get_type() == 'MISSION_CURRENT' and m.seq >= 2)
                elif name in ('manual_takeover', 'rtl_takeover'):
                    takeover_mode = 0 if name == 'manual_takeover' else 11
                    connection.mav.set_mode_send(1, 1, takeover_mode)
                    peer.wait(lambda m: m.get_type() == 'HEARTBEAT' and m.custom_mode == takeover_mode)
                    peer.named('USV_FAIL', sample_id)
                    peer.assert_mode_for(takeover_mode, duration=3)
                    assert peer.max_seq <= 1, 'operator takeover advanced mission'
                else:
                    if name == 'cancel':
                        peer.named('USV_FAIL', sample_id)
                    elif name == 'link_loss':
                        peer.payload_enabled = False
                    else:
                        # Wrong sources, IDs and non-finite values cannot release this sample.
                        peer.named('USV_DONE', sample_id, component=190)
                        peer.named('USV_DONE', sample_id, system=2)
                        for invalid_id in (0, sample_id + 1, sample_id + 0.5, 65536, float('nan'), float('inf')):
                            peer.named('USV_DONE', invalid_id)
                    peer.wait(lambda m: m.get_type() == 'HEARTBEAT' and m.custom_mode == 4)
                    if name in ('timeout', 'link_loss'):
                        if not peer.failures:
                            peer.wait(lambda m: bool(peer.failures), timeout=5)
                        assert len(peer.failures) == 1, 'expected one FCU cancellation notification'
                        failure = peer.failures[0]
                        assert failure.value == sample_id, 'FCU cancelled the wrong sampling ID'
                        failure_elapsed_ms = (failure.time_boot_ms - trigger.time_boot_ms) & 0xffffffff
                        lower, upper = (5900, 10000) if name == 'timeout' else (2900, 6000)
                        assert lower <= failure_elapsed_ms <= upper, 'wrong failure timing: %s ms' % failure_elapsed_ms
                        reason = 'USV: sampling timeout/link loss, HOLD'
                    else:
                        reason = 'USV: sampling failed/cancelled, HOLD'
                    if reason not in peer.texts:
                        peer.wait(lambda m: reason in peer.texts, timeout=5)
                    peer.named('USV_DONE', sample_id)  # Late success must not undo a failed sample.
                    peer.assert_mode_for(4)
                    assert peer.max_seq <= 1, 'failed sample advanced mission'
                print(json.dumps({'scenario': name, 'sample_id': sample_id,
                                  'fcu_failures': len(peer.failures), 'failure_elapsed_ms': failure_elapsed_ms,
                                  'max_mission_seq': peer.max_seq, 'result': 'passed'}), flush=True)
            finally:
                connection.close()
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=pathlib.Path, required=True)
    parser.add_argument('--scenario', choices=SCENARIOS + ('all',), default='all')
    parser.add_argument('--instance', type=int, default=178)
    args = parser.parse_args()
    binary = args.binary.resolve(strict=True)
    for name in SCENARIOS if args.scenario == 'all' else (args.scenario,):
        scenario(binary, name, args.instance)
