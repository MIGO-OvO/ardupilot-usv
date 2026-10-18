#!/usr/bin/env python3
"""AP_FLAKE8_CLEAN: two isolated UDP links; no serial ports or hardware commands."""
import argparse
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil  # noqa: E402


def udp_peer(system, component):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    return port, mavutil.mavlink_connection(
        'udpin:127.0.0.1:%d' % port, source_system=system, source_component=component)


def verify(binary):
    gcs_port, gcs = udp_peer(255, 190)
    companion_port, companion = udp_peer(1, 191)
    with tempfile.TemporaryDirectory(prefix='usv-event-route-') as folder:
        defaults = Path(folder) / 'protocol.parm'
        defaults.write_text('SERIAL0_PROTOCOL 2\nSERIAL1_PROTOCOL 2\n')
        with (Path(folder) / 'sitl.log').open('w') as output:
            process = subprocess.Popen([
                str(binary), '--model', 'rover', '--instance', '179', '--defaults', str(defaults),
                '--serial0', 'udpclient:127.0.0.1:%d' % gcs_port,
                '--serial1', 'udpclient:127.0.0.1:%d' % companion_port,
            ], cwd=folder, stdout=output, stderr=subprocess.STDOUT)
            try:
                assert gcs.wait_heartbeat(timeout=20), 'GCS link not ready'
                assert companion.wait_heartbeat(timeout=20), 'companion link not ready'
                gcs.mav.heartbeat_send(6, 8, 0, 0, 0)
                companion.mav.heartbeat_send(18, 8, 0, 0, 0)
                expected = [(73, 0, 'USVE1:' + 'a' * 44), (73, 1, 'tail')]
                deadline = time.monotonic() + 10
                received = set()
                last_send = 0
                while time.monotonic() < deadline and len(received) < len(expected):
                    if time.monotonic() - last_send > 1:
                        for identifier, sequence, text in expected:
                            companion.mav.statustext_send(5, text.encode('ascii'), identifier, sequence)
                        last_send = time.monotonic()
                    message = gcs.recv_match(type='STATUSTEXT', blocking=True, timeout=0.2)
                    if message and message.get_srcSystem() == 1 and message.get_srcComponent() == 191:
                        received.add((message.id, message.chunk_seq, message.text))
                assert set(expected) <= received, 'STATUSTEXT source/id/chunk/text was not preserved: %r' % received
                print('PASS: SITL routes complete MAVLink2 STATUSTEXT chunks from 1/191 across links unchanged')
            finally:
                gcs.close()
                companion.close()
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    verify(parser.parse_args().binary.resolve(strict=True))
