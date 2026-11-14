#!/usr/bin/env python3
"""Isolated AS2 DDS subscriber. No command client or RPC writer."""
import argparse
from collections import deque
import fcntl
import json
import math
from pathlib import Path
import signal
import struct
import time
import uuid

from cyclonedds.domain import DomainParticipant
from cyclonedds.qos import Policy, Qos
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

ROOT = Path(__file__).resolve().parents[1]


def atomic(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, allow_nan=False))
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=3600.)
    args = parser.parse_args()
    if not 0 < args.duration <= 86400:
        parser.error('duration out of range')
    output = ROOT/'runs/perception_live'
    output.mkdir(exist_ok=True)
    lock = (output/'telemetry.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    generation = uuid.uuid4().hex
    ChannelFactoryInitialize(0, 'end0')
    participant = DomainParticipant(0)
    qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(1))
    sport_reader = DataReader(participant, Topic(participant, 'rt/lf/sportmodestate', SportModeState_), qos)
    low_reader = DataReader(participant, Topic(participant, 'rt/lf/lowstate', LowState_), qos)
    stopped = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_:stopped.__setitem__(0, True))
    history = deque(maxlen=120)
    sport = low = axes = None
    epoch = 0
    began = written = last_history = time.monotonic()
    counter = 0
    print(json.dumps(dict(status='read_only_subscriber_started', robot_commands_sent=0)), flush=True)
    try:
        while not stopped[0] and time.monotonic()-began < args.duration:
            for msg in sport_reader.take(8):
                if not msg.sample_info.valid_data:
                    continue
                now = time.monotonic_ns()
                sport = dict(received_monotonic_ns=now, mode=int(msg.mode), error_code=int(msg.error_code),
                    rpy=list(msg.imu_state.rpy), position=list(msg.position), velocity=list(msg.velocity),
                    yaw_speed=float(msg.yaw_speed), progress=float(msg.progress), gait_type=int(msg.gait_type))
                if time.monotonic()-last_history >= .015:
                    if history and (math.hypot(sport['position'][0]-history[-1]['x'],sport['position'][1]-history[-1]['y']) > .5
                                    or now-history[-1]['timestamp_ns'] > 1_000_000_000):
                        epoch += 1
                        history.clear()
                    history.append(dict(x=sport['position'][0],y=sport['position'][1],yaw=sport['rpy'][2],
                                        timestamp_ns=now,frame_id='direct_sport_receipt:'+boot+':'+generation+':'+str(epoch)))
                    last_history = time.monotonic()
                counter += 1
            for msg in low_reader.take(8):
                if not msg.sample_info.valid_data:
                    continue
                raw = bytes(msg.wireless_remote)
                low = dict(received_monotonic_ns=time.monotonic_ns(), tick=int(msg.tick), mode_machine=int(msg.mode_machine))
                axes = {key:struct.unpack_from('<f', raw, offset)[0] for key,offset in [('lx',4),('rx',8),('ry',12),('ly',20)]}
                axes['keys'] = int.from_bytes(raw[2:4], 'little')
                if any(not math.isfinite(v) or abs(v)>1.05 for k,v in axes.items() if k!='keys'):
                    axes = None
            if time.monotonic()-written >= .04:
                atomic(output/'telemetry.json', dict(status='running',host_boot_id=boot,timestamp_ns=time.monotonic_ns(),
                    sport=sport,low=low,remote_axes=axes,pose_history=list(history),robot_commands_sent=0))
                written=time.monotonic()
            time.sleep(.005)
    finally:
        atomic(output/'telemetry.json', dict(status='finished',host_boot_id=boot,timestamp_ns=time.monotonic_ns(),robot_commands_sent=0))
        lock.close()
        print(json.dumps(dict(status='finished',samples=counter,elapsed_s=time.monotonic()-began,robot_commands_sent=0)),flush=True)


if __name__=='__main__':
    main()
