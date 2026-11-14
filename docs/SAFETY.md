# Safety model and hard limits

This is research software for a dynamic quadruped. It is not a certified safety
system and it does not replace a human operator, the robot's own protective
firmware, or the physical remote-control takeover. Read this document before
running anything against real hardware.

## 1. Default posture: no arm, no execute path

- `runner.py` refuses any configuration whose `execution_enabled` is not
  `false`. It publishes a read-only decision packet and never opens a motion
  client.
- `scripts/fusion_trial.py` is a **dry run** unless `--execute` is passed. In
  dry-run mode it reads packets, evaluates the gate and logs commands without
  creating a robot sink.
- Even with `--execute`, the AS2 sink is created only after the controller lease
  is acquired, the idle controller is stopped with a confirmed stop, and a fresh
  packet passes every preflight check.
- There is no "arm", "takeoff", gait or posture endpoint anywhere in this
  repository.

## 2. Hard limits

Defaults live in `src/vlx_async/execution.py` (`Limits`,
`GuardedActuator`) and can only be tightened by configuration.

| Limit | Default | Enforced by |
| --- | --- | --- |
| Forward speed | 0.05 m/s | `Limits.max_forward_mps` |
| Lateral speed | 0.03 m/s | `Limits.max_lateral_mps` |
| Combined linear speed | 0.05 m/s | `Limits.max_linear_mps` |
| Yaw rate | 0.10 rad/s | `Limits.max_yaw_rps` |
| Linear acceleration | 0.10 m/s^2 | `Limits.max_linear_accel_mps2` |
| Yaw acceleration | 0.20 rad/s^2 | `Limits.max_yaw_accel_rps2` |
| Heading gain | 2.0 | `Limits.heading_gain` |
| Telemetry age | 0.25 s | `Limits.max_telemetry_age_s` |
| Prediction age | 0.35 s | `Limits.max_prediction_age_s` |
| Control gap | 0.20 s | `Limits.max_control_gap_s` |
| Camera frame age | 0.50 s | `CommandGate` |
| Observed speed trip | 0.12 m/s | `config/execution_limits.json` |
| Body tilt (roll/pitch) | 0.35 rad | `CommandGate` |
| Remote axes / keys | 0.05 / 0 | `CommandGate` |
| Packet freshness | 200 ms | `packet_observation` |
| Executor watchdog | 0.20 s | `GuardedActuator` |
| Stop attempts | 3 | `GuardedActuator._stop` |
| Plan calls per task | 1 | `TaskConfig.max_plan_calls` |

`Limits.__post_init__` rejects non-finite or non-positive values, a control gap
above 0.20 s, a telemetry age above 0.25 s, a prediction age above 0.35 s and a
heading gain outside `[0.5, 10]`.

`config/execution_limits.json` stores the operator-selected *observed* speed
trip. It is a telemetry veto, not a motor command: if the robot is already
moving faster than this value, the gate holds. Keep it low (0.12 m/s by default)
until the site is characterised.

The web trial form (`trial_control.py`) accepts speed/acceleration values up to
large bounds for research. It does not know your site. If you use that form,
set small values explicitly; the defaults of `fusion_trial.py` and `Limits` are
the safe reference.

## 3. Execution gates, in order

`CommandGate.evaluate` can only return `allowed=true` when *all* of the
following hold:

1. the task key has not changed since the previous tick;
2. the monotonic control gap is in `(0, 0.20] s`;
3. the camera frame is at most 0.50 s old and the prediction at most 0.35 s old;
4. decision and observation agree on camera index, prediction index, stream,
   boot and instruction revision;
5. the frozen task plan is ready and not rejected (`task_execution_ready`);
6. telemetry is fresh and the robot reports sport mode 3, error code 0 and
   low-level mode 1;
7. roll/pitch are within 0.35 rad, velocity is finite and the observed speed is
   below the configured trip;
8. no other controller owns the robot, or this process holds the same lock;
9. the physical remote is idle: no keys and all sticks within 0.05;
10. the fused path contains eight valid body-frame points.

If a measured obstacle source is enabled, a measured obstacle in the corridor or
on the next path segment brakes the robot (`measured_obstacle_in_corridor`).
**It is disabled by default**: `config/motion_gates.json` and
`config/depth.json` both say `false`, and deleting either file falls back to
"enabled" (fail-safe) rather than silently removing the check.

## 4. Command shaping and stop semantics

- The requested velocity comes from the second recommended waypoint divided by
  `reference_seconds` (0.10 s), then clamped. The yaw request is proportional to
  the heading error (`heading_gain * |yaw|`), not `angle / 0.1 s`; a 44 degree
  error must not become 6 rad/s.
- The command is rate-limited by the configured linear/yaw acceleration over the
  measured tick interval. The previous command is kept, so a hold brakes within
  the acceleration window instead of restarting from zero.
- `GuardedActuator` independently re-checks the same bounds; a value outside the
  configured envelope is a latched `input_gate_rejected` fault.
- The watchdog trips at 0.20 s without a submitted command. The motion duration
  is also a hard stop. `close()` always attempts `StopMove`, up to three times,
  and records every return code. A stop timeout is never treated as a confirmed
  stop.
- After motion, the executor reports `stop_confirmed` and only then restores the
  previous controller. If the stop is unconfirmed, the trial manager latches
  `executor_stop_not_confirmed` and refuses to start another trial.

## 5. Remote takeover

The gate treats the physical remote as the highest-priority owner. Any non-zero
key, any stick axis beyond 0.05, missing remote samples or unknown owner state
produces `remote_input_or_unknown_state` / `robot_not_in_ready_mode` and the
command is not sent. When a perception clock is stale the actuator may brake
inside the acceleration window (keeping the gait alive), but robot and remote
vetoes are latched stops, not holds.

## 6. Control ownership

The controller lock is an existing `flock` file. `ControllerLease` opens it
read-only and takes an exclusive non-blocking lock; it never writes to or
deletes the file. If the lock is busy, `ControllerHandoff` may stop a recognised
idle controller only after:

- reading a controller record whose PID and process start ticks still match,
- verifying the recorded script is the expected manual-web controller,
- confirming the controller reports disabled, not busy, stop acknowledged and
  VLX inactive,
- collecting stationary telemetry evidence.

Anything it does not recognise is refused (`controller_identity_unavailable`,
`original_controller_not_idle`, `unexpected_controller_process`). The optional
stack integration is disabled unless `VLX_STACK_ROOT` points at an existing VLX
deployment.

## 7. Task-level bounds

- One planner request per task, before departure. No replanning while moving.
- The route is frozen in the body frame of the request image and is not
  re-validated by a model. It remains valid for the task window
  (`task_timeout_s`, default 120 s) and is bounded by `max_travel_m` (2.0 m).
- Arrival is `remaining_route_m <= arrival_distance_m` (0.35 m) measured along
  the route by odometry. A cross-track error above 1.5 m blocks the task.
- The runner has a hard wall-clock budget independent of task state, so a
  planner answer that never arrives ends as `plan_timeout`.
- The browser trial stops on heartbeat loss (2 s) and reports the reason.

## 8. What this system does NOT provide

- **No obstacle sensing by default.** The depth chain is removed from the
  default build; only the optional `depth_obstacles.py` can restore measured
  obstacle braking, and it is disabled. The robot will walk into an unmapped
  obstacle that the operator does not stop.
- **No collision avoidance in the planner.** The image route is a direction aid,
  not a collision-free path. The monocular projection cannot measure distance
  reliably; it is biased by camera height and pitch.
- **No dynamic-object tracking.** People and moving objects are not tracked; a
  task is frozen.
- **No formal guarantee.** The gates are ordinary Python checks running on the
  same machine as the planner client. A process crash, a kernel stall, a network
  partition, a DDS loss or a robot fault can defeat them.
- **No fleet or remote supervision.** Only one controller may own the robot, and
  a human must be able to reach the physical stop.

## 9. Pre-execution checklist

1. Read `docs/ARCHITECTURE.md` and this page.
2. Confirm `config/execution_limits.json` holds a speed you have tested.
3. Confirm `config/motion_gates.json` and `config/depth.json` match the sensors
   that are actually installed; do not disable a protection you rely on.
4. Verify the mount calibration (`height_m`, `pitch_rad`, intrinsics) on site.
5. Have the RC remote in hand, with the takeover tested.
6. Run the dry-run executor first and inspect `runs/execution_*/commands.jsonl`.
7. Start with `--duration` in the seconds range and a clear floor.
8. Keep a second person at the emergency stop; never run unattended.
