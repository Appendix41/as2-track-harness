# Architecture

This document describes the asynchronous navigation harness in
`src/vlx_async`. It is a research prototype: one fast vision-language-action
(VLA) policy supplies body-frame waypoints, one slow vision-language model
(VLM) is asked exactly once per task for a ground route in the image, and an
independent execution gate decides whether a bounded velocity command may be
sent.

## 1. Data flow

```
        existing VLX preview service (loopback, GET only)
        /control.json                /state.json
             |                            |
             v                            v
       FastReader (read_hz)          request_factory (one image per task)
             |                            |
             v                            v
       FastGate + Observation ----> SlowWorker ---> Responses API (one call/task)
             |                            |
             |                            v
             |                     project image route to body frame
             |                     (config/camera_mount.json intrinsics)
             v                            |
         Fusion.decide() <--- frozen RoutePlan
             |
             v
   runs/fusion_execution_packet.json  (status='shadow', written from a fresh tick)
             |
             +--> read-only dashboard (/state.json)
             |
             +--> scripts/fusion_trial.py: CommandGate -> GuardedActuator
                  (dry-run by default; --execute requires the controller lease)
```

The fast loop runs at `loop_hz` (default 20 Hz) and never blocks on the model.
The slow worker has at most one in-flight request and one undelivered result.
A task's one request is submitted only while the runner is still planning and
the task has not been closed.

## 2. Task lifecycle

Task identity is the VLX `task_key = (boot_id, stream_id, instruction_revision,
instruction)`. Only a new task key opens a new planning phase.

```
idle --new task_key--> planning --plan accepted--> executing --arrival / timeout--> completed
                          |                                                           |
                          +-- plan rejected / plan timeout --> blocked <--------------+
```

- **planning**: the robot is assumed stationary. Exactly one planner call is
  permitted (`TaskConfig.max_plan_calls` is forced to 1; `Fusion.accept`
  refuses a second plan for the same task).
- **ready / executing**: no planner request is possible. The accepted route is
  frozen in the body frame captured with the request image and re-expressed
  through the current pose on every tick.
- **completed**: the remaining route length along the frozen route is at most
  `arrival_distance_m` (default 0.35 m), or `max_travel_m` was exceeded. The
  task then holds until VLX reports a new task key.
- **blocked**: plan rejected (`plan_rejected:<reason>`), plan timeout, task
  timeout, or route drift beyond `max_cross_track_m`.

`TaskLoop` in `runner.py` owns this state machine. It also enforces a hard
wall-clock budget (`runner_budget`) so a missing plan or a dead preview cannot
keep the process alive forever.

## 3. Coordinate contract

- **VLX waypoints** are `8 x [x, y, yaw]` in the current body/motion frame:
  `x` forward (m), `y` left (m), `yaw` counter-clockwise (rad). They are
  short-horizon shape hints, not a metric plan.
- **Planner output** is 2-16 ordered normalized image points `(u, v)` with the
  origin at the top-left, expressed in the exact image sent with the request
  (`coordinate_frame = image_uv`). The model is never trusted to claim metric
  accuracy, and it cannot invent a tracking id.
- **Image to body frame** uses the declared mount in
  `config/camera_mount.json`: colour intrinsics, camera height and pitch, and a
  flat-ground assumption. Points at or above the horizon are dropped, points
  beyond `max_range_m` are clamped, and a route with fewer than two usable
  points is rejected. This projection is a direction aid; it is not a depth
  sensor.
- **Pose** comes from read-only telemetry (`RobotState`). `Pose.frame_id` must
  match between the planning instant and the current tick; an odometry reset is
  rejected instead of silently reusing a stale route. The pose used by a plan
  is the one captured with the request image, never the pose of the tick that
  consumes the answer.
- **Time** is monotonic on the same boot. Camera and prediction stamps are
  carried through the packet; the task key includes `boot_id` and `stream_id`,
  so a reboot or a new VLX stream starts a new task.

## 4. Fusion math

Given an accepted `RoutePlan` in the body frame at plan time and the current
pose:

1. **Align** the plan into the current body frame (`geometry.align`) and
   **crop** the passed portion by finding the closest point on the polyline to
   the origin (`task_plan.remaining_length`). Progress is measured in metres
   along the route the robot actually walks, which is self-consistent with the
   projection error.
2. **Complete** when the remaining length is `<= arrival_distance_m`, or when
   travelled distance exceeds `max_travel_m`; a large cross-track offset
   (`> max_cross_track_m`) blocks the task as `route_drift`.
3. **Heading blend.** The VLX heading is the direction of the first real VLX
   waypoint. The plan heading is a weighted mix of the route direction at
   `plan_lookahead_m` and the direction of the route endpoint
   (`plan_endpoint_weight`, default 1.0). The applied correction is

   ```
   delta  = wrap(plan_heading - vlx_heading)
   target = clamp(plan_weight * delta, +/- max_heading_change_rad)
   blend  = rate-limit(target, 6 deg per tick, keeping state)
   recommended = rotate(vlx_waypoints, blend)
   ```

   `plan_weight` (default 0.8) is literally "how much of the plan's direction
   is followed". The lookahead/endpoint mix exists because monocular
   perspective flattens a visible curve into a few centimetres near the robot.
4. **Candidates** are kept for diagnostics: eleven rotations of the VLX path
   scored by deviation from VLX, mismatch with the plan and endpoint error. With
   an accepted plan the recommended path is the weighted rotation above; without
   a plan the decision is `hold`.
5. **Rotate in place.** If the required correction exceeds 90 degrees, the
   decision sets `rotate_in_place`; the executor then commands pure yaw until
   the heading error is inside the threshold, instead of strafing.

The decision packet exposes `recommended` (8 body-frame points), `selection`
(`plan_aligned` | `vlx_only` | `hold`), `reason`, `task`, `task_execution_ready`,
`plan_status` and the heading diagnostics (`plan_heading_rad`,
`vlx_heading_rad`, `heading_gap_rad`, `blend_limited`, ...).

## 5. Execution gates

`task_execution_ready == true` and `reason == 'task_ready'` are necessary but
not sufficient. `CommandGate.evaluate` checks, in order:

1. same task as the previous tick (`execution_task_changed`);
2. control gap `<= max_control_gap_s` (0.20 s) — the watchdog;
3. camera frame age `<= 0.50 s` and prediction age `<= max_prediction_age_s`
   (0.35 s);
4. decision/prediction and task identity match the observation;
5. the frozen plan is ready (`task_execution_ready`), otherwise the reason is
   passed through as a hold;
6. robot telemetry freshness `<= max_telemetry_age_s` (0.25 s), sport `mode == 3`,
   `error_code == 0`, low-state `mode_machine == 1`;
7. body tilt (roll/pitch) `<= 0.35 rad`, finite velocity, observed speed
   `<= max_observed_speed_mps`;
8. no other controller owns the robot: original manual/VLX flags must be false,
   or the same controller lease must be held;
9. the physical remote is idle (`remote_axes` keys == 0 and all axes `<= 0.05`);
10. the fused path is valid (8 points).

When an obstacle-measurement source exists (`config/motion_gates.json` and
`config/depth.json` both enabled), the gate can also brake on a measured
obstacle in the corridor (`measured_obstacle_in_corridor`). Both are disabled by
default in this repository.

A passed gate yields `(vx, vy, wz)` derived from the second recommended point
(`distance / reference_seconds`), capped by `max_forward_mps`,
`max_lateral_mps`, `max_linear_mps` and `max_yaw_rps`. The yaw request is
proportional (`heading_gain * |yaw|`), not `angle / 0.1 s`. The command is then
ramped by `max_linear_accel_mps2` and `max_yaw_accel_rps2` over the measured
tick interval. `GuardedActuator` re-validates the same bounds, keeps a 0.20 s
watchdog and calls `StopMove` with bounded retries on any fault.

## 6. Isolation, credentials and evidence

- The preview is read over HTTP **loopback only**, with a GET allowlist
  (`/control.json`, `/state.json`, `/camera.json`), no redirects and no proxy.
- Writes are confined to `runs/`, `cache/`, `manifests/`, `docs/` and `config/`
  under the repository root (`storage.confined`). Every run directory contains
  `metadata.json`, `decisions.jsonl`, `request_<id>.json/.jpg`, `latest.json`,
  `summary.json` and, when a plan is accepted, `successful_fusion.json`.
- The planner key is read from `api_key_env` (default `DEEPSEEK_API_KEY`),
  never written to disk or logs, and redacted from HTTP error text. Requests use
  HTTPS and reject URL credentials, query strings and redirects. There are no
  automatic retries, so a failed call cannot silently double the cost.
- The executor publishes a packet only when the tick has valid evidence; the
  packet is valid for 200 ms, for the same boot and session, and the executor
  never refreshes old evidence.
- The observation dashboard (`observation.py`, `observation.html`) is read-only
  except for explicit, token- and origin-checked test-control POSTs. It never
  proxies arbitrary URLs and exposes no raw credentials.

## 7. Module map

| Module | Responsibility |
| --- | --- |
| `contracts.py` | Frozen dataclasses, validation, `FusionConfig` / `TaskConfig` |
| `source.py` | Loopback GET adapter, freshness gate, pose buffer |
| `providers.py` | Mock/Responses providers, payloads, schemas, decoding, worker |
| `ground.py` | Monocular pixel-to-ground projection |
| `task_plan.py` | Frozen route, alignment, progress, resampling, heading |
| `fusion.py` | Plan acceptance and per-tick heading blend / candidates |
| `geometry.py` | SE(2) alignment, rotation, path distance, collision helpers |
| `runner.py` | Task lifecycle, request factory, decision packet, dashboard |
| `execution.py` | Command gate, guarded actuator, controller lease, AS2 sink |
| `trial_control.py` | Session-bound trial/queue supervisor (optional stack) |
| `controller_handoff.py` | Idle-controller handoff, stop confirmation, restore |
| `robot_state.py` | Read-only telemetry snapshot and pose history |
| `observation.py` | Independent observation dashboard and test controls |
| `depth_obstacles.py` | Optional depth obstacle reader (disabled by default) |
| `replay.py` | Deterministic delay experiment and recorded VLX replay |
| `dashboard.py` | Read-only run dashboard used by the shadow runner |

## 8. Known limits

- No obstacle sensing is active by default: the robot moves on the plan, the
  remote takeover and the watchdog only. See [SAFETY.md](SAFETY.md).
- The monocular route carries a scale bias from the assumed height/pitch. It is
  used for direction and odometry progress, not for standoff distance.
- There is no visual-servoing loop; the plan is frozen. If the scene changes,
  the operator must stop and issue a new task.
- The optional web trial supervisor and controller handoff expect an existing
  VLX preview deployment and are disabled unless `VLX_STACK_ROOT` points at one.
