# Runbook

Everything here runs on Linux. The repository ships no virtual environment and
no model weights; tests and offline tools need only Python and numpy. For a real
VLM run you must obtain the upstream VLX-Go weights and provide the local preview
adapter — see "Running on an AS2" in [`../README.md`](../README.md).

## 1. Install

```bash
cd as2-track-harness
python3.11 -m venv .venv
. .venv/bin/activate
pip install -e '.[vision]'        # add nothing else for offline work
```

Optional extras:

- `.[hardware]` — Cyclone DDS and the Unitree SDK2 bindings for read-only
  telemetry and real-robot execution. If `unitree_sdk2py` is not available on
  your platform, install it from a Unitree SDK2 checkout and set `VLX_SDK_PATH`.
- The shell wrappers honour `PYTHON=/path/to/python` when you do not want to
  activate the venv.

## 2. Tests

```bash
bash scripts/test.sh
```

The script sets `PYTHONDONTWRITEBYTECODE=1`, `PYTHONNOUSERSITE=1`,
`PYTHONPATH=src`, creates `runs/` and `cache/tmp/` if needed, and runs
`python -m unittest discover -s tests -v`. Expected tail:

```
Ran 170 tests in 12.0s
OK
```

CI runs the same command on Python 3.11 and 3.12.

## 3. Mock shadow run without hardware

Terminal A serves a synthetic VLX preview **and** a synthetic telemetry snapshot
on the loopback port (the telemetry file is `runs/perception_live/telemetry.json`,
the runner's default path):

```bash
bash scripts/mock_preview.sh --port 8765
```

Terminal B runs the read-only runner with the mock planner:

```bash
bash scripts/shadow.sh --config config/shadow.json --duration 15 --port 0
```

The first line is JSON with the dashboard URL (`http://127.0.0.1:<port>/`). The
mock planner answers after `delay_s` (7 s), the plan is accepted, the task moves
to `executing`, and after the duration the runner writes
`runs/shadow_<timestamp>/summary.json` with `plan_accepted: 1`,
`reason:task_ready` and `robot_commands_sent: 0`. No API call is made and no
robot command exists.

Both the preview frames and the telemetry are synthetic. Never point an
`--execute` trial at `mock_preview.py`.

Inspect the result:

```bash
bash scripts/view_report.sh runs/shadow_<timestamp> --port 0
```

The saved report page shows the captured request image, the VLX path and the
fused recommendation. Stop it with Ctrl+C.

## 4. Replay a recorded run offline

`examples/demo_run` contains one real task: the request image, the plan the model
returned, 258 recorded VLX predictions and the sanitized config.

```bash
PYTHONPATH=src python -m vlx_async.replay \
  --predictions examples/demo_run/predictions.jsonl --max-records 200
```

This writes `runs/replay_<timestamp>/report.json` with two parts:

- a deterministic delay experiment (1/3/5/7 s injected planning latency, local
  ticks before the plan arrives);
- a recorded-replay summary (valid vs stale frames, observed prediction rate).

No network, camera or robot is touched. `--max-records` bounds the replay.

To inspect the recorded plan itself:

```bash
python -c "import json;d=json.load(open('examples/demo_run/plan.json'));\
print(d['instruction'], len(d['request_waypoints_uv']), d['api_latency_s'])"
```

## 5. Observation dashboard and browser controls

```bash
DEEPSEEK_API_KEY=... bash scripts/observe.sh --port 8768
```

Open `http://127.0.0.1:8768/`. The page is read-only except for explicit
test-control POSTs, which require the per-session token rendered into the page
and a matching `Origin`. It shows the live VLX record, the task phase, the plan
frame with its image-space route, the fused paths and the saved-history browser
under `runs/`.

The dashboard listens on `127.0.0.1` only. LAN exposure requires an explicit
`--listen <ipv4> --allow-client <client-ip>`, and the page's Host check follows
the bound address. The trial controls start a shadow or execute session through
a local runner; keep the default mode until the dry-run executor has been
verified on your site.

`bash scripts/observe.sh --prompt-api-key` reads the key from a hidden prompt
instead of the environment.

## 6. Executor dry run

```bash
bash scripts/fusion_trial.sh --duration 10
```

This reads `runs/fusion_execution_packet.json`, evaluates the real command gate
and logs `runs/execution_<timestamp>/commands.jsonl` without creating a robot
sink. Inspect `summary.json` (`motion_commands_sent`, `stop_confirmed`,
`last_reason`).

## 7. Real robot prerequisites

Do this only with an operator at the robot and the physical remote in hand.
Read [SAFETY.md](SAFETY.md) first.

1. **Preview service.** A VLX-Go deployment must be running and exposing
   `/control.json` and `/state.json` on `127.0.0.1:8765` in the format consumed
   by `src/vlx_async/source.py`. This repository does not ship the policy or
   its weights.
2. **Telemetry.** `scripts/telemetry_readonly.sh` must be able to read
   `SportModeState_` and `LowState_` over DDS. Set `VLX_DDS_LIB` if the DDS
   libraries are outside the default loader path. The snapshot is written to
   `runs/perception_live/telemetry.json`.
3. **Controller lock.** The existing controller must publish
   `reports/vlx-as2.lock` (and an idle manual-web state on `127.0.0.1:8767`).
   Point `VLX_CONTROLLER_LOCK` and `VLX_STACK_ROOT` at that deployment. If the
   lock or record is missing, the handoff refuses to proceed — that is intended.
4. **Mode.** The robot must be in sport mode 3 with error code 0 and low-level
   mode 1, and must not be moving.
5. Run the dry-run executor, confirm `stop_confirmed: true` on a previous
   session, then:

   ```bash
   bash scripts/fusion_trial.sh --execute --duration 5 \
     --speed 0.05 --lateral-speed 0.03 --yaw-speed 0.10 \
     --linear-acceleration 0.10 --yaw-acceleration 0.20
   ```

The executor acquires the controller lease, confirms the idle controller has
stopped, requires a fresh gate-passing packet, moves within the limits, stops,
and only then restores the previous controller. Any missing confirmation is
latched and blocks the next trial.

## 8. Troubleshooting

| Symptom | Likely cause |
| --- | --- |
| `source_not_running`, `invalid_envelope` | Preview not started, or wrong `source_url` |
| `camera_frame_stale`, `prediction_stale` | Preview loop is slower than 0.5 s / 0.35 s |
| `history_warming_up` | VLX has fewer than 31 valid history frames; wait |
| `plan_timeout` | No planner answer inside `plan_timeout_s`; check key, endpoint, model name |
| `API_HTTP_401` | `DEEPSEEK_API_KEY` missing/invalid for the configured endpoint |
| `API_base_URL_requires_HTTPS_without_credentials` | `base_url` must be HTTPS with no userinfo/query |
| `plan_route_above_horizon` | Route points above the horizon; check `height_m`/`pitch_rad` and the image |
| `route_drift` | Cross-track error above `max_cross_track_m`; issue a new task |
| `observed_speed_limit` | Robot already faster than `config/execution_limits.json` |
| `original_controller_active_or_unknown` | Another controller owns the robot; do not fight it |
| `executor_stop_not_confirmed` | A stop RPC failed; verify the robot is still, then clear the blocker |
| `controller_identity_unavailable` | `VLX_STACK_ROOT`/lock does not match a known controller |

## 9. What is not included

- VLX-Go weights and inference code, depth models, and the preview service.
- A hardware-in-the-loop test rig. CI is CPU-only and offline.
- Any obstacle-avoidance guarantee; see [SAFETY.md](SAFETY.md#8-what-this-system-does-not-provide).
