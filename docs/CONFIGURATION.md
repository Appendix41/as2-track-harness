# Configuration reference

All configuration is plain JSON. Unknown keys are ignored (except in
`provider`, where validation is strict), so files may carry a `note` field.
Paths may be absolute, or relative to the repository root when they must stay
under the confined write roots.

## 1. Files

| File | Read by | Purpose |
| --- | --- | --- |
| `config/shadow.json` | `runner.py` | Mock planner, fully offline shadow run |
| `config/deepseek_task.json` | `runner.py`, `probe_planner.py`, `trial_control.py` | Real planner, one route plan per task |
| `config/deepseek_high.json` | `runner.py` | Same, hybrid route+anchor response and 0.65 blend |
| `config/camera_mount.json` | `runner.py` / `ground.py` | Colour intrinsics, height, pitch for image-to-ground |
| `config/execution_limits.json` | `runtime_limits.py` | Operator-selected observed-speed trip |
| `config/motion_gates.json` | `motion_gates.py` | Whether measured obstacles may brake |
| `config/depth.json` | `depth_obstacles.py` | Whether the optional depth reader runs |
| `examples/demo_run/config.json` | reference only | Settings of the recorded demo task |

## 2. Runner configuration

```json
{
  "execution_enabled": false,
  "source_url": "http://127.0.0.1:8765",
  "loop_hz": 20,
  "read_hz": 10,
  "slow_min_interval_s": 2,
  "slow_mode": "route",
  "provider": { "...": "..." },
  "calibration_path": null,
  "context_path": null,
  "fusion": { "...": "..." },
  "task": { "...": "..." }
}
```

| Key | Default | Meaning |
| --- | --- | --- |
| `execution_enabled` | must be `false` | The runner **rejects** anything else; execution lives in `scripts/fusion_trial.py` |
| `source_url` | `http://127.0.0.1:8765` | Existing VLX preview; loopback only, GET allowlist |
| `loop_hz` | 20 | Local decision loop, 1-50 |
| `read_hz` | 10 | `/control.json` poll rate, 1-20 |
| `slow_min_interval_s` | 1 | Minimum spacing between planner submissions, 0.1-60 |
| `slow_mode` | `route` | `route`, `anchor` or `hybrid` |
| `camera_mount_path` | `config/camera_mount.json` | Optional override |
| `telemetry_path` | `runs/perception_live/telemetry.json` | Optional override; must stay in the confined roots |
| `owner_state_url` | `http://127.0.0.1:8767/manual/state.json` | Read-only owner flags from the existing controller |
| `calibration_path`, `context_path` | `null` | Reserved; the sidecar must be `geometry_verified` or it is not used |

### Provider

`kind = "mock"` for offline work:

| Key | Default | Meaning |
| --- | --- | --- |
| `delay_s` | 7 | Synthetic latency, 0-60 s |
| `angle_deg` | 20 | Rotation of the synthetic route |

`kind = "openai_responses"` for a real planner:

| Key | Default | Meaning |
| --- | --- | --- |
| `model` | required | Model name sent to the endpoint |
| `base_url` | `DEEPSEEK_BASE_URL` env, else `https://api.deepseek.com/v1` | HTTPS base URL; the client posts to `<base_url>/responses` |
| `api_key_env` | `DEEPSEEK_API_KEY` | Environment variable holding the key |
| `reasoning_effort` | unset | `low`, `medium`, `high`, `xhigh` or `max` |
| `timeout_s` | 20 | HTTP timeout, `0 < t <= 60` |
| `max_output_tokens` | 2400 | Response token budget |
| `max_calls` | 1 | Per-task call budget, 1-1000 (still one plan per task) |
| `max_total_calls` | 1000 | Runaway backstop for the process, 1-10000 |
| `image_detail` | `auto` | `auto`, `low` or `high` |
| `credential_source` | unset | `local_config` reads `~/.config/openai-compatible/config.toml` and `auth.json` instead of the environment |

The endpoint must speak the **OpenAI Responses API** (`POST /responses`,
`input_image`, `json_schema` output). Any OpenAI-compatible Responses gateway
works; this client does not implement the Chat Completions API. DeepSeek's
hosted API currently exposes Chat Completions, so point `DEEPSEEK_BASE_URL` at a
Responses-compatible deployment if the default does not answer.

The key is never written to disk. A missing key is reported as
`API_key_missing`, and the key is redacted from HTTP error messages.

### Fusion

`FusionConfig` fields (legacy key `max_slow_weight` maps to `plan_weight`):

| Key | Default | Meaning |
| --- | --- | --- |
| `plan_weight` | 0.85 | Share of the plan direction applied, `(0, 0.95]` |
| `min_confidence` | 0.50 | Minimum accepted plan confidence |
| `pose_skew_s` | 0.10 | Allowed pose/camera skew when interpolating |
| `max_cross_track_m` | 1.50 | Drift beyond this blocks the task |
| `max_heading_change_rad` | pi | Per-tick correction cap; pi allows a full turn |
| `plan_lookahead_m` | 2.0 | Route direction sample distance |
| `plan_endpoint_weight` | 1.0 | 1 = aim at route end, 0 = aim at the lookahead point |
| `plan_max_age_s` | 900 | Plan age accepted for a frozen task route |
| `footprint_radius_m` | 0.45 | Used only by optional obstacle geometry |

### Task

`TaskConfig` fields:

| Key | Default | Range / meaning |
| --- | --- | --- |
| `arrival_distance_m` | 0.35 | 0.05-2.0; along-route remaining length |
| `task_timeout_s` | 120 | Task window after the plan is accepted |
| `plan_timeout_s` | 90 | Time allowed for the one plan |
| `max_range_m` | 4.0 | 1-15; image points are clamped to this range |
| `max_travel_m` | 2.0 | 0.2-10; hard travel bound per task |
| `max_plan_calls` | 1 | Must be exactly 1; the runner refuses anything else |

`--duration` on the runner caps both `plan_timeout_s` and `task_timeout_s`.

### Execution preview limits

`execution_preview_limits` is an optional object subset of `Limits` (see
`docs/SAFETY.md`) used by the runner's **preview** gate. The real gate is
constructed by `scripts/fusion_trial.py` from its own flags and the saved
observed-speed limit.

## 3. Camera mount

```json
{
  "height_m": 0.5,
  "pitch_rad": 0.0,
  "intrinsics": {"width": 640, "height": 480, "fx": 605.37, "fy": 605.13,
                 "ppx": 330.89, "ppy": 257.27}
}
```

`height_m` must be 0.05-3.0 m and `|pitch_rad| <= 1.2`. Positive pitch points the
optical axis down. `height_hint_m` is accepted as a legacy alias. Replace the
example intrinsics with your own camera calibration. This transform is a
flat-ground approximation; it is not an obstacle sensor.

## 4. Execution limits

`config/execution_limits.json` holds exactly one key,
`observed_speed_limit_mps`, in `[0.01, 3.4e38]`. It is validated and persisted by
`runtime_limits.py`; a malformed file raises instead of silently widening the
limit. The default is 0.12 m/s.

## 5. Obstacle switches

- `config/motion_gates.json`: `{"obstacle_stop_enabled": false}`. A missing or
  malformed file counts as `true` (fail-safe).
- `config/depth.json`: `{"depth_obstacles_enabled": false}`. A missing or
  malformed file counts as `true`; in that case the runner reads the optional
  `runs/perception_live/obstacles.json` sidecar if one exists.

Enabling either without a real depth source does nothing useful: no obstacles
are measured, so the gate has nothing to brake on.

## 6. Environment variables

| Variable | Used by | Meaning |
| --- | --- | --- |
| `DEEPSEEK_API_KEY` | planner client | API key, default `api_key_env` |
| `DEEPSEEK_BASE_URL` | planner client | Default Responses base URL when the config omits `base_url` |
| `VLX_STACK_ROOT` | `trial_control.py`, `controller_handoff.py` | Existing VLX deployment to start/stop and hand off from; default `~/vlx-stack` |
| `VLX_CONTROLLER_LOCK` | both | Override the controller lock path |
| `VLX_OBSERVATION_HOST` | `trial_control.py` | Host used to probe the observation page; default `127.0.0.1` |
| `VLX_ALLOW_CLIENT` | `trial_control.py` | Extra `--allow-client` passed to the stack start script |
| `VLX_SDK_PATH` | executor, handoff | Directory containing `unitree_sdk2py` when it is not pip-installed |
| `VLX_DDS_LIB` | shell wrappers | Extra `LD_LIBRARY_PATH` for Cyclone DDS |
| `PYTHON` | shell wrappers | Interpreter to use; default `python3` |

## 7. Run artifacts

A shadow run writes to `runs/shadow_<timestamp>/`:

- `metadata.json` — config snapshot and output path;
- `decisions.jsonl` — one decision per tick;
- `request_<id>.json` / `.jpg` — the exact image and record sent to the planner;
- `latest.json` — last published decision (every 20 ticks);
- `successful_fusion.json` — first accepted plan (when one is accepted);
- `summary.json` — process summary.

`runs/fusion_execution_packet.json` is the cross-process hand-off between the
runner and the executor; it is fresh for 200 ms and for one session only.
