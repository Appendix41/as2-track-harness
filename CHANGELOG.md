# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and release tags use
[Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.1.0] — 2026-11-13

First public release.

### Added

- Asynchronous task lifecycle: one frozen planner call per task, odometry-tracked
  progress, arrival and travel bounds (`task_plan.py`, `fusion.py`).
- Fast/slow fusion: VLX-Go 8×3 body-frame waypoints rotated toward the planned
  heading, with a bounded per-tick heading rate.
- Safety-gated executor: freshness, mode, remote-takeover, tilt, speed and
  control-lease checks; acceleration-limited commands; command watchdog;
  `StopMove` with confirmed-stop semantics (`execution.py`, `motion_gates.py`).
- Read-only observation dashboard, JSONL decision logs, offline report viewer and
  recorded-run replay (`observation.py`, `report_viewer.py`, `replay.py`).
- Offline mock preview and shadow loop, so the whole decision path runs without
  a camera, an API key or a robot.
- 170 unit tests, CI on Python 3.11 and 3.12, sanitized example configurations,
  one recorded demo task, and English/Chinese documentation.

### Changed

- Repository extracted from the lab deployment: hard-coded hosts, machine paths,
  credential plumbing and internal gateway names removed. The API key is read
  from `DEEPSEEK_API_KEY` (or the variable named by `provider.api_key_env`) and
  never written to config, snapshots or logs.
- Default observed-speed trip set to 0.12 m/s.

### Known limitations

- No continuous replanning, no obstacle avoidance by default, no metric ground
  truth, Chinese-only dashboard labels, and no released model weights.

## History before the public release

- **2026-11-09** — Architecture change: one planner call per task, executed before
  departure; the depth/obstacle chain was removed from the default path.
- **2026-10-25** — First end-to-end step on the real AS2: 17 `Move` calls, 0.81 s,
  4.27 cm of onboard odometry, stopped by the observed-speed gate.
