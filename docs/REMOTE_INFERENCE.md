# Running the planner away from the robot

The harness reads two HTTP endpoints from a local preview service. Where the
inference runs is a transport question: the robot host, a second GPU box on the
LAN, or a rented server. The endpoints still have to be served locally, in the
robot host's clock and boot domain. This page covers what can move and what
cannot.

## 1. What the harness actually requires

From [`src/vlx_async/source.py`](../src/vlx_async/source.py):

- `source_url` is `http://127.0.0.1:<port>` (scheme `http`, path `/`). A non-loopback host is
  rejected with `source_must_be_local_preview`.
- `GET /control.json` and `GET /state.json` only; redirects and proxies are disabled; payload is
  capped at 2 MB.
- The payload must report the **robot host's** `host_boot_id` (the runner compares it with
  `/proc/sys/kernel/random/boot_id`).
- `camera_received_monotonic_ns` and `prediction_available_monotonic_ns` must be in the **robot
  host's monotonic clock**: the freshness gates compute `now_ns - stamp` with the local clock
  (camera ≤ 0.50 s, prediction ≤ 0.35 s).
- `valid_history_frames` must be 31 — the VLX-Go model needs a warmed 31-frame history.

So "transport" is free. "Who serves the endpoints, and in whose clock" is not.

## 2. Three deployment topologies

### A. All local — what we ran

```
[camera] -> preview adapter -> VLX-Go (same AGX) -> /control.json -> harness -> AS2
```

One host, one clock, one boot id. This is the path all the results in the README come from, and
what `scripts/mock_preview.py` fakes for the offline demo.

### B. Split: adapter local, inference remote — recommended for a remote GPU

```
[camera] -> local adapter on AGX -----------------------------+
                 |   JPEG frame (one per tick, keep history remote)  |
                 v                                                  |
          remote GPU: VLX-Go inference, per-session 31-frame history |
                 |   8x3 waypoints                                  |
                 v                                                  |
          local adapter re-stamps prediction_available_monotonic_ns |
                 |                                                  |
                 +--> /control.json (loopback, local boot id) -> harness -> AS2
```

The harness is unchanged. The adapter stays on the robot host, so the boot id and the monotonic
stamps are local; it only makes an **outbound** call to the inference server.

Two numbers decide whether this works for you:

- **Latency budget.** The prediction-age gate is 0.35 s. `inference + RTT + jitter` must fit
  inside it at your loop rate (the camera-age gate is 0.50 s). On a LAN this is easy; over WAN it
  usually is not, so either lower the rate, raise `max_prediction_age_s` on purpose, or keep the
  model on the LAN. Do not widen it silently: the gate is the only thing that notices a stale
  plan.
- **Bandwidth and history placement.** At 640×480 JPEG, one frame is ~30–50 KB; 10 Hz is
  ~0.3–0.5 MB/s (~3–4 Mbit/s), fine on LAN or 5G. Do **not** ship the 31-frame history with every
  request (~1.2 MB × 10/s); keep a session/history on the remote side and stream frames into it.
  VLX-Go's episode API already works this way.

### C. Fully remote preview service

You can run the whole preview (camera receiver + model + endpoints) on another box and tunnel it
back, but the harness rejects that today for three independent reasons:

1. the URL must be loopback;
2. `host_boot_id` must equal the robot host's boot id;
3. timestamps must be in the robot host's monotonic clock.

Supporting C means an explicit, reviewed change: allow a configured remote URL behind an opt-in
flag, accept a configured expected `boot_id` instead of the local one, and re-stamp on receipt (or
carry a measured clock offset). It also moves the trust boundary; see below. Until then the only
way to run C is a local proxy that re-stamps the payload, which is B.

## 3. Trust boundary

The preview endpoints are unauthenticated, GET-only and loopback-bound. The harness treats
waypoints as trusted input: the gates bound speed and freshness, but a lying or compromised source
can still steer the robot within those bounds.

- Prefer **B**: the robot host only makes outbound requests; nothing new listens on the network.
- If you must do **C**, put it behind SSH/VPN, TLS and an authenticating proxy, and treat the
  remote side as part of the safety envelope — it can feed the gate.
- Camera frames leaving the robot are a privacy decision, exactly like sending a frame to the
  DeepSeek endpoint.

## 4. Checklist for a remote-GPU adapter

- [ ] Serve `/control.json` and `/state.json` on `127.0.0.1`; use the same schema as
      [`scripts/mock_preview.py`](../scripts/mock_preview.py) (the reference implementation).
- [ ] Report the robot host's `host_boot_id` and local monotonic timestamps.
- [ ] Keep the 31-frame history where the frames land; stream frames incrementally.
- [ ] Budget `inference + RTT` inside `max_prediction_age_s`; fail closed (hold) on timeout.
- [ ] Never give the adapter a robot command path: it is a sensor/planner service, not a controller.
