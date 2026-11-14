# Example: one recorded task

One real task from a lab session, reduced to the files needed to inspect and
replay it offline. No credentials, host names or machine paths are included.

| File | What it is |
| --- | --- |
| `request.jpg` | The exact RGB frame the planner saw for this task |
| `plan.json` | The returned image-space route, the fused body-frame recommendation and the task/timing record |
| `predictions.jsonl` | 258 recorded VLX-Go predictions (8x3 waypoints each) from the same session |
| `config.json` | Sanitized configuration of the run (planner, fusion, task windows) |

Instruction: `GO to the farthest black seat`. The plan was accepted with ten
image-space waypoints, a 4.08 m projected route, and a 22.6 s API latency.

## Replay

From the repository root:

```bash
PYTHONPATH=src python -m vlx_async.replay \
  --predictions examples/demo_run/predictions.jsonl --max-records 200
```

The command writes `runs/replay_<timestamp>/report.json` with:

- a deterministic synthetic delay experiment (what the local loop does while the
  planner is still thinking);
- a recorded-replay summary of the 258 VLX records — valid frames vs. warm-up
  and stale-frame rejections, and the observed prediction rate.

It does not call a model, a camera or a robot.

To inspect the recorded plan without running anything:

```bash
python -c "import json;d=json.load(open('examples/demo_run/plan.json'));\
print(d['instruction']);print(len(d['request_waypoints_uv']),'image points')"
```

## What this example does not show

- It is not a navigation success: the robot was observed in a shadow run, no
  motion command was sent (`robot_commands_sent: 0`).
- The image-space route is not metric ground truth. It was projected with the
  declared camera mount and only used as a direction reference.
- The 258 recorded predictions are a bounded slice; they are not the full
  session and `--max-records` bounds the replay.

The image was captured by the authors in their lab and is included solely to
demonstrate the data flow.
