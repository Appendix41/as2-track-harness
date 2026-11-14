# Documentation

Four documents carry the design; everything else is in the code and its tests.

| Document | Read it when you want to… |
| --- | --- |
| [ARCHITECTURE.md](ARCHITECTURE.md) | understand the data flow, the task lifecycle, the fusion math and the route/arrival contract |
| [SAFETY.md](SAFETY.md) | know exactly which gates exist, their hard limits, the stop semantics, and what the system does **not** provide |
| [RUNBOOK.md](RUNBOOK.md) | install, run the offline demo, open the dashboard, prepare a real-robot session, and decode common error reasons |
| [CONFIGURATION.md](CONFIGURATION.md) | look up a configuration field, its default and its validation range |
| [REMOTE_INFERENCE.md](REMOTE_INFERENCE.md) | move the planner/inference off the robot (remote GPU, LAN or cloud) without breaking the HTTP contract, clock domain or safety gates |

Supporting material:

- [`../examples/README.md`](../examples/README.md) — the recorded demo task and how to replay it.
- [`../CHANGELOG.md`](../CHANGELOG.md) — release notes and the two pre-release architecture milestones.
- [`../README.md`](../README.md) — project overview, results, safety summary and license.
