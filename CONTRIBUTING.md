# Contributing

Thanks for taking the time to improve AS2 Track Harness. This is research software
that can command a real quadruped, so correctness and conservative defaults
matter more than feature count.

## Ground rules

- **Never** make execution, arming or robot RPC the default path. The runner and
  the executor stay dry-run/shadow unless a human explicitly asks otherwise.
- Do not add network calls to tests. The planner is exercised with the mock
  provider and synthetic observations only.
- Do not commit credentials, `.env` files, captured customer scenes or run
  artifacts (`runs/`, `cache/`).
- Keep the public contract in `src/vlx_async/contracts.py` and the safety gates
  in `src/vlx_async/execution.py` small and reviewable.

## Development setup

```bash
git clone <your-fork> as2-track-harness && cd as2-track-harness
python3.11 -m venv .venv && . .venv/bin/activate
pip install -e '.[vision]'
bash scripts/test.sh
```

`scripts/test.sh` sets `PYTHONPATH=src` and creates `runs/` and `cache/tmp/`
lazily, so it works from any working directory. Use `PYTHON=/path/to/python` to
run the suite with another interpreter.

There is no hardware in CI. If you change the execution gate, add a test with
synthetic telemetry and state the invariant you are protecting in the test
docstring.

## Style

- Python 3.11+, standard library first. Keep third-party imports lazy or in the
  optional extras when they are only needed for rendering or hardware.
- Type hints are welcome but not mandatory; dataclasses in `contracts.py` are
  validated in `__post_init__`, keep that pattern.
- No `print` debugging in library code; use the JSONL records the runner already
  writes under `runs/`.
- Keep comments about *why*, not *what*.

## Pull requests

1. Open an issue for non-trivial behaviour changes.
2. One logical change per PR.
3. Include the exact commands you ran and the test tail in the PR body.
4. Never paste API keys, tokens or private network addresses into a PR, issue or
   screenshot.

By contributing you agree that your contribution is licensed under
Apache-2.0.
