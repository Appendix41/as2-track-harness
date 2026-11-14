## What changed

<!-- One paragraph. Link the issue if there is one. -->

## Why

<!-- The problem this solves. For gate changes, name the invariant you protect. -->

## Safety impact

- [ ] No change to command bounds, telemetry gates, watchdog or control lease
- [ ] Execution remains disabled by default
- [ ] I did not add robot RPC or network calls to tests

## How it was verified

<!-- Exact commands, e.g. PYTHON=python3.11 bash scripts/test.sh, and the tail. -->

```
Ran 170 tests ...
OK
```

## Checklist

- [ ] `bash scripts/test.sh` passes
- [ ] No credentials, private IPs, host paths or run artifacts are included
- [ ] Relevant docs (`docs/ARCHITECTURE.md`, `docs/SAFETY.md`, `docs/CONFIGURATION.md`, `docs/RUNBOOK.md`) updated
- [ ] New config keys are documented and have conservative defaults
