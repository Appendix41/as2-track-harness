# Security policy

## Reporting a vulnerability

Please do not open a public issue for a security problem. Report it privately to
the maintainers through your repository host's private security advisory
feature, including:

- affected version or commit,
- a minimal reproduction,
- the impact you believe it has,
- whether any robot, credentials or private network were involved.

We aim to acknowledge reports within 7 days. There is no bug bounty.

## Scope and safety model

This repository controls a physical robot only in the explicit
`--execute` path of `scripts/fusion_trial.py`. The default runner publishes a
read-only decision packet and never opens a motion client. Read
[docs/SAFETY.md](docs/SAFETY.md) before enabling anything else.

In scope for reports:

- bypasses of the command gate, watchdog, speed/acceleration limits or control
  lease in `src/vlx_async/execution.py`;
- ways to make the observation dashboard start motion, proxy requests or leak
  the control token;
- credential disclosure in logs, packets, dashboards or saved runs;
- path traversal or writes outside `runs/` and `cache/`.

Out of scope:

- the robot's own firmware, the DDS transport, the vendor SDK or the model
  providers;
- consequences of deliberately disabling the documented limits;
- physical safety guarantees beyond the documented gates. This is research
  software with no obstacle sensing by default; keep an operator with the RC
  takeover in reach.

## Secrets

API keys are read from the environment variable named by the selected config
(`api_key_env`, default `DEEPSEEK_API_KEY`). They are never written to disk, never
logged and redacted from HTTP error messages. The observation dashboard reports
only whether a key is present. If you find a path that prints a key, treat it as
a vulnerability.
