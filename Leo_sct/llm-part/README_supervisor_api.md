## GPT-based Supervisor Design Helper

This folder contains two entry points:
- `request_supervisor_updates_manual.py` asks the GPT API for DES supervisor
  transitions when you already know the state/event lists (no C++ parsing).
- `run_pipeline.py` runs the full DES -> Nadzoru -> SCT pipeline, optionally
  calling the GPT helper first and then emitting YAML for the ROS node.

### Manual GPT helper

Quick start:

```
cd Leo_sct/des
python3 request_supervisor_updates_manual.py --profile find_marker
```

Key flags:

- `--profile NAME` – use a predefined goal/event set from `task_profiles.json`.
- `--states ...` – override the state list (defaults to clear/obs_*).
- `--controllable-events ...`, `--uncontrollable-events ...` – override event lists.
- `--existing-transition "state event next"` – include known transitions
  (repeatable), or provide `--transitions-file`.
- `--scenario "..."` – describe the operating context.
- `--constraint "..."` – add guidance bullets (repeatable).
- `--no-default-constraints` – drop the baked-in obstacle handling guidance.
- `--no-current` – ignore existing transitions even if provided.
- `--dry-run` – print the prompt without calling the API.
- `--save path.json` – store the returned JSON.
- `--model`, `--temperature`, `--timeout`, `--retries` – API tuning.
- `--list-profiles` – list available profiles and exit.

Example (dry-run prompt):

```
python3 request_supervisor_updates_manual.py \
  --profile find_marker \
  --scenario "Robot is in a crowded environment" \
  --constraint "Prefer slowing down or waiting instead of backing up" \
  --constraint "Avoid fast movements near people" \
  --dry-run
```

The script expects an API key via `OPENAI_API_KEY` or `../api_key.txt`. The
response JSON contains `transitions` lines (also echoed verbatim at the end)
that can be pasted into a DES file or fed into the pipeline.

### End-to-end pipeline

`run_pipeline.py` runs:
1) (Optional) GPT call to produce a JSON supervisor.
2) JSON -> Nadzoru XML.
3) Headless Nadzoru script to compute Sloc*.xml.
4) SCT YAML export for ROS.

Quick runs:

```
python3 run_pipeline.py --task explore --run-llm
python3 run_pipeline.py --task find_marker --skip-llm
python3 run_pipeline.py --task wall_follow --run-llm
```

Notes:
- Output YAMLs are written under `../swarm_basics/config` and `../leo_real/config`.
- LLM settings can be overridden via `--llm-profile`, `--llm-goal`,
  `--llm-constraint`, and `--llm-no-default-constraints`.

### Dependencies

- `requests` for the GPT API helper.
- `PyYAML` for the SCT YAML output in `run_pipeline.py`.
