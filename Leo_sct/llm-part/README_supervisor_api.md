## DES Pipeline (`run_pipeline.py`)

This directory is centered on `run_pipeline.py`, which builds a supervisor YAML
for ROS from task profiles and automata models.

## What `run_pipeline.py` does

For a selected `--task`, the pipeline executes:

1. Optional LLM generation of a supervisor JSON (`*_nadzoru.json`).
2. Validation of JSON transitions (event alphabet, determinism, uncontrollable
   totality, controllable presence, loop guards).
3. JSON to Nadzoru XML conversion via `json_to_nadzoru_xml.py` (`E*.xml`).
4. Headless Nadzoru script execution (`Sync`, `SupC`) to produce `Sloc*.xml`.
5. `Sloc*.xml` to SCT YAML export for ROS runtime.

It does not build or run C++ nodes directly.

## Files used by the pipeline

- `run_pipeline.py`: main entry point.
- `task_profiles.json`: per-task goal and event lists.
- `json_to_nadzoru_xml.py`: JSON to XML converter used by the pipeline.
- `hardcoded_find_obj/G1.xml`, `hardcoded_find_obj/G2.xml` (and task-specific
  variants): source plant/spec automata.
- `full_pipeline/`: generated intermediate artifacts (`E*.xml`, `Sloc*.xml`,
  `script.txt`, `*_nadzoru.json`).

Related but separate:

- `convert_to_nadzoru_xml.py` converts a `robot_navigation.cpp`-style source to
  XML, but it is not called by `des/run_pipeline.py`.

## Task to automata mapping

- `find_marker` -> `hardcoded_find_obj`
- `explore` -> `hardcoded_coverage`
- `wall_follow` -> `hardcoded_wall_follow`
- `zigzag` -> `hardcoded_zigzag`

## Usage

From `Leo_sct/des`:

```bash
python3 run_pipeline.py --task explore --run-llm
python3 run_pipeline.py --task find_marker --skip-llm
python3 run_pipeline.py --task wall_follow --run-llm
python3 run_pipeline.py --task zigzag --run-llm
```

Required flag:

- `--task {find_marker|explore|wall_follow|zigzag}`

LLM control flags:

- `--run-llm`: generate a new JSON from the built-in LLM step.
- `--skip-llm`: skip LLM and reuse latest JSON for the selected task.

Note: `RUN_LLM` defaults to `True` in code, so `--skip-llm` is the explicit way
to bypass LLM generation.

## Inputs and outputs

Input key source:

- `OPENAI_API_KEY` environment variable, or
- `../api_key.txt`

Generated files:

- `des/full_pipeline/<task>_<N>_nadzoru.json`
- `des/full_pipeline/E<N>.xml`
- `des/full_pipeline/Sloc<N>.xml`
- `des/full_pipeline/script.txt`

YAML outputs written to:

- `swarm_basics/config/<task>_sup_gpt_<N>.yaml`
- `swarm_basics/config/sup_gpt.yaml` (latest copy)
- `leo_real/config/<task>_sup_gpt_<N>.yaml`
- `leo_real/config/sup_gpt.yaml` (latest copy)

## Important behavior

- Output index `<N>` auto-increments based on existing `E*.xml` and `Sloc*.xml`.
- `G1`/`G2` are loaded explicitly from the selected task folder to avoid
  collisions with unrelated XMLs.
- If LLM output is invalid, pipeline stops before XML generation.
- If Nadzoru script fails or `Sloc<N>.xml` is missing, pipeline exits with a
  detailed error message.

## Prerequisites

- Python packages: `requests`, `PyYAML`
- Nadzoru2 source available at:
  `~/Documents/Nadzoru2`
  (used to import `machine.automaton`)

## Troubleshooting

- `Unknown task`: verify the name exists in `task_profiles.json`.
- `Expected input not found` with `--skip-llm`: no prior JSON exists for task.
- `Profile ... missing valid ... events list`: fix malformed profile schema.
- Nadzoru import errors: check `DEFAULT_NADZORU_ROOT` in `run_pipeline.py`.
