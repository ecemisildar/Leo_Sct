# New Prompt Runs

This folder contains the 8-prompt exploration set for new `run_pipeline.py` runs.

Generate one YAML per prompt:

```bash
python3 llm-part/run_pipeline.py --task explore --prompt-set-file "new prompt runs/prompt_groups_explore_8.txt" --prompt-repeats 1 --results-llm-dir "new prompt runs" --run-llm
```

The generated YAMLs will be under:

```text
new prompt runs/explore/new/prompt_*/run_*/
```

Run Gazebo simulations from those saved YAMLs:

```bash
python3 llm-part/run_saved_yaml_sim_batches.py --yaml-root "new prompt runs/explore" --results-root "new prompt runs/gazebo_results" --once-per-yaml
```
