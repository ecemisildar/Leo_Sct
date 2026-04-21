#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Dict, List


THIS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = THIS_DIR.parent
WORKSPACE_ROOT = THIS_DIR.parents[2]
DEFAULT_INSTALL_SETUP = WORKSPACE_ROOT / "install" / "setup.bash"

sys.path.insert(0, str(THIS_DIR))
from run_pipeline import DEFAULT_PROMPT_SET_FILE, DEFAULT_RESULTS_LLM_DIR, _parse_prompt_set_file


def _find_yaml_file(run_dir: Path) -> Path:
    yaml_paths = sorted(run_dir.glob("*.yaml"))
    if len(yaml_paths) != 1:
        raise SystemExit(f"Expected exactly one YAML in {run_dir}, found {len(yaml_paths)}")
    return yaml_paths[0]


def _build_pipeline_command(
    *,
    task: str,
    prompt_set_file: Path,
    prompt_repeats: int,
    results_llm_dir: Path,
) -> List[str]:
    cmd = [
        "python3",
        str(THIS_DIR / "run_pipeline.py"),
        "--task",
        task,
        "--prompt-set-file",
        str(prompt_set_file),
        "--prompt-repeats",
        str(prompt_repeats),
        "--results-llm-dir",
        str(results_llm_dir),
        "--run-llm",
    ]
    return cmd


def _build_launch_command(
    *,
    setup_bash: Path,
    run_duration: float,
    total_robots: int,
    results_dir: Path,
    metadata_yaml_path: Path,
    prompt_file_path: Path,
    headless: bool,
    auto_start: bool,
    spawn_layout: str,
    auto_start_supervisor: bool,
) -> str:
    parts = [
        f"source {shlex.quote(str(setup_bash))}",
        "ros2 launch swarm_basics leo_gz.launch.py",
        f"run_duration:={run_duration}",
        f"total_robots:={total_robots}",
        f"results_dir:={shlex.quote(str(results_dir))}",
        f"metadata_yaml_path:={shlex.quote(str(metadata_yaml_path))}",
        f"prompt_file_path:={shlex.quote(str(prompt_file_path))}",
        f"headless:={'true' if headless else 'false'}",
        f"auto_start:={'true' if auto_start else 'false'}",
        f"spawn_layout:={spawn_layout}",
        f"auto_start_supervisor:={'true' if auto_start_supervisor else 'false'}",
    ]
    return " && ".join(parts[:2]) + " " + " ".join(parts[2:])


def _run_command(command: List[str], *, cwd: Path) -> None:
    subprocess.run(command, cwd=cwd, check=True)


def _write_manifest(rows: List[Dict[str, str]], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "group",
                "prompt_ordinal",
                "repeat",
                "yaml_path",
                "prompt_path",
                "results_root",
                "run_output_dir",
                "status",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate grouped YAMLs, with optional grouped Gazebo experiments."
    )
    parser.add_argument("--task", default="explore")
    parser.add_argument("--prompt-set-file", default=str(DEFAULT_PROMPT_SET_FILE))
    parser.add_argument("--prompt-repeats", type=int, default=3)
    parser.add_argument("--results-llm-dir", default=str(DEFAULT_RESULTS_LLM_DIR))
    parser.add_argument(
        "--sim-results-dir",
        default=str(PROJECT_ROOT / "results_exp_batches"),
        help="Root folder for simulation outputs, grouped by prompt set structure.",
    )
    parser.add_argument("--run-duration", type=float, default=200.0)
    parser.add_argument("--total-robots", type=int, default=5)
    parser.add_argument("--spawn-layout", default="spread")
    parser.add_argument("--setup-bash", default=str(DEFAULT_INSTALL_SETUP))
    parser.add_argument(
        "--run-sim",
        action="store_true",
        help="Also launch the simulation batch after generating YAMLs.",
    )
    parser.add_argument(
        "--skip-yaml-generation",
        action="store_true",
        help="Reuse existing grouped YAMLs and skip calling run_pipeline.py.",
    )
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--no-headless", dest="headless", action="store_false")
    parser.add_argument("--auto-start", action="store_true", default=True)
    parser.add_argument("--no-auto-start", dest="auto_start", action="store_false")
    parser.add_argument("--auto-start-supervisor", action="store_true", default=True)
    parser.add_argument("--no-auto-start-supervisor", dest="auto_start_supervisor", action="store_false")
    parser.add_argument(
        "--stop-on-error",
        action="store_true",
        help="Abort the batch on the first failed simulation run.",
    )
    args = parser.parse_args()

    prompt_set_file = Path(args.prompt_set_file).resolve()
    results_llm_dir = Path(args.results_llm_dir).resolve()
    sim_results_dir = Path(args.sim_results_dir).resolve()
    setup_bash = Path(args.setup_bash).resolve()

    if args.run_sim and not setup_bash.exists():
        raise SystemExit(f"ROS setup file not found: {setup_bash}")

    prompt_entries = _parse_prompt_set_file(prompt_set_file)

    if not args.skip_yaml_generation:
        pipeline_cmd = _build_pipeline_command(
            task=args.task,
            prompt_set_file=prompt_set_file,
            prompt_repeats=args.prompt_repeats,
            results_llm_dir=results_llm_dir,
        )
        _run_command(pipeline_cmd, cwd=THIS_DIR)

    manifest_rows: List[Dict[str, str]] = []
    manifest_rows: List[Dict[str, str]] = []

    for entry in prompt_entries:
        group = entry["group"]
        prompt_ordinal = entry["ordinal"]
        for repeat in range(1, args.prompt_repeats + 1):
            yaml_run_dir = results_llm_dir / args.task / group / f"prompt_{prompt_ordinal}" / f"run_{repeat}"
            yaml_path = _find_yaml_file(yaml_run_dir)
            prompt_path = yaml_run_dir / "prompt.txt"
            if not prompt_path.exists():
                raise SystemExit(f"Prompt sidecar not found: {prompt_path}")

            sim_run_root = sim_results_dir / args.task / group / f"prompt_{prompt_ordinal}" / f"run_{repeat}"
            run_output_dir = ""
            status = "yaml_only"

            if args.run_sim:
                sim_run_root.mkdir(parents=True, exist_ok=True)
                before = {path.resolve() for path in sim_run_root.glob("run_*") if path.is_dir()}

                launch_cmd = _build_launch_command(
                    setup_bash=setup_bash,
                    run_duration=args.run_duration,
                    total_robots=args.total_robots,
                    results_dir=sim_run_root,
                    metadata_yaml_path=yaml_path,
                    prompt_file_path=prompt_path,
                    headless=args.headless,
                    auto_start=args.auto_start,
                    spawn_layout=args.spawn_layout,
                    auto_start_supervisor=args.auto_start_supervisor,
                )
                stdout_path = sim_run_root / "launch_stdout.log"
                stderr_path = sim_run_root / "launch_stderr.log"

                status = "ok"
                try:
                    with stdout_path.open("a", encoding="utf-8") as stdout_f, stderr_path.open(
                        "a", encoding="utf-8"
                    ) as stderr_f:
                        subprocess.run(
                            ["bash", "-lc", launch_cmd],
                            cwd=WORKSPACE_ROOT,
                            check=True,
                            stdout=stdout_f,
                            stderr=stderr_f,
                        )
                except subprocess.CalledProcessError:
                    status = "failed"
                    if args.stop_on_error:
                        raise

                after = sorted(path.resolve() for path in sim_run_root.glob("run_*") if path.is_dir())
                new_runs = [path for path in after if path not in before]
                run_output_dir = str(new_runs[-1]) if new_runs else ""

            manifest_rows.append(
                {
                    "group": group,
                    "prompt_ordinal": prompt_ordinal,
                    "repeat": str(repeat),
                    "yaml_path": str(yaml_path),
                    "prompt_path": str(prompt_path),
                    "results_root": str(sim_run_root),
                    "run_output_dir": run_output_dir,
                    "status": status,
                }
            )
            print(
                f"[{status.upper()}] group={group} prompt={prompt_ordinal} repeat={repeat} "
                f"yaml={yaml_path}"
                + (f" results={run_output_dir or sim_run_root}" if args.run_sim else "")
            )

    manifest_path = (sim_results_dir / args.task / "batch_manifest.csv") if args.run_sim else (
        results_llm_dir / args.task / "yaml_manifest.csv"
    )
    _write_manifest(manifest_rows, manifest_path)
    print(f"Wrote manifest to {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
