#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import shlex
import subprocess
from pathlib import Path
from typing import Iterable


THIS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = THIS_DIR.parent
WORKSPACE_ROOT = THIS_DIR.parents[2]
DEFAULT_SETUP_BASH = WORKSPACE_ROOT / "install" / "setup.bash"
DEFAULT_YAML_ROOT = PROJECT_ROOT / "results_llm" / "explore"
DEFAULT_RESULTS_ROOT = PROJECT_ROOT / "results_exp" / "explore"
EXPECTED_GROUPS = ("long", "medium", "short")
EXPECTED_PROMPTS = range(1, 6)
EXPECTED_RUNS = range(1, 4)
SPAWN_LAYOUTS = ("spread", "middle_circle", "random_safe")
GROUP_INPUT_DIRS = {
    "long": "long",
    "medium": "medium",
    "short": "vague",
    "vague": "vague",
}


def _iter_yaml_runs(yaml_root: Path) -> Iterable[tuple[str, str, str, Path, Path]]:
    for yaml_path in sorted(yaml_root.glob("*/*/run_*/*.yaml")):
        run_dir = yaml_path.parent
        prompt_file = run_dir / "prompt.txt"
        if not prompt_file.exists():
            raise SystemExit(f"Missing prompt.txt next to YAML: {yaml_path}")
        run_name = run_dir.name
        prompt_name = run_dir.parent.name
        group_name = run_dir.parent.parent.name
        yield group_name, prompt_name, run_name, yaml_path, prompt_file


def _iter_yaml_runs_interleaved(yaml_root: Path) -> Iterable[tuple[str, str, str, Path, Path]]:
    """Yield prompt_1 long/medium/short, then prompt_2 long/medium/short, etc."""
    yielded: set[Path] = set()
    for prompt in EXPECTED_PROMPTS:
        for group in EXPECTED_GROUPS:
            input_group = GROUP_INPUT_DIRS.get(group, group)
            prompt_dir = yaml_root / input_group / f"prompt_{prompt}"
            for yaml_path in sorted(prompt_dir.glob("run_*/*.yaml")):
                run_dir = yaml_path.parent
                prompt_file = run_dir / "prompt.txt"
                if not prompt_file.exists():
                    raise SystemExit(f"Missing prompt.txt next to YAML: {yaml_path}")
                yielded.add(yaml_path.resolve())
                yield group, f"prompt_{prompt}", run_dir.name, yaml_path, prompt_file

    # Include any non-standard saved YAMLs after the expected prompt/group order.
    for group, prompt_name, run_name, yaml_path, prompt_file in _iter_yaml_runs(yaml_root):
        if yaml_path.resolve() not in yielded:
            yield group, prompt_name, run_name, yaml_path, prompt_file


def _iter_expected_yaml_runs(yaml_root: Path) -> Iterable[tuple[str, str, str, Path | None, Path | None]]:
    for group in EXPECTED_GROUPS:
        for prompt in EXPECTED_PROMPTS:
            for run in EXPECTED_RUNS:
                input_group = GROUP_INPUT_DIRS.get(group, group)
                run_dir = yaml_root / input_group / f"prompt_{prompt}" / f"run_{run}"
                yaml_paths = sorted(run_dir.glob("*.yaml"))
                prompt_file = run_dir / "prompt.txt"
                yaml_path = yaml_paths[0] if len(yaml_paths) == 1 else None
                prompt_path = prompt_file if prompt_file.exists() else None
                yield group, f"prompt_{prompt}", f"run_{run}", yaml_path, prompt_path


def _single_yaml_run(yaml_path: Path) -> tuple[str, str, str, Path, Path]:
    run_dir = yaml_path.parent
    prompt_file = run_dir / "prompt.txt"
    if not prompt_file.exists():
        raise SystemExit(f"Missing prompt.txt next to YAML: {yaml_path}")

    run_name = run_dir.name
    prompt_name = run_dir.parent.name if run_dir.parent.name else "prompt_unknown"
    group_name = run_dir.parent.parent.name if run_dir.parent.parent.name else "group_unknown"
    return group_name, prompt_name, run_name, yaml_path, prompt_file


def _resolve_yaml_selector(yaml_root: Path, group: str, prompt: int, run: int) -> Path:
    input_group = GROUP_INPUT_DIRS.get(group, group)
    run_dir = yaml_root / input_group / f"prompt_{prompt}" / f"run_{run}"
    yaml_paths = sorted(run_dir.glob("*.yaml"))
    if len(yaml_paths) != 1:
        raise SystemExit(f"Expected exactly one YAML in {run_dir}, found {len(yaml_paths)}")
    return yaml_paths[0]


def _append_manifest_row(manifest_path: Path, row: dict[str, str]) -> None:
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not manifest_path.exists()
    with manifest_path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "group",
                "prompt",
                "run",
                "layout",
                "trial",
                "yaml_path",
                "prompt_path",
                "results_parent",
                "run_output_dir",
                "status",
            ],
        )
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def _run_artifacts_saved(run_dir: Path) -> bool:
    required = [
        "coverage_timeseries.csv",
        "coverage_paths.csv",
        "coverage_visited_cells.csv",
        "SAVE_STATUS.txt",
    ]
    return all((run_dir / name).exists() for name in required)


def _last_coverage_time(run_dir: Path) -> float | None:
    coverage_csv = run_dir / "coverage_timeseries.csv"
    if not coverage_csv.exists():
        return None
    last_time = None
    with coverage_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                last_time = float(row.get("time_s", ""))
            except (TypeError, ValueError):
                continue
    return last_time


def _find_existing_saved_run(results_parent: Path) -> Path | None:
    existing_runs = sorted(
        (path for path in results_parent.glob("run_*") if path.is_dir()),
        key=lambda path: path.stat().st_mtime,
    )
    for run_dir in reversed(existing_runs):
        if _run_artifacts_saved(run_dir):
            return run_dir
    return None


def _build_launch_cmd(
    *,
    setup_bash: Path,
    build_packages: list[str],
    run_duration: float,
    total_robots: int,
    spawn_layout: str,
    results_dir: Path,
    yaml_path: Path,
    prompt_path: Path,
    headless: bool,
    auto_start: bool,
    auto_start_supervisor: bool,
    skip_build: bool,
) -> str:
    commands = []
    if not skip_build:
        build_cmd = "colcon build"
        if build_packages:
            build_cmd += " --packages-select " + " ".join(
                shlex.quote(pkg) for pkg in build_packages
            )
        commands.append(build_cmd)
    commands.extend(
        [
            f"source {shlex.quote(str(setup_bash))}",
            " ".join(
                [
                    "ros2 launch swarm_basics leo_gz.launch.py",
                    f"run_duration:={run_duration}",
                    f"total_robots:={total_robots}",
                    f"spawn_layout:={spawn_layout}",
                    f"results_dir:={shlex.quote(str(results_dir))}",
                    f"metadata_yaml_path:={shlex.quote(str(yaml_path))}",
                    f"prompt_file_path:={shlex.quote(str(prompt_path))}",
                    f"headless:={'true' if headless else 'false'}",
                    f"auto_start:={'true' if auto_start else 'false'}",
                    f"auto_start_supervisor:={'true' if auto_start_supervisor else 'false'}",
                ]
            ),
        ]
    )
    return " && ".join(commands)


def _run_single_simulation(
    *,
    manifest_path: Path,
    group: str,
    prompt: str,
    run_name: str,
    layout: str,
    trial: int,
    results_parent: Path,
    yaml_path: Path,
    prompt_path: Path,
    setup_bash: Path,
    build_packages: list[str],
    run_duration: float,
    total_robots: int,
    headless: bool,
    auto_start: bool,
    auto_start_supervisor: bool,
    skip_build: bool,
    capture_output: bool,
    stop_on_error: bool,
    analyze_after_run: bool,
) -> str:
    results_parent.mkdir(parents=True, exist_ok=True)
    existing_run_dir = _find_existing_saved_run(results_parent)
    if existing_run_dir is not None:
        _append_manifest_row(
            manifest_path,
            {
                "group": group,
                "prompt": prompt,
                "run": run_name,
                "layout": layout,
                "trial": str(trial),
                "yaml_path": str(yaml_path),
                "prompt_path": str(prompt_path),
                "results_parent": str(results_parent),
                "run_output_dir": str(existing_run_dir),
                "status": "skipped_existing",
            },
        )
        print(
            f"[SKIPPED] group={group} prompt={prompt} run={run_name} "
            f"layout={layout} trial={trial} existing={existing_run_dir}"
        )
        return "skipped_existing"

    before_runs = {path.resolve() for path in results_parent.glob("run_*") if path.is_dir()}

    launch_cmd = _build_launch_cmd(
        setup_bash=setup_bash,
        build_packages=build_packages,
        run_duration=run_duration,
        total_robots=total_robots,
        spawn_layout=layout,
        results_dir=results_parent,
        yaml_path=yaml_path,
        prompt_path=prompt_path,
        headless=headless,
        auto_start=auto_start,
        auto_start_supervisor=auto_start_supervisor,
        skip_build=skip_build,
    )

    status = "ok"
    launch_failed = False
    try:
        if capture_output:
            stdout_path = results_parent / "launch_stdout.log"
            stderr_path = results_parent / "launch_stderr.log"
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
        else:
            subprocess.run(
                ["bash", "-lc", launch_cmd],
                cwd=WORKSPACE_ROOT,
                check=True,
            )
    except subprocess.CalledProcessError:
        launch_failed = True

    after_runs = sorted(path.resolve() for path in results_parent.glob("run_*") if path.is_dir())
    new_runs = [path for path in after_runs if path not in before_runs]
    run_output_dir = str(new_runs[-1]) if new_runs else ""
    saved_ok = bool(run_output_dir) and _run_artifacts_saved(Path(run_output_dir))
    duration_ok = False
    if saved_ok:
        coverage_end_s = _last_coverage_time(Path(run_output_dir))
        duration_ok = (
            coverage_end_s is not None
            and coverage_end_s >= max(0.0, run_duration - 5.0)
        )
        if not duration_ok:
            status = "short_run"

    if saved_ok and analyze_after_run:
        try:
            subprocess.run(
                [
                    "python3",
                    str(PROJECT_ROOT / "swarm_basics/swarm_basics/analyze_run_from_logs.py"),
                    "--results-dir",
                    str(Path(run_output_dir).parent),
                ],
                cwd=PROJECT_ROOT,
                check=True,
            )
        except subprocess.CalledProcessError:
            status = "analysis_failed"
            if stop_on_error:
                raise

    if launch_failed:
        if saved_ok and duration_ok:
            if status != "analysis_failed":
                status = "ok"
        else:
            if status != "short_run":
                status = "failed"

    _append_manifest_row(
        manifest_path,
        {
            "group": group,
            "prompt": prompt,
            "run": run_name,
            "layout": layout,
            "trial": str(trial),
            "yaml_path": str(yaml_path),
            "prompt_path": str(prompt_path),
            "results_parent": str(results_parent),
            "run_output_dir": run_output_dir,
            "status": status,
        },
    )
    print(
        f"[{status.upper()}] group={group} prompt={prompt} run={run_name} "
        f"layout={layout} trial={trial} results={run_output_dir or results_parent}"
    )
    if stop_on_error and status in {"failed", "short_run", "analysis_failed"}:
        raise subprocess.CalledProcessError(1, ["bash", "-lc", launch_cmd])
    return status


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run saved grouped YAML supervisors in Gazebo and save each simulation immediately."
    )
    parser.add_argument("--yaml-root", default=str(DEFAULT_YAML_ROOT))
    parser.add_argument(
        "--yaml-file",
        help="Run exactly one YAML file instead of scanning --yaml-root.",
    )
    parser.add_argument("--group", choices=["long", "medium", "short", "vague"])
    parser.add_argument("--prompt", type=int, help="Prompt ordinal within the group, 1-5.")
    parser.add_argument("--run", type=int, help="Run ordinal for the YAML, usually 1-3.")
    parser.add_argument("--results-root", default=str(DEFAULT_RESULTS_ROOT))
    parser.add_argument(
        "--results-parent",
        help=(
            "Exact parent directory for simulation run_* output. "
            "Only valid with --yaml-file or --group/--prompt/--run and --once-per-yaml."
        ),
    )
    parser.add_argument("--setup-bash", default=str(DEFAULT_SETUP_BASH))
    parser.add_argument("--run-duration", type=float, default=200.0)
    parser.add_argument("--total-robots", type=int, default=5)
    parser.add_argument(
        "--build-packages",
        nargs="*",
        default=["swarm_basics", "leo_image"],
        help="Packages to rebuild before each simulation run.",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Skip colcon build and reuse the current workspace install.",
    )
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument(
        "--layouts",
        nargs="+",
        default=["spread", "middle_circle"],
        choices=SPAWN_LAYOUTS,
    )
    parser.add_argument(
        "--once-per-yaml",
        action="store_true",
        help="Run exactly one Gazebo simulation for each YAML instead of iterating all layouts and trials.",
    )
    parser.add_argument(
        "--interleave-groups",
        action="store_true",
        help="Scan YAMLs as prompt_1 long/medium/short, then prompt_2 long/medium/short, etc.",
    )
    parser.add_argument(
        "--analyze-after-run",
        action="store_true",
        help="Run offline coverage/collision analysis immediately after each saved simulation.",
    )
    parser.add_argument(
        "--once-layout",
        default="spread",
        choices=SPAWN_LAYOUTS,
        help="Layout to use with --once-per-yaml. Default: spread.",
    )
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--no-headless", dest="headless", action="store_false")
    parser.add_argument("--auto-start", action="store_true", default=True)
    parser.add_argument("--no-auto-start", dest="auto_start", action="store_false")
    parser.add_argument("--auto-start-supervisor", action="store_true", default=True)
    parser.add_argument("--no-auto-start-supervisor", dest="auto_start_supervisor", action="store_false")
    parser.add_argument(
        "--capture-output",
        action="store_true",
        help="Write build/launch stdout and stderr to per-run log files instead of streaming to the terminal.",
    )
    parser.add_argument("--stop-on-error", action="store_true")
    args = parser.parse_args()

    yaml_root = Path(args.yaml_root).resolve()
    results_root = Path(args.results_root).resolve()
    exact_results_parent = Path(args.results_parent).resolve() if args.results_parent else None
    setup_bash = Path(args.setup_bash).resolve()
    manifest_path = results_root / "simulation_manifest.csv"

    selector_used = any(value is not None for value in (args.group, args.prompt, args.run))
    if selector_used and not all(value is not None for value in (args.group, args.prompt, args.run)):
        raise SystemExit("Use --group, --prompt, and --run together.")

    if args.yaml_file and selector_used:
        raise SystemExit("Use either --yaml-file or (--group --prompt --run), not both.")
    if exact_results_parent is not None and (not args.once_per_yaml or (not args.yaml_file and not selector_used)):
        raise SystemExit("--results-parent requires --once-per-yaml and a single YAML selection.")

    if selector_used:
        if not yaml_root.exists():
            raise SystemExit(f"YAML root not found: {yaml_root}")
        input_group = GROUP_INPUT_DIRS.get(args.group, args.group)
        run_dir = yaml_root / input_group / f"prompt_{args.prompt}" / f"run_{args.run}"
        yaml_paths = sorted(run_dir.glob("*.yaml"))
        prompt_path = run_dir / "prompt.txt"
        yaml_path = yaml_paths[0] if len(yaml_paths) == 1 else None
        prompt_file = prompt_path if prompt_path.exists() else None
        run_entries = [(args.group, f"prompt_{args.prompt}", f"run_{args.run}", yaml_path, prompt_file)]
    elif args.yaml_file:
        yaml_file = Path(args.yaml_file).resolve()
        if not yaml_file.exists():
            raise SystemExit(f"YAML file not found: {yaml_file}")
        run_entries = [_single_yaml_run(yaml_file)]
    else:
        if not yaml_root.exists():
            raise SystemExit(f"YAML root not found: {yaml_root}")
        iterator = _iter_yaml_runs_interleaved if args.interleave_groups else _iter_yaml_runs
        run_entries = list(iterator(yaml_root))

    if not run_entries:
        raise SystemExit(f"YAML root not found: {yaml_root}")
    if not setup_bash.exists():
        raise SystemExit(f"ROS setup file not found: {setup_bash}")

    failures = 0

    for group, prompt, run_name, yaml_path, prompt_path in run_entries:
        if yaml_path is None or prompt_path is None:
            results_parent = results_root / group / prompt / run_name
            _append_manifest_row(
                manifest_path,
                {
                    "group": group,
                    "prompt": prompt,
                    "run": run_name,
                    "layout": "",
                    "trial": "",
                    "yaml_path": str(yaml_path or ""),
                    "prompt_path": str(prompt_path or ""),
                    "results_parent": str(results_parent),
                    "run_output_dir": "",
                    "status": "skipped_missing",
                },
            )
            print(
                f"[SKIPPED] group={group} prompt={prompt} run={run_name} missing YAML or prompt.txt"
            )
            continue
        if args.once_per_yaml:
            results_parent = (
                exact_results_parent
                if exact_results_parent is not None
                else results_root / group / prompt / run_name / args.once_layout / "trial_1"
            )
            status = _run_single_simulation(
                manifest_path=manifest_path,
                group=group,
                prompt=prompt,
                run_name=run_name,
                layout=args.once_layout,
                trial=1,
                results_parent=results_parent,
                yaml_path=yaml_path,
                prompt_path=prompt_path,
                setup_bash=setup_bash,
                build_packages=args.build_packages,
                run_duration=args.run_duration,
                total_robots=args.total_robots,
                headless=args.headless,
                auto_start=args.auto_start,
                auto_start_supervisor=args.auto_start_supervisor,
                skip_build=args.skip_build,
                capture_output=args.capture_output,
                stop_on_error=args.stop_on_error,
                analyze_after_run=args.analyze_after_run,
            )
            if status == "failed":
                failures += 1
            continue
        for layout in args.layouts:
            for trial in range(1, args.trials + 1):
                results_parent = results_root / group / prompt / run_name / layout / f"trial_{trial}"
                status = _run_single_simulation(
                    manifest_path=manifest_path,
                    group=group,
                    prompt=prompt,
                    run_name=run_name,
                    layout=layout,
                    trial=trial,
                    results_parent=results_parent,
                    yaml_path=yaml_path,
                    prompt_path=prompt_path,
                    setup_bash=setup_bash,
                    build_packages=args.build_packages,
                    run_duration=args.run_duration,
                    total_robots=args.total_robots,
                    headless=args.headless,
                    auto_start=args.auto_start,
                    auto_start_supervisor=args.auto_start_supervisor,
                    skip_build=args.skip_build,
                    capture_output=args.capture_output,
                    stop_on_error=args.stop_on_error,
                    analyze_after_run=args.analyze_after_run,
                )
                if status == "failed":
                    failures += 1

    print(f"Wrote manifest to {manifest_path}")
    if failures:
        print(f"Completed with {failures} failed simulation(s).")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
