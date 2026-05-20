#!/usr/bin/env python3
"""Run a basic grid simulation with agents controlled by Nadzoru SCT automata."""

from __future__ import annotations

import argparse
import contextlib
import os
import random
import string
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import List

from automata import load_automaton
from grid_world import DELTAS, GridAgent, GridEnvironment


DEFAULT_AUTOMATON = Path(__file__).with_name("sample_explore_nadzoru.json")
DEFAULT_LOG_DIR = Path(__file__).with_name("logs")
RED_AREA_SIZE = 2
BLUE_AREA_SIZE = 2
OBJECT_COUNT = 4


class Tee:
    def __init__(self, *streams) -> None:
        self.streams = streams

    def write(self, text: str) -> int:
        for stream in self.streams:
            stream.write(text)
        return len(text)

    def flush(self) -> None:
        for stream in self.streams:
            stream.flush()


def parse_position(text: str) -> tuple[int, int]:
    try:
        x_text, y_text = text.split(",", 1)
        return int(x_text), int(y_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("positions must be written as x,y") from exc


def build_agents(args: argparse.Namespace, env: GridEnvironment) -> List[GridAgent]:
    automata_paths = args.automata or [DEFAULT_AUTOMATON]
    starts = args.start or default_starts(args.agents, env)
    if len(starts) < args.agents:
        raise SystemExit(f"Need at least {args.agents} start positions.")

    agents = []
    for idx in range(args.agents):
        path = automata_paths[idx % len(automata_paths)]
        automaton = load_automaton(path)
        start = starts[idx]
        if env.blocked(start):
            raise SystemExit(f"Start position {start} for agent {idx} is blocked.")
        agents.append(
            GridAgent(
                name=f"agent_{idx}",
                automaton=automaton,
                position=start,
                heading=args.heading,
                glyph=string.ascii_uppercase[idx % len(string.ascii_uppercase)],
                side_sensors=args.side_sensors,
            )
        )
    return agents


def default_red_area(width: int, height: int) -> set[tuple[int, int]]:
    origin_x = max(0, min(width - RED_AREA_SIZE, width // 2 - 1))
    origin_y = max(0, min(height - RED_AREA_SIZE, height // 2 - 1))
    return build_square_area((origin_x, origin_y), width, height, RED_AREA_SIZE, "Red")


def default_blue_area(width: int, height: int) -> set[tuple[int, int]]:
    origin_x = max(0, min(width - BLUE_AREA_SIZE, width // 2 - 1))
    origin_y = max(0, min(height - BLUE_AREA_SIZE, height // 2 - 1))
    return build_square_area((origin_x, origin_y), width, height, BLUE_AREA_SIZE, "Blue")


def default_objects(
    width: int,
    height: int,
    forbidden: set[tuple[int, int]],
    rng: random.Random,
) -> set[tuple[int, int]]:
    candidates = [
        (x, y)
        for y in range(height)
        for x in range(width)
        if (x, y) not in forbidden
    ]
    rng.shuffle(candidates)

    selected: list[tuple[int, int]] = []
    for candidate in candidates:
        if any(
            abs(candidate[0] - chosen[0]) + abs(candidate[1] - chosen[1]) <= 1
            for chosen in selected
        ):
            continue
        selected.append(candidate)
        if len(selected) == OBJECT_COUNT:
            return set(selected)

    raise SystemExit(
        "Could not place four separated 1-cell objects. Increase the grid size or set --object positions manually."
    )


def build_red_area(origin: tuple[int, int], width: int, height: int) -> set[tuple[int, int]]:
    return build_square_area(origin, width, height, RED_AREA_SIZE, "Red")


def build_blue_area(origin: tuple[int, int], width: int, height: int) -> set[tuple[int, int]]:
    return build_square_area(origin, width, height, BLUE_AREA_SIZE, "Blue")


def build_square_area(
    origin: tuple[int, int],
    width: int,
    height: int,
    size: int,
    label: str,
) -> set[tuple[int, int]]:
    origin_x, origin_y = origin
    cells = {
        (origin_x + dx, origin_y + dy)
        for dx in range(size)
        for dy in range(size)
    }
    invalid = [cell for cell in cells if not (0 <= cell[0] < width and 0 <= cell[1] < height)]
    if invalid:
        raise SystemExit(
            f"{label} area origin {origin} does not fit a {size}x{size} area "
            f"inside a {width}x{height} grid."
        )
    return cells


def default_separate_red_blue_areas(
    width: int,
    height: int,
) -> tuple[set[tuple[int, int]], set[tuple[int, int]]]:
    if width >= RED_AREA_SIZE + BLUE_AREA_SIZE:
        origin_y = max(0, min(height - RED_AREA_SIZE, height // 2 - 1))
        red_origin = (max(0, width // 3 - 1), origin_y)
        blue_origin = (min(width - BLUE_AREA_SIZE, (2 * width) // 3), origin_y)
    elif height >= RED_AREA_SIZE + BLUE_AREA_SIZE:
        origin_x = max(0, min(width - RED_AREA_SIZE, width // 2 - 1))
        red_origin = (origin_x, max(0, height // 3 - 1))
        blue_origin = (origin_x, min(height - BLUE_AREA_SIZE, (2 * height) // 3))
    else:
        raise SystemExit(
            "Need at least four cells of width or height to place separate 2x2 red and blue areas."
        )

    red_area = build_red_area(red_origin, width, height)
    blue_area = build_blue_area(blue_origin, width, height)
    if red_area & blue_area:
        raise SystemExit("Default red and blue areas overlap; use --red-area-origin and --blue-area-origin.")
    return red_area, blue_area


def should_enable_red_area(args: argparse.Namespace) -> bool:
    if args.red_area_origin is not None:
        return True
    return any(
        "prompt_8" in path.parts
        or "prompt_10" in path.parts
        or "prompt_11" in path.parts
        or "prompt_13" in path.parts
        for path in (args.automata or [])
    )


def should_enable_blue_area(args: argparse.Namespace) -> bool:
    if args.blue_area_origin is not None:
        return True
    return any(
        "prompt_9" in path.parts
        or "prompt_10" in path.parts
        or "prompt_12" in path.parts
        for path in (args.automata or [])
    )


def should_enable_objects(args: argparse.Namespace) -> bool:
    if args.object:
        return True
    return any("prompt_13" in path.parts for path in (args.automata or []))


def default_starts(count: int, env: GridEnvironment) -> List[tuple[int, int]]:
    free = []
    for y in range(env.height):
        for x in range(env.width):
            position = (x, y)
            if position not in env.obstacles and position not in env.objects:
                free.append(position)

    if count <= 1:
        return free[:count]

    starts = []
    last_index = len(free) - 1
    for idx in range(count):
        starts.append(free[round(idx * last_index / (count - 1))])
    return starts


def run(args: argparse.Namespace) -> int:
    rng = random.Random(args.seed)
    random.seed(args.seed)
    red_enabled = should_enable_red_area(args)
    blue_enabled = should_enable_blue_area(args)
    if (
        red_enabled
        and blue_enabled
        and args.red_area_origin is None
        and args.blue_area_origin is None
    ):
        red_area, blue_area = default_separate_red_blue_areas(args.width, args.height)
    else:
        blue_area = set()
        red_area = set()

    if args.red_area_origin is not None:
        red_area = build_red_area(args.red_area_origin, args.width, args.height)
    elif red_enabled and not red_area:
        red_area = default_red_area(args.width, args.height)

    if args.blue_area_origin is not None:
        blue_area = build_blue_area(args.blue_area_origin, args.width, args.height)
    elif blue_enabled and not blue_area:
        blue_area = default_blue_area(args.width, args.height)

    obstacles = set(args.obstacle or [])
    if args.object:
        objects = set(args.object)
    elif should_enable_objects(args):
        objects = default_objects(args.width, args.height, obstacles | red_area | blue_area, rng)
    else:
        objects = set()

    env = GridEnvironment(args.width, args.height, obstacles, red_area, blue_area, objects)
    agents = build_agents(args, env)

    if args.text:
        print("Initial grid:")
        print(env.render(agents))
        print()
    else:
        plotter = GridPlotter(env, agents, args.steps, args.save_plot)
        plotter.draw(0, [])

    for step_idx in range(1, args.steps + 1):
        reports = [agent.step(env, agents, rng) for agent in agents]
        if args.text and not args.quiet:
            print(f"Step {step_idx}:")
            # for report in reports:
                # collision = (
                #     f" collision={report['collision_event']}->{report['collision_hit']}"
                #     if report["collision"]
                #     else ""
                # )
                # print(
                #     "  {agent}: sense={perception} action={action} "
                #     "pos={position} heading={heading} state={state}{collision}".format_map(
                #         {**report, "collision": collision}
                #     )
                # )
            # print(env.render(agents))
            # print()
        elif not args.text:
            plotter.draw(step_idx, reports)
            if not args.quiet:
                print_plot_step_summary(step_idx, agents, reports)

    total_cells = env.width * env.height - len(env.obstacles)
    visited = set().union(*(agent.visited for agent in agents))
    total_collisions = sum(agent.collisions for agent in agents)
    if not args.text:
        plotter.finish()
    print(
        f"Visited {len(visited)}/{total_cells} free cells "
        f"({len(visited) / total_cells:.1%}) in {args.steps} steps."
    )
    print(f"Collisions: {total_collisions} total")
    for agent in agents:
        print(
            f"  {agent.name}: {agent.collisions} "
            f"(wall={agent.collision_by_type['wall']}, "
            f"obstacle={agent.collision_by_type['obstacle']}, "
            f"agent={agent.collision_by_type['agent']}, "
            f"object={agent.collision_by_type['object']}; "
            f"events={format_count_dict(agent.collision_by_event)}; "
            f"hits={format_count_dict(agent.collision_by_hit)})"
        )
    print_action_summary(agents)
    return 0


def run_with_logging(args: argparse.Namespace) -> int:
    if args.no_log:
        return run(args)

    log_path = args.log_file or make_run_log_path(args.log_dir)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log_file:
        stdout_tee = Tee(sys.stdout, log_file)
        stderr_tee = Tee(sys.stderr, log_file)
        with contextlib.redirect_stdout(stdout_tee), contextlib.redirect_stderr(stderr_tee):
            print(f"Writing simulation log to {log_path}")
            return run(args)


def make_run_log_path(log_dir: Path) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return log_dir / f"grid_sim_{timestamp}.log"


def aggregate_action_counts(agents: List[GridAgent]) -> Counter:
    counts = Counter()
    for agent in agents:
        counts.update(agent.action_counts)
    return counts


def format_action_percentages(counts: Counter) -> str:
    total = sum(counts.values())
    if total == 0:
        return "no actions"
    parts = []
    for action, count in sorted(counts.items()):
        parts.append(f"{action}={count / total:.1%} ({count})")
    return ", ".join(parts)


def print_action_summary(agents: List[GridAgent]) -> None:
    print(f"Actions overall: {format_action_percentages(aggregate_action_counts(agents))}")
    for agent in agents:
        print(f"  {agent.name}: {format_action_percentages(Counter(agent.action_counts))}")


def format_count_dict(counts: dict[str, int]) -> str:
    if not counts:
        return "none"
    return ", ".join(f"{name}={count}" for name, count in sorted(counts.items()))


def print_plot_step_summary(
    step_idx: int,
    agents: List[GridAgent],
    reports: list[dict],
) -> None:
    total_collisions = sum(agent.collisions for agent in agents)
    step_collisions = [
        f"{report['agent']}:{report['collision_event']}->{report['collision_hit']}"
        for report in reports
        if report["collision"]
    ]
    details = ", ".join(
        f"{agent.name}={agent.collisions}" for agent in agents
    )
    suffix = f" | new: {', '.join(step_collisions)}" if step_collisions else ""
    actions = format_action_percentages(aggregate_action_counts(agents))
    print(
        f"Step {step_idx}: collisions total={total_collisions} "
    )


class GridPlotter:
    def __init__(
        self,
        env: GridEnvironment,
        agents: List[GridAgent],
        total_steps: int,
        save_path: Path | None,
    ) -> None:
        try:
            os.environ.setdefault("MPLCONFIGDIR", "/tmp/basic_sim_matplotlib")
            Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
            import matplotlib.pyplot as plt
            from matplotlib.patches import Rectangle
        except ImportError as exc:
            raise SystemExit(
                "Matplotlib is required for plot mode. Install matplotlib or run with --text."
            ) from exc

        self.env = env
        self.agents = agents
        self.total_steps = total_steps
        self.save_path = save_path
        self.plt = plt
        self.Rectangle = Rectangle
        self.fig, self.ax = plt.subplots(figsize=(max(5, env.width), max(4, env.height)))
        self.agent_colors = [
            "#2563eb",
            "#dc2626",
            "#16a34a",
            "#ca8a04",
            "#9333ea",
            "#0891b2",
            "#ea580c",
            "#4f46e5",
        ]

    def draw(self, step_idx: int, reports: list[dict]) -> None:
        self.ax.clear()
        self.ax.set_xlim(-0.5, self.env.width - 0.5)
        self.ax.set_ylim(self.env.height - 0.5, -0.5)
        self.ax.set_aspect("equal")
        self.ax.set_xticks(range(self.env.width))
        self.ax.set_yticks(range(self.env.height))
        self.ax.grid(True, color="#d1d5db", linewidth=1)
        self.ax.tick_params(length=0)
        total_collisions = sum(agent.collisions for agent in self.agents)
        self.ax.set_title(
            f"SCT Grid Simulation - step {step_idx}/{self.total_steps} - "
            f"collisions: {total_collisions}"
        )

        for x, y in self.env.red_area:
            self.ax.add_patch(
                self.Rectangle(
                    (x - 0.5, y - 0.5),
                    1,
                    1,
                    facecolor="#ef4444",
                    edgecolor="#b91c1c",
                    alpha=0.35,
                    linewidth=1.5,
                )
            )

        for x, y in self.env.blue_area:
            self.ax.add_patch(
                self.Rectangle(
                    (x - 0.5, y - 0.5),
                    1,
                    1,
                    facecolor="#3b82f6",
                    edgecolor="#1d4ed8",
                    alpha=0.35,
                    linewidth=1.5,
                )
            )

        for x, y in self.env.obstacles:
            self.ax.add_patch(
                self.Rectangle(
                    (x - 0.5, y - 0.5),
                    1,
                    1,
                    facecolor="#111827",
                    edgecolor="#111827",
                )
            )

        for x, y in self.env.objects:
            self.ax.add_patch(
                self.Rectangle(
                    (x - 0.35, y - 0.35),
                    0.7,
                    0.7,
                    facecolor="#a16207",
                    edgecolor="#713f12",
                    linewidth=1.5,
                    zorder=2,
                )
            )

        for idx, agent in enumerate(self.agents):
            color = self.agent_colors[idx % len(self.agent_colors)]
            for x, y in agent.visited:
                self.ax.add_patch(
                    self.Rectangle(
                        (x - 0.5, y - 0.5),
                        1,
                        1,
                        facecolor=color,
                        alpha=0.12,
                        edgecolor="none",
                    )
                )

        for idx, agent in enumerate(self.agents):
            x, y = agent.position
            color = self.agent_colors[idx % len(self.agent_colors)]
            self.ax.scatter([x], [y], s=520, color=color, edgecolor="white", linewidth=2, zorder=3)
            self.ax.text(
                x,
                y,
                agent.glyph,
                ha="center",
                va="center",
                color="white",
                weight="bold",
                fontsize=12,
                zorder=4,
            )
            dx, dy = DELTAS[agent.heading]
            self.ax.arrow(
                x,
                y,
                dx * 0.26,
                dy * 0.26,
                color="white",
                width=0.025,
                head_width=0.16,
                length_includes_head=True,
                zorder=5,
            )

        self.ax.set_xlabel("")

        self.fig.tight_layout()
        if not self.save_path:
            self.plt.pause(0.25)

    def finish(self) -> None:
        if self.save_path:
            self.fig.savefig(self.save_path, dpi=160)
            print(f"Saved final plot to {self.save_path}")
            self.plt.close(self.fig)
            return
        self.plt.show()


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Simulate grid agents driven by Nadzoru-style SCT automata."
    )
    parser.add_argument(
        "--automata",
        nargs="+",
        type=Path,
        help="Nadzoru XML, generated JSON, or compact SCT YAML files. Reused round-robin.",
    )
    parser.add_argument("--agents", type=int, default=2)
    parser.add_argument("--steps", type=int, default=50)
    parser.add_argument("--width", type=int, default=8)
    parser.add_argument("--height", type=int, default=6)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--heading", choices=["N", "E", "S", "W"], default="E")
    parser.add_argument("--start", action="append", type=parse_position, help="Agent start x,y.")
    parser.add_argument("--obstacle", action="append", type=parse_position, help="Obstacle x,y.")
    parser.add_argument(
        "--object",
        action="append",
        type=parse_position,
        help="Movable 1-cell object x,y. If omitted, prompt_13 creates four default objects.",
    )
    parser.add_argument(
        "--red-area-origin",
        type=parse_position,
        help=(
            "Top-left x,y of the 2x2 red gathering area. If omitted, the red area "
            "is auto-enabled only for automata paths under prompt_8, prompt_10, prompt_11, or prompt_13."
        ),
    )
    parser.add_argument(
        "--blue-area-origin",
        type=parse_position,
        help=(
            "Top-left x,y of the 2x2 blue danger area. If omitted, the blue area "
            "is auto-enabled only for automata paths under prompt_9, prompt_10, or prompt_12."
        ),
    )
    parser.add_argument(
        "--side-sensors",
        action="store_true",
        help="Emit obstacle_left/right when side cells are blocked. By default only the front cell controls path_clear.",
    )
    parser.add_argument("--text", action="store_true", help="Use terminal ASCII output instead of the plot.")
    parser.add_argument("--save-plot", type=Path, help="Save the final plot image instead of opening a window.")
    parser.add_argument("--quiet", action="store_true", help="Only print the final coverage summary.")
    parser.add_argument(
        "--log-file",
        type=Path,
        help="Write a copy of terminal output to this exact log file.",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=DEFAULT_LOG_DIR,
        help="Directory for per-run simulation logs.",
    )
    parser.add_argument("--no-log", action="store_true", help="Do not write a simulation log file.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = make_parser()
    args = parser.parse_args(argv)
    if args.agents < 1:
        parser.error("--agents must be at least 1")
    if args.width < 1 or args.height < 1:
        parser.error("--width and --height must be at least 1")
    return run_with_logging(args)


if __name__ == "__main__":
    sys.exit(main())
