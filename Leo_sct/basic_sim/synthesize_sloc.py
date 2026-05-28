#!/usr/bin/env python3
"""Synthesize Sloc from sample/basic-sim G and E XML files using Nadzoru."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path



THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))
NADZORU_ROOT_CANDIDATES = (
    Path.home() / "Nadzoru2",
    Path.home() / "Documents" / "Nadzoru2",
)


def default_nadzoru_root() -> Path:
    for candidate in NADZORU_ROOT_CANDIDATES:
        if (candidate / "machine" / "automaton.py").is_file():
            return candidate
    return NADZORU_ROOT_CANDIDATES[0]


DEFAULT_NADZORU_ROOT = default_nadzoru_root()


def import_nadzoru(nadzoru_root: Path) -> type:
    if str(nadzoru_root) not in sys.path:
        sys.path.insert(0, str(nadzoru_root))
    try:
        from machine.automaton import Automaton
    except ImportError as exc:
        raise SystemExit(
            f"Could not import Nadzoru machine.automaton from {nadzoru_root}. "
            "Pass --nadzoru-root if your Nadzoru checkout is elsewhere."
        ) from exc
    return Automaton


def load_automaton(automaton_cls: type, path: Path, name: str):
    automaton = automaton_cls()
    automaton.load(str(path))
    automaton.set_name(name)
    return automaton


def save_automaton(automaton: object, path: Path, name: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    automaton.set_name(name)
    automaton.set_file_path_name(str(path))
    automaton.arrange_states_position()
    automaton.save()


def synthesize(args: argparse.Namespace) -> int:
    automaton_cls = import_nadzoru(args.nadzoru_root)

    plant = load_automaton(automaton_cls, args.g, "G")
    spec = load_automaton(automaton_cls, args.e, "E")

    gloc = plant
    kloc = automaton_cls.synchronization(gloc, spec)
    sloc = automaton_cls.sup_c(gloc, kloc)

    save_automaton(gloc, args.output_dir / "Gloc.xml", "Gloc")
    save_automaton(kloc, args.output_dir / "Kloc.xml", "Kloc")
    save_automaton(sloc, args.output_dir / "Sloc.xml", "Sloc")

    print(f"Wrote {args.output_dir / 'Gloc.xml'}")
    print(f"Wrote {args.output_dir / 'Kloc.xml'}")
    print(f"Wrote {args.output_dir / 'Sloc.xml'}")
    return 0


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build Gloc, Kloc, and Sloc from basic-sim G/E XML files."
    )
    parser.add_argument("--g", type=Path, default=Path(__file__).with_name("sample_G.xml"))
    parser.add_argument("--e", type=Path, default=Path(__file__).with_name("sample_all_actions.xml"))
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).with_name("synthesized"),
    )
    parser.add_argument("--nadzoru-root", type=Path, default=DEFAULT_NADZORU_ROOT)
    return parser


def main(argv: list[str] | None = None) -> int:
    return synthesize(make_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
