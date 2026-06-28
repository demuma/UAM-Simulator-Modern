#!/usr/bin/env python3
"""
Find CityGML tiles whose gml:Envelope covers a given coordinate.
Run this script in the folder containing the GML tiles or any parent folder.
"""

from __future__ import annotations

import argparse
import os
import re
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple


LOWER_RE = re.compile(r"<gml:lowerCorner[^>]*>\s*([^<]+)\s*</gml:lowerCorner>")
UPPER_RE = re.compile(r"<gml:upperCorner[^>]*>\s*([^<]+)\s*</gml:upperCorner>")


@dataclass
class Envelope:
    lower: Tuple[float, float]
    upper: Tuple[float, float]

    def contains(self, x: float, y: float) -> bool:
        return self.lower[0] <= x <= self.upper[0] and self.lower[1] <= y <= self.upper[1]

    def distance_to(self, x: float, y: float) -> float:
        """Minimum distance from point to envelope (0 if inside)."""
        dx = 0.0
        if x < self.lower[0]:
            dx = self.lower[0] - x
        elif x > self.upper[0]:
            dx = x - self.upper[0]

        dy = 0.0
        if y < self.lower[1]:
            dy = self.lower[1] - y
        elif y > self.upper[1]:
            dy = y - self.upper[1]

        return (dx * dx + dy * dy) ** 0.5


def iter_gml_files(root: str) -> Iterable[str]:
    for dirpath, _, filenames in os.walk(root):
        for name in filenames:
            if name.lower().endswith(".gml"):
                yield os.path.join(dirpath, name)


def parse_envelope(text: str) -> Optional[Envelope]:
    m_low = LOWER_RE.search(text)
    m_up = UPPER_RE.search(text)
    if not (m_low and m_up):
        return None

    try:
        low_parts = m_low.group(1).split()
        up_parts = m_up.group(1).split()
        lx, ly = float(low_parts[0]), float(low_parts[1])
        ux, uy = float(up_parts[0]), float(up_parts[1])
        return Envelope((lx, ly), (ux, uy))
    except Exception:
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Find GML tiles that cover a coordinate.")
    parser.add_argument("x", type=float, help="Easting (x) coordinate")
    parser.add_argument("y", type=float, help="Northing (y) coordinate")
    parser.add_argument(
        "--root",
        type=str,
        default=".",
        help="Root folder to search (default: current directory)",
    )
    parser.add_argument(
        "--closest",
        type=int,
        default=3,
        help="If no match, show N closest tiles by envelope distance (default: 3)",
    )
    args = parser.parse_args()

    x, y = args.x, args.y
    root = os.path.abspath(args.root)

    matches: List[Tuple[str, Envelope]] = []
    candidates: List[Tuple[float, str, Envelope]] = []

    for path in iter_gml_files(root):
        try:
            with open(path, "r", encoding="utf-8", errors="ignore") as f:
                text = f.read()
        except Exception:
            continue

        env = parse_envelope(text)
        if env is None:
            continue

        if env.contains(x, y):
            matches.append((path, env))
        else:
            dist = env.distance_to(x, y)
            candidates.append((dist, path, env))

    if matches:
        print("Found matching tiles:")
        for path, env in matches:
            rel = os.path.relpath(path, root)
            print(f"- {rel}")
            print(f"  lower: {env.lower[0]} {env.lower[1]}")
            print(f"  upper: {env.upper[0]} {env.upper[1]}")
        return 0

    print("No tile envelopes contain the coordinate.")
    candidates.sort(key=lambda t: t[0])
    for dist, path, env in candidates[: max(args.closest, 0)]:
        rel = os.path.relpath(path, root)
        print(f"- {rel} (distance: {dist:.3f} m)")
        print(f"  lower: {env.lower[0]} {env.lower[1]}")
        print(f"  upper: {env.upper[0]} {env.upper[1]}")

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
