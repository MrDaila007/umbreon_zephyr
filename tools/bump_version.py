#!/usr/bin/env python3
"""Bump FW_VERSION in src/version.h."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


VERSION_RE = re.compile(r'(#define\s+FW_VERSION\s+")(\d+)\.(\d+)\.(\d+)(")')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("version_file", type=Path, help="Path to src/version.h")
    parser.add_argument(
        "--part",
        choices=("major", "minor", "patch"),
        default="patch",
        help="Version part to bump when --set is not used",
    )
    parser.add_argument(
        "--set",
        dest="set_version",
        help="Set an explicit semantic version, for example 2.1.0",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Only print the current version",
    )
    return parser.parse_args()


def find_version(text: str) -> tuple[re.Match[str], tuple[int, int, int]]:
    match = VERSION_RE.search(text)
    if not match:
        raise SystemExit("FW_VERSION must be defined as major.minor.patch")
    return match, tuple(int(match.group(i)) for i in range(2, 5))


def validate_version(version: str) -> tuple[int, int, int]:
    match = re.fullmatch(r"(\d+)\.(\d+)\.(\d+)", version)
    if not match:
        raise SystemExit("--set must use major.minor.patch, for example 2.1.0")
    return tuple(int(part) for part in match.groups())


def bumped(version: tuple[int, int, int], part: str) -> tuple[int, int, int]:
    major, minor, patch = version
    if part == "major":
        return major + 1, 0, 0
    if part == "minor":
        return major, minor + 1, 0
    return major, minor, patch + 1


def main() -> None:
    args = parse_args()
    text = args.version_file.read_text(encoding="utf-8")
    match, current = find_version(text)
    current_text = ".".join(str(part) for part in current)

    if args.show:
        print(current_text)
        return

    new_version = (
        validate_version(args.set_version)
        if args.set_version
        else bumped(current, args.part)
    )
    new_text = ".".join(str(part) for part in new_version)
    replacement = f"{match.group(1)}{new_text}{match.group(5)}"
    args.version_file.write_text(
        text[: match.start()] + replacement + text[match.end() :],
        encoding="utf-8",
    )
    print(f"FW_VERSION {current_text} -> {new_text}")


if __name__ == "__main__":
    main()
