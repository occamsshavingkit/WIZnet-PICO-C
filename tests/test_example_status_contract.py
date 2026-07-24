#!/usr/bin/env python3
"""Reject example programs that ignore WIZnet startup failures."""

from __future__ import annotations

import re
import sys
from pathlib import Path


STARTUP_APIS = (
    "wizchip_spi_initialize",
    "wizchip_cris_initialize",
    "wizchip_reset",
    "wizchip_initialize",
    "network_initialize",
    "wizchip_1ms_timer_initialize",
)
EXCLUDED_SOURCES = {
    Path("can/can_loopback/wizchip_can_loopback.c"),
    Path("can/can_utils/wizchip_can_utils.c"),
}
API_CALL_RE = re.compile(r"\b(" + "|".join(STARTUP_APIS) + r")\s*\(")
ASSIGNMENT_RE = re.compile(r"(?<![=!<>])=(?!=)")
CONDITION_RE = re.compile(r"\b(?:if|while)\s*\(")


def mask_comments_and_literals(source: str) -> str:
    """Replace comments and literals with spaces while preserving positions."""
    masked = list(source)
    index = 0

    while index < len(source):
        if source.startswith("//", index):
            end = source.find("\n", index)
            end = len(source) if end == -1 else end
        elif source.startswith("/*", index):
            close = source.find("*/", index + 2)
            end = len(source) if close == -1 else close + 2
        elif source[index] in {'"', "'"}:
            quote = source[index]
            end = index + 1
            while end < len(source):
                if source[end] == "\\":
                    end += 2
                elif source[end] == quote:
                    end += 1
                    break
                else:
                    end += 1
        else:
            index += 1
            continue

        for position in range(index, min(end, len(source))):
            if masked[position] != "\n":
                masked[position] = " "
        index = end

    return "".join(masked)


def matching_parenthesis(source: str, opening: int) -> int | None:
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "(":
            depth += 1
        elif source[index] == ")":
            depth -= 1
            if depth == 0:
                return index
    return None


def condition_ranges(source: str) -> list[tuple[int, int]]:
    ranges = []
    for match in CONDITION_RE.finditer(source):
        opening = source.find("(", match.start(), match.end())
        closing = matching_parenthesis(source, opening)
        if closing is not None:
            ranges.append((opening, closing))
    return ranges


def is_checked_call(source: str, call_start: int, conditions: list[tuple[int, int]]) -> bool:
    if any(start < call_start < end for start, end in conditions):
        return True

    statement_start = max(
        source.rfind(";", 0, call_start),
        source.rfind("{", 0, call_start),
        source.rfind("}", 0, call_start),
    )
    return ASSIGNMENT_RE.search(source, statement_start + 1, call_start) is not None


def main() -> int:
    examples_dir = Path(__file__).resolve().parents[1] / "examples"
    failed = False

    missing_exclusions = sorted(
        relative_path
        for relative_path in EXCLUDED_SOURCES
        if not (examples_dir / relative_path).is_file()
    )
    for relative_path in missing_exclusions:
        print(f"FAIL {relative_path}: required excluded source is missing")
        failed = True

    for source_path in sorted(examples_dir.rglob("*.c")):
        relative_path = source_path.relative_to(examples_dir)
        source = mask_comments_and_literals(source_path.read_text(encoding="utf-8"))
        calls = list(API_CALL_RE.finditer(source))

        if relative_path in EXCLUDED_SOURCES:
            if calls:
                names = ", ".join(match.group(1) for match in calls)
                print(f"FAIL {relative_path}: excluded source calls {names}")
                failed = True
            else:
                print(f"PASS {relative_path}: excluded (no startup API calls)")
            continue

        conditions = condition_ranges(source)
        unchecked = [
            f"{match.group(1)} at line {source.count(chr(10), 0, match.start()) + 1}"
            for match in calls
            if not is_checked_call(source, match.start(), conditions)
        ]
        if unchecked:
            print(f"FAIL {relative_path}: unchecked " + ", ".join(unchecked))
            failed = True
        else:
            detail = f"{len(calls)} startup call(s) checked" if calls else "no startup API calls"
            print(f"PASS {relative_path}: {detail}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
