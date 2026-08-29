#!/usr/bin/env python3
"""Generate CMakePresets.json from Boards/<board>/App/<app>/CMakeLists.txt."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PRESETS_PATH = ROOT / "CMakePresets.json"
BOARD_DIR = ROOT / "Boards"
BUILD_TYPES = ("Debug", "Release")


def discover_apps() -> list[tuple[str, str]]:
    apps: list[tuple[str, str]] = []
    for board_path in sorted(p for p in BOARD_DIR.iterdir() if p.is_dir()):
        toolchain = board_path / "cmake" / "gcc-arm-none-eabi.cmake"
        app_root = board_path / "App"
        if not toolchain.is_file() or not app_root.is_dir():
            continue

        for app_path in sorted(p for p in app_root.iterdir() if p.is_dir()):
            if (app_path / "CMakeLists.txt").is_file():
                apps.append((board_path.name, app_path.name))

    return apps


def make_presets(apps: list[tuple[str, str]]) -> dict:
    configure_presets: list[dict] = [
        {
            "name": "base",
            "hidden": True,
            "generator": "Ninja",
            "binaryDir": "${sourceDir}/build/${presetName}",
        }
    ]
    build_presets: list[dict] = []

    for board, app in apps:
        for build_type in BUILD_TYPES:
            name = f"{board}-{app}-{build_type}"
            configure_presets.append(
                {
                    "name": name,
                    "inherits": "base",
                    "toolchainFile": f"${{sourceDir}}/Boards/{board}/cmake/gcc-arm-none-eabi.cmake",
                    "cacheVariables": {
                        "CMAKE_BUILD_TYPE": build_type,
                        "ACTIVE_APP": app,
                    },
                }
            )
            build_presets.append({"name": name, "configurePreset": name})

    return {
        "version": 3,
        "configurePresets": configure_presets,
        "buildPresets": build_presets,
    }


def encode_presets(data: dict) -> str:
    return json.dumps(data, indent=4, ensure_ascii=False) + "\n"


def read_normalized(path: Path) -> str:
    if not path.is_file():
        return ""
    return path.read_text(encoding="utf-8").replace("\r\n", "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--check",
        action="store_true",
        help="exit with non-zero status if CMakePresets.json is not up to date",
    )
    args = parser.parse_args()

    apps = discover_apps()
    if not apps:
        print("No apps found under Boards/<board>/App/<app>/CMakeLists.txt", file=sys.stderr)
        return 1

    generated = encode_presets(make_presets(apps))

    if args.check:
        current = read_normalized(PRESETS_PATH)
        if current != generated:
            print("CMakePresets.json is out of date. Run scripts/generate_cmake_presets.py", file=sys.stderr)
            return 1
        return 0

    PRESETS_PATH.write_text(generated, encoding="utf-8", newline="\n")
    print(f"Generated {PRESETS_PATH.relative_to(ROOT)} with {len(apps) * len(BUILD_TYPES)} presets.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
