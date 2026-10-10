#!/usr/bin/env python3
"""Download the Autoware sources wrapped by this package.

Only the files listed in ``PACKAGES`` are fetched, at the commits pinned in ``PINS``,
using a shallow partial clone with sparse checkout (a few MB instead of the full
repositories). CMake runs this script at configure time, so ``uv sync`` needs nothing
else; it can also be run by hand::

    python scripts/fetch_upstream.py build/third_party

To update Autoware, change the commits in ``PINS`` (and ``PACKAGES`` if needed).
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

# repository key -> (URL, pinned commit)
PINS: dict[str, tuple[str, str]] = {
    "universe": (
        "https://github.com/autowarefoundation/autoware_universe",
        "dc1fcea1d7ccae851466e7c4db3843c08304c6a5",
    ),
    "core": (
        "https://github.com/autowarefoundation/autoware_core",
        "e0a8ab847413b73d9a48e13369943ae90cd4859a",
    ),
    "utils": (
        "https://github.com/autowarefoundation/autoware_utils",
        "6b89a2e1d9151f1c94d3336b6b8fa54bb50c9217",
    ),
}

# (repository key, package path inside the repository, directories / files to fetch).
# Only what CMakeLists.txt compiles (or the upstream tests read) is fetched.
PACKAGES: list[tuple[str, str, tuple[str, ...]]] = [
    ("universe", "control/autoware_trajectory_follower_base", ("include", "src")),
    ("universe", "control/autoware_mpc_lateral_controller", ("include", "src", "param", "test")),
    ("universe", "control/autoware_pid_longitudinal_controller", ("include", "src", "test")),
    ("universe", "control/autoware_pure_pursuit", ("include", "src")),
    # Parameters, plus the node sources that src/trajectory_follower.cpp mirrors (not
    # compiled; fetched so that upstream changes show up when the pin is bumped).
    ("universe", "control/autoware_trajectory_follower_node", ("include", "src", "param")),
    ("core", "common/autoware_interpolation", ("include", "src")),
    (
        "core",
        "common/autoware_motion_utils",
        (
            "include",
            "src/distance",
            "src/marker",
            "src/resample",
            "src/trajectory/conversion.cpp",
            "src/trajectory/interpolation.cpp",
            "src/trajectory/path_with_lane_id.cpp",
            "src/trajectory/trajectory.cpp",
        ),
    ),
    ("core", "common/autoware_osqp_interface", ("include", "src")),
    ("core", "common/autoware_vehicle_info_utils", ("include", "src", "config")),
    ("utils", "autoware_utils", ("include",)),
    (
        "utils",
        "autoware_utils_geometry",
        ("include", "src/geometry/geometry.cpp", "src/geometry/pose_deviation.cpp"),
    ),
    ("utils", "autoware_utils_math", ("include",)),
    ("utils", "autoware_utils_system", ("include", "src")),
    ("utils", "autoware_utils_rclcpp", ("include",)),
    ("utils", "autoware_utils_visualization", ("include", "src")),
]


def git(*args: str, cwd: Path) -> None:
    subprocess.run(["git", *args], cwd=cwd, check=True)


def sparse_checkout(url: str, commit: str, paths: list[str], workdir: Path) -> None:
    """Check out only ``paths`` of ``commit`` into ``workdir``."""
    git("init", "-q", cwd=workdir)
    git("remote", "add", "origin", url, cwd=workdir)
    git("sparse-checkout", "set", "--no-cone", *paths, cwd=workdir)
    git("fetch", "-q", "--depth", "1", "--filter=blob:none", "origin", commit, cwd=workdir)
    git("checkout", "-q", "FETCH_HEAD", cwd=workdir)


def fetch(dest: Path) -> None:
    stamp = dest / ".stamp.json"
    expected = json.dumps({"pins": PINS, "packages": PACKAGES}, sort_keys=True)
    if stamp.exists() and stamp.read_text() == expected:
        return

    if dest.exists():
        shutil.rmtree(dest)
    for key, (url, commit) in PINS.items():
        packages = [(pkg, items) for k, pkg, items in PACKAGES if k == key]
        paths = ["/LICENSE"] + [f"/{pkg}/{item}" for pkg, items in packages for item in items]
        print(f"-- Fetching {url} @ {commit[:10]}")
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp)
            sparse_checkout(url, commit, paths, src)
            (dest / key).mkdir(parents=True)
            shutil.copy2(src / "LICENSE", dest / key / "LICENSE")
            for pkg, _ in packages:
                # Flatten "control/<pkg>" to "<key>/<pkg>" (layout used by CMakeLists.txt).
                shutil.copytree(src / pkg, dest / key / Path(pkg).name)
    stamp.write_text(expected)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dest", type=Path, help="output directory")
    fetch(parser.parse_args().dest)


if __name__ == "__main__":
    main()
