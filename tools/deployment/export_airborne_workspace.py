#!/usr/bin/env python3
"""Export a minimal ROS1 workspace for the aircraft computer.

The development repository deliberately contains simulation and analysis
assets.  This exporter uses an explicit allowlist so those files cannot leak
into an aircraft deployment through a recursive copy.
"""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
from typing import Iterable


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
DEPLOY_ROOT = REPO_ROOT / "deploy" / "airborne"
MANIFEST_PATH = DEPLOY_ROOT / "airborne_files.txt"
TEMPLATES_ROOT = DEPLOY_ROOT / "templates"

PACKAGE_TEMPLATES = {
    "detection_msgs": ("CMakeLists.txt", "package.xml"),
    "kcf_msgs": ("CMakeLists.txt", "package.xml"),
    "png_planner": ("CMakeLists.txt", "package.xml"),
}

FORBIDDEN_PARTS = {
    "build",
    "devel",
    "docs",
    "log",
    "results",
    "simulation",
    "tests",
    "third_party",
    "仿真结果",
}
FORBIDDEN_NAME_FRAGMENTS = (
    "multisim",
    "sitl",
    "_old",
    ".backup",
    "test_publisher",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a simulation-free aircraft catkin workspace."
    )
    parser.add_argument(
        "output",
        type=Path,
        help="new output directory, for example /tmp/fixed_camera_airborne_ws",
    )
    parser.add_argument(
        "--archive",
        type=Path,
        help="optional .tar.gz path written after validation",
    )
    return parser.parse_args()


def manifest_entries() -> list[Path]:
    entries: list[Path] = []
    for raw_line in MANIFEST_PATH.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        path = Path(line)
        if path.is_absolute() or ".." in path.parts:
            raise RuntimeError(f"unsafe manifest entry: {line}")
        entries.append(path)
    if not entries:
        raise RuntimeError("airborne manifest is empty")
    return entries


def git_revision() -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def copy_file(source: Path, destination: Path) -> None:
    if not source.is_file():
        raise FileNotFoundError(f"required airborne file is missing: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def iter_files(root: Path) -> Iterable[Path]:
    return sorted(path for path in root.rglob("*") if path.is_file())


def validate_export(workspace: Path) -> list[Path]:
    files = list(iter_files(workspace))
    if not files:
        raise RuntimeError("airborne workspace contains no files")

    for path in files:
        relative = path.relative_to(workspace)
        lowered_parts = {part.lower() for part in relative.parts}
        if lowered_parts & FORBIDDEN_PARTS:
            raise RuntimeError(f"forbidden deployment directory: {relative}")
        lower_name = path.name.lower()
        if any(fragment in lower_name for fragment in FORBIDDEN_NAME_FRAGMENTS):
            raise RuntimeError(f"forbidden deployment file: {relative}")

    models = [path for path in files if path.suffix == ".rknn"]
    if len(models) != 1 or models[0].name != "yolov8-NewDrone.rknn":
        raise RuntimeError(
            "airborne workspace must contain exactly yolov8-NewDrone.rknn"
        )

    required = (
        workspace / "src" / "CMakeLists.txt",
        workspace
        / "src"
        / "Fixedcamera_Planner"
        / "src"
        / "detection_msgs"
        / "msg"
        / "Detection.msg",
        workspace
        / "src"
        / "Fixedcamera_Planner"
        / "src"
        / "png_planner"
        / "src"
        / "fixed_camera_ibvs.cpp",
        workspace
        / "src"
        / "Fixedcamera_Planner"
        / "src"
        / "png_planner"
        / "src"
        / "radar_approach_handover.cpp",
    )
    missing = [path for path in required if not path.is_file()]
    if missing:
        raise RuntimeError(f"incomplete airborne workspace: {missing}")
    return files


def write_checksums(workspace: Path, files: Iterable[Path]) -> None:
    checksum_path = workspace / "AIRBORNE_SHA256SUMS.txt"
    lines = []
    for path in files:
        if path == checksum_path:
            continue
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        relative = path.relative_to(workspace).as_posix()
        lines.append(f"{digest}  {relative}")
    checksum_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def make_archive(workspace: Path, archive: Path) -> None:
    archive = archive.resolve()
    if archive.exists():
        raise FileExistsError(f"archive already exists: {archive}")
    archive.parent.mkdir(parents=True, exist_ok=True)
    def portable_permissions(info: tarfile.TarInfo) -> tarfile.TarInfo:
        # Windows does not preserve POSIX execute bits.  Set deterministic
        # modes in the transfer archive so field scripts run immediately
        # after extraction on the aircraft.
        if info.isdir():
            info.mode = 0o755
        elif Path(info.name).suffix in {".py", ".sh"}:
            info.mode = 0o755
        else:
            info.mode = 0o644
        return info

    with tarfile.open(archive, "w:gz") as handle:
        handle.add(
            workspace,
            arcname=workspace.name,
            filter=portable_permissions,
        )


def main() -> int:
    args = parse_args()
    output = args.output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(
            f"output already exists; choose a new empty path: {output}"
        )

    repository_destination = output / "src" / "Fixedcamera_Planner"
    try:
        for relative in manifest_entries():
            copy_file(REPO_ROOT / relative, repository_destination / relative)

        copy_file(
            TEMPLATES_ROOT / "workspace_CMakeLists.txt",
            output / "src" / "CMakeLists.txt",
        )
        for package, names in PACKAGE_TEMPLATES.items():
            package_destination = (
                repository_destination / "src" / package
            )
            for name in names:
                copy_file(
                    TEMPLATES_ROOT / package / name,
                    package_destination / name,
                )

        copy_file(
            DEPLOY_ROOT / "README.md",
            output / "README_AIRBORNE.md",
        )
        copy_file(
            DEPLOY_ROOT / "build_airborne.sh",
            output / "build_airborne.sh",
        )
        copy_file(
            DEPLOY_ROOT / "verify_airborne.sh",
            output / "verify_airborne.sh",
        )

        for path in iter_files(output):
            if path.suffix in {".py", ".sh"}:
                path.chmod(path.stat().st_mode | 0o111)

        revision = git_revision()
        (output / "AIRBORNE_BUILD_INFO.txt").write_text(
            "source_repository=https://github.com/toast33333/Fixedcamera_Planner.git\n"
            f"source_revision={revision}\n"
            "profile=px4_mavros_rk3588\n",
            encoding="utf-8",
        )

        files = validate_export(output)
        write_checksums(output, files)
        if args.archive:
            make_archive(output, args.archive)
    except Exception:
        # The output was newly created by this invocation, so removing only
        # this exact resolved path is safe and avoids leaving a partial bundle.
        if output.exists():
            shutil.rmtree(output)
        raise

    total_bytes = sum(path.stat().st_size for path in iter_files(output))
    print(f"airborne workspace: {output}")
    print(f"files: {len(list(iter_files(output)))}")
    print(f"size: {total_bytes / (1024 * 1024):.2f} MiB")
    if args.archive:
        print(f"archive: {args.archive.resolve()}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:  # concise CLI failure for field use
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
