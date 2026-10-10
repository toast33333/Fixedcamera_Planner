#!/usr/bin/env python3
"""Select a representative successful run for the client demo video.

The selector is read-only: it scans per-case ``summary.json`` files and
prints a ranking.  It deliberately favours a run close to the successful
group median instead of the fastest or most accurate (best-looking) run.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import statistics
from pathlib import Path


METRICS = (
    "handover_time_from_chase_s",
    "capture_time_from_chase_s",
    "radar_prediction_rmse_m",
    "post_handover_p95_fov_ratio",
)


def load_validation_module():
    """Load the client-summary gates from the sibling standalone script."""
    path = Path(__file__).with_name("summarize_full_chain_demo.py")
    spec = importlib.util.spec_from_file_location(
        "full_chain_client_validation", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load validation helpers from %s" % path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_successful_runs(root: Path, profile: str) -> list[dict]:
    validation = load_validation_module()
    runs: list[dict] = []
    for path in sorted(root.glob("*/summary.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue

        params_path = path.parent / "case_params.json"
        try:
            params = json.loads(params_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            params = {}
        if str(params.get("error_profile", "")).lower() != profile.lower():
            continue
        name = str(data.get("case_name", path.parent.name))
        candidate = dict(data)
        candidate["params"] = params
        candidate.update(validation.inspect_run_health(str(path.parent), data))
        if not validation.primary_validation_pass(candidate):
            continue
        fov_max = data.get("post_handover_max_fov_ratio")
        if not isinstance(fov_max, (int, float)) or not math.isfinite(fov_max):
            continue
        if float(fov_max) >= 1.0:
            continue
        if not all(
            isinstance(data.get(key), (int, float))
            and math.isfinite(float(data[key]))
            for key in METRICS
        ):
            continue

        item = {key: float(data[key]) for key in METRICS}
        item.update(
            {
                "case_name": name,
                "summary_path": str(path),
                "post_handover_tracking_availability_percent": data.get(
                    "post_handover_tracking_availability_percent"
                ),
                "post_handover_fov_exceedance_percent": data.get(
                    "post_handover_fov_exceedance_percent"
                ),
                "minimum_interceptor_altitude_m": candidate.get(
                    "minimum_interceptor_altitude_m"
                ),
                "capture_distance_at_success_m": data.get(
                    "capture_distance_at_success_m"
                ),
            }
        )
        runs.append(item)
    return runs


def rank_by_median_distance(runs: list[dict]) -> list[dict]:
    medians = {
        key: statistics.median(run[key] for run in runs) for key in METRICS
    }
    spans = {
        key: max(run[key] for run in runs) - min(run[key] for run in runs)
        for key in METRICS
    }

    ranked = []
    for run in runs:
        components = {
            key: (abs(run[key] - medians[key]) / spans[key] if spans[key] else 0.0)
            for key in METRICS
        }
        item = dict(run)
        item["representative_score"] = sum(components.values()) / len(METRICS)
        item["score_components"] = components
        item["group_medians"] = medians
        ranked.append(item)

    return sorted(ranked, key=lambda item: (item["representative_score"], item["case_name"]))


def select(root: Path, profile: str) -> list[dict]:
    runs = load_successful_runs(root, profile)
    if runs:
        return rank_by_median_distance(runs)
    raise SystemExit(
        "No primary-validation-passing %s-profile run with complete metrics was found."
        % profile)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_root", type=Path)
    parser.add_argument("--profile", default="field")
    parser.add_argument("--json", action="store_true", help="emit machine-readable output")
    args = parser.parse_args()

    ranked = select(args.result_root, args.profile)
    if args.json:
        print(json.dumps({"selected_profile": args.profile, "ranking": ranked}, indent=2))
        return

    print(f"selected_profile={args.profile}")
    for index, run in enumerate(ranked, 1):
        print(
            f"{index}. {run['case_name']} score={run['representative_score']:.4f} "
            f"handover={run['handover_time_from_chase_s']:.3f}s "
            f"capture={run['capture_time_from_chase_s']:.3f}s "
            f"radar_rmse={run['radar_prediction_rmse_m']:.3f}m "
            f"post_fov_p95={run['post_handover_p95_fov_ratio']:.4f}"
        )
    print(f"SELECTED_CASE={ranked[0]['case_name']}")


if __name__ == "__main__":
    main()
