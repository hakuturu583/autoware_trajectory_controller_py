"""Default parameters of the wrapped controllers.

The defaults are the upstream ``*.param.yaml`` files shipped with
``autoware_trajectory_follower_node`` and ``autoware_vehicle_info_utils`` (installed in
``config/`` next to this module), so the controllers behave exactly like a default
Autoware launch.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml

from . import _core

# The YAML files are installed next to the compiled module (this also works for
# editable installs, where the Python sources stay in the source tree).
CONFIG_DIR = Path(_core.__file__).resolve().parent / "config"

LATERAL_MODES = ("mpc", "pure_pursuit")
LONGITUDINAL_MODES = ("pid",)

# Parameters that Autoware normally loads from shared launch configuration
# (autoware_launch: common/nearest_search.param.yaml).
NEAREST_SEARCH_DEFAULTS: dict[str, Any] = {
    "ego_nearest_dist_threshold": 3.0,  # [m]
    "ego_nearest_yaw_threshold": 1.046,  # [rad] = 60 [deg]
}


def flatten(params: Mapping[str, Any], prefix: str = "") -> dict[str, Any]:
    """Flatten nested dicts into ROS 2 style dotted parameter names."""
    out: dict[str, Any] = {}
    for key, value in params.items():
        name = f"{prefix}{key}"
        if isinstance(value, Mapping):
            out.update(flatten(value, prefix=f"{name}."))
        else:
            out[name] = value
    return out


def load_param_file(path: str | Path) -> dict[str, Any]:
    """Load a ROS 2 parameter YAML file (``/**: ros__parameters: ...``) as a flat dict."""
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    out: dict[str, Any] = {}
    if isinstance(data, Mapping) and any(
        isinstance(v, Mapping) and "ros__parameters" in v for v in data.values()
    ):
        for node_params in data.values():
            if isinstance(node_params, Mapping):
                out.update(flatten(node_params.get("ros__parameters", {})))
    elif isinstance(data, Mapping):
        out.update(flatten(data))
    return out


def vehicle_info_parameters() -> dict[str, Any]:
    """Default vehicle description (the Autoware sample vehicle)."""
    return load_param_file(CONFIG_DIR / "vehicle_info.param.yaml")


def lateral_parameters(mode: str = "mpc") -> dict[str, Any]:
    if mode not in LATERAL_MODES:
        raise ValueError(f"lateral mode must be one of {LATERAL_MODES}, got {mode!r}")
    return load_param_file(CONFIG_DIR / f"{mode}.param.yaml")


def longitudinal_parameters(mode: str = "pid") -> dict[str, Any]:
    if mode not in LONGITUDINAL_MODES:
        raise ValueError(f"longitudinal mode must be one of {LONGITUDINAL_MODES}, got {mode!r}")
    return load_param_file(CONFIG_DIR / f"{mode}.param.yaml")


def common_parameters() -> dict[str, Any]:
    params = load_param_file(CONFIG_DIR / "trajectory_follower_node.param.yaml")
    params.update(NEAREST_SEARCH_DEFAULTS)
    return params


def default_parameters(
    lateral: str | None = "mpc",
    longitudinal: str | None = "pid",
    overrides: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Assemble the full parameter set for a controller.

    Args:
        lateral: ``"mpc"``, ``"pure_pursuit"`` or ``None`` (no lateral parameters).
        longitudinal: ``"pid"`` or ``None`` (no longitudinal parameters).
        overrides: values that replace the defaults. Nested dicts are flattened, so
            ``{"steering_offset": {"max_update_th": 0.02}}`` is the same as
            ``{"steering_offset.max_update_th": 0.02}``.
    """
    params = common_parameters()
    params.update(vehicle_info_parameters())
    if lateral is not None:
        params.update(lateral_parameters(lateral))
        params["lateral_controller_mode"] = lateral
    if longitudinal is not None:
        params.update(longitudinal_parameters(longitudinal))
        params["longitudinal_controller_mode"] = longitudinal
    if overrides:
        params.update(flatten(overrides))
    return params
