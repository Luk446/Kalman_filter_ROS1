#!/usr/bin/env python3
"""Lightweight visualization for exported ROS bag CSVs."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CSV_FILES: Dict[str, str] = {
	"cmd_vel": "cmd_vel.csv",
	"fake_gps": "fake_gps.csv",
	"kalman_estimate": "kalman_estimate.csv",
	"odom": "odom.csv",
	"odom1": "odom1.csv",
	"imu": "imu.csv",
}

XY_COLUMNS: Iterable[Tuple[str, str]] = (
	("pose_pose_position_x", "pose_pose_position_y"),
	("pose_position_x", "pose_position_y"),
	("position_x", "position_y"),
	("x", "y"),
)

QUAT_COLUMN_SETS = (
	(
		"pose_pose_orientation_x",
		"pose_pose_orientation_y",
		"pose_pose_orientation_z",
		"pose_pose_orientation_w",
	),
	(
		"pose_orientation_x",
		"pose_orientation_y",
		"pose_orientation_z",
		"pose_orientation_w",
	),
	(
		"orientation_x",
		"orientation_y",
		"orientation_z",
		"orientation_w",
	),
)


def load_csv(path: Path) -> Optional[pd.DataFrame]:
	if not path.exists():
		print(f"[warn] Missing file: {path}")
		return None
	df = pd.read_csv(path)
	if df.empty or "time" not in df.columns:
		return df
	df = df.sort_values("time").reset_index(drop=True)
	df["t_rel"] = df["time"] - df["time"].iloc[0]
	return df


def extract_xy(df: Optional[pd.DataFrame]) -> Optional[Tuple[np.ndarray, np.ndarray]]:
	if df is None:
		return None
	for col_x, col_y in XY_COLUMNS:
		if col_x in df.columns and col_y in df.columns:
			return df[col_x].to_numpy(), df[col_y].to_numpy()
	return None


def extract_yaw(df: Optional[pd.DataFrame]) -> Optional[np.ndarray]:
	if df is None:
		return None
	for quat_columns in QUAT_COLUMN_SETS:
		if all(col in df.columns for col in quat_columns):
			qx, qy, qz, qw = (df[col].to_numpy() for col in quat_columns)
			numerator = 2.0 * (qw * qz + qx * qy)
			denominator = 1.0 - 2.0 * ((qy * qy) + (qz * qz))
			return np.arctan2(numerator, denominator)  # type: ignore[attr-defined]
	for key in ("yaw", "pose_yaw", "theta"):
		if key in df.columns:
			return df[key].to_numpy()
	return None


def interpolate_xy(df: Optional[pd.DataFrame], sample_times: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
	if df is None or "time" not in df.columns:
		return None
	xy = extract_xy(df)
	if xy is None:
		return None
	ref_times = df["time"].to_numpy()
	if ref_times.size < 2:
		return None
	x_vals, y_vals = xy
	interp_x = np.interp(sample_times, ref_times, x_vals)
	interp_y = np.interp(sample_times, ref_times, y_vals)
	return interp_x, interp_y


def squared_position_error(
	ref_df: Optional[pd.DataFrame],
	target_df: Optional[pd.DataFrame],
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
	if (
		ref_df is None
		or target_df is None
		or "time" not in target_df.columns
		or "t_rel" not in target_df.columns
	):
		return None
	target_xy = extract_xy(target_df)
	if target_xy is None:
		return None
	interp = interpolate_xy(ref_df, target_df["time"].to_numpy())
	if interp is None:
		return None
	x_ref, y_ref = interp
	x_tar, y_tar = target_xy
	diff_x = x_tar - x_ref
	diff_y = y_tar - y_ref
	squared_err = diff_x * diff_x + diff_y * diff_y
	return target_df["t_rel"].to_numpy(), squared_err


def plot_xy(ax, df, label, style="-"):
	xy = extract_xy(df)
	if xy is None:
		return
	x_vals, y_vals = xy
	ax.plot(x_vals, y_vals, style, label=label, linewidth=1.5)


def plot_series(ax, df, label, color, extractor):
	if df is None or "t_rel" not in df.columns:
		return
	values = extractor(df)
	if values is None:
		return
	ax.plot(df["t_rel"], values, color=color, label=label)


def plot_squared_error(ax, ref_df, df, label, color):
	result = squared_position_error(ref_df, df)
	if result is None:
		return
	times, errors = result
	ax.plot(times, errors, color=color, label=label)


def plot_data(datasets: Dict[str, Optional[pd.DataFrame]]) -> None:
	fig, axes = plt.subplots(1, 2, figsize=(12, 6))
	ax_xy, ax_err = axes.flatten()

	plot_xy(ax_xy, datasets.get("odom"), "ground_truth", "k--")
	plot_xy(ax_xy, datasets.get("odom1"), "odom1", "r:")
	plot_xy(ax_xy, datasets.get("fake_gps"), "fake_gps", "g^")
	plot_xy(ax_xy, datasets.get("kalman_estimate"), "kalman_estimate", "b-")
	ax_xy.set_title("Trajectories")
	ax_xy.set_xlabel("x [m]")
	ax_xy.set_ylabel("y [m]")
	ax_xy.axis("equal")
	ax_xy.grid(True)
	ax_xy.legend()

	ref = datasets.get("odom")
	plot_squared_error(ax_err, ref, datasets.get("kalman_estimate"), "kalman vs ground truth", "b")
	ax_err.set_title("Kalman squared position error")
	ax_err.set_xlabel("time [s]")
	ax_err.set_ylabel("error [m^2]")
	ax_err.grid(True)
	ax_err.legend()

	fig.tight_layout()
	plt.show()


def collect_data(base_dir: Path) -> Dict[str, Optional[pd.DataFrame]]:
	datasets: Dict[str, Optional[pd.DataFrame]] = {}
	for name, filename in CSV_FILES.items():
		path = base_dir / filename
		df = load_csv(path)
		if df is not None:
			print(f"[info] Loaded {len(df)} rows from {path}")
		elif name == "odom":
			print("[hint] Run rosbag_export.py to generate odom.csv if you want ground truth plotted.")
		datasets[name] = df
	return datasets


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Plot exported ROS bag CSVs")
	parser.add_argument(
		"base_dir",
		nargs="?",
		default="v3",
		help="Directory containing CSV exports (default: v3)",
	)
	return parser.parse_args()


def main() -> None:
	args = parse_args()
	base_dir = Path(args.base_dir).expanduser().resolve()
	print(f"[info] Using CSV directory: {base_dir}")
	datasets = collect_data(base_dir)
	plot_data(datasets)


if __name__ == "__main__":
	main()


















