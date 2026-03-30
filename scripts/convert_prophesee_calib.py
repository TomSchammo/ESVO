#!/usr/bin/env python3
"""
Convert Prophesee calibration (JSON) to ESVO format (YAML).

Convention:
  - master (cam_0) = left
  - slave (cam_1) = right
  - T_slave_to_master = T_right_left

Usage:
  python convert_prophesee_calib.py \
    --master-intrinsics master_intrinsics.json \
    --slave-intrinsics slave_intrinsics.json \
    --extrinsics extrinsics.json \
    --output-dir ./output
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_intrinsics(path: Path) -> dict:
    """Load Prophesee intrinsics JSON."""
    with open(path) as f:
        data = json.load(f)

    K = np.array(data["K"]).reshape(3, 3)

    assert np.array_equal(K[-1][:], np.array(
        [0, 0, 1], dtype=np.float32)), "Coeffiecent matrix is malformed"

    D = np.array(data["D"])
    width = data["width"]
    height = data["height"]
    model_type = data.get("type", "pinhole")

    return {
        "K": K,
        "D": D,
        "width": width,
        "height": height,
        "type": model_type,
    }


def load_extrinsics(path: Path) -> dict:
    """Load Prophesee extrinsics JSON (T_slave_to_master)."""
    with open(path) as f:
        data = json.load(f)

    # NOTE(tom): in stereo setup the first is the only slave entry
    assert len(data["T_slave_master"]) >= 1, "Extrinsics are misformed, expected at least one slave entry"

    if len(data["T_slave_master"]) > 1:
        # TODO color highlight
        print(f"Found {len(data["T_slave_master"])} slave entries! Will  ignore the last {len(data["T_slave_master"])-1}")

    slave_data = data["T_slave_master"][0]

    rvec = np.array(slave_data["rvec"])
    tvec = np.array(slave_data["tvec"])

    return {"rvec": rvec, "tvec": tvec}


def compute_stereo_rectification(
    K_left: np.ndarray,
    D_left: np.ndarray,
    K_right: np.ndarray,
    D_right: np.ndarray,
    R: np.ndarray,
    T: np.ndarray,
    image_size: tuple,
) -> dict:
    """
    Compute stereo rectification parameters.

    Args:
        K_left, K_right: 3x3 camera matrices
        D_left, D_right: distortion coefficients
        R: 3x3 rotation matrix (right to left)
        T: 3x1 translation vector (right to left)
        image_size: (width, height)

    Returns:
        Dictionary with R1, R2, P1, P2, Q
    """
    # pinhole model
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K_left, D_left,
        K_right, D_right,
        image_size,
        R, T,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0,  # 0 = all pixels valid, 1 = no black borders
    )

    return {
        "R1": R1,
        "R2": R2,
        "P1": P1,
        "P2": P2,
        "Q": Q,
    }


def build_T_matrix(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Build 3x4 transformation matrix [R | t]."""
    T = np.hstack([R, t.reshape(3, 1)])
    return T


def matrix_to_yaml_list(mat: np.ndarray) -> list:
    """Convert numpy matrix to flat list for YAML (row-major)."""
    return mat.flatten().tolist()


def write_esvo_yaml(
    path: Path,
    width: int,
    height: int,
    camera_name: str,
    distortion_model: str,
    K: np.ndarray,
    D: np.ndarray,
    R_rect: np.ndarray,
    P: np.ndarray,
    T_right_left: np.ndarray,
):
    """Write ESVO-compatible calibration YAML."""

    # ESVO expects 4 distOptional coefficients for both models, so the last one is truncated
    D_out = D[:4] if len(D) > 4 else D

    data = {
        "image_width": width,
        "image_height": height,
        "camera_name": camera_name,
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": matrix_to_yaml_list(K),
        },
        "distortion_model": distortion_model,
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(D_out),
            "data": D_out.tolist(),
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": matrix_to_yaml_list(R_rect),
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": matrix_to_yaml_list(P),
        },
    }

    data["T_right_left"] = {
        "rows": 3,
        "cols": 4,
        "data": matrix_to_yaml_list(T_right_left),
    }

    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=None, sort_keys=False)

    print(f"Written: {path}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert Prophesee calibration to ESVO format"
    )
    parser.add_argument(
        "--master-intrinsics", "-m",
        type=Path,
        required=True,
        help="Path to master (left) camera intrinsics JSON",
    )
    parser.add_argument(
        "--slave-intrinsics", "-s",
        type=Path,
        required=True,
        help="Path to slave (right) camera intrinsics JSON",
    )
    parser.add_argument(
        "--extrinsics", "-e",
        type=Path,
        required=True,
        help="Path to extrinsics JSON (T_slave_master)",
    )
    parser.add_argument(
        "--output-dir", "-o",
        type=Path,
        default=Path("."),
        help="Output directory for left.yaml and right.yaml",
    )
    parser.add_argument(
        "--camera-name-prefix",
        type=str,
        default="evk4",
        help="Prefix for camera names in output YAML",
    )

    args = parser.parse_args()

    # load calibration data
    print(f"Loading master intrinsics from: {args.master_intrinsics}")
    master = load_intrinsics(args.master_intrinsics)

    assert master["type"] == "pinhole", "expected pinhole model"

    print(f"Loading slave intrinsics from: {args.slave_intrinsics}")
    slave = load_intrinsics(args.slave_intrinsics)

    print(f"Loading extrinsics from: {args.extrinsics}")
    extrinsics = load_extrinsics(args.extrinsics)

    # convert rvec to rotation matrix
    R, _ = cv2.Rodrigues(extrinsics["rvec"])
    T = extrinsics["tvec"]

    print(f"\nRotation matrix (R_slave_to_master):\n{R}")
    print(f"\nTranslation vector (t_slave_to_master):\n{T}")

    # build T_right_left (= T_slave_to_master)
    T_right_left = build_T_matrix(R, T)
    print(f"\nT_right_left (3x4):\n{T_right_left}")

    # Image size (should be same for both cameras)
    image_size = (master["width"], master["height"])
    print(f"Image size: {image_size}")

    # compute stereo rectification
    # convention: master=left, slave=right
    # R, T transform from right (slave) to left (master)
    print("\nComputing stereo rectification...")
    rect = compute_stereo_rectification(
        K_left=master["K"],
        D_left=master["D"],
        K_right=slave["K"],
        D_right=slave["D"],
        R=R,
        T=T,
        image_size=image_size,
    )

    print(f"\nR1 (left rectification):\n{rect['R1']}")
    print(f"\nR2 (right rectification):\n{rect['R2']}")
    print(f"\nP1 (left projection):\n{rect['P1']}")
    print(f"\nP2 (right projection):\n{rect['P2']}")

    # compute baseline from P2
    # P2[0,3] = -fx * baseline, so baseline = -P2[0,3] / P2[0,0]
    baseline = -rect["P2"][0, 3] / rect["P2"][0, 0]
    print(f"\nBaseline: {baseline:.6f} m")

    # create output directory
    args.output_dir.mkdir(parents=True, exist_ok=True)

    # write left.yaml (master)
    write_esvo_yaml(
        path=args.output_dir / "left.yaml",
        width=master["width"],
        height=master["height"],
        camera_name=f"{args.camera_name_prefix}_left",
        distortion_model="plumb_bob",
        K=master["K"],
        D=master["D"],
        R_rect=rect["R1"],
        P=rect["P1"],
        T_right_left=T_right_left,
    )

    # write right.yaml (slave)
    write_esvo_yaml(
        path=args.output_dir / "right.yaml",
        width=slave["width"],
        height=slave["height"],
        camera_name=f"{args.camera_name_prefix}_right",
        distortion_model="plumb_bob",
        K=slave["K"],
        D=slave["D"],
        R_rect=rect["R2"],
        P=rect["P2"],
        T_right_left=T_right_left,
    )

    print(f"\nDone! Output written to: {args.output_dir}")
    print("\nConvention used:")
    print("  - master (cam_0) = left")
    print("  - slave (cam_1) = right")
    print("  - T_right_left = T_slave_to_master")


if __name__ == "__main__":
    main()
