#!/usr/bin/env python3
"""Convert all_telop vive.csv (T0-relative TCP) to coiks path_robot_*.csv.

T0 = FK of right TCP at q=0 (vertical-arm / all-zero joints).
Matches TcpRelativeCsvWriter reconstruction:
  p_abs = p0 + p_rel
  R_abs = R0 @ R_rel
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def se3_from_xyz_quat_wxyz(xyz, quat_wxyz):
    import pinocchio as pin
    from pinocchio import Quaternion

    q = Quaternion(
        float(quat_wxyz[0]),
        float(quat_wxyz[1]),
        float(quat_wxyz[2]),
        float(quat_wxyz[3]),
    )
    q.normalize()
    return pin.SE3(q.matrix(), np.asarray(xyz, dtype=float))


def fk_t0(urdf_path: Path, ee_frame: str):
    import pinocchio as pin

    # Kinematics only; mesh package:// paths are unused.
    model = pin.buildModelFromUrdf(str(urdf_path))
    if not model.existFrame(ee_frame):
        raise SystemExit(f"URDF 中无 frame '{ee_frame}'")
    fid = model.getFrameId(ee_frame)
    data = model.createData()
    q = pin.neutral(model)  # all-zero for revolute OpenArm = vertical
    pin.framesForwardKinematics(model, data, q)
    T0 = data.oMf[fid].copy()
    print(f"[INFO] T0 from {ee_frame} at q=0: p={T0.translation.T}")
    return T0


def convert(vive_path: Path, out_path: Path, T0) -> int:
    import pinocchio as pin

    p0 = T0.translation.copy()
    R0 = T0.rotation.copy()

    with vive_path.open(newline="") as fin, out_path.open("w", newline="") as fout:
        reader = csv.DictReader(fin)
        required = ("x", "y", "z", "qw", "qx", "qy", "qz")
        if reader.fieldnames is None or any(c not in reader.fieldnames for c in required):
            raise SystemExit(
                f"vive CSV missing columns {required}; got {reader.fieldnames}"
            )
        writer = csv.writer(fout)
        writer.writerow(["x", "y", "z", "qx", "qy", "qz", "qw"])
        n = 0
        for row in reader:
            p_rel = np.array(
                [float(row["x"]), float(row["y"]), float(row["z"])], dtype=float
            )
            quat_wxyz = np.array(
                [
                    float(row["qw"]),
                    float(row["qx"]),
                    float(row["qy"]),
                    float(row["qz"]),
                ],
                dtype=float,
            )
            # Match TcpRelativeCsvWriter: p_rel = p - p0, R_rel = R0^T * R
            # Reconstruct: p = p0 + p_rel, R = R0 * R_rel  (not SE3 multiply)
            T_rel = se3_from_xyz_quat_wxyz(np.zeros(3), quat_wxyz)
            R_abs = R0 @ T_rel.rotation
            p_abs = p0 + p_rel
            quat = pin.Quaternion(R_abs)
            quat.normalize()
            xyzw = quat.coeffs()  # x,y,z,w
            writer.writerow(
                [
                    f"{p_abs[0]:.10f}",
                    f"{p_abs[1]:.10f}",
                    f"{p_abs[2]:.10f}",
                    f"{xyzw[0]:.10f}",
                    f"{xyzw[1]:.10f}",
                    f"{xyzw[2]:.10f}",
                    f"{xyzw[3]:.10f}",
                ]
            )
            n += 1
    return n


def write_qinit(out_path: Path, n_joints: int = 7) -> None:
    with out_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["problem_id"] + [f"q{i}" for i in range(1, n_joints + 1)])
        writer.writerow([0] + [0.0] * n_joints)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--vive",
        type=Path,
        default=Path("/home/vector/pika_ros/data/001/sensor/vive.csv"),
    )
    ap.add_argument(
        "--urdf",
        type=Path,
        default=ROOT / "urdf" / "openarm" / "openarm_right.urdf",
    )
    ap.add_argument(
        "--ee",
        type=str,
        default="link_ee",
        help="Frame for T0 FK (default link_ee = right TCP alias)",
    )
    ap.add_argument(
        "--out",
        type=Path,
        default=ROOT / "data" / "path_robot_15.csv",
    )
    ap.add_argument(
        "--qinit",
        type=Path,
        default=ROOT / "data" / "qinit_robot_openarm_random_10000samples.csv",
    )
    args = ap.parse_args()

    if not args.vive.is_file():
        raise SystemExit(f"找不到 vive CSV: {args.vive}")
    if not args.urdf.is_file():
        raise SystemExit(f"找不到 URDF: {args.urdf}")

    T0 = fk_t0(args.urdf, args.ee)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    n = convert(args.vive, args.out, T0)
    write_qinit(args.qinit)
    print(f"[INFO] wrote {n} poses -> {args.out}")
    print(f"[INFO] wrote qinit (7 zeros) -> {args.qinit}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
