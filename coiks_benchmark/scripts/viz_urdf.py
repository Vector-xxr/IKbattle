#!/usr/bin/env python3
"""用 Pinocchio + MeshCat 可视化 coiks_benchmark 基准数据。

模式:
  - csv:    播放 data/path_robot_<id>.csv 连续末端轨迹（简易 IK）
  - random: 交叉显示 qinit/targets 第 index..index+count-1 题
  - neutral: 中性位静态

依赖::

    pip install -r scripts/requirements-viz.txt

示例::

    cd coiks_benchmark
    python3 scripts/viz_urdf.py --robot_id 7 --mode csv --index 0 --count 200
    python3 scripts/viz_urdf.py --robot_id 7 --mode random --index 10 --count 5 --dt 0.5
    python3 scripts/viz_urdf.py --robot_id 1 --mode neutral
"""

from __future__ import annotations

import argparse
import atexit
import csv
import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]  # coiks_benchmark/
URDF_ROOT = ROOT / "urdf"
DATA_ROOT = ROOT / "data"

# 与 benchmark.cpp 中 robot_id → URDF / 数据名一致
ROBOTS = {
    0: ("toyota/hsr/hsr_arm.urdf", "hsr"),
    1: ("denso/vs050/vs050.urdf", "vs050"),
    2: ("ufactory/xarm6/xarm6.urdf", "xarm6"),
    3: ("abb/irb140/irb140.urdf", "irb140"),
    4: ("kinova/jaco/jaco_arm.urdf", "jaco"),
    5: ("ur/ur5/ur5.urdf", "ur5"),
    6: ("ur/ur10/ur10.urdf", "ur10"),
    7: ("kinova/gen3/gen3.urdf", "gen3"),
    8: ("kuka/iiwa/iiwa.urdf", "iiwa"),
    9: ("ufactory/xarm7/xarm7.urdf", "xarm7"),
    10: ("franka/panda.urdf", "panda"),
    11: ("rethink/sawyer/sawyer.urdf", "sawyer"),
    12: ("abb/yumi/yumi.urdf", "yumi"),
    13: ("fetch/fetch_arm.urdf", "fetch"),
    14: ("tiago/tiago.urdf", "tiago"),
}

OLD_MESH_PREFIXES = (
    "/home/colan/simulations/URDFs/robots/",
)

EE_FRAME = "link_ee"
IK_MAX_ITER = 50
IK_TOL = 1e-4
IK_DAMP = 1e-4


def rewrite_mesh_paths(urdf_path: Path) -> Path:
    """将绝对 mesh 路径改写到本仓库 urdf/，写入临时文件。"""
    text = urdf_path.read_text(encoding="utf-8", errors="ignore")
    urdf_root = str(URDF_ROOT.resolve()) + "/"
    for old in OLD_MESH_PREFIXES:
        text = text.replace(old, urdf_root)
    text = text.replace("meshes//", "meshes/")

    fd, tmp_name = tempfile.mkstemp(prefix="coiks_viz_", suffix=".urdf")
    os.close(fd)
    tmp_path = Path(tmp_name)
    tmp_path.write_text(text, encoding="utf-8")
    atexit.register(lambda p=tmp_path: p.unlink(missing_ok=True))
    return tmp_path


def resolve_robot(robot_id: int) -> tuple[Path, str]:
    if robot_id not in ROBOTS:
        raise SystemExit(
            f"robot_id 须在 0..14，收到 {robot_id}。可选: "
            + ", ".join(f"{i}:{ROBOTS[i][0]}" for i in ROBOTS)
        )
    rel, key = ROBOTS[robot_id]
    urdf = (URDF_ROOT / rel).resolve()
    if not urdf.is_file():
        raise FileNotFoundError(f"找不到 URDF: {urdf}")
    return urdf, key


def resolve_urdf_path(args) -> tuple[Path, str | None, int]:
    """返回 (urdf_path, name_key_or_None, robot_id)。"""
    robot_id = args.robot_id
    if args.urdf:
        p = Path(args.urdf)
        if not p.is_file():
            p = ROOT / args.urdf
        if not p.is_file():
            raise FileNotFoundError(f"找不到 URDF: {args.urdf}")
        key = ROBOTS[robot_id][1] if robot_id in ROBOTS else None
        return p.resolve(), key, robot_id

    urdf, key = resolve_robot(robot_id)
    return urdf, key, robot_id


def load_path(robot_id: int):
    """读 path_robot_<id>.csv → list of (xyz, quat_xyzw)."""
    import numpy as np

    path = DATA_ROOT / f"path_robot_{robot_id}.csv"
    if not path.is_file():
        raise FileNotFoundError(f"找不到路径 CSV: {path}")

    poses = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            xyz = np.array(
                [float(row["x"]), float(row["y"]), float(row["z"])], dtype=float
            )
            # 文件列为 qx,qy,qz,qw
            quat_xyzw = np.array(
                [
                    float(row["qx"]),
                    float(row["qy"]),
                    float(row["qz"]),
                    float(row["qw"]),
                ],
                dtype=float,
            )
            poses.append((xyz, quat_xyzw))
    if not poses:
        raise ValueError(f"路径 CSV 为空: {path}")
    return poses


def load_qinit(name_key: str):
    import numpy as np

    csv_path = DATA_ROOT / f"qinit_robot_{name_key}_random_10000samples.csv"
    if not csv_path.is_file():
        raise FileNotFoundError(f"找不到 qinit CSV: {csv_path}")

    rows = []
    with csv_path.open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        q_cols = [i for i, h in enumerate(header) if h.startswith("q")]
        if not q_cols:
            raise ValueError(f"qinit 无关节列: {csv_path}")
        for row in reader:
            rows.append([float(row[i]) for i in q_cols])
    return np.asarray(rows, dtype=float)


def load_target_q(name_key: str, nq: int):
    """读 targets 中位姿后的关节列 q1..qn。"""
    import numpy as np

    csv_path = DATA_ROOT / f"targets_robot_{name_key}_random_10000samples.csv"
    if not csv_path.is_file():
        raise FileNotFoundError(f"找不到 targets CSV: {csv_path}")

    with csv_path.open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        try:
            q1_idx = header.index("q1")
        except ValueError as e:
            raise ValueError(
                f"targets 缺少关节列 q1..（文件 {csv_path.name}，列={header}）"
            ) from e
        rows = []
        for row in reader:
            vals = [float(x) for x in row[q1_idx : q1_idx + nq]]
            if len(vals) != nq:
                raise ValueError(
                    f"targets 关节数列不足 nq={nq}（文件 {csv_path.name}）"
                )
            rows.append(vals)
    return np.asarray(rows, dtype=float)


def clamp_slice(index: int, count: int | None, n: int, default_count: int):
    """返回合法 [start, end) 与实际 count。"""
    if n <= 0:
        raise ValueError("数据长度为 0")
    if index < 0 or index >= n:
        raise IndexError(f"index={index} 越界（有效 0..{n - 1}）")
    remain = n - index
    if count is None:
        count = min(default_count, remain)
    else:
        count = max(0, min(count, remain))
    if count == 0:
        raise ValueError("count 截断后为 0")
    return index, index + count, count


def se3_from_xyz_quat_xyzw(xyz, quat_xyzw):
    import pinocchio as pin
    from pinocchio import Quaternion
    import numpy as np

    q = Quaternion(
        float(quat_xyzw[3]),
        float(quat_xyzw[0]),
        float(quat_xyzw[1]),
        float(quat_xyzw[2]),
    )
    q.normalize()
    return pin.SE3(q.matrix(), np.asarray(xyz, dtype=float))


def find_ee_frame_id(model):
    import pinocchio as pin

    if model.existFrame(EE_FRAME):
        return model.getFrameId(EE_FRAME)
    # 回退：最后一个 frame
    print(
        f"[WARN] 未找到 frame '{EE_FRAME}'，使用最后一个 frame id={model.nframes - 1}",
        file=sys.stderr,
    )
    return model.nframes - 1


def ik_to_pose(model, data, frame_id, q, X_des, max_iter=IK_MAX_ITER, tol=IK_TOL):
    """阻尼伪逆迭代 IK。成功返回 (True, q_new)，失败 (False, q)。"""
    import pinocchio as pin
    import numpy as np

    q = np.asarray(q, dtype=float).copy()
    for _ in range(max_iter):
        pin.framesForwardKinematics(model, data, q)
        pin.computeJointJacobians(model, data, q)
        oMf = data.oMf[frame_id]
        iMd = oMf.actInv(X_des)
        err = pin.log(iMd).vector
        if np.linalg.norm(err) < tol:
            return True, q
        J = pin.computeFrameJacobian(
            model, data, q, frame_id, pin.ReferenceFrame.LOCAL
        )
        JJt = J @ J.T
        damp = IK_DAMP * np.eye(6)
        v = J.T @ np.linalg.solve(JJt + damp, err)
        q = pin.integrate(model, q, v)
    pin.framesForwardKinematics(model, data, q)
    oMf = data.oMf[frame_id]
    err = pin.log(oMf.actInv(X_des)).vector
    ok = np.linalg.norm(err) < tol * 10
    return ok, q


def print_viewer_url(viz) -> None:
    if not hasattr(viz, "viewer") or viz.viewer is None:
        return
    url = getattr(viz.viewer, "url", None)
    if callable(url):
        try:
            url = url()
        except TypeError:
            pass
    if url is None:
        url = str(viz.viewer)
    print(f"[INFO] MeshCat: {url}")


def wait_exit() -> None:
    try:
        input("按回车退出...")
    except EOFError:
        pass


def run_path_animation(
    viz,
    model,
    data,
    frame_id,
    robot_id: int,
    name_key: str | None,
    index: int,
    count: int | None,
    dt: float,
):
    import pinocchio as pin
    import numpy as np

    poses = load_path(robot_id)
    start, end, n_play = clamp_slice(index, count, len(poses), default_count=200)
    print(
        f"[INFO] path 模式: path_robot_{robot_id}.csv "
        f"点 [{start}, {end}) 共 {n_play} 个，dt={dt}"
    )

    if name_key is not None:
        try:
            qinit = load_qinit(name_key)
            q = qinit[0].copy()
            print(f"[INFO] 路径 IK 种子: qinit[0]")
        except Exception as e:
            print(f"[WARN] 无法读 qinit，改用 neutral: {e}", file=sys.stderr)
            q = pin.neutral(model)
    else:
        q = pin.neutral(model)

    if q.shape[0] != model.nq:
        raise ValueError(f"种子 q 维数 {q.shape[0]} != nq={model.nq}")

    viz.display(q)
    fail = 0
    for k, (xyz, quat) in enumerate(poses[start:end]):
        X_des = se3_from_xyz_quat_xyzw(xyz, quat)
        ok, q_new = ik_to_pose(model, data, frame_id, q, X_des)
        if ok:
            q = q_new
            viz.display(q)
        else:
            fail += 1
            print(f"[WARN] path IK failed at k={start + k}", file=sys.stderr)
        if (k + 1) % 20 == 0 or k == 0:
            print(f"[path] k={start + k}/{end - 1}")
        time.sleep(dt)

    print(f"[INFO] path 播放结束（失败 {fail}/{n_play}）")
    wait_exit()


def run_random_toggle(
    viz,
    model,
    name_key: str,
    index: int,
    count: int | None,
    dt: float,
):
    import numpy as np

    qinit = load_qinit(name_key)
    q_goal = load_target_q(name_key, model.nq)
    n = min(len(qinit), len(q_goal))
    if len(qinit) != len(q_goal):
        print(
            f"[WARN] qinit 行数 {len(qinit)} 与 targets 行数 {len(q_goal)} 不一致，"
            f"取 min={n}",
            file=sys.stderr,
        )
    if qinit.shape[1] != model.nq:
        raise ValueError(f"qinit 关节数 {qinit.shape[1]} != nq={model.nq}")

    start, end, n_play = clamp_slice(index, count, n, default_count=20)
    print(
        f"[INFO] random 模式: {name_key} 题 [{start}, {end}) 共 {n_play} 道，"
        f"题内 dt={dt}，跨题 2*dt={2 * dt}"
    )

    for i in range(start, end):
        print(f"[q_init]  i={i}")
        viz.display(np.asarray(qinit[i], dtype=float))
        time.sleep(dt)

        print(f"[target]  i={i}")
        viz.display(np.asarray(q_goal[i], dtype=float))
        time.sleep(2 * dt)

    print("[INFO] random 播放结束")
    wait_exit()


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Pinocchio + MeshCat：可视化 path / 随机 IK 基准数据"
    )
    ap.add_argument(
        "--robot_id",
        type=int,
        default=1,
        help="机器人编号 0–14（与 launch 中 robot_id 一致），默认 1",
    )
    ap.add_argument(
        "--urdf",
        type=str,
        default=None,
        help="直接指定 URDF（仍建议提供 robot_id 以定位 CSV）",
    )
    ap.add_argument(
        "--mode",
        type=str,
        choices=("neutral", "csv", "random"),
        default="csv",
        help="neutral | csv(path 轨迹) | random(qinit/target 交叉)",
    )
    ap.add_argument(
        "--index",
        type=int,
        default=0,
        help="起始下标：csv=路径点，random=题号，默认 0",
    )
    ap.add_argument(
        "--count",
        type=int,
        default=None,
        help="连续条数：csv 默认 min(200,剩余)，random 默认 min(20,剩余)",
    )
    ap.add_argument(
        "--dt",
        type=float,
        default=None,
        help="时间间隔秒：csv 默认 0.03；random 默认 0.5（跨题为 2*dt）",
    )
    ap.add_argument(
        "--no-open",
        action="store_true",
        help="不自动打开浏览器",
    )
    args = ap.parse_args()

    if args.dt is None:
        args.dt = 0.5 if args.mode == "random" else 0.03

    try:
        import pinocchio as pin
        from pinocchio.visualize import MeshcatVisualizer
    except ImportError as e:
        print(
            "缺少 Python 依赖。请先安装:\n"
            "  pip install -r scripts/requirements-viz.txt\n"
            f"原始错误: {e}",
            file=sys.stderr,
        )
        return 1

    try:
        urdf_path, name_key, robot_id = resolve_urdf_path(args)
    except (FileNotFoundError, SystemExit) as e:
        print(e, file=sys.stderr)
        return 1

    if args.mode in ("csv", "random") and name_key is None:
        print(
            "[ERROR] csv/random 模式需要有效 --robot_id 以定位 data/ CSV",
            file=sys.stderr,
        )
        return 1

    urdf_viz = rewrite_mesh_paths(urdf_path)
    package_dirs = [
        str(URDF_ROOT.resolve()),
        str(urdf_path.parent.resolve()),
    ]

    print(f"[INFO] URDF: {urdf_path}")
    print(f"[INFO] mode={args.mode}, robot_id={robot_id}, name={name_key}")

    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        str(urdf_viz), package_dirs=package_dirs
    )
    data = model.createData()
    frame_id = find_ee_frame_id(model)
    print(
        f"[INFO] nq={model.nq}, njoints={model.njoints} "
        f"(含 universe；自由度看 nq), ee_frame_id={frame_id}"
    )

    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=not args.no_open)
    viz.loadViewerModel()
    print_viewer_url(viz)

    try:
        if args.mode == "neutral":
            q = pin.neutral(model)
            viz.display(q)
            print(f"[INFO] q = {q}")
            wait_exit()
        elif args.mode == "csv":
            run_path_animation(
                viz,
                model,
                data,
                frame_id,
                robot_id,
                name_key,
                args.index,
                args.count,
                args.dt,
            )
        else:
            run_random_toggle(
                viz, model, name_key, args.index, args.count, args.dt
            )
    except (FileNotFoundError, ValueError, IndexError) as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n[INFO] 已中断")
        return 130

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
