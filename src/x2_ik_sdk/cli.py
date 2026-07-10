from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .config import ArmSide, X2IKConfig
from .solver import X2ArmIKSolver


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline X2 arm IK demo")
    parser.add_argument("--urdf", type=Path, help="Override URDF path")
    parser.add_argument("--side", choices=[side.value for side in ArmSide], default="right")
    parser.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--target-offset", nargs=3, type=float, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--arm-pos", nargs=14, type=float, help="Seed arm_pos[14]. Default is ready pose.")
    parser.add_argument("--print-json", action="store_true")
    parser.add_argument("--limits", action="store_true", help="Print arm joint limits and exit")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    cfg = X2IKConfig.default_omnipicker()
    if args.urdf:
        cfg = X2IKConfig(urdf_path=args.urdf)

    solver = X2ArmIKSolver(cfg)
    if args.limits:
        for name, lower, upper in solver.joint_limits_for_arm_pos():
            print(f"{name:35s} {lower: .6f} {upper: .6f}")
        return

    side = ArmSide(args.side)
    arm_pos = args.arm_pos if args.arm_pos is not None else solver.ready_arm_pos()
    if args.target is not None:
        target = np.asarray(args.target, dtype=float)
    elif args.target_offset is not None:
        current = np.asarray(solver.fk_xyz(side, arm_pos), dtype=float)
        target = current + np.asarray(args.target_offset, dtype=float)
    else:
        raise SystemExit("Provide --target or --target-offset")

    result = solver.solve_position(side=side, target_xyz=target, current_arm_pos=arm_pos)
    if args.print_json:
        payload = asdict(result)
        payload["side"] = result.side.value
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return

    print(f"success: {result.success}")
    print(f"message: {result.message}")
    print(f"side: {result.side.value}")
    print(f"ee_frame: {result.ee_frame}")
    print(f"iterations: {result.iterations}")
    print(f"error_norm: {result.error_norm:.8f}")
    print(f"target_xyz: {result.target_xyz}")
    print(f"final_xyz:  {result.final_xyz}")
    print("arm_pos[14]:")
    print(np.array2string(np.asarray(result.arm_pos), precision=6, suppress_small=True, separator=", "))


if __name__ == "__main__":
    main()
