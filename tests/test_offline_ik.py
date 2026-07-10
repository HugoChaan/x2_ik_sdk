from x2_ik_sdk import ArmSide, X2ArmIKSolver, X2IKConfig


def test_right_arm_offset_ik():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    current = solver.fk_xyz(ArmSide.RIGHT, seed)
    target = [current[0] + 0.01, current[1], current[2] + 0.01]
    result = solver.solve_position(ArmSide.RIGHT, target, seed)
    assert result.success
    assert result.error_norm < 2e-4
    assert len(result.arm_pos) == 14


def test_left_arm_offset_ik():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    current = solver.fk_xyz(ArmSide.LEFT, seed)
    target = [current[0] + 0.01, current[1], current[2] + 0.01]
    result = solver.solve_position(ArmSide.LEFT, target, seed)
    assert result.success
    assert result.error_norm < 2e-4
    assert len(result.arm_pos) == 14


def test_right_arm_pose_ik_keeps_current_orientation():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    current_xyz = solver.fk_xyz(ArmSide.RIGHT, seed)
    current_rpy = solver.fk_rpy(ArmSide.RIGHT, seed)
    target_xyz = [current_xyz[0] + 0.005, current_xyz[1], current_xyz[2] + 0.005]
    result = solver.solve_pose(ArmSide.RIGHT, target_xyz, current_rpy, seed)
    assert result.success
    assert result.position_error_norm is not None
    assert result.orientation_error_norm is not None
    assert result.position_error_norm < 2e-4
    assert result.orientation_error_norm < 1e-3
    assert len(result.arm_pos) == 14
