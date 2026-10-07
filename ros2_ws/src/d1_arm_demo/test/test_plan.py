from d1_arm_demo.arm_demo_node import DEFAULT_WAYPOINTS, NEUTRAL
from d1_arm_demo.plan import build_plan, changed_joints, multi_joint_rows, split_move

N = len(NEUTRAL)


def pose(**kw):
    p = [0.0] * N
    for k, v in kw.items():
        p[int(k[1:]) - 1] = v
    return p


def assert_single_joint_steps(start, plan):
    prev = start
    for step in plan:
        assert len(changed_joints(prev, step)) == 1, (prev, step)
        prev = step


def test_default_from_neutral_goes_out_and_back_the_same_way():
    plan = build_plan(NEUTRAL, DEFAULT_WAYPOINTS, True, NEUTRAL)
    assert plan == [pose(j1=1.57), pose(j1=1.57, j2=1.2), pose(j1=1.57), NEUTRAL]
    assert_single_joint_steps(NEUTRAL, plan)


def test_without_return_ends_at_last_waypoint():
    plan = build_plan(NEUTRAL, DEFAULT_WAYPOINTS, False, NEUTRAL)
    assert plan[-1] == DEFAULT_WAYPOINTS[-1]
    assert_single_joint_steps(NEUTRAL, plan)


def test_second_run_without_return_backtracks_instead_of_jumping():
    # 1回目を return_to_neutral:=false で終えた姿勢から2回目を始める
    last = DEFAULT_WAYPOINTS[-1]
    plan = build_plan(last, DEFAULT_WAYPOINTS, False, NEUTRAL)
    assert plan[:2] == [pose(j1=1.57), NEUTRAL]
    assert plan[-1] == last
    assert_single_joint_steps(last, plan)


def test_multi_joint_waypoint_is_split():
    wps = [NEUTRAL, pose(j1=0.8, j2=-0.6, j3=0.4, j5=0.5)]
    assert multi_joint_rows(wps) == [1]
    plan = build_plan(NEUTRAL, wps, True, NEUTRAL)
    assert_single_joint_steps(NEUTRAL, plan)
    assert wps[1] in plan
    assert plan[-1] == NEUTRAL


def test_non_neutral_first_waypoint_is_reached_and_left_one_joint_at_a_time():
    wps = [pose(j1=0.5, j2=0.3), pose(j1=0.5, j2=0.6)]
    plan = build_plan(NEUTRAL, wps, True, NEUTRAL)
    assert_single_joint_steps(NEUTRAL, plan)
    assert plan[-1] == NEUTRAL


def test_split_move_retracts_higher_joints_first():
    assert split_move(pose(j1=1.0, j2=1.0), NEUTRAL) == [pose(j1=1.0), NEUTRAL]
    assert split_move(NEUTRAL, pose(j1=1.0, j2=1.0)) == [pose(j1=1.0), pose(j1=1.0, j2=1.0)]


def test_default_waypoints_have_no_multi_joint_rows():
    assert multi_joint_rows(DEFAULT_WAYPOINTS) == []
