"""Pink/Pinocchio planner for the Panther gripper-centre move card.

Run inside the isolated pin-pink environment.  The planner never touches ROS.
"""

import json
import math
import sys

import numpy as np
import pinocchio as pin
import pink
from pink.barriers import PositionBarrier
from pink.tasks import FrameTask, PostureTask


SIDE_JOINTS = {
    "left": tuple(f"joint_la{i}" for i in range(8)),
    "right": tuple(f"joint_ra{i}" for i in range(8)),
}
CENTER_FRAMES = {
    "left": ("left_connector", "link_lf03", "link_lf13"),
    "right": ("right_connector", "link_rf03", "link_rf13"),
}


def _rotation_matrix(values):
    matrix = np.asarray(values, dtype=float).reshape(3, 3)
    if not np.isfinite(matrix).all():
        raise ValueError("target rotation is not finite")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-4):
        raise ValueError("target rotation is not orthonormal")
    if np.linalg.det(matrix) < 0:
        raise ValueError("target rotation is left-handed")
    return matrix


def _set_joint(model, q, name, value):
    if not model.existJointName(name):
        raise ValueError(f"joint missing from URDF: {name}")
    joint = model.joints[model.getJointId(name)]
    if joint.nq != 1:
        raise ValueError(f"joint is not single-DoF: {name}")
    q[joint.idx_q] = value


def _build(request):
    side = request["side"]
    names = SIDE_JOINTS[side]
    urdf = request["urdf"]
    measured = request["measured"]
    lifting = float(request["lifting_m"])

    full = pin.buildModelFromXML(urdf)
    q = pin.neutral(full)
    for name, value in measured.items():
        if full.existJointName(name):
            _set_joint(full, q, name, float(value))
    _set_joint(full, q, "body_joint", lifting)
    # Fingers are locked open: the centre of symmetric fingers does not depend
    # on the opening, and collision checking is deliberately out of scope.
    for suffix in ("joint_lf", "joint_lf11", "joint_rf", "joint_rf11"):
        if full.existJointName(suffix):
            _set_joint(full, q, suffix, 0.0)

    data = full.createData()
    pin.forwardKinematics(full, data, q)
    pin.updateFramePlacements(full, data)
    connector, tip_a, tip_b = CENTER_FRAMES[side]
    frame_id = full.getFrameId(connector)
    if frame_id >= len(full.frames):
        raise ValueError(f"connector frame missing: {connector}")
    parent_joint = full.frames[frame_id].parentJoint
    base = data.oMf[frame_id]
    first = data.oMf[full.getFrameId(tip_a)]
    second = data.oMf[full.getFrameId(tip_b)]
    centre = pin.SE3(base.rotation, (first.translation + second.translation) / 2.0)
    placement = data.oMi[parent_joint].inverse() * centre
    frame_name = f"{side}_gripper_center"
    if not full.existFrame(frame_name):
        full.addFrame(pin.Frame(frame_name, parent_joint, placement, pin.FrameType.OP_FRAME))

    locked = [
        joint_id for joint_id in range(1, full.njoints)
        if full.names[joint_id] not in names
    ]
    model = pin.buildReducedModel(full, locked, q)
    if model.nq != len(names):
        raise ValueError("unexpected reduced model size")
    data = model.createData()
    order = [model.joints[model.getJointId(name)].idx_q for name in names]
    current = np.empty(model.nq)
    current[order] = [float(measured[name]) for name in names]

    limits = request["limits"]
    for name, index in zip(names, order):
        low, high = limits[name]
        model.lowerPositionLimit[index] = max(float(low), model.lowerPositionLimit[index])
        model.upperPositionLimit[index] = min(float(high), model.upperPositionLimit[index])

    return model, data, current, order, frame_name


def _pose_at(model, data, q, frame):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[model.getFrameId(frame)]


def fk(request):
    model, data, current, order, frame = _build(request)
    pose = _pose_at(model, data, current, frame)
    return {
        "position": pose.translation.tolist(),
        "rotation": pose.rotation.reshape(-1).tolist(),
        "frame": frame,
    }


def plan(request):
    model, data, current, order, frame = _build(request)
    target_position = np.asarray(request["target_position"], dtype=float)
    target_rotation = _rotation_matrix(request["target_rotation"])
    if not np.isfinite(target_position).all():
        raise ValueError("target position is not finite")
    box_min = np.asarray(request["box_min"], dtype=float)
    box_max = np.asarray(request["box_max"], dtype=float)
    if np.any(target_position < box_min) or np.any(target_position > box_max):
        raise ValueError("target outside commissioned workspace")

    target = pin.SE3(target_rotation, target_position)
    barrier = PositionBarrier(frame, p_min=box_min, p_max=box_max)
    task = FrameTask(frame, position_cost=1.0, orientation_cost=1.0, lm_damping=1e-3)
    posture = PostureTask(cost=1e-3)

    def solve(seed):
        configuration = pink.Configuration(model, data, seed.copy())
        task.set_target(target)
        posture.set_target(seed)
        q = seed.copy()
        for _ in range(200):
            try:
                configuration.update(q)
                configuration.check_limits()
                pose = configuration.get_transform_frame_to_world(frame)
                error = pin.log6(target.inverse() * pose).vector
                if np.linalg.norm(error[:3]) <= 1e-3 and np.linalg.norm(error[3:]) <= 2e-2:
                    if np.min(barrier.compute_barrier(configuration)) < -1e-8:
                        raise ValueError("IK solution outside workspace")
                    return q.copy()
                velocity = pink.solve_ik(
                    configuration, [task, posture], 0.02, solver="daqp", barriers=[barrier]
                )
                q = pin.integrate(model, q, velocity * 0.02)
            except Exception:
                raise
        raise ValueError("IK did not converge")

    seeds = [current]
    neutral = np.clip(np.zeros(model.nq), model.lowerPositionLimit, model.upperPositionLimit)
    if not np.allclose(neutral, current):
        seeds.append(neutral)
    # Deterministic extra seeds reduce the chance of stopping in a local
    # minimum when the fixed start orientation makes the arm redundant.
    rng = np.random.default_rng(0)
    for _ in range(4):
        seeds.append(rng.uniform(model.lowerPositionLimit, model.upperPositionLimit))
    best = None
    best_margin = -math.inf
    best_distance = math.inf
    for seed in seeds:
        try:
            candidate = solve(seed)
        except Exception:
            continue
        margins = np.minimum(candidate - model.lowerPositionLimit,
                             model.upperPositionLimit - candidate)
        margin = float(np.min(margins))
        distance = float(np.max(np.abs(candidate - current)))
        # Prefer the shortest joint-space solution; use limit margin only as a
        # tie-breaker. A high-margin random branch can otherwise demand a huge
        # elbow/wrist sweep for a small Cartesian move.
        if (distance < best_distance - 1e-9
                or (abs(distance - best_distance) <= 1e-9 and margin > best_margin)):
            best, best_margin, best_distance = candidate, margin, distance
    if best is None:
        raise ValueError("no reachable IK solution for this target")

    max_step = math.radians(float(request["max_step_deg"]))
    if max_step <= 0:
        raise ValueError("max_step_deg must be positive")
    distance = float(np.max(np.abs(best - current)))
    count = max(1, int(math.ceil(distance / max_step)))
    waypoints = []
    for index in range(1, count + 1):
        q = pin.interpolate(model, current, best, index / count)
        configuration = pink.Configuration(model, data, q)
        configuration.check_limits()
        if np.min(barrier.compute_barrier(configuration)) < -1e-8:
            raise ValueError("planned path leaves workspace")
        waypoints.append(np.rad2deg(q[order]).tolist())

    return {
        "frame": frame,
        "waypoints": waypoints,
        "waypoint_count": len(waypoints),
        "joint_margin_deg": math.degrees(best_margin),
        "target_position": target_position.tolist(),
    }


def plan_joints(request):
    """Return to a known joint pose, without imposing the Cartesian start attitude."""
    model, data, current, order, frame = _build(request)
    target = np.asarray(request["target_joints_rad"], dtype=float)
    if target.shape != (8,) or not np.isfinite(target).all():
        raise ValueError("return target must contain eight finite joint angles")
    goal = current.copy()
    goal[order] = target
    low, high = model.lowerPositionLimit, model.upperPositionLimit
    if np.any(goal < low) or np.any(goal > high):
        raise ValueError("return target outside joint limits")
    # Measured encoder noise may be slightly outside a hard stop. Never widen
    # target limits: only allow a monotonic return from that measured value.
    tolerance = math.radians(float(request.get("hold_limit_tolerance_deg", 0.5)))
    if not np.isfinite(current).all() or np.any(current < low - tolerance) or np.any(current > high + tolerance):
        raise ValueError("current joints outside feedback limit tolerance")
    step = math.radians(float(request["max_step_deg"]))
    if not math.isfinite(step) or not 0 < step <= math.radians(15):
        raise ValueError("return max step must be in (0, 15] degrees")
    delta = goal - current
    distance = float(np.max(np.abs(delta)))
    count = max(1, int(math.ceil(distance / step)))
    # Check the interpolated path more densely than the transmitted waypoints.
    samples = max(1, int(math.ceil(distance / math.radians(1))))
    box_min, box_max = np.asarray(request["box_min"]), np.asarray(request["box_max"])
    for i in range(samples + 1):
        q = current + delta * i / samples
        position = _pose_at(model, data, q, frame).translation
        if np.any(position < box_min - 1e-8) or np.any(position > box_max + 1e-8):
            raise ValueError("return path leaves configured workspace; no trajectory sent")
    pose = _pose_at(model, data, goal, frame)
    return {
        "frame": frame,
        "waypoints": [np.rad2deg((current + delta * i / count)[order]).tolist()
                      for i in range(1, count + 1)],
        "waypoint_count": count,
        "target_position": pose.translation.tolist(),
        "target_rotation": pose.rotation.reshape(-1).tolist(),
        "target_joints_deg": np.rad2deg(target).tolist(),
    }


def main():
    try:
        request = json.load(sys.stdin)
        action = request.get("action")
        if action == "fk":
            result = fk(request)
        elif action == "plan":
            result = plan(request)
        elif action == "plan_joints":
            result = plan_joints(request)
        else:
            raise ValueError(f"unknown planner action: {action}")
        print(json.dumps(result, allow_nan=False))
    except Exception as exc:
        print(json.dumps({"error": f"{type(exc).__name__}: {exc}"}, allow_nan=False))


if __name__ == "__main__":
    main()
