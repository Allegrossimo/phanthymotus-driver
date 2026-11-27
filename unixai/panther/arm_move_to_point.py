"""Panther single-arm absolute gripper-center move."""

import json
import math
import subprocess
import threading
from pathlib import Path
from uuid import uuid4

from contracts import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, fresh, intersect_joint_limits
from device import _notify_completion


SIDE_JOINTS = {"left": LEFT_ARM_JOINTS, "right": RIGHT_ARM_JOINTS}


def _finite(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


class ArmMovePointPlugin:
    name = "arm_point"

    def __init__(self, device, config):
        self.device = device
        self.cfg = dict(config)
        self._lock = threading.RLock()
        self._worker = None
        self._cancel = threading.Event()
        self._action_id = None
        self._handle = None
        self._switched = False
        self._restore_activate = []
        self._restore_deactivate = []
        self._holding = False
        self._started = False
        self._moved = False
        self._start_pose = None
        self._start_joints = None
        self._start_lifting = None
        self._last_result = None
        self._side = None
        # Last pre-move joint feedback per arm survives stop/prepare, but not
        # driver restart. Never infer an unrecorded historical pose.
        self._previous = {}

    def get_tool(self):
        lower = self.cfg["workspace_min_m"]
        upper = self.cfg["workspace_max_m"]
        geometry = (
            "XYZ 为 base_link 下夹爪中心的绝对坐标（m），不是位移。"
            "三轴范围必须同时满足，且仍须满足联合可达性：存在限位内的关节角 q，"
            "使 FK_position(q,h)=(x,y,z)，FK_rotation(q,h)=prepare 记录的姿态（求解容差内）。"
            "几何必要条件（当前 URDF 保守外包球）："
            "(x-0.162991)^2+(y-y0)^2+(z-h-0.498)^2 <= 0.912^2，"
            "左臂 y0=0.0595，右臂 y0=-0.0595，h 为当前 lifting 高度（0.4–0.65 m）。"
            "球内不保证可达；还受姿态、关节限位及路径工作空间约束，以 preview 为准。"
            "未进行整臂碰撞检测。"
        )
        return {
            "name": self.name,
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Panther one-arm absolute gripper-center move. "
                "Operator ensures clear path; no collision checking."
            ),
            "inputSchema": {
                "type": "object",
                "required": ["action"],
                "additionalProperties": False,
                "properties": {
                    "action": {"type": "string",
                               "enum": ["start", "prepare", "info", "preview", "move", "preview_return", "return", "stop", "cancel"]},
                    "side": {"type": "string", "enum": ["left", "right"], "default": "left"},
                    "x": {"type": "number", "minimum": lower[0], "maximum": upper[0],
                          "description": f"X ∈ [{lower[0]:g}, {upper[0]:g}] m；base_link 绝对坐标，+前/-后。模型零位下垂：左约 0.15018，右约 0.13923。范围内仍需 preview 验证。"},
                    "y": {"type": "number", "minimum": lower[1], "maximum": upper[1],
                          "description": f"Y ∈ [{lower[1]:g}, {upper[1]:g}] m；base_link 绝对坐标，+左/-右。模型零位下垂：左约 0.26824，右约 -0.27576。范围内仍需 preview 验证。"},
                    "z": {"type": "number", "minimum": lower[2], "maximum": upper[2],
                          "description": f"Z ∈ [{lower[2]:g}, {upper[2]:g}] m；base_link 绝对坐标，+上。模型零位下垂：Z=h-0.19931，h 为 lifting 高度；h=0.6 时 Z≈0.40069。实际初始位姿以 prepare/info 返回为准。"},
                    "confirm_motion": {"type": "boolean", "const": True},
                    "return_to": {"type": "string",
                                  "enum": ["回到上一位置", "回到开机初始位置"],
                                  "default": "回到上一位置",
                                  "description": "上一位置=本卡片最近一次已接受move前记录的关节姿态（重启不保留）；开机初始位置=配置的下垂初始化姿态，目前8关节全零，并非历史开机实测快照。lifting保持当前高度。"},
                },
                "x-action-params": {
                    "start": {"params": ["side"]},
                    "prepare": {"params": ["side"], "description": "Capture the initial gripper pose and reserve the arm"},
                    "info": {"params": []},
                    "preview": {"params": ["side", "x", "y", "z"],
                                "description": "仅规划，不运动。" + geometry},
                    "move": {"params": ["side", "x", "y", "z", "confirm_motion"],
                             "description": "重新规划后执行；每段关节变化≤15°。" + geometry},
                    "preview_return": {"params": ["side", "return_to"],
                                       "description": "仅预览返回路径。使用已知关节目标，不锁定当前末端姿态、不需要XYZ；检查关节限位和夹爪中心路径，未做整臂避障。"},
                    "return": {"params": ["side", "return_to", "confirm_motion"],
                               "description": "返回上一位置或配置的开机下垂初始姿态。不需要XYZ，末端姿态随关节一起恢复；lifting不动，每段≤15°。路径检查失败则不运动；请先preview_return。"},
                    "stop": {"params": []},
                    "cancel": {"params": []},
                },
                "x-resource": ["arm_l", "arm_r"],
                "x-is-dangerous": True,
                "x-completion": {"actions": ["move", "return"], "timeout": 75},
                "x-hooks": {
                    "on_interrupt_motion": {"action": "cancel"},
                    "on_interrupt_all": {"action": "stop"},
                },
            },
        }

    def start(self):
        # The card lifecycle is driven by MCP actions.  Do not reserve the arm
        # or read feedback at driver startup.
        return None

    def stop(self):
        return self._stop()

    def dispatch(self, action, args):
        if action == "stop" or action == "cancel":
            return self._stop()
        if action == "info":
            return self.info()
        if action == "start":
            return self._start(args)
        if action == "prepare":
            with self._lock:
                if (self._started and self._worker is None and not self._moved
                        and args.get("side", self._side) == self._side):
                    return self.info()
                return self._start(args)
        if action == "preview":
            return self._preview(args)
        if action == "move":
            return self._move(args)
        if action in ("preview_return", "return"):
            return self._return(args, preview=action == "preview_return")
        raise ValueError(f"unsupported arm_move_to_point action: {action}")

    def info(self):
        with self._lock:
            return {
                "state": ("moving" if self._worker else
                          "holding" if self._holding else
                          "ready" if self._started else "idle"),
                "side": self._side,
                "allow_motion": self.cfg.get("allow_motion") is True,
                "start_pose": self._start_pose,
                "last_result": self._last_result,
                "previous_available": {side: side in self._previous for side in SIDE_JOINTS},
                "initial_pose_source": "arm_control.home_pose_deg (configured, not a boot snapshot)",
            }

    def _joints(self):
        message, received = self.device._get("joints")
        if message is None or not fresh(received, 2.0):
            raise RuntimeError("fresh Panther joint feedback is required")
        return dict(zip(message.name, message.position))

    def _lifting(self):
        state = self.device.lifting_state()
        if not state.get("available") or not state.get("fresh"):
            raise RuntimeError("fresh lifting feedback is required")
        positions = state.get("position") or []
        if not positions or not math.isfinite(float(positions[0])):
            raise RuntimeError("lifting position is unavailable")
        value = float(positions[0])
        lifting = self.device._cfg.get("plugins", {}).get("lifting", {})
        low = float(lifting.get("min_position_m", 0.4))
        high = float(lifting.get("max_position_m", 0.65))
        if not low <= value <= high:
            raise RuntimeError(f"lifting {value:.3f} m is outside [{low:g}, {high:g}] m")
        return value

    def _planner(self, request):
        result = subprocess.run(
            [self.cfg["planner_python"], str(Path(__file__).with_name("arm_ik.py"))],
            input=json.dumps(request, allow_nan=False),
            text=True, capture_output=True, timeout=30.0,
        )
        if result.returncode != 0:
            raise RuntimeError(f"planner failed: {result.stderr.strip() or result.returncode}")
        reply = json.loads(result.stdout)
        if "error" in reply:
            raise RuntimeError(reply["error"])
        return reply

    def _base_request(self, side, joints, lifting):
        names = SIDE_JOINTS[side]
        limits = intersect_joint_limits(
            names, self.device._model_joint_limits, self.device._joint_limits
        )
        return {
            "side": side,
            # Live robot_description can omit the lifting chain. Geometry uses
            # the complete vendor model; joint limits still intersect live data.
            "urdf": self.device._model_urdf,
            "measured": {name: joints[name] for name in names},
            "limits": {name: list(bounds) for name, bounds in limits.items()},
            "lifting_m": lifting,
            "box_min": self.cfg["workspace_min_m"],
            "box_max": self.cfg["workspace_max_m"],
            "max_step_deg": self.cfg["max_joint_step_deg"],
        }

    def _start(self, args):
        with self._lock:
            if self._worker is not None:
                raise RuntimeError("arm_move_to_point is moving")
            if self._started:
                raise RuntimeError("arm_move_to_point already started; stop first")
            side = args.get("side", "left")
            if side not in SIDE_JOINTS:
                raise ValueError("side must be left or right")
            self.device.reserve_arm(self)
            try:
                joints = self._joints()
                lifting = self._lifting()
                request = self._base_request(side, joints, lifting)
                request["action"] = "fk"
                pose = self._planner(request)
            except Exception:
                self.device.release_arm(self)
                raise
            self._side = side
            self._started = True
            self._moved = False
            self._holding = False
            self._start_joints = joints
            self._start_lifting = lifting
            self._start_pose = pose
            return self.info()

    def _target(self, args):
        side = args.get("side", self._side)
        if side not in SIDE_JOINTS:
            raise ValueError("side must be left or right")
        if side != self._side:
            raise ValueError("side cannot change after start")
        x = _finite(args.get("x"), "x")
        y = _finite(args.get("y"), "y")
        z = _finite(args.get("z"), "z")
        return side, [x, y, z]

    def _preview(self, args):
        with self._lock:
            if not self._started or self._start_pose is None:
                raise RuntimeError("start must be called before preview")
            if self._moved:
                raise RuntimeError("this card already moved; stop and start again")
            side, target = self._target(args)
            joints = self._joints()
            if any(abs(joints[name] - self._start_joints[name]) > 1e-3
                   for name in SIDE_JOINTS[side]):
                raise RuntimeError("arm moved after start; start again")
            if abs(self._lifting() - self._start_lifting) > 1e-3:
                raise RuntimeError("lifting changed after start; start again")
            request = self._base_request(side, joints, self._start_lifting)
            request.update(action="plan", target_position=target,
                           target_rotation=self._start_pose["rotation"])
            result = self._planner(request)
            return {"state": "reachable", **result}

    def _move(self, args):
        with self._lock:
            if self._worker is not None:
                raise RuntimeError("arm_move_to_point is moving")
            if not self._started:
                raise RuntimeError("start must be called before move")
            if self._moved:
                raise RuntimeError("this card already moved; stop and start again")
            if args.get("confirm_motion") is not True:
                return {"state": "error", "code": "CONFIRMATION_REQUIRED"}
            if self.cfg.get("allow_motion") is not True:
                raise RuntimeError("arm_move_to_point is not commissioned for motion")
            self.device._ensure_motion_ready()
            self._cancel.clear()
            action_id = str(args.get("action_id") or f"panther_point_{uuid4().hex}")
            self._action_id = action_id
            self._worker = threading.Thread(
                target=self._run, args=(action_id, args), daemon=True,
                name="panther_arm_move_point",
            )
            self._worker.start()
            return {"state": "running", "action_id": action_id}

    def _run(self, action_id, args):
        result = {"ok": False, "side": self._side, "target": None, "waypoints": 0}
        status = "error"
        try:
            side, target = self._target(args)
            result["target"] = target
            joints = self._joints()
            if any(abs(joints[name] - self._start_joints[name]) > 1e-3
                   for name in SIDE_JOINTS[side]):
                raise RuntimeError("arm moved after start")
            if abs(self._lifting() - self._start_lifting) > 1e-3:
                raise RuntimeError("lifting changed after start")
            request = self._base_request(side, joints, self._start_lifting)
            request.update(action="plan", target_position=target,
                           target_rotation=self._start_pose["rotation"])
            planned = self._planner(request)
            waypoints = planned["waypoints"]
            result["waypoints"] = len(waypoints)
            self._execute(side, waypoints, joints, self._start_lifting, remember=True)
            result["ok"] = True
            status = "completed"
            with self._lock:
                self._holding = True
                self._moved = True
        except Exception as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
            status = "cancelled" if self._cancel.is_set() else "error"
            self._restore_controllers()
        finally:
            with self._lock:
                self._worker = None
                self._last_result = result
            _notify_completion(action_id, status, result, self.name)

    def _return_request(self, args):
        side = args.get("side", self._side or "left")
        if side not in SIDE_JOINTS:
            raise ValueError("side must be left or right")
        choice = args.get("return_to", "回到上一位置")
        if choice not in ("回到上一位置", "回到开机初始位置"):
            raise ValueError("unknown return destination")
        if choice == "回到上一位置" and side not in self._previous:
            raise RuntimeError("没有上一位置记录：此版本仅记录本卡片后续运动前的姿态。可选择回到开机初始位置。")
        if not self._started:
            self._start({"side": side})
        if side != self._side:
            raise RuntimeError("请先 cancel 释放当前手臂，再选择另一侧返回")
        joints, lifting = self._joints(), self._lifting()
        names = SIDE_JOINTS[side]
        request = self._base_request(side, joints, lifting)
        if choice == "回到上一位置":
            previous = self._previous[side]
            if abs(previous["lifting_m"] - lifting) > 1e-3:
                raise RuntimeError("lifting高度已改变，不能恢复上一空间位置；请先恢复原lifting高度")
            target = [previous["joints"][name] for name in names]
            # Clip only recorded encoder noise at a hard stop, never a user goal.
            tolerance = math.radians(self.device._cfg["plugins"]["arm_control"].get("hold_limit_tolerance_deg", 0.5))
            for index, name in enumerate(names):
                low, high = request["limits"][name]
                if not low - tolerance <= target[index] <= high + tolerance:
                    raise ValueError("saved return pose is outside current joint limits")
                target[index] = min(high, max(low, target[index]))
        else:
            home = self.device._cfg["plugins"]["arm_control"]["home_pose_deg"]
            target = [math.radians(_finite(home[name], name)) for name in names]
        request.update(action="plan_joints", target_joints_rad=target,
                       hold_limit_tolerance_deg=self.device._cfg["plugins"]["arm_control"].get("hold_limit_tolerance_deg", 0.5))
        return side, choice, joints, lifting, request

    def _return(self, args, preview=False):
        with self._lock:
            if self._worker is not None:
                raise RuntimeError("arm_move_to_point is moving; cancel first")
            if not preview:
                if args.get("confirm_motion") is not True:
                    return {"state": "error", "code": "CONFIRMATION_REQUIRED"}
                if self.cfg.get("allow_motion") is not True:
                    raise RuntimeError("arm_move_to_point is not commissioned for motion")
                self.device._ensure_motion_ready()
            side, choice, joints, lifting, request = self._return_request(args)
            if preview:
                return {"state": "reachable", "return_to": choice, **self._planner(request)}
            self._cancel.clear()
            self._handle = None
            action_id = str(args.get("action_id") or f"panther_return_{uuid4().hex}")
            self._worker = threading.Thread(target=self._run_return,
                args=(action_id, side, choice, joints, lifting, request), daemon=True,
                name="panther_arm_return")
            self._worker.start()
            return {"state": "running", "action_id": action_id, "return_to": choice}

    def _run_return(self, action_id, side, choice, joints, lifting, request):
        result = {"ok": False, "side": side, "return_to": choice}
        status = "error"
        try:
            planned = self._planner(request)
            result.update(target=planned["target_position"], waypoints=planned["waypoint_count"])
            self._execute(side, planned["waypoints"], joints, lifting)
            with self._lock:
                self._holding = True
                self._moved = True
            result["ok"] = True
            status = "completed"
        except Exception as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
            status = "cancelled" if self._cancel.is_set() else "error"
            try:
                self._restore_controllers()
            except Exception as restore_error:
                result["restore_error"] = str(restore_error)
        finally:
            with self._lock:
                self._worker = None
                self._last_result = result
            _notify_completion(action_id, status, result, self.name)

    def _execute(self, side, waypoints, planned_start, lifting, remember=False):
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        if self._cancel.is_set():
            raise RuntimeError("cancelled before execution")
        self.device._ensure_motion_ready()
        names = SIDE_JOINTS[side]
        current = self._joints()
        if any(abs(current[name] - planned_start[name]) > 1e-3 for name in names):
            raise RuntimeError("arm moved during planning; preview again")
        if abs(self._lifting() - lifting) > 1e-3:
            raise RuntimeError("lifting changed during planning")
        controller = f"{side}_arm_joint_trajectory_position_controller"
        forward = "dual_arm_forward_position_controller"
        states = self.device.controller_states()
        activate = [controller] if states.get(controller) != "active" else []
        deactivate = [forward] if states.get(forward) == "active" else []
        if activate or deactivate:
            self.device._switch_controllers_exact(activate, deactivate)
            self._restore_activate = deactivate
            self._restore_deactivate = activate
            self._switched = True

        trajectory = JointTrajectory()
        trajectory.joint_names = list(names)
        current = self._joints()
        first = JointTrajectoryPoint()
        first.positions = [current[name] for name in names]
        first.time_from_start = Duration(sec=0, nanosec=0)
        trajectory.points = [first]

        segment = float(self.cfg["segment_duration_s"])
        for index, waypoint in enumerate(waypoints, start=1):
            point = JointTrajectoryPoint()
            point.positions = [math.radians(value) for value in waypoint]
            whole = index * segment
            point.time_from_start = Duration(
                sec=int(whole), nanosec=int((whole % 1.0) * 1e9)
            )
            trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        client = self.device._arm_actions[side]
        if not client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f"{side} FollowJointTrajectory action is unavailable")
        if self._cancel.is_set():
            raise RuntimeError("cancelled before goal submission")
        handle = self.device._wait_future(client.send_goal_async(goal), 5.0)
        if not handle.accepted:
            raise RuntimeError(f"{side} trajectory goal was rejected")
        with self._lock:
            self._handle = handle
            if remember:
                self._previous[side] = {"joints": {name: current[name] for name in names},
                                        "lifting_m": lifting}
            if self._cancel.is_set():
                handle.cancel_goal_async()
        timeout = float(len(waypoints) * segment + 15.0)
        result_future = handle.get_result_async()
        try:
            wrapped = self.device._wait_future(result_future, timeout)
        except Exception:
            handle.cancel_goal_async()
            raise
        if wrapped.result.error_code != 0:
            raise RuntimeError(
                f"{side} trajectory failed: {wrapped.result.error_string}"
            )
        if self._cancel.is_set():
            raise RuntimeError("cancelled")

    def _restore_controllers(self):
        if not self._switched:
            return
        states = self.device.controller_states(max_cache_age_s=0.0)
        activate = [name for name in self._restore_activate if states.get(name) != "active"]
        deactivate = [name for name in self._restore_deactivate if states.get(name) == "active"]
        self.device._switch_controllers_exact(activate, deactivate)
        self._switched = False

    def _stop(self):
        with self._lock:
            self._cancel.set()
            handle = self._handle
            worker = self._worker
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=15.0)
            if worker.is_alive():
                # Do not release the arm or change controllers while the
                # trajectory action may still be in flight.
                return {"state": "stopping", "stopped": False,
                        "message": "trajectory cancel not yet confirmed; retry stop"}
        self._restore_controllers()
        with self._lock:
            if self._holding or self._started:
                try:
                    self.device.release_arm(self)
                except Exception:
                    pass
            self._worker = None
            self._handle = None
            self._holding = False
            self._started = False
            self._moved = False
            self._start_pose = None
            self._start_joints = None
            self._start_lifting = None
            self._side = None
            return {"state": "idle", "stopped": True}
