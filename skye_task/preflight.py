"""Read-only planning from current feedback. Never enables, uploads, runs or stops."""

from copy import deepcopy
from datetime import datetime, timezone
from .geometry import local_displacement, pose_error, translated
from .sdk import MotionError
from .sequence import FIXED_AXES, Sequence


def check_current_plan(sdk, config, report, on_event=lambda name, detail: None):
    report.update(mode="read_only_plan_check", status="checking", source="current_feedback",
                  started_at=datetime.now(timezone.utc).isoformat(), segments=[], error="")
    try:
        report["metadata"] = sdk.connect()
        feedback = sdk.read()
        validator = Sequence(sdk, config)
        # Linear work is performed at receive height. A read-only check must not move the lift.
        validator.fixed = {"body": config.body, "head": config.head, "lift": config.receive_lift}
        validator._guard(feedback, require_position=False)
        if any(state >= 3 for state in feedback.trajectory_states):
            raise MotionError("当前双臂有轨迹正在执行，不能以此状态检查规划")
        sdk.initialize_kinematics(feedback)
        report["metadata"]["tool_xyzabc_mm_deg"] = sdk.tools
        report["current_joints_deg"] = deepcopy(feedback.joints)
        report["fixed_feedback"] = {name: feedback.rt[name]["fb_pos"][:size]
                                    for name, size in FIXED_AXES.items()}
        segments = (
            ("forward", [[config.forward_mm, 0, 0]] * 2, config.forward),
            ("down", [[0, 0, -config.down_mm]] * 2, config.down),
            ("exit", [[0, config.left_exit_mm, 0], [0, -config.right_exit_mm, 0]], config.exit),
        )
        predicted = deepcopy(feedback)
        for name, deltas, motion in segments:
            starts = [sdk.forward(i, predicted.joints[i]) for i in range(2)]
            shoulders = sdk.shoulders(predicted)
            local = [local_displacement(shoulders[i], deltas[i]) for i in range(2)]
            targets = [translated(starts[i], local[i]) for i in range(2)]
            record = {"name": name, "status": "planning", "reference_joints_deg": predicted.joints,
                      "robot_displacements_mm": deltas, "arm_base_displacements_mm": local,
                      "arm_base_start_matrices": starts, "arm_base_target_matrices": targets,
                      "null_space_strategy": "nearest_reference_joints"}
            report["segments"].append(record)
            on_event(name, {"status": "planning", "robot_displacements_mm": deltas})
            trajectories = sdk.plan_linear(predicted, starts, targets, motion)
            for i, trajectory in enumerate(trajectories):
                if not validator._near(trajectory.startpoint, predicted.joints[i]):
                    raise MotionError(f"{name}: arm{i} 轨迹首点偏离参考关节值")
                distance, angle = pose_error(sdk.forward(i, trajectory.endpoint), targets[i])
                if distance > config.position_tolerance_mm or angle > config.orientation_tolerance_deg:
                    raise MotionError(f"{name}: arm{i} 规划终点未达到目标位姿")
                predicted.rt["arms"][i]["fb"]["fb_pos"] = trajectory.endpoint
            record.update(status="planned", point_counts=[t.count for t in trajectories],
                          endpoint_joints_deg=[t.endpoint for t in trajectories])
        record = {"name": "return_ready", "status": "planning"}
        report["segments"].append(record)
        counts = []
        for i in range(2):
            if validator._near(predicted.joints[i], config.ready[i]):
                counts.append(0)
                continue
            trajectory = sdk.plan_joint(i, predicted.joints[i], config.ready[i])
            if (not validator._near(trajectory.startpoint, predicted.joints[i])
                    or not validator._near(trajectory.endpoint, config.ready[i])):
                raise MotionError(f"return_ready: arm{i} 规划端点不匹配")
            counts.append(trajectory.count)
        record.update(status="planned", point_counts=counts)
        report["status"] = "planned"
        return True
    except Exception as error:
        report["status"] = "failed"
        report["error"] = str(error)
        if report["segments"] and report["segments"][-1]["status"] == "planning":
            report["segments"][-1].update(status="failed", error=str(error))
        return False
    finally:
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
