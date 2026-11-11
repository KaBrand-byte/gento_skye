"""Small rigid-transform helpers, with no third-party dependencies."""

import math


def local_displacement(shoulder_in_robot, delta_robot):
    """Transform a vector with R transpose (translations do not apply to vectors)."""
    return [sum(shoulder_in_robot[j][i] * delta_robot[j] for j in range(3))
            for i in range(3)]


def translated(pose, delta):
    target = [list(row) for row in pose]
    for i in range(3):
        target[i][3] += delta[i]
    return target


def pose_error(actual, target):
    distance = math.sqrt(sum((actual[i][3] - target[i][3]) ** 2 for i in range(3)))
    trace = sum(actual[i][j] * target[i][j] for i in range(3) for j in range(3))
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))
    return distance, angle
