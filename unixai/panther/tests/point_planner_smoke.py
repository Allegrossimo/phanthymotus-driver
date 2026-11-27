"""Run with the image's /opt/panther-pink/bin/python; no ROS or motion."""
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, '/work')
import arm_ik

xml = Path('/work/resource/panther.urdf').read_text()
model = pin.buildModelFromXML(xml)
for side in ('left', 'right'):
    names = arm_ik.SIDE_JOINTS[side]
    limits = {n: [float(model.lowerPositionLimit[model.joints[model.getJointId(n)].idx_q]),
                  float(model.upperPositionLimit[model.joints[model.getJointId(n)].idx_q])]
              for n in names}
    request = dict(side=side, urdf=xml, measured=dict.fromkeys(names, 0.0),
                   lifting_m=0.5, limits=limits, box_min=[-.2, -.45, 0],
                   box_max=[.88, .45, 1.2], max_step_deg=15)
    pose = arm_ik.fk(request)
    raised = arm_ik.fk(dict(request, lifting_m=0.6))
    assert abs(raised['position'][2] - pose['position'][2] - .1) < 1e-8
    target = np.array(pose['position']) + [.05, 0, 0]
    result = arm_ik.plan(dict(request, target_position=target, target_rotation=pose['rotation']))
    previous = np.zeros(8)
    for waypoint in result['waypoints']:
        q = np.deg2rad(waypoint)
        assert np.max(np.abs(q - previous)) <= np.deg2rad(15) + 1e-8
        for name, value in zip(names, q):
            assert limits[name][0] - 1e-8 <= value <= limits[name][1] + 1e-8
        previous = q
    achieved = arm_ik.fk(dict(request, measured=dict(zip(names, previous))))
    assert np.linalg.norm(np.array(achieved['position']) - target) < .0011
    returned = arm_ik.plan_joints(dict(request, measured=dict(zip(names, previous)),
                                      target_joints_rad=[0.0] * 8))
    assert np.allclose(returned['waypoints'][-1], np.zeros(8))
    last = previous
    for waypoint in returned['waypoints']:
        angles = np.deg2rad(waypoint)
        assert np.max(np.abs(angles - last)) <= np.deg2rad(15) + 1e-8
        last = angles
    try:
        arm_ik.plan_joints(dict(request, target_joints_rad=[10.0] * 8))
    except ValueError as exc:
        assert 'joint limits' in str(exc)
    else:
        raise AssertionError('out-of-limit return accepted')
    try:
        arm_ik.plan(dict(request, target_position=[2, 2, 2], target_rotation=pose['rotation']))
    except ValueError as exc:
        assert 'outside' in str(exc)
    else:
        raise AssertionError('out-of-box target accepted')
    print(side, 'PASS', result['waypoint_count'], 'waypoints')
