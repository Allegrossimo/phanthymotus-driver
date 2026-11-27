"""No ROS motion: exercise return dispatch with simulated feedback/planning."""
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, '/work')
from arm_move_to_point import ArmMovePointPlugin, SIDE_JOINTS

names = SIDE_JOINTS['left']
device = SimpleNamespace(
    _cfg={'plugins': {'arm_control': {'home_pose_deg': dict.fromkeys(names, 0.0)}}},
    reserve_arm=Mock(), release_arm=Mock())
plugin = ArmMovePointPlugin(device, {'allow_motion': True})
plugin._joints = lambda: dict.fromkeys(names, .1)
plugin._lifting = lambda: .6
plugin._base_request = lambda side, joints, lifting: {'limits': dict.fromkeys(names, [-1., 1.])}
plugin._planner = lambda request: {'position': [0., 0., .4], 'rotation': [1., 0., 0., 0., 1., 0., 0., 0., 1.]}
try:
    plugin.dispatch('preview_return', {'side': 'left', 'return_to': '回到上一位置'})
except RuntimeError as exc:
    assert '没有上一位置' in str(exc)
else:
    raise AssertionError('missing history was invented')
assert not plugin._started
side, choice, joints, lifting, req = plugin._return_request({'side': 'left', 'return_to': '回到开机初始位置'})
assert req['target_joints_rad'] == [0.] * 8
assert plugin._started
plugin._previous['left'] = {'joints': dict.fromkeys(names, .2), 'lifting_m': .6}
plugin.dispatch('cancel', {})
assert 'left' in plugin._previous
req = plugin._return_request({'side': 'left', 'return_to': '回到上一位置'})[-1]
assert req['target_joints_rad'] == [.2] * 8
plugin._lifting = lambda: .5
try:
    plugin._return_request({'side': 'left', 'return_to': '回到上一位置'})
except RuntimeError as exc:
    assert 'lifting' in str(exc)
else:
    raise AssertionError('changed lift accepted')
plugin._worker = object()
try:
    plugin.dispatch('preview_return', {'side': 'left'})
except RuntimeError as exc:
    assert 'moving' in str(exc)
else:
    raise AssertionError('concurrent return accepted')
print('return state checks PASS')
