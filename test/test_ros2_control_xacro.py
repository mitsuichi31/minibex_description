"""
Static regression tests for minibex.urdf.xacro's hardware_mode switch (Stage 5).

These do not launch any node; they only xacro-expand the URDF and inspect
the resulting <ros2_control> block for each hardware_mode value.
"""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

XACRO_FILE = Path(__file__).resolve().parent.parent / 'urdf' / 'minibex.urdf.xacro'

EXPECTED_JOINTS = {
    'BL_collar_joint', 'BL_hip_joint', 'BL_knee_joint',
    'BR_collar_joint', 'BR_hip_joint', 'BR_knee_joint',
    'FL_collar_joint', 'FL_hip_joint', 'FL_knee_joint',
    'FR_collar_joint', 'FR_hip_joint', 'FR_knee_joint',
}


def _expand(extra_args=()):
    result = subprocess.run(
        ['xacro', str(XACRO_FILE), *extra_args], capture_output=True, text=True, check=True
    )
    return ET.fromstring(result.stdout)


def _expand_fail(extra_args=()):
    return subprocess.run(
        ['xacro', str(XACRO_FILE), *extra_args], capture_output=True, text=True
    )


def test_hardware_mode_none_by_default_omits_ros2_control():
    root = _expand()
    assert root.find('ros2_control') is None


def test_mock_and_bxi_export_the_same_full_mit_interfaces():
    expected_commands = {'position', 'velocity', 'kp', 'kd', 'effort'}
    expected_states = {'position', 'velocity', 'effort', 'temperature'}
    for mode, expected_plugin in (
        ('mock', 'mock_components/GenericSystem'),
        ('bxi', 'bxi_hardware/BxiSystemInterface'),
    ):
        root = _expand([
            f'hardware_mode:={mode}', 'config_file:=/tmp/bxi.yaml', 'spec_dir:=/tmp/specs',
        ])
        control = root.find('ros2_control')
        assert control.find('hardware/plugin').text.strip() == expected_plugin
        joints = control.findall('joint')
        assert {j.get('name') for j in joints} == EXPECTED_JOINTS
        assert len(joints) == 12
        for joint in joints:
            cmd_ifaces = {c.get('name') for c in joint.findall('command_interface')}
            state_ifaces = {s.get('name') for s in joint.findall('state_interface')}
            assert cmd_ifaces == expected_commands, f'{joint.get("name")}: {cmd_ifaces}'
            assert state_ifaces == expected_states, f'{joint.get("name")}: {state_ifaces}'


def test_missing_mujoco_model_is_rejected():
    result = _expand_fail(['hardware_mode:=mujoco'])
    assert result.returncode != 0, 'xacro must fail when mujoco_model is not set'
    assert 'mujoco_model is required when hardware_mode is mujoco' in result.stderr


def test_unknown_hardware_mode_is_rejected():
    result = _expand_fail(['hardware_mode:=nonsense'])
    assert result.returncode != 0
    assert 'hardware_mode must be none, mock, bxi, or mujoco' in result.stderr


def test_mujoco_exports_position_only_joints_and_odom_params():
    root = _expand(['hardware_mode:=mujoco', 'mujoco_model:=/tmp/scene.xml'])
    control = root.find('ros2_control')
    assert control.find('hardware/plugin').text.strip() == 'mujoco_ros2_control/MujocoSystemInterface'

    params = {p.get('name'): p.text for p in control.find('hardware').findall('param')}
    assert params['mujoco_model'] == '/tmp/scene.xml'
    assert params['odom_free_joint_name'] == 'base_joint'
    assert params['odom_topic'] == '/odom'

    joints = control.findall('joint')
    assert {j.get('name') for j in joints} == EXPECTED_JOINTS
    for joint in joints:
        cmd_ifaces = {c.get('name') for c in joint.findall('command_interface')}
        state_ifaces = {s.get('name') for s in joint.findall('state_interface')}
        assert cmd_ifaces == {'position'}, f'{joint.get("name")}: {cmd_ifaces}'
        assert state_ifaces == {'position', 'velocity', 'effort'}, f'{joint.get("name")}: {state_ifaces}'


def test_mujoco_exports_imu_sensor():
    root = _expand(['hardware_mode:=mujoco', 'mujoco_model:=/tmp/scene.xml'])
    control = root.find('ros2_control')
    sensors = control.findall('sensor')
    assert len(sensors) == 1
    imu = sensors[0]
    assert imu.get('name') == 'imu'
    params = {p.get('name'): p.text for p in imu.findall('param')}
    assert params == {'mujoco_type': 'imu', 'mujoco_sensor_name': 'imu'}
    state_ifaces = {s.get('name') for s in imu.findall('state_interface')}
    assert state_ifaces == {
        'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w',
        'angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z',
        'linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z',
    }
