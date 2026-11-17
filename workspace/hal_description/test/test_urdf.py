# Structural checks of urdf/hal_062.urdf.xacro (T1 in docs/NAV2_DESCRIPTION_PORT_PLAN.md).
# Runs xacro on the SOURCE tree (not the installed share) so it works before/without `colcon build`
# of this package, but it needs zed_wrapper to be sourced ($(find zed_wrapper)).
import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
XACRO_FILE = os.path.join(PKG_DIR, 'urdf', 'hal_062.urdf.xacro')
FAKE_PREFIX = tempfile.mkdtemp(prefix='hal_description_test_')

ROVER_LINKS = {
    'base_footprint', 'chassis',
    'left_suspension_u', 'right_suspension_u', 'left_suspension_l', 'right_suspension_l',
    'wheel_link_LF', 'wheel_link_LM', 'wheel_link_LR',
    'wheel_link_RF', 'wheel_link_RM', 'wheel_link_RR',
}
CAMERA_LINKS = {
    'zed_camera_link', 'zed_camera_center',
    'zed_left_camera_frame', 'zed_left_camera_frame_optical',
    'zed_right_camera_frame', 'zed_right_camera_frame_optical',
}
LEGACY_LINKS = {'camera_frame', 'zed2_camera_link',
                'zed2_left_camera_optical_frame', 'zed2_right_camera_optical_frame'}


def run_xacro(**mappings):
    # $(find hal_description) must resolve to the SOURCE tree during this test:
    # build a fake ament prefix in /tmp
    env = dict(os.environ)
    share = os.path.join(FAKE_PREFIX, 'share', 'hal_description')
    index = os.path.join(FAKE_PREFIX, 'share', 'ament_index', 'resource_index', 'packages')
    os.makedirs(index, exist_ok=True)
    open(os.path.join(index, 'hal_description'), 'w').close()
    if not os.path.islink(share):
        os.symlink(PKG_DIR, share)
    env['AMENT_PREFIX_PATH'] = FAKE_PREFIX + os.pathsep + env.get('AMENT_PREFIX_PATH', '')
    args = ['xacro', XACRO_FILE] + [f'{k}:={v}' for k, v in mappings.items()]
    out = subprocess.run(args, capture_output=True, text=True, env=env)
    assert out.returncode == 0, f'xacro failed:\n{out.stderr}'
    return out.stdout


def parse(urdf_xml):
    root = ET.fromstring(urdf_xml)
    links = {link.get('name') for link in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}
    return root, links, joints


@pytest.fixture(scope='module')
def real_urdf():
    return run_xacro(simulation='false')


@pytest.fixture(scope='module')
def sim_urdf():
    return run_xacro(simulation='true')


def test_check_urdf_passes(real_urdf, tmp_path):
    check_urdf = shutil.which('check_urdf')
    if check_urdf is None:
        pytest.skip('check_urdf (urdfdom tools) not available')
    f = tmp_path / 'hal_062.urdf'
    f.write_text(real_urdf)
    out = subprocess.run([check_urdf, str(f)], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr
    assert 'root Link: base_footprint' in out.stdout


def test_links_present_and_no_legacy(real_urdf):
    _, links, _ = parse(real_urdf)
    missing = (ROVER_LINKS | CAMERA_LINKS) - links
    assert not missing, f'missing links: {missing}'
    assert not (LEGACY_LINKS & links), f'legacy links still present: {LEGACY_LINKS & links}'


def test_single_root_is_base_footprint(real_urdf):
    _, links, joints = parse(real_urdf)
    children = {j.find('child').get('link') for j in joints.values()}
    roots = links - children
    assert roots == {'base_footprint'}, f'roots: {roots}'
    # every child link exists and no link has two parents
    parents = [j.find('child').get('link') for j in joints.values()]
    assert len(parents) == len(set(parents)), 'a link has more than one parent joint'
    assert set(parents) <= links


def test_camera_chain(real_urdf):
    _, _, joints = parse(real_urdf)
    mount = joints['camera_mount_joint']
    assert mount.find('parent').get('link') == 'chassis'
    assert mount.find('child').get('link') == 'zed_camera_link'
    assert mount.get('type') == 'fixed'
    xyz = [float(v) for v in mount.find('origin').get('xyz').split()]
    assert xyz == pytest.approx([0.342, 0.0, 0.1])
    assert joints['zed_camera_center_joint'].find('parent').get('link') == 'zed_camera_link'
    optical = joints['zed_left_camera_optical_joint'].find('child').get('link')
    assert optical == 'zed_left_camera_frame_optical'


def test_camera_args_propagate():
    _, links, joints = parse(run_xacro(simulation='false', camera_name='zedx', camera_model='zed2',
                                       camera_x='0.5', camera_z='0.2', camera_pitch='0.1'))
    assert 'zedx_camera_link' in links and 'zedx_left_camera_frame_optical' in links
    assert 'zed_camera_link' not in links
    origin = joints['camera_mount_joint'].find('origin')
    assert [float(v) for v in origin.get('xyz').split()] == pytest.approx([0.5, 0.0, 0.2])
    assert [float(v) for v in origin.get('rpy').split()] == pytest.approx([0.0, 0.1, 0.0])


def test_real_rover_all_joints_fixed(real_urdf):
    _, _, joints = parse(real_urdf)
    not_fixed = {n: j.get('type') for n, j in joints.items() if j.get('type') != 'fixed'}
    assert not not_fixed, f'non-fixed joints on the real rover: {not_fixed}'
    root = ET.fromstring(real_urdf)
    assert root.find('gazebo') is None, 'Gazebo plugins must not be included when simulation=false'


def test_simulation_variant(sim_urdf):
    root, links, joints = parse(sim_urdf)
    assert joints['left_rocker_joint'].get('type') == 'revolute'
    assert joints['wheel_lr'].get('type') == 'continuous'
    assert root.find('gazebo') is not None
    assert (ROVER_LINKS | CAMERA_LINKS) <= links
    # gazebo sensor block must reference an existing link (was camera_frame in the old stack)
    refs = {g.get('reference') for g in root.findall('gazebo') if g.get('reference')}
    assert refs <= links, f'gazebo references to unknown links: {refs - links}'


def test_mesh_files_exist(real_urdf):
    root = ET.fromstring(real_urdf)
    meshes = {m.get('filename') for m in root.iter('mesh')}
    ours = {m for m in meshes if m.startswith('package://hal_description/')}
    assert ours, 'no hal_description meshes referenced'
    for m in ours:
        rel = m[len('package://hal_description/'):]
        assert os.path.isfile(os.path.join(PKG_DIR, rel)), f'missing mesh {rel}'
    leftovers = {m for m in meshes if 'meshes_gazebo' in m or 'hal_main' in m}
    assert not leftovers, f'old package references: {leftovers}'


def test_gps_link_both_variants(real_urdf, sim_urdf):
    for urdf in (real_urdf, sim_urdf):
        _, links, joints = parse(urdf)
        assert 'gps_link' in links
        assert joints['gps_joint'].find('parent').get('link') == 'chassis'


def test_imu_link_only_in_simulation(real_urdf, sim_urdf):
    # real rover: zed_imu_link comes from the ZED wrapper (publish_imu_tf),
    # a second publisher would clash
    _, links, _ = parse(real_urdf)
    assert 'zed_imu_link' not in links
    # simulation: identity under the left (depth) camera frame, as the wrapper publishes it
    _, links, joints = parse(sim_urdf)
    imu = joints['zed_imu_joint']
    assert imu.find('parent').get('link') == 'zed_left_camera_frame'
    assert imu.find('child').get('link') == 'zed_imu_link'
    assert [float(v) for v in imu.find('origin').get('xyz').split()] == pytest.approx([0, 0, 0])


def _sensors(urdf):
    root = ET.fromstring(urdf)
    return {s.get('name'): s for g in root.findall('gazebo') for s in g.findall('sensor')}


def test_sim_sensors_match_zed_wrapper(sim_urdf):
    sensors = _sensors(sim_urdf)
    expected = {
        'zed_imu': ('imu', '/zed/zed_node/imu/data', 'zed_imu_link'),
        'zed_magnetometer': ('magnetometer', '/zed/zed_node/imu/mag', 'zed_imu_link'),
        'gps': ('navsat', '/fix', 'gps_link'),
    }
    for name, (stype, topic, frame) in expected.items():
        s = sensors[name]
        assert s.get('type') == stype
        assert s.find('topic').text == topic
        assert s.find('frame_id').text == frame


def test_sim_config_overrides_defaults(tmp_path):
    cfg = tmp_path / 'sim.yaml'
    cfg.write_text('sensors:\n  imu: {rate: 123.0}\n  gps: {topic: /gps/fix}\n')
    sensors = _sensors(run_xacro(simulation='true', sim_config=str(cfg)))
    assert float(sensors['zed_imu'].find('update_rate').text) == 123.0
    assert sensors['gps'].find('topic').text == '/gps/fix'
    # keys missing in the YAML keep their defaults
    assert float(sensors['zed_magnetometer'].find('update_rate').text) == 50.0
