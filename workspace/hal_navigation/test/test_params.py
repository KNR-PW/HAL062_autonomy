# Static checks of the Nav2 parameter files (T3.1 in docs/NAV2_DESCRIPTION_PORT_PLAN.md):
# the Nav2 1.5 migration rules and the frame/topic consistency the launch file relies on.
import glob
import os
import struct

import pytest
import yaml

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PARAM_FILES = sorted(glob.glob(os.path.join(PKG_DIR, 'config', 'nav2_params*.yaml')))
EXPECTED_NODES = {
    'map_server', 'controller_server', 'smoother_server', 'planner_server', 'behavior_server',
    'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'local_costmap', 'global_costmap',
}
CMD_VEL_NODES = ['controller_server', 'velocity_smoother', 'behavior_server']


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def params(doc, node):
    return doc[node]['ros__parameters']


def walk(node, path=()):
    if isinstance(node, dict):
        for k, v in node.items():
            yield from walk(v, path + (k,))
    else:
        yield path, node


@pytest.fixture(params=PARAM_FILES, ids=[os.path.basename(p) for p in PARAM_FILES])
def doc(request):
    return load(request.param)


def test_param_files_exist():
    assert PARAM_FILES, 'no config/nav2_params*.yaml'
    assert os.path.join(PKG_DIR, 'config', 'nav2_params.yaml') in PARAM_FILES


def test_expected_nodes_and_no_legacy_sections(doc):
    assert EXPECTED_NODES <= set(doc), EXPECTED_NODES - set(doc)
    legacy = {k for k in doc if k.endswith('_rclcpp_node') or k.endswith('_client')}
    assert not legacy, f'legacy Humble helper-node sections: {legacy}'
    for cm in ('local_costmap', 'global_costmap'):
        inner = set(doc[cm]) - {cm}
        assert not inner, f'{cm}: legacy sub-sections {inner}'


def test_plugin_names_use_double_colon(doc):
    for path, value in walk(doc):
        if path and path[-1] == 'plugin':
            assert '::' in value and '/' not in value, f'{".".join(path)} = {value}'


def test_no_custom_costmap_plugins(doc):
    for cm in ('local_costmap', 'global_costmap'):
        p = params(doc[cm], cm)
        for layer in p['plugins']:
            plugin = p[layer]['plugin']
            assert plugin.startswith('nav2_costmap_2d::'), \
                f'{cm}.{layer}: {plugin} (no custom plugins in this port)'


def test_cmd_vel_is_unstamped_twist(doc):
    for node in CMD_VEL_NODES:
        assert params(doc, node).get('enable_stamped_cmd_vel') is False, node


def test_behavior_server_nav2_15_keys(doc):
    p = params(doc, 'behavior_server')
    for key in ('local_frame', 'global_frame', 'robot_base_frame',
                'local_costmap_topic', 'local_footprint_topic'):
        assert key in p, key
    assert 'costmap_topic' not in p and 'footprint_topic' not in p
    assert p['local_frame'] == 'odom_2d' and p['global_frame'] == 'map_2d'


def test_bt_navigator_has_no_builtin_plugin_list(doc):
    p = params(doc, 'bt_navigator')
    assert 'plugin_lib_names' not in p, 'Nav2 1.5 loads built-in BT plugins automatically'
    assert p['global_frame'] == 'map_2d' and p['robot_base_frame'] == 'rover_2d'


def test_frames_consistent(doc):
    g = params(doc['global_costmap'], 'global_costmap')
    lc = params(doc['local_costmap'], 'local_costmap')
    assert g['global_frame'] == 'map_2d' and g['robot_base_frame'] == 'rover_2d'
    assert lc['global_frame'] == 'odom_2d' and lc['robot_base_frame'] == 'rover_2d'
    assert params(doc, 'map_server')['frame_id'] == 'map_2d'
    assert params(doc, 'behavior_server')['robot_base_frame'] == 'rover_2d'


def test_odom_topic_consistent(doc):
    bt = params(doc, 'bt_navigator')['odom_topic']
    vs = params(doc, 'velocity_smoother')['odom_topic']
    cs = params(doc, 'controller_server')['odom_topic']
    assert bt == vs == cs == '/visual_slam/tracking/odometry'


def test_costmap_obstacle_sources(doc):
    lc = params(doc['local_costmap'], 'local_costmap')
    gc = params(doc['global_costmap'], 'global_costmap')
    for cm in (lc, gc):
        assert 'obstacle_layer' in cm['plugins']
        cloud = cm['obstacle_layer']['cloud']
        assert cloud['data_type'] == 'PointCloud2'
        assert cloud['topic'].startswith('/zed/zed_node/')
        assert 0 < cloud['min_obstacle_height'] < cloud['max_obstacle_height']
    assert lc['obstacle_layer']['cloud']['topic'] == gc['obstacle_layer']['cloud']['topic']
    local_cloud = lc['obstacle_layer']['cloud']
    assert local_cloud['obstacle_max_range'] <= 2 * max(lc['width'], lc['height'])
    # plugin order: static map first, live obstacles on top, inflation last
    assert gc['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer']


def test_speed_limits_consistent(doc):
    fp = params(doc, 'controller_server')['FollowPath']
    vs = params(doc, 'velocity_smoother')
    assert fp['max_vel_x'] <= vs['max_velocity'][0] + 1e-9
    assert fp['max_vel_theta'] <= vs['max_velocity'][2] + 1e-9


def test_maps_complete():
    maps = sorted(glob.glob(os.path.join(PKG_DIR, 'maps', 'costmap_*.yaml')))
    names = {os.path.basename(m)[len('costmap_'):-len('.yaml')] for m in maps}
    assert names == {'blank', 'blank_large', 'erc', 'mel'}, names
    for m in maps:
        d = load(m)
        image = os.path.join(os.path.dirname(m), d['image'])
        assert os.path.isfile(image), f'{m}: image {d["image"]} missing'
        assert len(d['origin']) == 3 and d['resolution'] > 0
        # the rover starts at (0, 0) of the map frame: it must lie inside every map
        # (the old blank_large yaml had origin -1000 with a 205 m image -> planner error 203)
        with open(image, 'rb') as f:
            width, height = struct.unpack('>II', f.read(32)[16:24])
        ox, oy = d['origin'][0], d['origin'][1]
        x_max, y_max = ox + width * d['resolution'], oy + height * d['resolution']
        assert ox < 0 < x_max, f'{m}: x=0 outside [{ox}, {x_max}]'
        assert oy < 0 < y_max, f'{m}: y=0 outside [{oy}, {y_max}]'
