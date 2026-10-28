import os
import xml.etree.ElementTree as ET

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

PACKAGE = 'proxima_description'

VARIANTS = [
    ('proxima.urdf.xacro', {'manipulator': 'none'}),
    ('proxima.urdf.xacro', {'manipulator': '6DOF'}),
    ('proxima.urdf.xacro', {'manipulator': '6DOF', 'manipulator_only': 'true'}),
]


def _variant_id(variant):
    file_name, mappings = variant
    args = ','.join(f'{k}={v}' for k, v in mappings.items())
    return f'{file_name}[{args}]'


@pytest.fixture(params=VARIANTS, ids=_variant_id)
def robot(request):
    file_name, mappings = request.param
    path = os.path.join(get_package_share_directory(PACKAGE), 'robots', file_name)
    doc = xacro.process_file(path, mappings=dict(mappings))
    return ET.fromstring(doc.toxml())


def _links(robot):
    return [link.get('name') for link in robot.findall('link')]


def _joints(robot):
    return robot.findall('joint')


def test_names_are_unique(robot):
    links = _links(robot)
    joints = [joint.get('name') for joint in _joints(robot)]
    assert len(links) == len(set(links)), f'duplicate links: {links}'
    assert len(joints) == len(set(joints)), f'duplicate joints: {joints}'


def test_joints_reference_existing_links(robot):
    links = set(_links(robot))
    for joint in _joints(robot):
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        assert parent in links, f'{joint.get("name")}: unknown parent {parent}'
        assert child in links, f'{joint.get("name")}: unknown child {child}'


def test_single_tree(robot):
    links = set(_links(robot))
    children = [joint.find('child').get('link') for joint in _joints(robot)]
    assert len(children) == len(set(children)), 'a link has more than one parent'

    roots = links - set(children)
    assert len(roots) == 1, f'expected exactly one root link, got {sorted(roots)}'

    edges = {}
    for joint in _joints(robot):
        edges.setdefault(joint.find('parent').get('link'), []).append(
            joint.find('child').get('link')
        )
    reached, stack = set(), list(roots)
    while stack:
        link = stack.pop()
        reached.add(link)
        stack.extend(edges.get(link, []))
    assert reached == links, f'unreachable links: {sorted(links - reached)}'


def test_meshes_exist(robot):
    for mesh in robot.iter('mesh'):
        uri = mesh.get('filename')
        assert uri.startswith('package://'), f'mesh is not a package:// URI: {uri}'
        package, _, relative = uri[len('package://') :].partition('/')
        path = os.path.join(get_package_share_directory(package), relative)
        assert os.path.isfile(path), f'missing mesh: {uri}'
