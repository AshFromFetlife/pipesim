"""Self-contained, textured glTF skins for the editor's physical mannequins.

Imported skins never replace collision geometry or inertial properties. All
dependencies are embedded on import, so loading a saved design cannot make the
browser fetch arbitrary network URLs or neighbouring files from the workspace.
"""
from __future__ import annotations
import base64
import copy
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import struct
import urllib.parse
from .document import DocumentError

MAX_ASSET_BYTES = 20 * 1024 * 1024
MAX_NODES = 4096
SEGMENTS = {'pelvis', 'lumbar', 'thorax', 'neck', 'head'} | {
    side + '_' + name for side in ('left', 'right')
    for name in ('clavicle', 'upper_arm', 'forearm', 'hand', 'thigh', 'shin', 'foot')}
EXTRA_MODES = {'fixed_to_parent', 'fixed_angle_to_parent', 'weighted_ball_joint', 'damped_spring'}
VRM_SEGMENTS = dict(zip(
    ('hips', 'spine', 'chest', 'neck', 'head', 'leftShoulder', 'rightShoulder',
     'leftUpperArm', 'rightUpperArm', 'leftLowerArm', 'rightLowerArm',
     'leftHand', 'rightHand', 'leftUpperLeg', 'rightUpperLeg',
     'leftLowerLeg', 'rightLowerLeg', 'leftFoot', 'rightFoot'),
    ('pelvis', 'lumbar', 'thorax', 'neck', 'head', 'left_clavicle', 'right_clavicle',
     'left_upper_arm', 'right_upper_arm', 'left_forearm', 'right_forearm',
     'left_hand', 'right_hand', 'left_thigh', 'right_thigh',
     'left_shin', 'right_shin', 'left_foot', 'right_foot')))

SEGMENT_ORDER = ['pelvis', 'lumbar', 'thorax', 'neck', 'head'] + [
    side + '_' + name for side in ('left', 'right')
    for name in ('clavicle', 'upper_arm', 'forearm', 'hand', 'thigh', 'shin', 'foot')]
NAME_ALIASES = {
    'pelvis': ['hips', 'hip', 'pelvis', 'roothips'],
    'lumbar': ['spine', 'spine01', 'spine1', 'abdomen', 'lowerback'],
    'thorax': ['chest', 'upperchest', 'spine2', 'spine02', 'spine3', 'spine03', 'upperback', 'chestupper'],
    'neck': ['neck', 'neck1', 'neck01'], 'head': ['head', 'head1', 'head01'],
}
for _side in ('left', 'right'):
    _short = _side[0]
    for _segment, _names in {
            'clavicle': ['shoulder', 'clavicle', 'collar'], 'upper_arm': ['upperarm', 'arm'],
            'forearm': ['lowerarm', 'forearm'], 'hand': ['hand', 'wrist'],
            'thigh': ['upperleg', 'upleg', 'thigh'], 'shin': ['lowerleg', 'leg', 'calf', 'shin'],
            'foot': ['foot', 'ankle']}.items():
        NAME_ALIASES[_side + '_' + _segment] = [value for name in _names
                                                for value in (_side + name, name + _side, _short + name, name + _short)]
for _segment in SEGMENT_ORDER:
    NAME_ALIASES[_segment].append(_segment.replace('_', ''))
NAME_KEYWORDS = {
    'pelvis': ['pelvis', 'hips', 'hip'], 'lumbar': ['abdomen', 'lowerback', 'spine'],
    'thorax': ['upperchest', 'chest', 'upperback', 'spine'], 'neck': ['neck'], 'head': ['head'],
    'clavicle': ['shoulder', 'clavicle', 'collar'], 'upper_arm': ['upperarm', 'arm'],
    'forearm': ['forearm', 'lowerarm'], 'hand': ['hand', 'wrist'],
    'thigh': ['thigh', 'upperleg', 'upleg'], 'shin': ['shin', 'calf', 'lowerleg', 'leg'],
    'foot': ['foot', 'ankle'],
}
INFERENCE_CHAINS = [['pelvis', 'lumbar', 'thorax', 'neck', 'head']] + [
    [side + '_' + part for part in ('clavicle', 'upper_arm', 'forearm', 'hand')]
    for side in ('left', 'right')] + [
    [side + '_' + part for part in ('thigh', 'shin', 'foot')] for side in ('left', 'right')]


def _canonical_bone_name(name):
    value = re.sub(r'^.*[|:]', '', str(name or '').lower())
    value = re.sub(r'^(mixamorig|bip0?1|def|org|j_bip_c|j_bip_l|j_bip_r)[_. -]*', '', value)
    return re.sub(r'[^a-z0-9]', '', value)


def _bone_side(name):
    raw = re.sub(r'([a-z])([A-Z])', r'\1 \2', str(name or '')).lower()
    tokens = [token for token in re.split(r'[^a-z0-9]+', raw) if token]
    compact = _canonical_bone_name(name)
    if 'left' in compact or 'l' in tokens: return 'left'
    if 'right' in compact or 'r' in tokens: return 'right'
    terms = {term for values in NAME_KEYWORDS.values() for term in values}
    if any(compact.startswith('l' + term) or compact.endswith(term + 'l') for term in terms): return 'left'
    if any(compact.startswith('r' + term) or compact.endswith(term + 'r') for term in terms): return 'right'
    return None


def _bone_name_score(name, segment):
    compact = _canonical_bone_name(name)
    side = segment.split('_', 1)[0] if segment.startswith(('left_', 'right_')) else None
    found_side = _bone_side(name)
    if side and found_side != side: return -1
    if not side and found_side: return -1
    part = segment.split('_', 1)[1] if side else segment
    if part == 'upper_arm' and ('forearm' in compact or 'lowerarm' in compact): return -1
    if part == 'shin' and any(value in compact for value in ('upperleg', 'upleg', 'thigh')): return -1
    scores = []
    for alias in NAME_ALIASES[segment]:
        if compact == alias: scores.append(140 + len(alias))
        elif alias in compact: scores.append(90 + len(alias))
    for keyword in NAME_KEYWORDS[part]:
        if compact == keyword: scores.append(120 + len(keyword))
        elif keyword in compact: scores.append(60 + len(keyword))
    return (max(scores) + (25 if side else 0)) if scores else -1


def _auto_bone_map(nodes, joint_ids, mapping, parents=None):
    """Add unambiguous name matches, then conservative skeletal-chain inferences."""
    result = dict(mapping)
    used = set(result.values())
    proposals = []
    for segment in SEGMENT_ORDER:
        if segment in result: continue
        scored = [(i, _bone_name_score(nodes[i].get('name', ''), segment)) for i in joint_ids
                  if i not in used and nodes[i].get('name', '').strip()]
        scored = [(i, score) for i, score in scored if score >= 0]
        if not scored: continue
        best = max(score for _, score in scored)
        matches = [i for i, score in scored if score == best]
        if len(matches) == 1: proposals.append((best, segment, matches[0]))
    for _, segment, i in sorted(proposals, reverse=True):
        if segment not in result and i not in used: result[segment] = i; used.add(i)
    if parents:
        def joint_parent(i):
            parent = parents.get(i)
            while parent is not None and parent not in joint_ids: parent = parents.get(parent)
            return parent
        children = {i: [] for i in joint_ids}
        for i in joint_ids:
            parent = joint_parent(i)
            if parent in children: children[parent].append(i)
        changed = True
        while changed:
            changed = False
            for chain in INFERENCE_CHAINS:
                for before, after in zip(chain, chain[1:]):
                    if before in result and after not in result:
                        candidates = [i for i in children.get(result[before], []) if i not in used]
                        if len(candidates) == 1: result[after] = candidates[0]; used.add(candidates[0]); changed = True
                    if after in result and before not in result:
                        parent = joint_parent(result[after])
                        if parent is not None and parent not in used: result[before] = parent; used.add(parent); changed = True
    return result


def _node_matrix(node):
    if 'matrix' in node: return node['matrix']
    x, y, z, w = node.get('rotation', [0, 0, 0, 1])
    sx, sy, sz = node.get('scale', [1, 1, 1])
    tx, ty, tz = node.get('translation', [0, 0, 0])
    return [(1 - 2*y*y - 2*z*z)*sx, (2*x*y + 2*z*w)*sx, (2*x*z - 2*y*w)*sx, 0,
            (2*x*y - 2*z*w)*sy, (1 - 2*x*x - 2*z*z)*sy, (2*y*z + 2*x*w)*sy, 0,
            (2*x*z + 2*y*w)*sz, (2*y*z - 2*x*w)*sz, (1 - 2*x*x - 2*y*y)*sz, 0,
            tx, ty, tz, 1]


def _matrix_multiply(a, b):
    return [sum(a[row + k*4] * b[k + column*4] for k in range(4))
            for column in range(4) for row in range(4)]


def _fail(message):
    raise DocumentError('Human model: ' + message)


def _decode(value):
    if not isinstance(value, str) or len(value) > (MAX_ASSET_BYTES + 2) // 3 * 4:
        _fail('asset exceeds the 20 MiB import limit')
    try:
        result = base64.b64decode(value, validate=True)
    except (ValueError, TypeError) as exc:
        raise DocumentError('Human model: invalid base64 file content') from exc
    if len(result) > MAX_ASSET_BYTES:
        _fail('asset exceeds the 20 MiB import limit')
    return result


def _relative_name(value):
    if not isinstance(value, str) or not value or '\\' in value:
        _fail('bundle filenames must be relative paths using forward slashes')
    value = urllib.parse.unquote(value)
    path = PurePosixPath(value)
    if (path.is_absolute() or any(p in ('..', '.git', '.codex', '.agents') for p in path.parts)
            or ':' in value or '?' in value or '#' in value or '\\' in value or '\0' in value):
        _fail('bundle paths must stay inside the selected model folder')
    return path.as_posix()


def _read_container(raw, suffix):
    binary = None
    if suffix in ('.glb', '.vrm'):
        if len(raw) < 20 or raw[:4] != b'glTF':
            _fail('invalid GLB header')
        _, version, length = struct.unpack_from('<4sII', raw)
        if version != 2 or length != len(raw):
            _fail('only complete glTF 2.0 GLB files are supported')
        offset = 12
        payload = None
        while offset < len(raw):
            if offset + 8 > len(raw): _fail('truncated GLB chunk')
            size, kind = struct.unpack_from('<II', raw, offset)
            offset += 8
            if size % 4 or offset + size > len(raw): _fail('invalid GLB chunk length')
            chunk = raw[offset:offset + size]
            offset += size
            if kind == 0x4E4F534A:
                if payload is not None or offset != 20 + size: _fail('invalid GLB JSON chunk order')
                payload = chunk
            elif kind == 0x004E4942:
                if binary is not None: _fail('multiple GLB binary chunks')
                binary = chunk
        if payload is None: _fail('GLB has no JSON chunk')
    else:
        payload = raw
    try:
        document = json.loads(payload.decode('utf-8'), parse_constant=lambda x: _fail('non-finite number'))
    except (ValueError, UnicodeError, RecursionError) as exc:
        raise DocumentError('Human model: invalid glTF JSON') from exc
    if (not isinstance(document, dict) or not isinstance(document.get('asset'), dict)
            or document['asset'].get('version') != '2.0'):
        _fail('a glTF 2.0 asset is required')
    for key in ('nodes', 'skins', 'meshes', 'buffers', 'bufferViews', 'accessors', 'images', 'textures', 'materials', 'scenes'):
        value = document.get(key, [])
        if not isinstance(value, list) or any(not isinstance(item, dict) for item in value):
            _fail(key + ' must be an array of objects')
    if not isinstance(document.get('extensions', {}), dict): _fail('extensions must be an object')
    for extension in ('VRMC_vrm', 'VRM'):
        value = document.get('extensions', {}).get(extension, {})
        if not isinstance(value, dict) or not isinstance(value.get('humanoid', {}), dict):
            _fail('invalid VRM humanoid metadata')
        bones = value.get('humanoid', {}).get('humanBones', {} if extension == 'VRMC_vrm' else [])
        if extension == 'VRMC_vrm' and (not isinstance(bones, dict) or any(not isinstance(v, dict) for v in bones.values())):
            _fail('invalid VRM humanoid bone map')
        if extension == 'VRM' and (not isinstance(bones, list) or any(not isinstance(v, dict) or 'bone' not in v for v in bones)):
            _fail('invalid VRM humanoid bone list')
    return document, binary


def _embed(document, binary, siblings, filename):
    buffers = []
    total = 0
    allowed_data_types = {'application/octet-stream', 'application/gltf-buffer', 'image/png', 'image/jpeg', 'image/webp'}
    for kind in ('buffers', 'images'):
        for index, item in enumerate(document.get(kind, [])):
            if not isinstance(item, dict): _fail('invalid ' + kind + ' entry')
            if 'uri' not in item:
                if kind == 'buffers':
                    if index != 0 or binary is None: _fail('missing binary buffer')
                    payload = binary
                else:
                    if 'bufferView' not in item: _fail('image is missing its data')
                    if item.get('mimeType') not in ('image/png', 'image/jpeg', 'image/webp'):
                        _fail('embedded textures must be PNG, JPEG or WebP')
                    continue
            else:
                uri = item['uri']
                if not isinstance(uri, str): _fail('invalid resource URI')
                if uri.startswith('data:'):
                    match = re.fullmatch(r'data:([^;,]+);base64,(.*)', uri, re.DOTALL)
                    if not match or match[1] not in allowed_data_types: _fail('unsupported embedded resource type')
                    payload = _decode(match[2])
                else:
                    relative = _relative_name(uri)
                    key = (PurePosixPath(filename).parent / relative).as_posix()
                    if key not in siblings and relative in siblings: key = relative
                    if key not in siblings:
                        # A normal browser multi-file picker supplies basenames,
                        # even when glTF references a textures/ subdirectory.
                        matches = [name for name in siblings if PurePosixPath(name).name == PurePosixPath(relative).name]
                        if len(matches) > 1: _fail('ambiguous companion filename: ' + relative)
                        if matches: key = matches[0]
                    if key not in siblings: _fail('select the companion file ' + relative + ' together with the model')
                    payload = siblings[key]
                    mime = 'application/octet-stream' if kind == 'buffers' else {
                        '.png': 'image/png', '.jpg': 'image/jpeg', '.jpeg': 'image/jpeg', '.webp': 'image/webp'
                    }.get(PurePosixPath(relative).suffix.lower())
                    if mime is None: _fail('textures must be PNG, JPEG or WebP')
                    item['uri'] = 'data:' + mime + ';base64,' + base64.b64encode(payload).decode('ascii')
            total += len(payload)
            if total > MAX_ASSET_BYTES: _fail('embedded resources exceed the 20 MiB import limit')
            if kind == 'buffers':
                length = item.get('byteLength')
                if type(length) is not int or length < 0 or length > len(payload): _fail('invalid buffer byteLength')
                buffers.append(payload)
    # URI-bearing extensions cannot bypass the offline resource policy.
    def inspect(value, depth=0):
        if depth > 64: _fail('asset nesting is too deep')
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'uri' and (not isinstance(child, str) or not child.startswith('data:')):
                    _fail('external extension resources are not supported')
                inspect(child, depth + 1)
        elif isinstance(value, list):
            for child in value: inspect(child, depth + 1)
    inspect(document)
    return buffers


def _metadata(document, buffers):
    unsupported = {'KHR_draco_mesh_compression', 'EXT_meshopt_compression', 'KHR_texture_basisu'}
    if unsupported.intersection(document.get('extensionsUsed', [])):
        _fail('compressed meshes or KTX2 textures need an uncompressed glTF/GLB export')
    nodes = document.get('nodes', [])
    skins = document.get('skins', [])
    meshes = document.get('meshes', [])
    accessors = document.get('accessors', [])
    views = document.get('bufferViews', [])
    if not isinstance(nodes, list) or not 1 <= len(nodes) <= MAX_NODES:
        _fail('the rig must have between 1 and 4096 nodes')
    if not skins or not meshes: _fail('a rigged mesh with skin weights is required')
    def index(value, items, label):
        if type(value) is not int or not 0 <= value < len(items): _fail('invalid ' + label + ' index')
        return items[value]
    for view in views:
        buffer = index(view.get('buffer'), buffers, 'buffer')
        offset, length = view.get('byteOffset', 0), view.get('byteLength')
        if type(offset) is not int or type(length) is not int or offset < 0 or length < 0 or offset + length > len(buffer):
            _fail('buffer view exceeds its binary data')
    for accessor in accessors:
        if type(accessor.get('count')) is not int or not 0 <= accessor['count'] <= 5_000_000:
            _fail('invalid accessor count')
        component_size = {5120: 1, 5121: 1, 5122: 2, 5123: 2, 5125: 4, 5126: 4}.get(accessor.get('componentType'))
        count = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT2': 4, 'MAT3': 9, 'MAT4': 16}.get(accessor.get('type'))
        if component_size is None or count is None: _fail('invalid accessor component type')
        if 'bufferView' in accessor:
            view = index(accessor['bufferView'], views, 'buffer view')
            element = component_size * count
            # Small integer matrices pad each column to a four-byte boundary.
            if accessor['type'] in ('MAT2', 'MAT3'):
                dimension = 2 if accessor['type'] == 'MAT2' else 3
                element = ((dimension * component_size + 3) // 4 * 4) * dimension
            stride, offset = view.get('byteStride', element), accessor.get('byteOffset', 0)
            if type(stride) is not int or stride < element or type(offset) is not int or offset < 0:
                _fail('invalid accessor offset or stride')
            required = offset + (accessor['count'] - 1) * stride + element if accessor['count'] else offset
            if required > view['byteLength']: _fail('accessor exceeds its buffer view')
        elif 'sparse' not in accessor:
            _fail('accessor is missing its buffer view')
    for image in document.get('images', []):
        if 'bufferView' in image: index(image['bufferView'], views, 'image buffer view')
    parents = {}
    for node_id, node in enumerate(nodes):
        if not isinstance(node, dict): _fail('invalid rig node')
        if 'name' in node and not isinstance(node['name'], str): _fail('bone names must be strings')
        for key, length in (('translation', 3), ('rotation', 4), ('scale', 3), ('matrix', 16)):
            if key in node:
                values = node[key]
                if not isinstance(values, list) or len(values) != length or any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
                    _fail('invalid node ' + key)
        for child in node.get('children', []):
            index(child, nodes, 'child node')
            if child in parents or child == node_id: _fail('rig nodes must form a tree')
            parents[child] = node_id
        if 'mesh' in node: index(node['mesh'], meshes, 'mesh')
        if 'skin' in node: index(node['skin'], skins, 'skin')
    for node_id in range(len(nodes)):
        seen = set()
        while node_id in parents:
            if node_id in seen: _fail('cyclic rig hierarchy')
            seen.add(node_id)
            node_id = parents[node_id]
    joint_ids = set()
    for skin in skins:
        if not skin.get('joints'): _fail('skin has no bones')
        for joint in skin['joints']:
            index(joint, nodes, 'skin joint')
            joint_ids.add(joint)
        if 'inverseBindMatrices' in skin:
            matrices = index(skin['inverseBindMatrices'], accessors, 'inverse bind accessor')
            if matrices.get('type') != 'MAT4' or matrices['count'] < len(skin['joints']):
                _fail('invalid inverse bind matrices')
    vertices = 0
    uv_sets = set()
    skinned = False
    for node in nodes:
        if 'skin' not in node or 'mesh' not in node: continue
        primitives = meshes[node['mesh']].get('primitives', [])
        if not isinstance(primitives, list): _fail('mesh primitives must be an array')
        for primitive in primitives:
            if not isinstance(primitive, dict): _fail('invalid mesh primitive')
            attributes = primitive.get('attributes', {})
            if not isinstance(attributes, dict): _fail('invalid mesh attributes')
            if not {'POSITION', 'JOINTS_0', 'WEIGHTS_0'} <= attributes.keys():
                _fail('each skinned mesh needs positions, joint indices and vertex weights')
            position = index(attributes['POSITION'], accessors, 'position accessor')
            vertices += position['count']
            for name, value in attributes.items():
                attribute = index(value, accessors, 'vertex attribute')
                if attribute['count'] != position['count']: _fail('vertex attribute lengths do not match')
                if name.startswith('TEXCOORD_'): uv_sets.add(name)
            skinned = True
    if not skinned or vertices == 0: _fail('the model contains no weighted mesh vertices')
    if vertices > 2_000_000: _fail('the model exceeds two million skinned vertices')
    extensions = document.get('extensions', {})
    standard = extensions.get('VRMC_vrm', {}).get('humanoid', {}).get('humanBones', {})
    if not standard:
        standard = {item['bone']: item for item in extensions.get('VRM', {}).get('humanoid', {}).get('humanBones', [])}
    bone_map = {VRM_SEGMENTS[name]: value['node'] for name, value in standard.items()
                if name in VRM_SEGMENTS and isinstance(value, dict) and value.get('node') in joint_ids}
    bone_map = _auto_bone_map(nodes, sorted(joint_ids), bone_map, parents)
    world_matrices = {}
    def world_matrix(node_id):
        if node_id not in world_matrices:
            local = _node_matrix(nodes[node_id])
            world_matrices[node_id] = (_matrix_multiply(world_matrix(parents[node_id]), local)
                                       if node_id in parents else local)
        return world_matrices[node_id]
    def nearest_joint_parent(node_id):
        parent = parents.get(node_id)
        while parent is not None and parent not in joint_ids: parent = parents.get(parent)
        return parent
    bones = []
    for i in sorted(joint_ids):
        authored_name = nodes[i].get('name', '').strip()
        matrix = world_matrix(i)
        parent = parents.get(i)
        bones.append({'name': authored_name or 'Bone ' + str(i), 'named': bool(authored_name), 'index': i,
                      'parent': parent,
                      'parent_name': ((nodes[parent].get('name', '').strip() or 'Bone ' + str(parent))
                                      if parent is not None else None),
                      'parent_bone': nearest_joint_parent(i),
                      'position': [matrix[12], matrix[13], matrix[14]]})
    return {'format': 'glTF 2.0', 'bones': bones, 'bone_map': bone_map,
            'vertex_count': vertices, 'mesh_count': len(meshes), 'skin_count': len(skins),
            'texture_count': len(document.get('textures', [])), 'uv_sets': sorted(uv_sets)}


def prepare_import(filename, content_base64, files=None):
    """Validate selected files and return a portable, offline-only glTF/GLB."""
    filename = _relative_name(filename)
    suffix = PurePosixPath(filename).suffix.lower()
    if suffix not in ('.glb', '.gltf', '.vrm'): _fail('choose a rigged .glb, .gltf or .vrm file')
    raw = _decode(content_base64)
    siblings = {}
    total = len(raw)
    if files is not None and not isinstance(files, list): _fail('companion files must be a list')
    for item in files or []:
        if not isinstance(item, dict) or not {'name', 'content_base64'} <= item.keys():
            _fail('each companion file needs a name and base64 content')
        name = _relative_name(item['name'])
        if name in siblings: _fail('duplicate companion filename: ' + name)
        value = _decode(item['content_base64'])
        total += len(value)
        if total > MAX_ASSET_BYTES: _fail('selected files exceed the 20 MiB import limit')
        siblings[name] = value
    document, binary = _read_container(raw, suffix)
    buffers = _embed(document, binary, siblings, filename)
    metadata = _metadata(document, buffers)
    payload = json.dumps(document, separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode('utf-8')
    if suffix in ('.glb', '.vrm'):
        payload += b' ' * (-len(payload) % 4)
        chunks = struct.pack('<II', len(payload), 0x4E4F534A) + payload
        if binary is not None: chunks += struct.pack('<II', len(binary), 0x004E4942) + binary
        payload = struct.pack('<4sII', b'glTF', 2, 12 + len(chunks)) + chunks
        suffix = '.glb'
    if len(payload) > MAX_ASSET_BYTES * 1.4: _fail('embedded asset exceeds the import limit')
    return payload, suffix, metadata


def import_human_model(directory, base, filename, content_base64, files=None):
    payload, suffix, metadata = prepare_import(filename, content_base64, files)
    directory, base = Path(directory), Path(base)
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / (hashlib.sha256(payload).hexdigest() + suffix)
    if target.is_symlink() or not target.resolve().is_relative_to(directory.resolve()):
        _fail('asset cache paths must stay inside the model directory')
    if not target.exists(): target.write_bytes(payload)
    config = {'file': Path(os.path.relpath(target, base)).as_posix(), 'name': PurePosixPath(filename).name,
              'enabled': True, 'bone_map': metadata['bone_map'], 'extra_bones': {},
              'default_extra_mode': 'fixed_to_parent', 'metadata': metadata}
    return {'render_model': config, 'metadata': metadata, 'asset': target}


def validate_stored_asset(path):
    """Apply the offline resource policy to files referenced by saved designs."""
    path = Path(path)
    if path.suffix.lower() not in ('.gltf', '.glb', '.vrm'): _fail('invalid saved asset extension')
    if path.stat().st_size > MAX_ASSET_BYTES * 1.4: _fail('saved asset exceeds the import limit')
    document, binary = _read_container(path.read_bytes(), path.suffix.lower())
    buffers = _embed(document, binary, {}, path.name)
    return _metadata(document, buffers)


def validate_render_model(config):
    if not isinstance(config, dict): _fail('render_model must be an object')
    if not isinstance(config.get('file'), str) or Path(config['file']).suffix.lower() not in ('.glb', '.gltf', '.vrm'):
        _fail('render_model.file must reference a glTF, GLB or VRM asset')
    if 'enabled' in config and not isinstance(config['enabled'], bool): _fail('enabled must be true or false')
    if config.get('default_extra_mode', 'fixed_to_parent') not in EXTRA_MODES: _fail('unknown extra bone mode')
    mapping = config.get('bone_map', {})
    if not isinstance(mapping, dict) or set(mapping) - SEGMENTS: _fail('invalid human segment in bone_map')
    if any(not isinstance(value, str) and (type(value) is not int or value < 0) for value in mapping.values()):
        _fail('bone mappings must be names or nonnegative glTF node indices')
    offsets = config.get('bone_offsets', {})
    if not isinstance(offsets, dict) or set(offsets) - SEGMENTS: _fail('invalid human segment in bone_offsets')
    for segment, value in offsets.items():
        if (not isinstance(value, list) or len(value) != 3
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
            _fail('bone offset for ' + segment + ' needs three finite angles')
    extras = config.get('extra_bones', {})
    if not isinstance(extras, dict): _fail('extra_bones must be an object')
    for bone, settings in extras.items():
        if not isinstance(settings, dict) or settings.get('mode', 'fixed_to_parent') not in EXTRA_MODES:
            _fail('invalid settings for extra bone ' + bone)
        for key in ('mass', 'stiffness', 'damping', 'limit_deg'):
            if key in settings and (type(settings[key]) not in (int, float) or not math.isfinite(settings[key]) or settings[key] < 0):
                _fail(key + ' must be finite and nonnegative')
        if 'rotation_deg' in settings:
            value = settings['rotation_deg']
            if not isinstance(value, list) or len(value) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
                _fail('extra bone rotation_deg needs three finite angles')


def scene_humans(assembly):
    """Neutral local geometry calibrates a skin independently of the posed body."""
    from .human import humanoid
    result = []
    instances = list(assembly.doc.get('objects', []))
    instances += [entry['instance'] for entry in assembly.doc.get('expanded_objects', [])]
    present = set()
    for instance in instances:
        if instance.get('template') != 'human' or not instance.get('render_model') or instance['id'] in present: continue
        if not any(pid.startswith(instance['id'] + '/') for pid in assembly.parts): continue
        present.add(instance['id'])
        parameters = {key: copy.deepcopy(value) for key, value in instance.get('parameters', {}).items()
                      if key in ('stature_mm', 'mass_kg', 'measurements', 'flexibility')}
        neutral = humanoid(**parameters)
        result.append({'id': instance['id'], 'render_model': copy.deepcopy(instance['render_model']),
                       'stature_mm': parameters.get('stature_mm', 1750),
                       'rest_parts': [{'id': p['id'], 'pose': p['pose']} for p in neutral['parts']],
                       'joints': neutral['joints']})
    return result
