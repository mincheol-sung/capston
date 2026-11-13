"""Persistence compatibility, complete-file commits and failure recovery contracts."""

from copy import deepcopy
from dataclasses import asdict
import json
import os
import stat
import zipfile

import numpy as np

import pytest

from semantic_slam.object_map import ObjectMap
from semantic_slam.object_map import ObjectRecord
from semantic_slam.object_map import Observation
from semantic_slam.pose_graph import NodePose
from semantic_slam.pose_graph import PoseGraph


def legacy_data(schema=2, anchored=True):
    """Build a legacy fixture without relying on the current writer."""
    points = [[0.01, 0.1, 0.2], [0.11, 0.2, 0.3], [0.21, 0.3, 0.4]]
    covariance = [0.01, 0, 0, 0, 0.02, 0, 0, 0, 0.03]
    item = {
        'object_id': 'chair-id', 'class_id': 56, 'class_name': 'chair',
        'position': [0.11, 0.2, 0.3], 'center': [0.11, 0.2, 0.3],
        'extent': [0.2, 0.2, 0.2], 'points': points, 'covariance': covariance,
        'last_seen': 2.0, 'count': 3, 'alias': 'window chair', 'status': 1,
        'movement_votes': 1, 'history': [[2.0, 0.11, 0.2, 0.3]],
    }
    if anchored:
        item['anchors'] = [{
            'graph_id': 'graph', 'trajectory_id': 0, 'left': 1.0, 'right': 2.0,
            'weight': 1.0, 'stamp': 2.0, 'points': deepcopy(points),
            'covariance': covariance[:],
            'pose': [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0], 'frozen': True,
        }]
    return {'schema': schema, 'map_id': 'lab', 'graph_id': 'graph' if anchored else '',
            'revision': 7, 'objects': [item]}


def memory_at(tmp_path, anchored=True):
    """Create an in-memory map with no implicit read or write of a user's DB."""
    data = legacy_data(anchored=anchored)
    memory = ObjectMap('lab', str(tmp_path / 'objects.json'), graph_id=data['graph_id'])
    record = ObjectRecord(**data['objects'][0])
    memory.objects[record.object_id] = record
    memory.revision, memory.dirty = 7, True
    return memory


@pytest.mark.parametrize('schema,anchored', [(1, False), (1, True), (2, False), (2, True)])
def test_legacy_load_does_not_write_until_dirty(tmp_path, schema, anchored):
    data = legacy_data(schema, anchored)
    path = tmp_path / 'legacy.json'
    path.write_text(json.dumps(data))
    before = path.read_bytes()
    memory = ObjectMap('lab', str(path), graph_id=data['graph_id'])
    assert not memory.dirty and memory.revision == 7
    memory.save()
    assert path.read_bytes() == before
    memory.set_alias('chair-id', 'new alias')
    memory.save()
    assert zipfile.is_zipfile(path)
    loaded = ObjectMap('lab', str(path), graph_id=data['graph_id'])
    assert loaded.objects['chair-id'].alias == 'new alias'


@pytest.mark.parametrize('anchored', [False, True])
def test_stored_float64_roundtrip_preserves_all_fields(tmp_path, anchored):
    memory = memory_at(tmp_path, anchored)
    expected = asdict(memory.objects['chair-id'])
    memory.save()
    assert not memory.dirty
    with zipfile.ZipFile(memory.path) as archive:
        assert set(archive.namelist()) == {'metadata.json', 'points.bin'}
        assert all(info.compress_type == zipfile.ZIP_STORED for info in archive.infolist())
        data = json.loads(archive.read('metadata.json'))
        raw = archive.read('points.bin')
    assert data['schema'] == 4 and data['generation']
    assert 'sha256' not in data and data['points_bytes'] == len(raw)
    reference = data['objects'][0]['points']
    assert reference == {'offset': 0, 'count': 3, 'dtype': '<f8', 'shape': [3, 3]}
    assert np.array_equal(np.frombuffer(raw[:72], dtype='<f8').reshape(3, 3), expected['points'])
    loaded = ObjectMap('lab', str(memory.path), graph_id=memory.graph_id)
    assert asdict(loaded.objects['chair-id']) == expected and loaded.revision == 7
    assert loaded.rebuild_ids == ({'chair-id'} if anchored else set())


def test_empty_map_and_clean_save(tmp_path):
    memory = ObjectMap('lab', str(tmp_path / 'empty.json'))
    memory.save()
    assert not memory.path.exists()
    memory.dirty = True
    memory.save()
    before = memory.path.read_bytes()
    assert not ObjectMap('lab', str(memory.path)).objects
    memory.save()
    assert memory.path.read_bytes() == before


def test_legacy_failed_load_never_changes_live_objects_or_revision(tmp_path):
    memory = memory_at(tmp_path)
    data = legacy_data()
    data['revision'] = 99
    data['objects'][0]['object_id'] = 'partially-loaded'
    invalid = deepcopy(data['objects'][0])
    invalid['object_id'], invalid['position'] = 'bad', [float('nan'), 0, 0]
    data['objects'].append(invalid)
    memory.path.write_text(json.dumps(data))
    previous_objects = memory.objects
    memory.rebuild_ids = {'existing-rebuild'}
    with pytest.raises(ValueError):
        memory.load()
    assert memory.objects is previous_objects and set(memory.objects) == {'chair-id'}
    assert memory.revision == 7 and memory.rebuild_ids == {'existing-rebuild'}


@pytest.mark.parametrize('corruption', [
    'duplicate', 'unknown_schema', 'boolean_schema', 'truncated',
])
def test_invalid_legacy_db_is_not_an_empty_or_partially_loaded_map(tmp_path, corruption):
    memory = memory_at(tmp_path)
    data = legacy_data()
    if corruption == 'duplicate':
        data['objects'].append(deepcopy(data['objects'][0]))
    elif corruption == 'unknown_schema':
        data['schema'] = 99
    elif corruption == 'boolean_schema':
        data['schema'] = True
    text = json.dumps(data)
    memory.path.write_text(text[:-8] if corruption == 'truncated' else text)
    previous = memory.objects
    with pytest.raises(ValueError):
        memory.load()
    assert memory.objects is previous and memory.revision == 7


def test_successful_reload_replaces_previous_objects_and_rebuild_ids(tmp_path):
    memory = memory_at(tmp_path)
    memory.save()
    memory.objects['extra'] = deepcopy(memory.objects['chair-id'])
    memory.rebuild_ids = {'extra'}
    memory.revision = 99
    memory.load()
    assert set(memory.objects) == {'chair-id'} and memory.rebuild_ids == {'chair-id'}
    assert memory.revision == 7


@pytest.mark.parametrize('failure', ['write', 'file_fsync', 'replace', 'directory_fsync'])
def test_failed_save_keeps_dirty_and_a_complete_db(tmp_path, monkeypatch, failure):
    memory = memory_at(tmp_path)
    memory.save()
    old = memory.path.read_bytes()
    memory.set_alias('chair-id', 'changed')
    original_fsync, original_replace = os.fsync, os.replace
    original_writestr = zipfile.ZipFile.writestr
    events = []

    def fsync(fd):
        event = 'directory_fsync' if stat.S_ISDIR(os.fstat(fd).st_mode) else 'file_fsync'
        events.append(event)
        if event == failure:
            raise OSError(f'injected {event}')
        original_fsync(fd)

    def replace(source, destination):
        events.append('replace')
        if failure == 'replace':
            raise OSError('injected replace')
        original_replace(source, destination)

    def writestr(archive, *args, **kwargs):
        if failure == 'write':
            raise OSError('injected write')
        return original_writestr(archive, *args, **kwargs)

    with monkeypatch.context() as patch:
        patch.setattr(os, 'fsync', fsync)
        patch.setattr(os, 'replace', replace)
        patch.setattr(zipfile.ZipFile, 'writestr', writestr)
        with pytest.raises(OSError, match='injected'):
            memory.save()
    assert memory.dirty
    assert not memory.path.with_suffix('.json.tmp').exists()
    loaded = ObjectMap('lab', str(memory.path), graph_id='graph')
    if failure == 'directory_fsync':
        assert loaded.objects['chair-id'].alias == 'changed'
        assert events == ['file_fsync', 'replace', 'directory_fsync']
    else:
        assert memory.path.read_bytes() == old
        assert loaded.objects['chair-id'].alias == 'window chair'
    memory.save()
    assert not memory.dirty
    reloaded = ObjectMap('lab', str(memory.path), graph_id='graph')
    assert reloaded.objects['chair-id'].alias == 'changed'


@pytest.mark.parametrize('partial', [b'partial' * 10000, b'PK\x03\x04truncated'],
                         ids=['long-partial', 'truncated-zip'])
def test_orphan_temporary_file_does_not_block_repeated_dirty_saves(tmp_path, partial):
    memory = memory_at(tmp_path)
    temporary = memory.path.with_suffix('.json.tmp')
    temporary.write_bytes(partial)
    for alias in ('first', 'second'):
        memory.set_alias('chair-id', alias)
        memory.save()
        assert not memory.dirty and not temporary.exists()
        loaded = ObjectMap('lab', str(memory.path), graph_id='graph')
        assert loaded.objects['chair-id'].alias == alias


def bound_memory(tmp_path, stamp):
    """Build a record through the real bind/update producers, including snap boundaries."""
    graph = PoseGraph('graph')
    graph.active = 1
    graph.replace(1, [NodePose(t, [0, 0, 0], [0, 0, 0, 1]) for t in (1.0, 1.5, 2.0)])
    anchor, points, covariance = graph.bind(
        stamp, [[2, 0, 0], [2, 0.1, 0], [2, 0, 0.1]], np.eye(3) * 0.01)
    memory = ObjectMap('lab', graph_id='graph')
    memory.path = tmp_path / 'bound.json'
    memory.update([Observation(56, 'chair', stamp, points.tolist(),
                               covariance.ravel().tolist(), anchor=anchor)])
    return memory


def test_bind_snapped_capture_time_survives_archive_roundtrip(tmp_path):
    memory = bound_memory(tmp_path, 1.5 - 5e-8)
    record = next(iter(memory.objects.values()))
    assert record.anchors[0]['stamp'] < record.anchors[0]['left']
    memory.save()
    loaded = ObjectMap('lab', str(memory.path), graph_id='graph')
    assert asdict(loaded.objects[record.object_id]) == asdict(record)


def test_schema2_snapped_capture_time_loads_and_migrates(tmp_path):
    memory = bound_memory(tmp_path, 1.5 - 5e-8)
    record = next(iter(memory.objects.values()))
    # A schema-2 writer never had first_seen (added in schema 4).
    legacy = {key: value for key, value in asdict(record).items() if key != 'first_seen'}
    data = {'schema': 2, 'map_id': 'lab', 'graph_id': 'graph', 'revision': 0,
            'objects': [legacy]}
    memory.path.write_text(json.dumps(data))
    loaded = ObjectMap('lab', str(memory.path), graph_id='graph')
    assert loaded.objects[record.object_id].anchors == record.anchors
    loaded.set_alias(record.object_id, 'migrated')
    loaded.save()
    assert zipfile.is_zipfile(memory.path)
    assert ObjectMap('lab', str(memory.path), graph_id='graph').objects[
        record.object_id].anchors == record.anchors


def test_seeded_bind_boundary_roundtrips_only_accepted_captures(tmp_path):
    rng = np.random.default_rng(42)
    accepted, rejected = 0, 0
    for node_stamp in (1.0, 1.5, 2.0):
        offsets = [0.0, -5e-8, 5e-8, -1e-7, 1e-7, -2e-7, 2e-7]
        offsets.extend(rng.uniform(-2e-7, 2e-7, 32))
        stamps = [node_stamp + delta for delta in offsets]
        for boundary in (node_stamp - 1e-7, node_stamp, node_stamp + 1e-7):
            stamps.extend([np.nextafter(boundary, -np.inf), np.nextafter(boundary, np.inf)])
        for stamp in stamps:
            try:
                memory = bound_memory(tmp_path, float(stamp))
            except LookupError:
                rejected += 1
                assert stamp < 1.0 - 1e-7 or stamp > 2.0
                continue
            record = next(iter(memory.objects.values()))
            memory.save()
            restored = ObjectMap('lab', str(memory.path), graph_id='graph')
            assert asdict(restored.objects[record.object_id]) == asdict(record)
            accepted += 1
    assert accepted > 50 and rejected > 0


@pytest.mark.parametrize('offset', [-2e-7, 2e-7])
def test_invalid_single_node_anchor_outside_snap_tolerance_is_rejected(tmp_path, offset):
    memory = bound_memory(tmp_path, 1.5)
    record = next(iter(memory.objects.values()))
    record.anchors[0]['stamp'] += offset
    with pytest.raises(ValueError, match='anchor association'):
        memory.save()


@pytest.mark.parametrize('corruption', [
    'schema', 'length', 'offset', 'count', 'shape', 'dtype', 'nan', 'anchor_nan',
    'duplicate_id', 'capacity', 'anchor_capacity', 'map', 'graph', 'revision',
    'missing_entry', 'truncated', 'crc',
])
def test_archive_corruption_is_rejected_without_partial_load(tmp_path, corruption):
    memory = memory_at(tmp_path)
    memory.save()
    with zipfile.ZipFile(memory.path) as archive:
        data = json.loads(archive.read('metadata.json'))
        raw = archive.read('points.bin')
    reference = data['objects'][0]['points']
    if corruption == 'schema':
        data['schema'] = 99
    elif corruption == 'length':
        data['points_bytes'] += 8
    elif corruption == 'offset':
        reference['offset'] = len(raw)
    elif corruption == 'count':
        reference['count'] = -1
    elif corruption == 'shape':
        reference['shape'] = [1, 9]
    elif corruption == 'dtype':
        reference['dtype'] = '<f4'
    elif corruption in ('nan', 'anchor_nan'):
        values = np.frombuffer(raw, dtype='<f8').copy()
        values[0 if corruption == 'nan' else 9] = np.nan
        raw = values.tobytes()
    elif corruption == 'duplicate_id':
        data['objects'].append(deepcopy(data['objects'][0]))
    elif corruption == 'capacity':
        memory.max_objects = 0
    elif corruption == 'anchor_capacity':
        memory.max_anchors = 0
    elif corruption == 'map':
        data['map_id'] = 'other'
    elif corruption == 'graph':
        data['graph_id'] = 'other'
    elif corruption == 'revision':
        data['revision'] = -1
    with zipfile.ZipFile(memory.path, 'w', compression=zipfile.ZIP_STORED) as archive:
        archive.writestr('metadata.json', json.dumps(data))
        if corruption != 'missing_entry':
            archive.writestr('points.bin', raw)
    if corruption == 'truncated':
        memory.path.write_bytes(memory.path.read_bytes()[:-20])
    elif corruption == 'crc':
        payload = bytearray(memory.path.read_bytes())
        start = payload.index(raw)
        payload[start] ^= 1
        memory.path.write_bytes(payload)
    previous_objects = memory.objects
    memory.rebuild_ids = {'existing-rebuild'}
    with pytest.raises(ValueError):
        memory.load()
    assert memory.objects is previous_objects and memory.revision == 7
    assert memory.rebuild_ids == {'existing-rebuild'}


@pytest.mark.parametrize('anchored', [False, True])
def test_invalid_geometry_save_preserves_previous_file(tmp_path, anchored):
    memory = memory_at(tmp_path, anchored)
    memory.save()
    before = memory.path.read_bytes()
    record = memory.objects['chair-id']
    points = record.anchors[0]['points'] if anchored else record.points
    points[0][0] = float('inf')
    memory.dirty = True
    with pytest.raises(ValueError):
        memory.save()
    assert memory.dirty and memory.path.read_bytes() == before
    assert not memory.path.with_suffix('.json.tmp').exists()


@pytest.mark.parametrize('shape', [(3, 3), (9, 1), (9,)])
@pytest.mark.parametrize('existing', [False, True])
def test_covariance_shape_roundtrip_on_create_and_update(tmp_path, shape, existing):
    points = [[0, 0, 1], [0.1, 0, 1], [0, 0.1, 1]]
    matrix = np.array([[0.04, 0.01, 0.002], [0.01, 0.05, 0.003],
                       [0.002, 0.003, 0.06]])
    memory = ObjectMap('lab', str(tmp_path / 'covariance.json'))
    original_id = None
    if existing:
        original_id = memory.update([Observation(
            56, 'chair', 1.0, points, (np.eye(3) * 0.01).ravel().tolist())])[0]
    object_id = memory.update([Observation(
        56, 'chair', 2.0, points, matrix.reshape(shape).tolist())])[0]
    if existing:
        assert object_id == original_id
    memory.save()
    loaded = ObjectMap('lab', str(memory.path))
    expected = matrix.ravel().tolist()
    assert memory.objects[object_id].covariance == expected
    assert loaded.objects[object_id].covariance == expected
    assert isinstance(loaded.objects[object_id].covariance, list)
    assert len(loaded.objects[object_id].covariance) == 9
    assert loaded.objects[object_id].count == (2 if existing else 1)


def _rewrite_metadata(path, edit):
    with zipfile.ZipFile(path) as archive:
        data = json.loads(archive.read('metadata.json'))
        raw = archive.read('points.bin')
    edit(data)
    with zipfile.ZipFile(path, 'w', compression=zipfile.ZIP_STORED) as archive:
        archive.writestr('metadata.json', json.dumps(data))
        archive.writestr('points.bin', raw)


def _saved_chair(tmp_path):
    memory = ObjectMap('lab', str(tmp_path / 'objects.json'), confirm_count=3)
    rng = np.random.default_rng(13)
    cov = (np.eye(3) * 0.03 ** 2).ravel().tolist()
    for k in range(3):
        points = (np.array([3.0, 1.0, 0.4]) + rng.uniform(-.2, .2, (50, 3))).tolist()
        memory.update([Observation(56, 'chair', 100.0 + k, points, cov, .9, .9)])
    memory.save()
    return memory.path


def test_writer_records_first_seen_under_schema_4(tmp_path):
    path = _saved_chair(tmp_path)
    with zipfile.ZipFile(path) as archive:
        data = json.loads(archive.read('metadata.json'))
    # Schema 3 readers reject schema 4 by number instead of failing on a new field.
    assert data['schema'] == 4 and data['objects'][0]['first_seen'] == 100.0


def test_schema_3_archive_loads_without_first_seen_and_never_merges_as_newer(tmp_path):
    path = _saved_chair(tmp_path)

    def downgrade(data):
        data['schema'] = 3
        del data['objects'][0]['first_seen']
    _rewrite_metadata(path, downgrade)
    memory = ObjectMap('lab', str(path), confirm_count=3)
    record = next(iter(memory.objects.values()))
    # Unknown, not estimated from the last 8 history entries (that is later than
    # the truth and would merge two real objects whose lifetimes overlapped).
    assert record.first_seen == 0.0 and memory._first_seen(record) is None


def test_newer_schema_is_rejected_as_newer_version(tmp_path):
    path = _saved_chair(tmp_path)
    _rewrite_metadata(path, lambda data: data.update(schema=5))
    with pytest.raises(ValueError, match='newer version'):
        ObjectMap('lab', str(path))


@pytest.mark.parametrize('schema, field', [(4, 'heading'), (3, 'first_seen')])
def test_record_field_unknown_to_its_schema_is_rejected_by_name(tmp_path, schema, field):
    path = _saved_chair(tmp_path)

    def add_field(data):
        data['schema'] = schema
        data['objects'][0].setdefault(field, 1.0)
    _rewrite_metadata(path, add_field)
    with pytest.raises(ValueError, match=f"unknown to schema {schema}: \\['{field}'\\]"):
        ObjectMap('lab', str(path))
