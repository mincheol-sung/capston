"""Native voxel reduction must preserve exact first-occurrence ordering."""

import numpy as np

import pytest

voxel = pytest.importorskip('semantic_slam._voxel')


@pytest.mark.parametrize('count', [0, 1, 50, 8192, 50000])
def test_native_indices_equal_reference_for_duplicates_and_full_uint64_range(count):
    rng = np.random.default_rng(18)
    keys = rng.integers(0, 1000, count, dtype=np.uint64)
    if count > 1:
        keys[::7] = np.iinfo(np.uint64).max
        keys[::11] = 0
    _, reference = np.unique(keys, return_index=True)
    actual = np.frombuffer(voxel.first_indices(keys), dtype=np.int64)
    assert np.array_equal(actual, np.sort(reference))


@pytest.mark.parametrize('keys', [
    np.ones(4, dtype=float), np.ones(4, dtype=np.int32),
    np.ones((2, 2), dtype=np.uint64), np.arange(8, dtype=np.uint64)[::2],
])
def test_native_reducer_rejects_incompatible_buffers(keys):
    with pytest.raises((ValueError, BufferError)):
        voxel.first_indices(keys)
