"""Geometric edge cases for the offline disparity-partition prototype."""
import unittest
import numpy as np
from experiment_disparity_partition import detect_edges, partition, runs


def scene(ranges, depth=None):
    angles=np.deg2rad(np.arange(-180,180))
    r=np.array(ranges,float)
    arrays=dict(strict_depth=np.full(360,2.) if depth is None else np.array(depth,float),
                hard_depth=np.full(360,2.),angles=angles)
    for sensor in range(2):
        arrays[f'scan_{sensor}_ranges']=r.copy()
        arrays[f'scan_{sensor}_valid']=np.full(360,sensor==0)
        arrays[f'scan_{sensor}_hits']=r[:,None]*np.column_stack((np.cos(angles),np.sin(angles)))
    return arrays


class PartitionTests(unittest.TestCase):
    def test_circular_seam_is_not_a_gap(self):
        mask=np.zeros(360,bool);mask[[358,359,0,1]]=True
        self.assertEqual([r.tolist() for r in runs(mask)],[[358,359,0,1]])

    def test_uncapped_edge_detected_but_distant_flat_reach_not_split(self):
        r=np.full(360,4.);r[90:270]=6.
        data=scene(r)
        self.assertEqual(len(detect_edges(data)),2)
        result=partition(data)
        self.assertEqual(result['region_count'],1)
        self.assertTrue(all(e['status']=='no_local_depth_support' for e in result['edges']))

    def test_spike_does_not_become_boundary(self):
        r=np.full(360,2.);r[100]=5.
        self.assertEqual(detect_edges(scene(r)),[])

    def test_invalid_beams_not_bridged(self):
        r=np.full(360,2.);r[180:]=4.
        data=scene(r);data['scan_0_valid'][178:182]=False
        self.assertEqual(detect_edges(data),[])

    def test_full_circle_two_edges_and_exact_coverage(self):
        r=np.full(360,2.);r[90:270]=4.
        depth=np.full(360,1.);depth[90:270]=2.
        result=partition(scene(r,depth))
        self.assertEqual(result['region_count'],2)
        bins=[b for region in result['regions'] for b in region['bins']]
        self.assertEqual(sorted(bins),list(range(360)))

    def test_zero_gap_fallback_and_no_invented_free_space(self):
        data=scene(np.full(360,2.),np.zeros(360))
        result=partition(data)
        self.assertTrue(result['fallback']);self.assertEqual(result['region_count'],1)
        data['hard_depth'][:]=0
        result=partition(data)
        self.assertEqual(result['region_count'],0)


if __name__=='__main__': unittest.main()
