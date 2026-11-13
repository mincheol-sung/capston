"""Dual-LiDAR disparity regions. Pure array geometry shared by the gap strategy.

Port of the recorded-data prototype; no runtime dependency on tools or docs.
"""
from dataclasses import dataclass, asdict
import time
import numpy as np


@dataclass
class GapConfig:
    total_rollouts: int = 500
    rollouts_per_guide: int = 100  # Legacy input only; allocation now uses minimum budgets.
    max_guides: int = 3
    timeout: float = .18
    nominal_min_rollouts: int = 100
    guide_min_rollouts: int = 30
    allocation_exploration: float = .20
    allocation_temperature_floor: float = 1.
    optimization_timeout: float = .14  # cap, also bounded by remaining cycle time
    optimization_mppi_reserve: float = .10  # cold start or changed prediction shape
    optimization_mppi_reserve_min: float = .05
    optimization_mppi_margin: float = .015
    optimization_maxiter: int = 45
    optimization_ftol: float = 1e-6
    optimization_constraint_tolerance: float = 1e-5
    optimization_target_tolerance_m: float = .01
    value_progress_weight: float = 8.
    value_straight_weight: float = 4.
    value_heading_weight: float = 3.
    value_delta_x_weight: float = .5
    value_delta_y_weight: float = .5
    value_delta_omega_weight: float = .055
    cache_max_age: float = .6
    cache_max_translation_m: float = .5
    cache_min_overlap: float = .5
    cache_max_angle_deg: float = 20.
    adjacent_jump_m: float = .4
    relative_jump: float = .12
    side_window_beams: int = 3
    side_median_jump_m: float = .4
    side_spread_fraction: float = .35
    minimum_cut_separation_deg: float = 20.
    minimum_region_bins: int = 6
    geometry_window_bins: int = 5
    geometry_jump_m: float = .25
    maximum_projection_bins: int = 8
    minimum_depth_m: float = .5
    max_depth_m: float = 2.
    radial_step_m: float = .02
    extra_margin_m: float = .05
    grid_resolution_m: float = .04
    grid_extent_m: float = 4.
    min_displacement_m: float = .25
    max_target_error_m: float = .35
    nominal_rms_m: float = .22
    nominal_tail_m: float = .30
    guide_rms_m: float = .25
    target_reserve_m: float = .10
    reach_scale: float = .90
    target_floor_m: float = .25
    path_lookahead_m: float = 1.

    def validate(self):
        values=asdict(self)
        if not all(np.isfinite(v) for v in values.values()):
            raise ValueError('gap parameters must be finite')
        integers=('total_rollouts','rollouts_per_guide','max_guides',
                  'side_window_beams','minimum_region_bins','geometry_window_bins','maximum_projection_bins',
                  'nominal_min_rollouts','guide_min_rollouts','optimization_maxiter')
        if any(int(values[k]) != values[k] for k in integers):
            raise ValueError('gap counts must be integers')
        if not 0 <= self.max_guides <= 3 or self.rollouts_per_guide < 2:
            raise ValueError('gap supports 0..3 extra guides with at least two samples each')
        if min(self.nominal_min_rollouts,self.guide_min_rollouts) < 2:
            raise ValueError('gap groups require at least two samples')
        if self.nominal_min_rollouts+self.max_guides*self.guide_min_rollouts > self.total_rollouts:
            raise ValueError('gap minimum group budgets exceed total rollouts')
        for key,value in values.items():
            if key in ('max_guides','extra_margin_m','relative_jump','maximum_projection_bins',
                       'allocation_exploration') or key.startswith('value_'):
                if value < 0: raise ValueError(f'gap.{key} must be nonnegative')
            elif value <= 0: raise ValueError(f'gap.{key} must be positive')
        if not 0 <= self.allocation_exploration <= 1 or not 0 < self.cache_min_overlap <= 1:
            raise ValueError('invalid gap allocation/cache fraction')
        if self.cache_max_angle_deg > 180:
            raise ValueError('invalid gap cache angle')
        if self.optimization_mppi_reserve >= self.timeout:
            raise ValueError('gap timeout must leave time before the MPPI reserve')
        if self.optimization_mppi_reserve_min > self.optimization_mppi_reserve:
            raise ValueError('gap minimum MPPI reserve must not exceed cold-start reserve')
        if self.optimization_target_tolerance_m >= self.min_displacement_m:
            raise ValueError('gap target tolerance must be smaller than minimum movement')
        if not 1 <= self.minimum_region_bins <= 360 or not 1 <= self.geometry_window_bins <= 180:
            raise ValueError('invalid gap angular window')
        if self.maximum_projection_bins > 180 or self.minimum_cut_separation_deg > 180:
            raise ValueError('invalid gap angular separation')
        if self.side_window_beams > 100 or self.reach_scale > 1:
            raise ValueError('invalid gap window or reach scale')
        if self.max_depth_m <= self.minimum_depth_m or self.grid_extent_m <= self.max_depth_m:
            raise ValueError('gap grid must contain the complete depth horizon')
        if self.radial_step_m > self.minimum_depth_m or self.grid_resolution_m > self.minimum_depth_m:
            raise ValueError('gap spatial resolution is too coarse')
        size=2*self.grid_extent_m/self.grid_resolution_m
        if size > 1000 or abs(size-round(size)) > 1e-6:
            raise ValueError('gap grid must have an integral size <=1000 cells per axis')
        return self


CONFIG=asdict(GapConfig())


def runs(mask):
    """Circular runs with every true bin represented exactly once."""
    if np.all(mask): return [np.arange(len(mask))]
    starts = np.flatnonzero(mask & ~np.roll(mask, 1))
    result = []
    for start in starts:
        ids = []
        while mask[start % len(mask)]:
            ids.append(start % len(mask)); start += 1
        result.append(np.array(ids, int))
    return result


def detect_edges(arrays, config=CONFIG):
    """Same-sensor adjacent disparity with persistent median level change."""
    found = []
    w = config['side_window_beams']
    for sensor in range(2):
        r = arrays[f'scan_{sensor}_ranges']
        valid = arrays[f'scan_{sensor}_valid']
        hits = arrays[f'scan_{sensor}_hits']
        pair_valid = valid[:-1] & valid[1:]
        differences = np.zeros(len(r)-1)
        np.subtract(r[1:], r[:-1], out=differences, where=pair_valid)
        candidates = np.flatnonzero(pair_valid & (np.abs(differences) >= config['adjacent_jump_m']))
        for i in candidates:
            if i < w-1 or i >= len(r)-w: continue
            if not np.all(valid[i-w+1:i+w+1]): continue
            left, right = r[i-w+1:i+1], r[i+1:i+w+1]
            ml, mr = float(np.median(left)), float(np.median(right))
            threshold = max(config['adjacent_jump_m'], config['relative_jump']*min(ml, mr))
            adjacent = float(r[i+1]-r[i]); median_jump = mr-ml
            if abs(adjacent) < threshold or abs(median_jump) < max(threshold, config['side_median_jump_m']): continue
            if adjacent*median_jump <= 0: continue
            # A new level must persist across each three-beam side; isolated
            # spikes and rapid alternating returns are not stable edges.
            spread = max(float(np.ptp(left)), float(np.ptp(right)))
            if spread > config['side_spread_fraction']*abs(median_jump): continue
            near = i if r[i] <= r[i+1] else i+1
            xy = hits[near]
            angle = float(np.degrees(np.arctan2(xy[1], xy[0])))
            index = int(np.floor((angle+180) % 360 + .5)) % 360
            found.append(dict(sensor=sensor, beam_before=int(i), near_beam=int(near),
                              angle_deg=angle, cut_bin=index, near_hit_xy=xy.tolist(),
                              range_before=float(r[i]), range_after=float(r[i+1]),
                              median_before=ml, median_after=mr,
                              jump_m=abs(adjacent), median_jump_m=abs(median_jump),
                              threshold_m=threshold, score=abs(median_jump)/threshold,
                              status='outside_open_run'))
    return found


def project_edges(edges, depth, config=CONFIG):
    """Require raw edges to affect local body reach, not only distant scenery.

    Boundary can move by at most eight bins because hit bearing and inflated
    body-clearance geometry differ. No boundary is created without a raw edge.
    """
    n=len(depth); w=config['geometry_window_bins']; reach=config['maximum_projection_bins']
    for edge in edges:
        original=edge['cut_bin']; edge['raw_cut_bin']=original
        sign=np.sign(edge['median_after']-edge['median_before'])
        supported=[]
        for offset in range(-reach,reach+1):
            j=(original+offset)%n
            left=depth[(j+np.arange(-w,0))%n];right=depth[(j+np.arange(0,w))%n]
            if np.any(left < config['minimum_depth_m']) or np.any(right < config['minimum_depth_m']):continue
            difference=float(np.median(right)-np.median(left))
            if difference*sign < config['geometry_jump_m']-1e-12:continue
            supported.append((abs(difference),-abs(offset),j,offset,difference))
        if not supported:
            edge['status']='no_local_depth_support';continue
        _,_,j,offset,difference=max(supported)
        edge.update(cut_bin=int(j),projection_bins=int(offset),geometry_jump_m=difference,
                    status='outside_open_run')


def partition(arrays, config=CONFIG):
    start = time.perf_counter()
    minimum = config['minimum_region_bins']
    depth = arrays['strict_depth']
    initial = [x for x in runs(depth >= config['minimum_depth_m']) if len(x) >= minimum]
    fallback = not initial
    if fallback: depth = arrays['hard_depth']
    components = [x for x in runs(depth >= config['minimum_depth_m']) if len(x) >= minimum]
    edges = detect_edges(arrays, config)
    project_edges(edges, depth, config)
    regions = []
    for ci, ids in enumerate(components):
        full_circle = len(ids) == 360
        positions = {int(v): k for k, v in enumerate(ids)}
        eligible = []
        for edge in edges:
            if edge['status']=='no_local_depth_support': continue
            if edge['cut_bin'] not in positions: continue
            pos = positions[edge['cut_bin']]
            edge['component'] = ci; edge['position'] = pos
            if not full_circle and (pos < minimum or len(ids)-pos < minimum):
                edge['status'] = 'near_open_endpoint'; continue
            eligible.append(edge)
        kept = []
        for edge in sorted(eligible, key=lambda e: (-e['score'], e['sensor'], e['beam_before'])):
            pos = edge['position']
            distances = [abs(pos-e['position']) for e in kept]
            if full_circle: distances = [min(d, 360-d) for d in distances]
            if any(d < max(minimum, config['minimum_cut_separation_deg']) for d in distances):
                edge['status'] = 'near_stronger_cut'; continue
            edge['status'] = 'selected'; kept.append(edge)
        cuts = sorted(e['position'] for e in kept)
        if full_circle:
            if len(cuts) < 2:
                for edge in kept: edge['status'] = 'single_cut_on_full_circle'
                chunks = [ids]
            else:
                chunks = [np.array([ids[j % 360] for j in range(a, b)])
                          for a, b in zip(cuts, cuts[1:]+[cuts[0]+360])]
        else:
            bounds = [0]+cuts+[len(ids)]
            chunks = [ids[a:b] for a, b in zip(bounds[:-1], bounds[1:])]
        for chunk in chunks:
            assert len(chunk) >= minimum
            regions.append(dict(id=len(regions)+1, component=ci, bins=chunk.tolist(),
                                first_angle_deg=int(chunk[0])-180,
                                last_angle_deg=int(chunk[-1])-180,
                                bin_count=len(chunk), min_depth_m=float(depth[chunk].min()),
                                max_depth_m=float(depth[chunk].max())))
    expected = np.sort(np.concatenate(components)) if components else np.array([], int)
    actual = np.sort(np.concatenate([r['bins'] for r in regions])) if regions else np.array([], int)
    np.testing.assert_array_equal(actual, expected)
    assert len(np.unique(actual)) == len(actual)
    return dict(initial_open_runs=len(initial), fallback=fallback, open_runs=len(components),
                regions=regions, edges=edges, region_count=len(regions),
                selected_cut_count=sum(e['status']=='selected' for e in edges),
                partition_ms=(time.perf_counter()-start)*1000)


