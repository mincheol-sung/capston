#!/usr/bin/env python3
"""Paired, offline gap-definition/ranking ablations on recorded ROS bags.

Run a frozen copy of this file and the planner modules for reproducible results.
No ROS node is started and no commands are published. Every trial is a fresh
single-snapshot optimization, not a counterfactual closed-loop replay.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks

from render_lidar_gap_flow import (
    ROOT, RADIUS, read_bag, prepare_snapshot, circular_runs, seed_control,
    recorded_predictions, SnapshotGapOptimizer, config_from_yaml, stamp,
)
from dra_mppi.mppi_controller import MppiController
from dra_mppi.mecanum_model import rollout
from dra_mppi.common_cost import common_costs, project_path

METHODS = ['fixed5_warm', 'nominal', 'radial_R', 'radial_noR',
           'radial_adaptive_noR', 'connected_noR', 'valley_noR',
           'valley_cost', 'valley_cost_gate']
BAGS = ['mpc_chain_20260923_135325', 'mpc_chain_20260923_113151',
        'mpc_chain_20260923_135606']
SEEDS = [7, 23]


def serial(value):
    if isinstance(value, np.ndarray):
        return serial(value.tolist())
    if isinstance(value, dict):
        return {str(k): serial(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serial(v) for v in value]
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if np.isfinite(value) else None
    return value


def write_json(path, value):
    path.write_text(json.dumps(serial(value), ensure_ascii=False, indent=2)+'\n')


def definitions(p, kind):
    angles, depth = p['angles'], p['d_free']
    candidates = []
    if kind == 'radial':
        for level in (1.5, 1., .5):
            for ids in circular_runs(depth >= level):
                if len(ids) < 6:
                    continue
                index = ids[len(ids)//2]
                angle = angles[index]
                if any(abs(math.atan2(math.sin(angle-g['angle']), math.cos(angle-g['angle']))) < math.radians(12)
                       and len(np.intersect1d(ids, g['ids'])) for g in candidates):
                    continue
                candidates.append(dict(angle=angle, index=index, ids=ids, level=level))
        return candidates
    runs = circular_runs(depth >= .5)
    if kind == 'connected':
        for ids in runs:
            if len(ids) >= 6:
                index = ids[len(ids)//2]
                candidates.append(dict(angle=angles[index], index=index, ids=ids,
                                       level=float(depth[index])))
        return candidates
    # One angular interval per substantial depth peak, bounded by valleys.
    # Smoothing selects extrema only; the actual clearance remains unsmoothed.
    smooth = gaussian_filter1d(depth, 2., mode='wrap')
    for ids in runs:
        if len(ids) < 6:
            continue
        if len(ids) == len(depth):
            # Cut the circle at its lowest depth; join boundary troughs.
            start = int(np.argmin(smooth))
            ids = (start+np.arange(len(depth))) % len(depth)
        values = smooth[ids]
        peaks = find_peaks(np.r_[-.1, values, -.1], prominence=.25, distance=20)[0]-1
        peaks = peaks[(peaks >= 0) & (peaks < len(ids))]
        if not len(peaks):
            peaks = np.array([int(np.argmax(values))])
        boundaries = [0]
        for left, right in zip(peaks[:-1], peaks[1:]):
            boundaries.append(left+int(np.argmin(values[left:right+1]))+1)
        boundaries.append(len(ids))
        for peak, start, end in zip(peaks, boundaries[:-1], boundaries[1:]):
            if end-start < 6:
                continue
            index = ids[peak]
            candidates.append(dict(angle=angles[index], index=index, ids=ids[start:end],
                                   level=float(depth[index])))
    return candidates


def cheap_cost(c, p, controls, paths):
    terms = common_costs(c.config, np.zeros(3), p['path'], controls, paths,
                         p['current'], return_components=False)
    static, _ = c._static_obstacle_cost(np.zeros(3), paths, p['field'], None)
    return sum(terms)+static


def path_progress(p, trajectories):
    full = np.concatenate((np.zeros((len(trajectories), 1, 2)), trajectories[..., :2]), axis=1)
    arc = project_path(full, p['path'], .2, .5)[1]
    return arc[:, -1]-arc[:, 0]


def make_pool(p, c, nominal_path, kind, adaptive):
    started = time.perf_counter()
    groups = definitions(p, kind)
    for i, g in enumerate(groups):
        g['id'] = f'{kind}_{i}'
        direction = np.array([np.cos(g['angle']), np.sin(g['angle'])])
        length = g['level']
        if adaptive:
            length = max(.1, min(length, p['d_free'][g['index']]-.10))
            far = seed_control(c, direction*length, p['current'])
            reach = float(np.dot(rollout(np.zeros(3), far, .2)[-1, :2], direction))
            length = min(length, max(.25, .90*reach))
        g['target_distance'] = length
        g['target'] = length*direction
        g['controls'] = seed_control(c, g['target'], p['current'])
        g['trajectory'] = rollout(np.zeros(3), g['controls'], .2)
        delta = np.linalg.norm(g['trajectory'][:, :2]-nominal_path[:, :2], axis=1)
        g['rms'] = float(np.sqrt(np.mean(delta**2)))
        g['tail'] = float(np.mean(delta[-3:]))
        g['reach_error'] = float(np.linalg.norm(g['trajectory'][-1, :2]-g['target']))
        g['displacement'] = float(np.linalg.norm(g['trajectory'][-1, :2]))
        g['collision'] = bool(p['field'].trajectory_collisions(
            g['trajectory'][None, :, :2], np.zeros(2), RADIUS)[0])
        swept = p['field'].trajectory_clearances(g['trajectory'][None, :, :2], np.zeros(2))[0]
        g['clearance'] = float(np.min(swept)-RADIUS)
        tangent = p['path'][min(3, len(p['path'])-1)]-p['path'][0]
        tangent /= max(np.linalg.norm(tangent), 1e-9)
        g['progress_proxy'] = float(np.dot(g['trajectory'][-1, :2], tangent))
        g['cross_track'] = float(np.min(np.linalg.norm(p['path']-g['trajectory'][-1, :2], axis=1)))
        P = np.clip(g['progress_proxy']/1.5, 0, 1)
        C = np.clip(g['clearance']/.4, 0, 1)
        D = np.clip(g['rms']/.8, 0, 1)
        R = np.exp(-g['cross_track']/.6)
        g['score_noR'] = float(.4*P+.25*C+.2*D)
        g['score_R'] = float(g['score_noR']+.15*R)
    if groups:
        costs = cheap_cost(c, p, np.array([g['controls'] for g in groups]),
                           np.array([g['trajectory'] for g in groups]))
        for g, cost in zip(groups, costs):
            g['cheap_cost'] = float(cost)
    return groups, (time.perf_counter()-started)*1000


def nominal_threat(predictions, path):
    if predictions is None or predictions.empty:
        return False
    sigma = np.sqrt(np.maximum(0., np.linalg.eigvalsh(predictions.covariances)[..., -1]))
    radii = predictions.obstacle_radii
    if radii is None:
        radii = np.full(len(predictions.means), .25)
    distance = np.linalg.norm(predictions.means-path[None, None, :, :2], axis=-1)
    overlap = distance <= RADIUS+radii[:, None, None]+2*sigma
    weights = predictions.mode_weights
    if weights.ndim == 2:
        weights = weights[..., None]
    return bool(np.any(overlap & (weights > .05)))


def select(p, pool, rule, nominal_collision, nominal_cost, escape):
    started = time.perf_counter()
    items = []
    for original in pool:
        g = original.copy()
        if not nominal_collision and g['rms'] < .22 and g['tail'] < .30:
            g['reason'] = 'duplicate_nominal'
        elif g['collision']:
            g['reason'] = 'collision'
        elif g['reach_error'] > .35:
            g['reason'] = 'unreachable'
        elif rule in ('R', 'noR') and g['score_'+rule] < .25:
            g['reason'] = 'score_threshold'
        elif rule == 'gate' and not escape and not (
                g['cheap_cost'] < nominal_cost-max(.5, .05*abs(nominal_cost))):
            g['reason'] = 'no_predicted_gain'
        else:
            g['reason'] = 'eligible'
        items.append(g)
    def key(g):
        return -g['score_'+rule] if rule in ('R', 'noR') else g['cheap_cost']
    chosen = []
    for g in sorted(items, key=key):
        if g['reason'] != 'eligible':
            continue
        if any(np.sqrt(np.mean(np.sum((g['trajectory'][:, :2]-s['trajectory'][:, :2])**2, axis=1))) < .25
               for s in chosen):
            g['reason'] = 'duplicate_gap'
        elif len(chosen) >= 3:
            g['reason'] = 'capacity'
        else:
            g['reason'] = 'selected'
            chosen.append(g)
    return items, chosen, (time.perf_counter()-started)*1000


def run_optimizer(p, cfg, nominal, chosen, predictions, method, seed):
    start = time.perf_counter()
    # Ranking decides membership, not the random-number stream assigned to a
    # given candidate. Identical selected sets must yield identical solver input.
    chosen = sorted(chosen, key=lambda g: g['id'])
    if method == 'fixed5_warm':
        run_cfg = replace(cfg, seed=seed, guide_timeout=10., strategy='guide_conditioned')
        c = MppiController(run_cfg)
        c._guide_optimizer.nominals[:] = nominal
        c._guide_optimizer.initialized[:] = True
    else:
        seeds = np.array([nominal]+[g['controls'] for g in chosen])
        names = ['nominal']+[g['id'] for g in chosen]
        counts = [500-100*len(chosen)]+[100]*len(chosen)
        run_cfg = replace(cfg, seed=seed, guide_timeout=10., strategy='standard',
                          guide_angles_deg=tuple(float(i) for i in range(len(seeds))))
        c = MppiController(run_cfg)
        c.config.strategy = 'guide_conditioned'
        c._guide_optimizer = SnapshotGapOptimizer(c, seeds, names, counts)
    setup_ms = (time.perf_counter()-start)*1000
    try:
        tick = time.perf_counter()
        result = c.compute(np.zeros(3), p['path'], p['current'], predictions,
                           visualization_rollouts=0, distance_field=p['field'])
        compute_ms = (time.perf_counter()-tick)*1000
        assert len(result.sampled_controls) == 500
        selected_index = result.candidate_names.index(result.selected_mode)
        progress = float(path_progress(p, result.trajectory[None])[0])
        safe = result.safety_status == 'ok'
        moving = safe and result.selected_mode != 'stop'
        output = dict(status=result.safety_status, safe=safe, moving=moving,
                      selected=result.selected_mode, cost=result.cost,
                      progress_m=progress, displacement_m=float(np.linalg.norm(result.trajectory[-1, :2])),
                      max_probability=result.max_collision_probability,
                      finite_fraction=float(np.mean(np.isfinite(result.rollout_costs))),
                      guide_count=len(result.candidate_names)-1,
                      compute_ms=compute_ms, setup_ms=setup_ms,
                      sample_counts=result.guide_rollout_counts,
                      underresolved_steps=result.final_risk_diagnostics.underresolved_steps,
                      final_mc_area_max=float(np.max(result.final_mc_region_areas, initial=0.)),
                      selected_index=selected_index, final_costs=result.mode_final_costs,
                      candidate_names=result.candidate_names)
        arrays = dict(trajectory=result.trajectory, representatives=result.mode_trajectories,
                      representative_controls=result.mode_control_sequences)
        if safe:
            assert result.max_collision_probability <= cfg.dynamic_risk.hard_threshold+1e-12
            assert not p['field'].trajectory_collisions(result.trajectory[None, :, :2], np.zeros(2), RADIUS)[0]
        return output, arrays
    finally:
        c.close()


def input_check(data, snap, p):
    epoch = p['epoch']
    if snap['record_ns']*1e-9-epoch > .25:
        raise ValueError('stale_odometry')
    scans = [snap['topics'][n][1] for n in ('/scan_front_carto','/scan_rear_carto')]
    if any(not 0 <= epoch-stamp(m) <= .25 for m in scans):
        raise ValueError('stale_scan')
    if abs(stamp(scans[0])-stamp(scans[1])) > .05:
        raise ValueError('scan_pair_skew')
    if not 0 <= p['ref_age'] <= .4:
        raise ValueError('stale_reference')
    length = np.sum(np.linalg.norm(np.diff(p['path'],axis=0),axis=1))
    if length < .5 or np.linalg.norm(p['path'][-1]) < .25:
        raise ValueError('terminal_path')


def protocol():
    return dict(methods=METHODS, bags=BAGS, seeds=SEEDS,
                design_bag=BAGS[0], holdout_bags=BAGS[1:],
                sampling='Every approximately 2 seconds from +5 s; no outcome-based frame selection.',
                comparison='Paired snapshots and control-noise seeds. K=500, T=10, dt=.2, existing cost and MC settings unchanged.',
                ordering='Selected seeds sorted by stable pool ID before solving; score-rank permutations alone cannot change noise assignment.',
                radial='Previous 0.5/1.0/1.5 m intervals; center angle; 12 deg cross-level merging.',
                connected='One interval per connected d_free>=0.5 m angular run, minimum 6 deg; midpoint direction.',
                valley='Split each connected run at substantial depth valleys: Gaussian sigma=2 angle bins, peak prominence=0.25 m, separation=20 deg. Deepest peak direction, unsmoothed clearance.',
                adaptive_target='min(available depth-0.10m, max(0.25m, 0.90*2s constrained projected reach)); never move target beyond available depth.',
                noR='Remove 0.15*R from prior score without renormalization; retain V>=0.25. Also report whether membership changes before/after threshold.',
                cost='Rank collision-free, nonduplicate, reachable seeds by existing common_costs+static cost. No custom value-score threshold; up to three.',
                gate='Same cost rank; when nominal healthy require seed cost improvement > max(0.5, 5% abs(nominal cost)). Escape allows all eligible seeds if nominal static-invalid, nominal arc progress<0.15m, or nominal intersects a 2-sigma person envelope. This envelope is a trigger, not collision probability.',
                fixed5='Context baseline: initialize every fixed guide nominal with the same recorded reference, then existing 0.25 anchor blend. Not reconstruction of historical independent guide states.',
                unimplemented='No closed-loop simulation, persistent gap IDs, MPC execution, goal-rate measurement, or real-time ROS/full-stack deadline validation.',
                outcomes=['safe/moving selection count', 'paired final objective on mutually safe moving trials', 'arc progress',
                          'finite rollout fraction', 'number of raw/selected candidates', 'rearward seed fraction',
                          'single-snapshot wall time including input prep, proposals, setup, solver, teardown'],
                timeout_s=10., max_gap_guides=3, allocation='nominal 500-100*gap_count; each gap 100',
                frozen_source='Source snapshot retained in result directory; thresholds fixed before any multiscene results.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--bags-root', type=Path, default=Path('/home/smoc/2026amr_ws/bags'))
    parser.add_argument('--max-per-bag', type=int)
    args = parser.parse_args()
    out = args.output.resolve();out.mkdir(parents=True, exist_ok=True)
    (out/'scenes').mkdir(exist_ok=True)
    write_json(out/'protocol.json', protocol())
    files = list((ROOT/'dra_mppi').glob('*.py'))+list((ROOT/'tools').glob('*.py'))+[ROOT/'config/dra_mppi.yaml']
    hashes = {str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
    write_json(out/'source_hashes.json', hashes)
    cfg = config_from_yaml()
    write_json(out/'configuration.json', vars(cfg)|{'dynamic_risk':vars(cfg.dynamic_risk)})
    trials_path, scene_path = out/'trials.jsonl', out/'scene_audit.jsonl'
    accepted, skipped = 0, 0
    with trials_path.open('w') as trial_file, scene_path.open('w') as scene_file:
        for bag_index, name in enumerate(BAGS):
            print(f'READ {name}', flush=True)
            data = read_bag(args.bags_root/name)
            snapshots = data['snapshots'][:args.max_per_bag] if args.max_per_bag else data['snapshots']
            print(f'SNAPSHOTS {name} {len(snapshots)}', flush=True)
            for frame_index, snap in enumerate(snapshots):
                scene_id = f'b{bag_index}_{frame_index:03d}'
                audit = dict(scene=scene_id, bag=name, split='design' if bag_index==0 else 'holdout',
                             offset=snap['elapsed'], record_ns=snap['record_ns'])
                try:
                    t = time.perf_counter()
                    p = prepare_snapshot(data, snap)
                    input_check(data, snap, p)
                    c = MppiController(replace(cfg, strategy='standard'))
                    c.gap_path = p['path']
                    predictions = recorded_predictions(p, c)
                    nominal = c._constrain_controls(p['nominal'][None],p['current'])[0]
                    nominal_path = rollout(np.zeros(3),nominal,.2)
                    nominal_collision = bool(p['field'].trajectory_collisions(nominal_path[None,:,:2],np.zeros(2),RADIUS)[0])
                    nom_cost = float(cheap_cost(c,p,nominal[None],nominal_path[None])[0])
                    nom_progress = float(path_progress(p,nominal_path[None])[0])
                    escape = nominal_collision or nom_progress < .15 or nominal_threat(predictions,nominal_path)
                    prep_ms = (time.perf_counter()-t)*1000
                except Exception as exc:
                    audit.update(accepted=False, reason=f'{type(exc).__name__}: {exc}')
                    scene_file.write(json.dumps(serial(audit),ensure_ascii=False)+'\n');scene_file.flush()
                    skipped += 1
                    continue
                accepted += 1
                pools = {}
                for key,kind,adaptive in [('radial','radial',False),('radial_adaptive','radial',True),
                                          ('connected','connected',True),('valley','valley',True)]:
                    pools[key] = make_pool(p,c,nominal_path,kind,adaptive)
                method_specs = {'radial_R':('radial','R'),'radial_noR':('radial','noR'),
                                'radial_adaptive_noR':('radial_adaptive','noR'),
                                'connected_noR':('connected','noR'),'valley_noR':('valley','noR'),
                                'valley_cost':('valley','cost'),'valley_cost_gate':('valley','gate')}
                proposals = {}
                for method,(key,rule) in method_specs.items():
                    pool,pool_ms = pools[key]
                    items,chosen,selection_ms = select(p,pool,rule,nominal_collision,nom_cost,escape)
                    proposals[method] = (items,chosen,pool_ms+selection_ms)
                audit.update(accepted=True, prepare_ms=prep_ms, nominal_collision=nominal_collision,
                             nominal_cost=nom_cost,nominal_progress=nom_progress,escape=escape,
                             prediction_count=0 if predictions is None else len(predictions.means),
                             current=p['current'], source_epoch=p['epoch'],
                             proposals={method:[{k:v for k,v in g.items() if k not in ('controls','trajectory','ids')}
                                                for g in items] for method,(items,_,_) in proposals.items()})
                scene_file.write(json.dumps(serial(audit),ensure_ascii=False)+'\n');scene_file.flush()
                arrays = dict(grid=p['grid'],scan_grid=p['scan_grid'],map_fill=p['map_fill'],
                              full_path=p['full_path'],nominal_path=nominal_path,angles=p['angles'],d_free=p['d_free'])
                for i,s in enumerate(p['scans']):
                    arrays[f'scan_{i}_hits']=s['hits'][s['valid']]
                for method,(items,chosen,_) in proposals.items():
                    arrays[method+'_seeds'] = np.array([g['trajectory'] for g in chosen])
                    arrays[method+'_targets'] = np.array([g['target'] for g in chosen])
                # Rotate execution order to avoid always warming one method first.
                for repeat,seed in enumerate(SEEDS):
                    order = np.roll(np.array(METHODS),-(frame_index+repeat)%len(METHODS))
                    for method in order:
                        items,chosen,proposal_ms = proposals.get(method,([],[],0.))
                        row = dict(scene=scene_id,bag=name,split=audit['split'],offset=snap['elapsed'],
                                   method=method,seed=seed,prepare_ms=prep_ms,proposal_ms=proposal_ms,
                                   raw_gaps=len(items),selected_gaps=len(chosen),escape=escape,
                                   rearward_seeds=sum(g['progress_proxy']<0 for g in chosen),
                                   selected_gap_ids=[g['id'] for g in chosen],
                                   selected_angles=[float(np.rad2deg(g['angle'])) for g in chosen],
                                   reach_rejected=sum(g['reason']=='unreachable' for g in items))
                        try:
                            trial_start=time.perf_counter()
                            output,paths=run_optimizer(p,cfg,nominal,chosen,predictions,method,seed)
                            wall_ms=(time.perf_counter()-trial_start)*1000
                            row.update(output)
                            row['offline_total_ms']=prep_ms+proposal_ms+wall_ms
                            row['optimizer_call_wall_ms']=wall_ms
                            if repeat==0:
                                arrays[method+'_trajectory']=paths['trajectory']
                        except Exception as exc:
                            row.update(status='exception',safe=False,moving=False,error=f'{type(exc).__name__}: {exc}')
                        trial_file.write(json.dumps(serial(row),ensure_ascii=False)+'\n');trial_file.flush()
                np.savez_compressed(out/'scenes'/f'{scene_id}.npz',**arrays)
                if accepted%5==0:
                    print(f'PROGRESS accepted={accepted} skipped={skipped} bag={name} t={snap["elapsed"]:.1f}',flush=True)
            del data
    write_json(out/'run_summary.json',dict(accepted_scenes=accepted,skipped_scenes=skipped,
                                         expected_trials=accepted*len(METHODS)*len(SEEDS)))
    for name,digest in hashes.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest, f'Frozen source changed: {name}'
    print(f'DONE accepted={accepted} skipped={skipped}',flush=True)


if __name__ == '__main__':
    main()
