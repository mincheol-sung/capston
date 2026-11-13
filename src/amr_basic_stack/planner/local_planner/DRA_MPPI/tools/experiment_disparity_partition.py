"""Real dual-LiDAR disparity partition prototype; no planner/robot commands.

Extract: source ROS/workspace setup, then run this file.
Reuse extraction: python3 this_file.py --render-only
The detector operates on uncapped sensor ranges; the colored region uses the
previous conservative body-clearance field. No candidate selection/MPPI is run.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'docs/gap_multiscene_2026-11-04'
OUT = BASE / 'disparity_partition_10_scenes'
CONFIG = dict(adjacent_jump_m=.4, relative_jump=.12, side_window_beams=3,
              side_median_jump_m=.4, side_spread_fraction=.35,
              minimum_cut_separation_deg=20., minimum_region_bins=6,
              geometry_window_bins=5, geometry_jump_m=.25, maximum_projection_bins=8,
              minimum_depth_m=.5, angle_step_deg=1., max_depth_m=2.,
              body_radius_m=.20, extra_margin_m=.05,
              retry_extra_margin_m=0., grid_resolution_m=.04,
              grid_padding_m=float(np.sqrt(2)*.04), sweep_padding_m=.01)


def dump(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


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


def choose_scenes():
    rows = [json.loads(x) for x in (BASE/'scene_audit.jsonl').read_text().splitlines()]
    selected = []
    for prefix, qs in [('b0_', [.2,.5,.8]), ('b1_', [.2,.5,.8]), ('b2_', [.1,.35,.65,.9])]:
        valid = [r for r in rows if r['accepted'] and r['scene'].startswith(prefix) and r['scene'] != 'b1_088']
        for q in qs:
            row = valid[round(q*(len(valid)-1))]
            selected.append({k:row[k] for k in ['scene','bag','offset','record_ns']})
    assert len(selected) == 10 and len({s['scene'] for s in selected}) == 10
    return selected


def extract(scenes):
    import sys
    sys.path.insert(0, str(BASE/'source_snapshot/tools'))
    import experiment_gap_multiscene as ex
    records = {s['record_ns']:s for s in scenes}
    for bag in dict.fromkeys(s['bag'] for s in scenes):
        print('READ', bag, flush=True)
        data = ex.read_bag(Path('/home/smoc/2026amr_ws/bags')/bag)
        for snapshot in data['snapshots']:
            if snapshot['record_ns'] not in records: continue
            scene = records[snapshot['record_ns']]
            p = ex.prepare_snapshot(data, snapshot)
            ex.input_check(data, snapshot, p)
            with np.load(BASE/'scenes'/f'{scene["scene"]}.npz') as previous:
                for key in ['grid', 'angles', 'd_free', 'full_path']:
                    np.testing.assert_allclose(p[key], previous[key], atol=1e-10)
            angle=p['angles']; dist=np.arange(0., 2.02, .02)
            rays=np.stack([np.cos(angle),np.sin(angle)],axis=1)[:,None,:]*dist[None,:,None]
            clearance=p['field'].query(rays)-np.sqrt(2)*.04-.01
            hard=np.max(np.where(np.logical_and.accumulate(clearance>.2,axis=1),dist,0),axis=1)
            arrays={k:p[k] for k in ['grid','scan_grid','map_fill','full_path','angles']}
            arrays.update(strict_depth=p['d_free'], hard_depth=hard)
            for i, scan in enumerate(p['scans']):
                for key in ['ranges','angles','valid','beam_times','origins','hits']:
                    arrays[f'scan_{i}_{key}']=scan[key]
            np.savez_compressed(OUT/'data'/f'{scene["scene"]}.npz', **arrays)
            print('MATCHED', scene['scene'], flush=True)


def plot_scene(scene, arrays, result, number):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Circle, Polygon
    colors=['#008e85','#8853b5','#cf8a22','#387bb7','#bd5770','#71963e','#946145','#786ab5','#449ca3','#b97d39']
    ink='#19364e'
    fig=plt.figure(figsize=(16,10), facecolor='#f7f9fc')
    fig.text(.045,.95,f'{number:02d}  실제 LiDAR 거리 급변 분할 · {result["region_count"]}개 구역',fontsize=24,weight='bold',color=ink)
    fig.text(.045,.905,f'{scene["scene"]}  |  {scene["bag"]} +{scene["offset"]:.3f} s  |  열린 구간 {result["open_runs"]}개 + 내부 급변 경계 {result["selected_cut_count"]}개',fontsize=12,color=ink)
    ax=fig.add_axes([.045,.19,.49,.65])
    grid=arrays['grid'];view=np.zeros_like(grid);view[grid<0]=1
    view[arrays['map_fill']&(grid==0)]=2;view[grid>=65]=3
    ax.imshow(view,extent=(-4,4,-4,4),origin='lower',vmin=0,vmax=3,
              cmap=ListedColormap(['white','#d6dee7','#e2ecfc','#283f54']),interpolation='nearest')
    rawax=fig.add_axes([.61,.60,.34,.23])
    for i,col in enumerate(['#bf7737','#4e78a2']):
        valid=arrays[f'scan_{i}_valid'];hits=arrays[f'scan_{i}_hits'][valid]
        ax.scatter(hits[:,0],hits[:,1],s=2,color=col,alpha=.65)
        rawax.scatter(np.degrees(np.arctan2(hits[:,1],hits[:,0])),arrays[f'scan_{i}_ranges'][valid],s=1.5,color=col,label=['앞','뒤'][i],alpha=.55)
    rawax.set(xlim=(-180,180),xlabel='현재 로봇 기준 hit 방향 [°]',ylabel='센서 원시 거리 [m]',title='2 m로 자르기 전 거리에서 경계 검출')
    rawax.legend(fontsize=9,loc='upper right');rawax.grid(alpha=.15)
    da=fig.add_axes([.61,.28,.34,.22])
    depth=arrays['hard_depth'] if result['fallback'] else arrays['strict_depth']
    degrees=np.degrees(arrays['angles'])
    da.plot(degrees,depth,color='#a4aeb9',lw=1)
    for region in result['regions']:
        ids=np.array(region['bins']);ang=arrays['angles'][ids];rad=depth[ids]
        xy=rad[:,None]*np.stack((np.cos(ang),np.sin(ang)),axis=1)
        col=colors[(region['id']-1)%len(colors)]
        polygon=xy if len(ids)==360 else np.vstack(([0,0],xy))
        ax.add_patch(Polygon(polygon,facecolor=col,alpha=.25,edgecolor='none'))
        outline=np.vstack((xy,xy[0])) if len(ids)==360 else xy
        ax.plot(outline[:,0],outline[:,1],color=col,lw=2)
        mid=len(ids)//2;label_radius=max(.31,.62*rad[mid])
        label=label_radius*np.array([np.cos(ang[mid]),np.sin(ang[mid])])
        ax.text(*label,f'G{region["id"]}',color=col,fontsize=13,weight='bold',ha='center',
                bbox=dict(facecolor='white',edgecolor='none',alpha=.85,pad=1.5))
        for chunk in np.split(ids,np.flatnonzero(np.diff(ids)<0)+1):
            da.fill_between(degrees[chunk],0,depth[chunk],color=col,alpha=.25)
            da.plot(degrees[chunk],depth[chunk],color=col,lw=2)
    for e in result['edges']:
        if e['status']!='selected': continue
        angle=np.deg2rad(e['cut_bin']-180-.5)
        radius=max(depth[e['cut_bin']],depth[(e['cut_bin']-1)%360])
        endpoint=radius*np.array([np.cos(angle),np.sin(angle)])
        ax.plot([0,endpoint[0]],[0,endpoint[1]],'--',color=ink,lw=1.2)
        da.axvline(np.rad2deg(angle),color=ink,ls='--',lw=1)
        rawax.scatter(e['angle_deg'],min(e['range_before'],e['range_after']),marker='*',s=95,c=ink,zorder=7)
    ax.add_patch(Circle((0,0),.2,facecolor=ink,edgecolor='white',zorder=9))
    ax.annotate('',(.4,0),(0,0),arrowprops=dict(arrowstyle='->',color=ink,lw=2),zorder=10)
    ax.set(xlim=(-2.3,2.3),ylim=(-2.3,2.3),aspect='equal',xlabel='x [m] · 전방 +',ylabel='y [m] · 좌측 +')
    ax.grid(alpha=.15)
    if not result['regions']:
        ax.text(0,-.7,'몸체 진입 기준을 만족하는\n0.5 m 이상 열린 구간 없음',ha='center',fontsize=13,color='#b74656',bbox=dict(facecolor='white',edgecolor='none',alpha=.9))
    da.axhline(.5,color='#b74656',ls=':',lw=1)
    da.set(xlim=(-180,180),ylim=(0,2.15),xlabel='현재 로봇 기준 방향 [°]',ylabel='몸체 진입 깊이 [m]',title='열린 구간 안에서만 급변 경계로 분할')
    da.grid(alpha=.15)
    info=f'확인된 거리 급변 {len(result["edges"])}개 → 내부 분할 경계 {result["selected_cut_count"]}개'
    info+='\n추가 여유 0.05 m → 0 m 재시도 적용' if result['fallback'] else '\n몸체 반경 0.20 m + 추가 여유 0.05 m 적용'
    fig.text(.61,.20,info,fontsize=12,color=ink,va='top',linespacing=1.6)
    fig.text(.045,.11,'★: 원시 거리 급변 / 점선: 몸체 깊이 변화에 맞춘 경계(최대 ±8° 보정) · 색칠: 몸체 진입 가능 영역',fontsize=11,color=ink)
    fig.text(.045,.07,'흰색: scan free / 연파랑: 기록 지도 보완 / 회색: 미확인 / 진한색: 장애물. 미관측을 free로 바꾸지 않음.',fontsize=10,color='#526c82')
    fig.text(.045,.035,'동일 규칙의 자동 분할 결과. seed 선택·MPPI·주행·5 Hz 검증은 이번 그림에 포함하지 않음.',fontsize=11,color='#526c82')
    file=f'{number:02d}_{scene["scene"]}.png';fig.savefig(OUT/file,dpi=170);plt.close(fig)
    return file


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--render-only',action='store_true');args=parser.parse_args()
    OUT.mkdir(exist_ok=True);(OUT/'data').mkdir(exist_ok=True)
    scenes=choose_scenes()
    dump(OUT/'configuration.json',dict(parameters=CONFIG,scenes=scenes,
         selection='Chronological accepted-scene quantiles: b0,b1 .2/.5/.8; b2 .1/.35/.65/.9; b1_088 excluded before selection.',
         status='Experimental automatic disparity partition only; production v1 unchanged.',
         script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    if not args.render_only: extract(scenes)
    from matplotlib import font_manager
    import matplotlib.pyplot as plt
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),'axes.unicode_minus':False,'font.size':11})
    results=[]
    for n,scene in enumerate(scenes,1):
        with np.load(OUT/'data'/f'{scene["scene"]}.npz') as data:
            result=partition(data)
            file=plot_scene(scene,data,result,n)
        result.update(scene=scene['scene'],bag=scene['bag'],offset=scene['offset'],png=file)
        results.append(result)
        print(scene['scene'], 'regions',result['region_count'],'cuts',result['selected_cut_count'],'fallback',result['fallback'],flush=True)
    dump(OUT/'results.json',results)
    with (OUT/'regions.csv').open('w') as f:
        cols=['scene','id','component','first_angle_deg','last_angle_deg','bin_count','min_depth_m','max_depth_m']
        writer=csv.DictWriter(f,fieldnames=cols);writer.writeheader()
        for r in results:
            for g in r['regions']:writer.writerow({'scene':r['scene'],**{k:g[k] for k in cols[1:]}})
    from PIL import Image
    atlas=Image.new('RGB',(3200,5000),'#f7f9fc')
    for i,r in enumerate(results):
        with Image.open(OUT/r['png']) as im:atlas.paste(im.resize((1600,1000),Image.Resampling.LANCZOS),((i%2)*1600,(i//2)*1000))
    atlas.save(OUT/'00_all_10_scenes.png')
    # Compact map-only comparison for viewing all ten scenes in conversation.
    from PIL import ImageDraw,ImageFont
    overview=Image.new('RGB',(3500,1510),'#f7f9fc');draw=ImageDraw.Draw(overview)
    heading=ImageFont.truetype(font,42);small=ImageFont.truetype(font,25)
    draw.text((45,20),'실제 LiDAR 10장면 · 거리 급변 + 몸체 진입 깊이로 자동 분할',font=heading,fill='#19364e')
    for i,r in enumerate(results):
        x=(i%5)*700;y=105+(i//5)*680
        draw.text((x+45,y),f'{i+1:02d}  {r["scene"]}  ·  {r["region_count"]}개 구역',font=small,fill='#19364e')
        with Image.open(OUT/r['png']) as im:
            w,h=im.size
            crop=im.crop((int(.04*w),int(.15*h),int(.54*w),int(.87*h)))
            crop.thumbnail((680,620),Image.Resampling.LANCZOS)
            overview.paste(crop,(x+(700-crop.width)//2,y+42))
    draw.text((45,1470),'같은 규칙 · 구역 수 고정 없음 · 기존 188.248초 장면 제외 · seed/MPPI 실행 결과 아님',font=small,fill='#526c82')
    overview.save(OUT/'00_partition_overview.png')
    print('DONE',OUT,flush=True)


if __name__=='__main__': main()
