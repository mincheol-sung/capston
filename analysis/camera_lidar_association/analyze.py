"""Read-only analysis of recorded source flags and reconstructed associations.

Run with /opt/ros/jazzy and workspace install/setup.bash sourced.
Reconstruction uses current default calibration; output agreement is checked.
"""
import sys, json, math
from pathlib import Path
from collections import Counter, defaultdict
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = WORKSPACE_ROOT / 'src' / 'amr_basic_stack'
sys.path[:0] = [str(PACKAGE_ROOT/'perception/sensor_fusion'), str(PACKAGE_ROOT/'perception/object_tracker')]
from sensor_fusion.camera_lidar_fusion_node import PolarNoiseModel, RepresentativeOffset, rot
from object_tracker.core.association import assign

def stamp(m): return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def quant(v):
    return dict(zip(('p50','p90','p95','max'), map(float,np.quantile(v,[.5,.9,.95,1])))) if v else {}
def xy(o): return np.array([o.position.x,o.position.y])
cm = PolarNoiseModel(.321e-3,2.741,.560e-3,1.450)
lm = PolarNoiseModel(.331e-3,.354,6.5e-3,0,1.6e-3,4.5)
offset = RepresentativeOffset(-10.98e-3,1.281,13e-3)
def corrected(o):
    p=np.array([.086291,-.020494])+rot(-.043670)@(xy(o)-np.array([.086291,-.020494]))
    return np.array(offset.apply(*p))

def analyze(path):
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    names=['/detections','/lidar_clusters','/detections_fused','/lidar_obstacles','/odom','/rosout']
    types={t.name:get_message(t.type) for t in r.get_all_topics_and_types() if t.name in names}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    frames=defaultdict(list); logs=[]
    while r.has_next():
        topic,raw,t=r.read_next(); m=deserialize_message(raw,types[topic])
        if topic=='/rosout':
            if 'fusion' in m.name: logs.append(m.msg)
        else: frames[topic].append((t,m))
    cams={stamp(m):m for _,m in frames['/detections']}
    lids={stamp(m):m for _,m in frames['/lidar_clusters']}
    obstacles=frames['/lidar_obstacles']; ot=np.array([t for t,_ in obstacles],dtype=np.int64)
    od=frames['/odom']; odt=np.array([stamp(m) for _,m in od],dtype=np.int64)
    classes=defaultdict(Counter); bins=defaultdict(Counter); motion=defaultdict(Counter)
    dt=[]; residual=[]; rawres=[]; chi=[]; agreement=[]; ambiguous=0; matches=0; reconstructed=0
    sources=Counter(); missing=0; outputstamps=[]; empty=0; examples=[]
    rejected=defaultdict(list); nearest=defaultdict(list); near_reject=defaultdict(Counter)
    for t,f in frames['/detections_fused']:
        outputstamps.append(stamp(f)); empty+=not bool(f.objects)
        for o in f.objects:
            sources[str(o.source)]+=1
            if o.source in (0,2):
                classes[o.class_name][str(o.source)]+=1
                d=math.hypot(o.position.x,o.position.y)
                bins['<2m' if d<2 else '2-4m' if d<4 else '4-6m' if d<6 else '>=6m'][str(o.source)]+=1
                if len(odt):
                    k=int(np.argmin(abs(odt-stamp(f)))); v=od[k][1].twist.twist
                    speed=math.hypot(v.linear.x,v.linear.y)
                    motion['stationary' if speed<.05 and abs(v.angular.z)<.05 else 'moving'][str(o.source)]+=1
        c=cams.get(stamp(f))
        if c is None or not len(ot): missing+=1; continue
        k=int(np.argmin(abs(ot-t))); ob=obstacles[k][1]; l=lids.get(stamp(ob))
        if l is None or abs(int(ot[k])-t)>20_000_000: missing+=1; continue
        if c.header.frame_id!=l.header.frame_id: missing+=1; continue
        dt.append(abs(stamp(c)-stamp(l))/1e6)
        cp=[xy(o) for o in c.objects]; lp=[corrected(o) for o in l.objects]
        cr=[cm.cov(*p) for p in cp]; lr=[lm.cov(*p) for p in lp]
        costs=np.full((len(cp),len(lp)),np.inf)
        for i,p in enumerate(cp):
            J=rot(math.atan2(p[1],p[0])); Q=J@np.diag([.00508**2,.00695**2])@J.T
            for j,q in enumerate(lp):
                delta=p-q; costs[i,j]=delta@np.linalg.solve(cr[i]+lr[j]+Q,delta)
        matched,_,_=assign(costs,9.21)
        sem=[o for o in f.objects if o.source in (0,2)]
        if len(sem)!=len(cp): missing+=1; continue
        reconstructed+=1
        for i,p in enumerate(cp):
            o=sem[i]; agreement.append(int((o.source==2)==(i in matched)))
            if o.source==0:
                label=o.class_name; near_reject[label]['total']+=1
                if len(lp):
                    nd=min(float(np.linalg.norm(p-q)) for q in lp)
                    nearest[label].append(nd); rejected[label].append(float(np.min(costs[i])))
                    near_reject[label]['within_20cm']+=int(nd<=.2)
                    near_reject[label]['within_50cm']+=int(nd<=.5)
                else: near_reject[label]['no_lidar_objects']+=1
            if i in matched:
                j=matched[i]; Ic=np.linalg.inv(cr[i]); Il=np.linalg.inv(lr[j]); z=np.linalg.solve(Ic+Il,Ic@p+Il@lp[j])
                # Validate both selected source and recorded fused coordinates.
                if o.source==2 and np.linalg.norm(z-xy(o))<1e-5:
                    matches+=1; residual.append(float(np.linalg.norm(p-lp[j]))); rawres.append(float(np.linalg.norm(p-xy(l.objects[j])))); chi.append(float(costs[i,j]))
                    ambiguous+=int(np.sum(costs[i]<=9.21)>1)
                elif len(examples)<5: examples.append({'stamp_ns':stamp(f),'class':o.class_name,'source':int(o.source),'position_difference_m':float(np.linalg.norm(z-xy(o)))})
    def rates(counts):
        return {k:{'camera_only':v['0'],'fused':v['2'],'match_pct':100*v['2']/max(1,v['0']+v['2'])} for k,v in counts.items()}
    totalcam=sum(len(m.objects) for _,m in frames['/detections'])
    Path(__file__).with_name(path.name+'_unmatched.json').write_text(json.dumps({k:{'counts':dict(v),'nearest_distance_m':quant(nearest[k]),'best_chi2':quant(rejected[k])} for k,v in near_reject.items()},indent=2))
    semantic=sources['0']+sources['2']
    return {'bag':path.name,'frames':{k:len(v) for k,v in frames.items()},'camera_objects_all_frames':totalcam,'sources':dict(sources),'semantic_match_pct':100*sources['2']/max(1,semantic),'classes':rates(classes),'distance_bins_output_position':rates(bins),'motion':rates(motion),'empty_fusion_frames':empty,'fusion_gap_ms':quant((np.diff(sorted(outputstamps))/1e6).tolist()),'paired_dt_ms':quant(dt),'reconstructed_frames':reconstructed,'unreconstructed_frames':missing,'reconstructed_source_agreement_pct':100*np.mean(agreement) if agreement else None,'exact_position_validated_matches':matches,'corrected_pair_distance_m':quant(residual),'raw_pair_distance_m':quant(rawres),'matched_chi2':quant(chi),'multiple_gated_candidates':ambiguous,'reconstruction_disagreements_examples':examples,'fusion_logs':logs}

if __name__=='__main__':
    out=[analyze(p) for p in sorted((WORKSPACE_ROOT/'bags').glob('mpc_chain_*'))]
    target=Path(__file__).with_name('results.json'); target.write_text(json.dumps(out,indent=2,ensure_ascii=False))
    print(json.dumps(out,indent=2,ensure_ascii=False))
