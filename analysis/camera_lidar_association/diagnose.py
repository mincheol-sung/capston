"""Offline association diagnostics and sensitivity checks; no true labels.

All counterfactual acceptance counts are geometric matches, NOT accuracy.
Ego compensation assumes a static object and interpolates recorded odom pose.
The +50ms scan scenario is a sensitivity check, not recovered point timing.
"""
from analyze import *

def diagnose(path):
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    names=['/detections','/lidar_clusters','/detections_fused','/lidar_obstacles','/odometry/filtered','/scan','/scan_front','/scan_rear','/tf_static','/rosout']
    types={t.name:get_message(t.type) for t in r.get_all_topics_and_types() if t.name in names}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    frames=defaultdict(list); scan=defaultdict(list); logs=[]; tf=[]
    while r.has_next():
        name,raw,t=r.read_next(); m=deserialize_message(raw,types[name])
        if name.startswith('/scan'):
            scan[name].append([m.time_increment,m.scan_time,m.angle_increment,len(m.ranges)])
        elif name=='/rosout':
            if m.name in ('rgbd_detection_node','lidar_cluster_node','dual_scan_merger','camera_lidar_fusion_node'): logs.append(m.msg)
        elif name=='/tf_static':
            tf.extend({'parent':x.header.frame_id,'child':x.child_frame_id,'translation':[x.transform.translation.x,x.transform.translation.y,x.transform.translation.z],'quaternion':[x.transform.rotation.x,x.transform.rotation.y,x.transform.rotation.z,x.transform.rotation.w]} for x in m.transforms)
        else: frames[name].append((t,m))
    cams={stamp(m):m for _,m in frames['/detections']}; lids={stamp(m):m for _,m in frames['/lidar_clusters']}
    obs=frames['/lidar_obstacles']; ot=np.array([t for t,_ in obs],dtype=np.int64)
    od=[]
    for _,m in frames['/odometry/filtered']:
        p=m.pose.pose.position; q=m.pose.pose.orientation
        od.append([stamp(m),p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
    od=np.array(sorted(od)); origin=od[0,0]; od[:,0]=(od[:,0]-origin)*1e-9; od[:,3]=np.unwrap(od[:,3])
    def pose(t):
        seconds=(t-origin)*1e-9
        return np.array([np.interp(seconds,od[:,0],od[:,j]) for j in (1,2,3)])
    count=Counter(); cls=defaultdict(Counter); residuals=defaultdict(lambda:defaultdict(list)); sensitivity=defaultdict(Counter)
    floor=[0,.02,.05,.1]
    sample=[]
    for t,f in frames['/detections_fused']:
        c=cams.get(stamp(f)); k=int(np.argmin(abs(ot-t))); l=lids.get(stamp(obs[k][1]))
        if c is None or l is None or abs(int(ot[k])-t)>20_000_000: continue
        cp=np.array([xy(o) for o in c.objects]); lp=np.array([corrected(o) for o in l.objects])
        if not len(cp): continue
        count['camera_observations']+=len(cp)
        if not len(lp):
            count['no_lidar_candidates']+=len(cp); continue
        cr=np.array([cm.cov(*p) for p in cp]); lr=np.array([lm.cov(*p) for p in lp])
        variants={'baseline':lp}
        target=pose(stamp(c))
        for name,shift in [('ego_header',0),('ego_header_plus_50ms',50_000_000)]:
            src=pose(stamp(l)+shift)
            variants[name]=(lp@rot(src[2]).T+src[:2]-target[:2])@rot(target[2])
        for name,points in variants.items():
            costs=np.zeros((len(cp),len(lp)))
            Sc=[]
            for i,p in enumerate(cp):
                J=rot(math.atan2(p[1],p[0])); Q=J@np.diag([.00508**2,.00695**2])@J.T
                for j,q in enumerate(points):
                    # Rotate LiDAR covariance with the ego transform, too.
                    R=lr[j]
                    if name!='baseline':
                        shift=50_000_000 if name.endswith('50ms') else 0
                        angle=pose(stamp(l)+shift)[2]-target[2]; A=rot(angle); R=A@R@A.T
                    S=cr[i]+R+Q; delta=p-q; costs[i,j]=delta@np.linalg.solve(S,delta)
                    if name=='baseline': Sc.append(S)
            matched,_,_=assign(costs,9.21)
            for i,o in enumerate(c.objects):
                sensitivity[name]['observations']+=1
                sensitivity[name]['matched']+=int(i in matched)
                if o.class_name=='person':
                    sensitivity[name]['person_observations']+=1; sensitivity[name]['person_matched']+=int(i in matched)
            if name!='baseline': continue
            for i,p in enumerate(cp):
                o=c.objects[i]; label=o.class_name
                count['accepted']+=int(i in matched)
                if i in matched: continue
                hasvalid=bool(np.any(costs[i]<=9.21))
                cause='assignment_competition' if hasvalid else 'outside_gate'
                count[cause]+=1; cls[label][cause]+=1
                j=int(np.argmin(np.linalg.norm(points-p,axis=1)))
                diff=p-points[j]; J=rot(math.atan2(p[1],p[0])); dr,dt=J.T@diff
                residuals[label]['nearest_radial_abs_m'].append(abs(float(dr)))
                residuals[label]['nearest_tangential_abs_m'].append(abs(float(dt)))
                residuals[label]['nearest_lidar_width_m'].append(l.objects[j].bbox.size_x)
                if label=='person' and np.linalg.norm(diff)<=.2 and len(sample)<8:
                    sample.append({'camera_stamp_ns':stamp(c),'camera_xy':p.tolist(),'camera_z':o.position.z,'lidar_xy_corrected':points[j].tolist(),'lidar_width_m':l.objects[j].bbox.size_x,'chi2':float(costs[i,j]),'radial_difference_m':float(dr),'tangential_difference_m':float(dt)})
            for sigma in floor[1:]:
                enlarged=np.zeros_like(costs)
                for i,p in enumerate(cp):
                    for j,q in enumerate(points):
                        delta=p-q; enlarged[i,j]=delta@np.linalg.solve(Sc[i*len(lp)+j]+sigma**2*np.eye(2),delta)
                ma,_,_=assign(enlarged,9.21)
                key=f'additional_pair_sigma_{sigma}m'
                sensitivity[key]['matched']+=len(ma)
                sensitivity[key]['person_matched']+=sum(c.objects[i].class_name=='person' for i in ma)
    return {'bag':path.name,'reconstructed_counts':dict(count),'unmatched_causes_by_class':{k:dict(v) for k,v in cls.items()},'nearest_unmatched_residuals':{k:{metric:quant(vals) for metric,vals in data.items()} for k,data in residuals.items()},'counterfactuals_not_accuracy':{k:dict(v) for k,v in sensitivity.items()},'scan_timing':{k:{'frames':len(v),'median_time_increment_s':float(np.median(np.array(v)[:,0])),'median_scan_time_s':float(np.median(np.array(v)[:,1])),'median_angle_increment_deg':float(np.median(np.array(v)[:,2])*180/math.pi),'median_points':float(np.median(np.array(v)[:,3]))} for k,v in scan.items()},'recorded_static_transforms':tf,'startup_and_warning_logs':logs,'close_person_rejection_examples':sample}

if __name__=='__main__':
    result=[diagnose(p) for p in sorted((WORKSPACE_ROOT/'bags').glob('mpc_chain_*'))]
    Path(__file__).with_name('diagnosis.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
    for d in result:
        print(json.dumps({k:d[k] for k in ('bag','reconstructed_counts','counterfactuals_not_accuracy','scan_timing','close_person_rejection_examples')},indent=2))
