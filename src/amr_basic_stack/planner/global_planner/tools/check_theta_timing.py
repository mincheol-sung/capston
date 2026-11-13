#!/usr/bin/env python3
"""Isolated saved-map Theta* goal/replan callbacks; no robot or live TF."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.ndimage import label
import yaml
import rclpy
from geometry_msgs.msg import PoseStamped
from global_planner.common.ros_map_loader import load_ros_map
from global_planner.theta_star.costmap import Costmap
import global_planner.rviz_theta_star_node as module


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[1]
    params=yaml.safe_load((root/'config/theta_star.yaml').read_text())['rviz_theta_star_node']['ros__parameters']
    rows=[]; maps=[]
    for map_name in ('smoc_lab','test_imu_scan'):
        yaml_path=root/'maps'/map_name/(map_name+'.yaml')
        tick=time.perf_counter();rosmap=load_ros_map(yaml_path)
        costmap=Costmap(rosmap.grid_map,robot_size=math.ceil(params['robot_size']/rosmap.resolution))
        setup_ms=(time.perf_counter()-tick)*1000
        labels,n=label(costmap.inflated_map==0)
        sizes=np.bincount(labels.ravel());sizes[0]=0
        y,x=np.where(labels==np.argmax(sizes));cells=np.column_stack((x,y))
        # Reproducible endpoints in the largest free component, no live pose.
        center=np.mean(cells,axis=0);start=cells[np.argmin(np.sum((cells-center)**2,axis=1))]
        distances=np.linalg.norm(cells-start,axis=1)*rosmap.resolution
        maps.append(dict(map=map_name,shape=list(rosmap.grid_map.shape),resolution=rosmap.resolution,
            load_and_inflation_ms=setup_ms,map_sha256=hashlib.sha256(rosmap.image_path.read_bytes()).hexdigest()))
        for requested_distance in (3.,8.):
            goal=cells[np.argmin(abs(distances-requested_distance))]
            start_world=rosmap.grid_to_world(start);goal_world=rosmap.grid_to_world(goal)
            node_params={**params,'map':map_name,'maps_dir':str(root/'maps'),
                'path_topic':'/theta_timing/global_path','waypoint_marker_topic':'/theta_timing/waypoints',
                'goal_topic':'/theta_timing/goal','initial_pose_topic':'/theta_timing/initial',
                'traversable_map_topic':'/theta_timing/traversable'}
            file=a.output/f'{map_name}_params.yaml';file.write_text(yaml.safe_dump({'rviz_theta_star_node':{'ros__parameters':node_params}}))
            rclpy.init(args=['--ros-args','--params-file',str(file)])
            node=module.RvizThetaStarNode();node.timer.cancel() if hasattr(node,'timer') else None
            node.initial_pose=start_world
            node._planning_start=lambda:(start_world,'timing_fixture')
            selected_costmap=costmap
            node._planning_costmap=lambda **kwargs:selected_costmap
            goal_msg=PoseStamped();goal_msg.header.frame_id=node.map_frame
            goal_msg.pose.position.x,goal_msg.pose.position.y=goal_world;goal_msg.pose.orientation.w=1.
            original=module.theta_star;original_plan=node._plan;current={}
            def timed_search(*args,**kw):
                tick=time.perf_counter();result=original(*args,**kw);current['search_ms']=(time.perf_counter()-tick)*1000;return result
            def timed_plan(*args,**kw):
                tick=time.perf_counter()
                try:return original_plan(*args,**kw)
                finally:current['plan_ms']=(time.perf_counter()-tick)*1000
            module.theta_star=timed_search;node._plan=timed_plan
            try:
                first_path=None
                for repeat in range(3):
                    current={};tick=time.perf_counter();node._on_goal(goal_msg);elapsed=(time.perf_counter()-tick)*1000
                    first_path=list(node.current_path)
                    rows.append(dict(map=map_name,requested_distance=requested_distance,phase='initial_goal',repeat=repeat,
                        callback_ms=elapsed,success=bool(first_path),poses=len(first_path),start=start_world,goal=goal_world,**current))
                if not first_path:continue
                # A newly observed obstacle blocks the old path midpoint.
                midpoint=first_path[len(first_path)//2];mx,my=rosmap.world_to_grid(midpoint)
                grid=rosmap.grid_map.copy();yy,xx=np.ogrid[:grid.shape[0],:grid.shape[1]]
                grid[(xx-mx)**2+(yy-my)**2 <= (.2/rosmap.resolution)**2]=1
                tick=time.perf_counter();changed=Costmap(grid,robot_size=math.ceil(params['robot_size']/rosmap.resolution))
                update_ms=(time.perf_counter()-tick)*1000
                selected_costmap=changed
                for repeat in range(3):
                    node.current_path=list(first_path);node.current_goal=goal_world;node.current_goal_verified=False
                    node._replan_pending=False;current={};tick=time.perf_counter();node._validate_current_path();elapsed=(time.perf_counter()-tick)*1000
                    did_replan='plan_ms' in current
                    success=did_replan and bool(node.current_path) and not node._replan_pending
                    rows.append(dict(map=map_name,requested_distance=requested_distance,phase='blocked_path_replan',repeat=repeat,
                        callback_ms=elapsed,success=success,replan_called=did_replan,poses=len(node.current_path),
                        map_inflation_ms=update_ms,start=start_world,goal=goal_world,**current))
            finally:
                module.theta_star=original;node.destroy_node();rclpy.shutdown()
            (a.output/'theta_raw.json').write_text(json.dumps(dict(maps=maps,rows=rows),indent=2))
            print(map_name,requested_distance,'done',flush=True)
    summaries=[]
    for name in ('smoc_lab','test_imu_scan'):
        for distance in (3.,8.):
            for phase in ('initial_goal','blocked_path_replan'):
                group=[r for r in rows if r['map']==name and r['requested_distance']==distance and r['phase']==phase]
                if not group:continue
                summaries.append(dict(map=name,requested_distance=distance,phase=phase,attempts=len(group),successes=sum(r['success'] for r in group),
                    callback_mean_ms=float(np.mean([r['callback_ms'] for r in group])),callback_max_ms=max(r['callback_ms'] for r in group),
                    search_mean_ms=float(np.mean([r.get('search_ms',0) for r in group]))))
    (a.output/'theta_summary.json').write_text(json.dumps(dict(summaries=summaries,maps=maps,
        limitation='Saved-map synthetic endpoints and injected obstacle; direct actual ROS node callbacks with fixed pose/costmap fixtures, publication included. Not live operational pose/goal, no DDS scheduling/TF wait. 3 repeats only; no p99 claim. Validation timer 1Hz can add up to 1s detection delay.'),indent=2))


if __name__=='__main__':main()
