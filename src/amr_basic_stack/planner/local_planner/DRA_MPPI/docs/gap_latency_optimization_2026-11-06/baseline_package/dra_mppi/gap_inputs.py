"""Time-aligned scan observations and conservative scan/map fusion for gap."""
from dataclasses import dataclass
import numpy as np
from .geometry import transform_points
from .grid_costmap import DistanceField, GridMetadata


class GapInputUnavailable(ValueError):
    """Missing, stale or inconsistent scan input; not an empty valid gap set."""


@dataclass(frozen=True)
class GapScan:
    ranges: np.ndarray
    valid: np.ndarray
    origins: np.ndarray  # per-beam sensor origins, planning frame
    hits: np.ndarray     # per-beam hits, planning frame; invalid entries ignored

    def validate(self):
        n=len(self.ranges)
        if not 2 <= n <= 20000 or self.valid.shape != (n,) or self.origins.shape != (n,2) or self.hits.shape != (n,2):
            raise GapInputUnavailable('invalid scan array shape')
        if self.valid.dtype != np.bool_ or not np.any(self.valid):
            raise GapInputUnavailable('scan has no time-aligned finite returns')
        if (not np.all(np.isfinite(self.ranges[self.valid]))
                or np.any(self.ranges[self.valid] <= 0)
                or not np.all(np.isfinite(self.origins))
                or not np.all(np.isfinite(self.hits))):
            raise GapInputUnavailable('nonfinite scan geometry')
        return self


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def deskew_scan(message, history, sensor_to_base, epoch):
    """Interpolate recorded planning-frame base poses; never extrapolate beams."""
    r=np.asarray(message.ranges,dtype=float)
    metadata=np.array([message.angle_min,message.angle_increment,message.time_increment,
                       message.range_min,message.range_max],float)
    if (r.ndim!=1 or not 2<=len(r)<=20000 or not np.all(np.isfinite(metadata))
            or message.angle_increment<=0 or message.time_increment<0
            or message.range_min<0 or message.range_max<=message.range_min):
        raise GapInputUnavailable('invalid LaserScan metadata')
    h=np.asarray(history,float)
    if h.ndim!=2 or h.shape[1]!=4 or len(h)<2 or not np.all(np.isfinite(h)):
        raise GapInputUnavailable('insufficient odometry history for scan deskew')
    times=h[:,0]
    if np.any(np.diff(times)<=0) or epoch>times[-1]+1e-6:
        raise GapInputUnavailable('nonmonotonic odometry or unbracketed scan epoch')
    if np.asarray(sensor_to_base).shape!=(3,) or not np.all(np.isfinite(sensor_to_base)):
        raise GapInputUnavailable('invalid LiDAR extrinsics')
    start=stamp_seconds(message.header.stamp)
    if start<=0 or start>epoch:
        raise GapInputUnavailable('scan begins after the current odometry epoch')
    beam_times=start+np.arange(len(r))*message.time_increment
    valid=np.isfinite(r)&(r>=message.range_min)&(r<=message.range_max)&(r>0)&(beam_times>=times[0])&(beam_times<=epoch)
    # Do not interpolate through an odometry dropout.
    upper=np.clip(np.searchsorted(times,beam_times,side='right'),1,len(times)-1)
    valid &= (times[upper]-times[upper-1] <= .10+1e-9)
    poses=h[:,1:].copy();poses[:,2]=np.unwrap(poses[:,2])
    interp=np.column_stack([np.interp(beam_times,times,poses[:,j]) for j in range(3)])
    x,y,yaw=sensor_to_base;cs=np.cos(interp[:,2]);sn=np.sin(interp[:,2])
    origins=interp[:,:2]+np.column_stack((cs*x-sn*y,sn*x+cs*y))
    bearing=message.angle_min+np.arange(len(r))*message.angle_increment+yaw+interp[:,2]
    hits=origins+np.where(valid,r,0)[:,None]*np.column_stack((np.cos(bearing),np.sin(bearing)))
    return GapScan(r.copy(),valid,origins,hits).validate()


def inverse_pose(pose):
    x,y,a=pose;c,s=np.cos(a),np.sin(a)
    return (-c*x-s*y,s*x-c*y,-a)


def fuse_scans(scans, state, field, source_to_grid, config, radius):
    """Local scan grid with valid upstream-map supplementation, like the replay."""
    import cv2
    if scans is None or len(scans)!=2 or field is None:
        raise GapInputUnavailable('gap requires both LiDAR observations and a map')
    ext=config.grid_extent_m;res=config.grid_resolution_m;n=round(2*ext/res)
    free=np.zeros((n,n),np.uint8);occupied=np.zeros_like(free)
    transform=inverse_pose(state);local=[]
    def pixel(xy):return np.floor((xy+ext)/res).astype(np.int32)
    for scan in scans:
        scan.validate()
        origins=transform_points(scan.origins,transform);hits=transform_points(scan.hits,transform)
        valid=scan.valid;r=scan.ranges
        delta=hits-origins;norm=np.linalg.norm(delta,axis=1)
        unit=np.divide(delta,norm[:,None],out=np.zeros_like(delta),where=norm[:,None]>1e-12)
        for i in np.flatnonzero(valid[:-1]&valid[1:]):
            if abs(r[i+1]-r[i])>.25:continue
            depth=max(0.,min(r[i],r[i+1],1.5*ext)-res)
            vertices=np.vstack((origins[i],origins[i+1],origins[i+1]+depth*unit[i+1],origins[i]+depth*unit[i]))
            cv2.fillConvexPoly(free,pixel(vertices),1)
        # Clip before converting arbitrarily large finite sensor returns to ints.
        measured=hits[valid];measured=measured[np.all((measured>=-ext)&(measured<ext),axis=1)]
        cells=pixel(measured);occupied[cells[:,1],cells[:,0]]=1
        local.append(dict(ranges=r,valid=valid,hits=hits))
    coords=-ext+(np.arange(n)+.5)*res;xx,yy=np.meshgrid(coords,coords)
    free[xx*xx+yy*yy<radius**2]=1
    grid=np.full((n,n),-1,np.int16);grid[free!=0]=0;grid[occupied!=0]=100
    world=transform_points(np.stack((xx,yy),axis=-1),state)
    mapped=field.occupancy_at(world,source_to_grid)
    grid[(grid<0)&(mapped>=0)&(mapped<65)]=0
    grid[mapped>=65]=100
    fused=DistanceField.from_occupancy(grid,GridMetadata(n,n,res,-ext,-ext,0.),'gap_local',unknown_is_occupied=True)
    return fused,transform,local


def directional_depth(field,config,radius):
    angles=np.deg2rad(np.arange(-180.,180.))
    distance=np.arange(0.,config.max_depth_m+1e-9,config.radial_step_m)
    rays=np.stack((np.cos(angles),np.sin(angles)),axis=1)[:,None,:]*distance[None,:,None]
    # Include both grid-cell and sweep padding, and the map boundary.
    clearance=field.state_clearances(rays)-field.metadata.resolution*.25
    def depth(margin):
        prefix=np.logical_and.accumulate(clearance>radius+margin,axis=1)
        return np.max(np.where(prefix,distance,0.),axis=1)
    return angles,depth(config.extra_margin_m),depth(0.)
