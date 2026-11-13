"""Raster and derivative regression checks for batched computations."""
from dataclasses import replace
import numpy as np
import pytest

from dra_mppi.gap_inputs import GapScan,fuse_scans,inverse_pose
from dra_mppi.gap_geometry import GapConfig
from dra_mppi.geometry import transform_points
from dra_mppi.grid_costmap import DistanceField,GridMetadata
from dra_mppi.gap_optimization import rollout_jacobian
from dra_mppi.mecanum_model import rollout


def scalar_scan_grid(scans,state,cfg,radius):
    """Pre-optimization scalar rasterization as an independent test oracle.

    Upstream map is entirely unknown in this fixture, so it cannot hide a
    difference in scan free-space filling. Overlap is a union, never parity.
    """
    import cv2
    ext,res=cfg.grid_extent_m,cfg.grid_resolution_m;n=round(2*ext/res)
    free=np.zeros((n,n),np.uint8);occupied=np.zeros_like(free)
    def pixel(x):return np.floor((x+ext)/res).astype(np.int32)
    for scan in scans:
        origins=transform_points(scan.origins,inverse_pose(state))
        hits=transform_points(scan.hits,inverse_pose(state))
        delta=hits-origins;norm=np.linalg.norm(delta,axis=1)
        unit=np.divide(delta,norm[:,None],out=np.zeros_like(delta),where=norm[:,None]>1e-12)
        for i in range(len(scan.ranges)-1):
            if not(scan.valid[i]and scan.valid[i+1]):continue
            if abs(scan.ranges[i+1]-scan.ranges[i])>.25:continue
            depth=max(0.,min(scan.ranges[i],scan.ranges[i+1],1.5*ext)-res)
            poly=np.vstack((origins[i],origins[i+1],origins[i+1]+depth*unit[i+1],origins[i]+depth*unit[i]))
            cv2.fillConvexPoly(free,pixel(poly),1)
        measured=hits[scan.valid]
        measured=measured[np.all((measured>=-ext)&(measured<ext),axis=1)]
        cells=pixel(measured);occupied[cells[:,1],cells[:,0]]=1
    coords=-ext+(np.arange(n)+.5)*res;xx,yy=np.meshgrid(coords,coords)
    free[xx*xx+yy*yy<radius**2]=1
    grid=np.full((n,n),-1,np.int16);grid[free!=0]=0;grid[occupied!=0]=100
    return grid


@pytest.mark.parametrize('paired',[True,False])
@pytest.mark.parametrize('state',[np.zeros(3),np.array([.4,-.2,.7])])
def test_batched_raster_matches_scalar_for_overlap_disparity_and_empty_pairs(paired,state):
    cfg=replace(GapConfig(),grid_resolution_m=.04,grid_extent_m=4.)
    n=200;radius=.2
    unknown=DistanceField.from_occupancy(np.full((n,n),-1),GridMetadata(n,n,.04,-4,-4,0))
    scans=[]
    for sensor in range(2):
        angles=np.linspace(-np.pi,np.pi,301)+sensor*.12
        ranges=2.+.1*np.sin(angles*3)
        ranges[60:90]=.01 # Very near returns: nonnegative free-depth clamp.
        ranges[120:160]=7. # Far returns: capped polygon depth, clipped hits.
        ranges[200:220]=ranges[199]+.25 # Boundary at the continuity threshold.
        valid=np.ones(len(ranges),bool)
        valid[5:15]=False
        if not paired:valid[1::2]=False
        origins=np.column_stack((np.linspace(.1,.2,len(ranges)),np.full(len(ranges),.1*sensor)))
        hits=origins+ranges[:,None]*np.column_stack((np.cos(angles),np.sin(angles)))
        scans.append(GapScan(ranges,valid,transform_points(origins,state),transform_points(hits,state)))
    expected=scalar_scan_grid(scans,state,cfg,radius)
    actual,_,_=fuse_scans(tuple(scans),state,unknown,None,cfg,radius)
    np.testing.assert_array_equal(actual.occupancy,expected)
    reference=DistanceField.from_occupancy(expected,GridMetadata(n,n,.04,-4,-4,0))
    np.testing.assert_array_equal(actual.clearance,reference.clearance)


@pytest.mark.parametrize('n',[1,3,17])
def test_batched_jacobian_matches_actual_model_at_different_horizons(n):
    x=np.random.default_rng(90+n).normal(0,.2,3*n);dt=.13;eps=1e-6
    state,jac=rollout_jacobian(x,n,dt)
    np.testing.assert_allclose(state,rollout(np.zeros(3),x.reshape(n,3),dt),atol=1e-12)
    numeric=np.stack([(rollout(np.zeros(3),(x+eps*e).reshape(n,3),dt)
                       -rollout(np.zeros(3),(x-eps*e).reshape(n,3),dt))/(2*eps)for e in np.eye(3*n)],axis=-1)
    np.testing.assert_allclose(jac,numeric,atol=2e-8)
