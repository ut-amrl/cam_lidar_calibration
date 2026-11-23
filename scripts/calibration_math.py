"""Offline target/ground solvers; meters, radians, T_destination_source.

Plane registration follows the iterative target-plane approach also used in
memo/datagen/calib/calibrate.py. No installed memo package is required.
"""
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def transform(points, matrix):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def board_points(columns, rows, square):
    xy = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square
    return np.c_[xy, np.zeros(len(xy))].astype(np.float32)


def plane_fit(points, threshold=0.02, up=None, iterations=250):
    if len(points) < 30:
        raise ValueError('Fewer than 30 points available for a plane')
    rng = np.random.default_rng(17)
    best = None
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b-a, c-a)
        if np.linalg.norm(n) < 1e-8:
            continue
        n /= np.linalg.norm(n)
        d = -n @ a
        if up is not None:
            if n @ up < 0:
                n, d = -n, -d
            if n @ up < np.cos(np.deg2rad(25)) or d < 0.2:
                continue
        mask = np.abs(points @ n + d) < threshold
        if best is None or mask.sum() > best.sum():
            best = mask
    if best is None or best.sum() < 30:
        raise ValueError('No supported plane found')
    center = points[best].mean(axis=0)
    _, _, v = np.linalg.svd(points[best]-center, full_matrices=False)
    n = v[-1]
    if up is not None and n @ up < 0:
        n = -n
    d = -n @ center
    mask = np.abs(points @ n+d) < threshold
    return n, float(d), mask


def ground_transform(normal, height):
    """Ground parent, lidar child; lidar origin is exactly (0,0,height)."""
    n = normal / np.linalg.norm(normal)
    forward = np.array([1., 0., 0.]) - n[0] * n
    forward /= np.linalg.norm(forward)
    matrix = np.eye(4)
    matrix[:3, :3] = np.vstack([forward, np.cross(n, forward), n])
    matrix[:3, 3] = [0., 0., height]
    return matrix


def fit_ground(scans, threshold=.025):
    fits = []
    for cloud in scans:
        p = cloud[np.isfinite(cloud).all(axis=1)]
        radius = np.linalg.norm(p[:, :2], axis=1)
        p = p[(radius > 1.) & (radius < 10.) & (p[:, 2] < -.2) & (p[:, 2] > -2.5)]
        if len(p) > 5000:
            p = p[np.random.default_rng(0).choice(len(p), 5000, replace=False)]
        try:
            n, d, mask = plane_fit(p, threshold, up=np.array([0., 0., 1.]))
        except ValueError:
            continue
        # In indoor VLP-16 scans the floor can be <30% of all downward returns
        # because furniture/walls occupy the rest. Require spatial support too.
        extent=np.ptp(p[mask,:2],axis=0)
        if mask.sum() >= 100 and mask.mean() >= .15 and min(extent) >= 1.0:
            fits.append((n, d, float(np.sqrt(np.mean((p[mask] @ n+d)**2)))))
    if len(fits) < 5:
        raise ValueError(f'Ground plane has only {len(fits)} supported scans; need five')
    normals = np.array([f[0] for f in fits]); heights = np.array([f[1] for f in fits])
    n = np.median(normals, axis=0); n /= np.linalg.norm(n)
    angles = np.rad2deg(np.arccos(np.clip(normals @ n, -1, 1)))
    keep = (np.abs(heights-np.median(heights)) < .05) & (angles < 3.)
    if keep.sum() < max(5, .7*len(fits)):
        raise ValueError('Ground estimates vary too much; record a stationary rigid rig on a flat floor')
    n = np.median(normals[keep], axis=0); n /= np.linalg.norm(n)
    d = float(np.median(heights[keep]))
    checks = dict(scans_used=int(keep.sum()), height_m=d, height_std_m=float(heights[keep].std()),
                  normal_in_lidar=n.tolist(), plane_equation='normal dot p_lidar + height = 0',
                  median_plane_rms_m=float(np.median([f[2] for f,k in zip(fits,keep) if k])),
                  yaw='fixed by projected lidar +X; absolute heading is unobservable', xy='LiDAR origin fixed to x=0,y=0')
    return ground_transform(n,d), checks


def reprojection_errors(obj, detections, k, distortion):
    values = []
    for det in detections:
        ok, rv, tv = cv2.solvePnP(obj, det['corners'], k, distortion)
        if not ok:
            values.append(float('inf')); continue
        uv = cv2.projectPoints(obj, rv, tv, k, distortion)[0].reshape(-1,2)
        values.append(float(np.sqrt(np.mean(np.sum((uv-det['corners'])**2, axis=1)))))
    return np.array(values)


def intrinsics(obj, detections, size):
    if len(detections) < 15:
        raise ValueError(f'Need 15 checkerboard views, found {len(detections)}')
    # Keep the final third of each repeating five-view group held out. Nearby
    # frames are sampled sparsely; these are held-out views, not a new session.
    train = [v for i,v in enumerate(detections) if i%5 not in (3,4)]
    test = [v for i,v in enumerate(detections) if i%5 in (3,4)]
    def fit(views):
        _, k, d, _, _ = cv2.calibrateCamera([obj]*len(views),
            [v['corners'].astype(np.float32) for v in views], size, None, None)
        if not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[0,0], k[1,1])<=0:
            raise ValueError('Camera calibration produced invalid parameters')
        return k, d.ravel()
    kh, dh = fit(train)
    heldout = reprojection_errors(obj,test,kh,dh)
    # Trim large corner-detection outliers, then refit all remaining views.
    k,d=fit(detections); errors=reprojection_errors(obj,detections,k,d)
    limit=max(1.,float(np.median(errors)*3))
    keep=errors<limit
    if keep.sum()<15:
        raise ValueError('Too few views remain after corner-error rejection')
    k,d=fit([v for v,q in zip(detections,keep) if q])
    points=np.concatenate([v['corners'] for v in detections])
    cells=np.clip((points/np.array(size)*8).astype(int),0,7)
    coverage=len(np.unique(cells,axis=0))/64
    return k,d,dict(views=len(detections),fit_views=int(keep.sum()), heldout_views=len(test),
        heldout_rms_px=float(np.sqrt(np.mean(heldout**2))),coverage_fraction=coverage,
        per_view_rms_px=reprojection_errors(obj,detections,k,d).tolist()),test


def make_planes(obj, detections, k, distortion, columns, rows, square):
    outline=np.array([[-1,-1,0],[columns,-1,0],[columns,rows,0],[-1,rows,0]],float)*square
    planes=[]
    for v in detections:
        if 'cloud' not in v: continue
        ok,rv,tv=cv2.solvePnP(obj,v['corners'],k,distortion)
        if not ok: continue
        r=cv2.Rodrigues(rv)[0];n=r[:,2]
        planes.append(dict(view=v,n=n,d=float(n@tv.ravel()),outline=outline@r.T+tv.ravel()))
    return planes


def select_board(plane,matrix,thickness,margin=.02):
    source=plane['view']['cloud'];camera=transform(source,matrix)
    corners=plane['outline'];ex=corners[1]-corners[0];ey=corners[3]-corners[0]
    lx,ly=np.linalg.norm(ex),np.linalg.norm(ey)
    rel=camera-corners[0];u=rel@ex/lx;v=rel@ey/ly
    mask=(np.abs(camera@plane['n']-plane['d'])<thickness)&(u>margin)&(u<lx-margin)&(v>margin)&(v<ly-margin)
    return source[mask]


def solve_target(planes,initial,min_points=20):
    """Iteratively resegment within each image-derived target outline, robust plane fit."""
    matrix=initial.copy();segments=[];solution=None
    for thickness in (.35,.18,.08,.04,.025):
        segments=[]
        for p in planes:
            points=select_board(p,matrix,thickness)
            if len(points)<min_points:continue
            if len(points)>250:points=points[np.linspace(0,len(points)-1,250).astype(int)]
            # At tight passes reject planes whose point support has no 2D extent.
            if thickness<=.08:
                _,s,_=np.linalg.svd(points-points.mean(axis=0),full_matrices=False)
                if s[1]/np.sqrt(len(points))<.025:continue
            segments.append((p,points))
        if len(segments)<8:raise ValueError(f'Only {len(segments)} target planes have enough LiDAR points')
        normals=np.array([p['n'] for p,_ in segments]);singular=np.linalg.svd(normals,compute_uv=False)
        if singular[-1]/singular[0]<.05:raise ValueError('Insufficient board pitch/yaw diversity')
        def residual(x):
            r=Rotation.from_rotvec(x[:3]).as_matrix()
            # Equal total weight per board instead of per-return density.
            return np.concatenate([((pts@r.T+x[3:])@p['n']-p['d'])/np.sqrt(len(pts))
                                   for p,pts in segments])
        x0=np.r_[Rotation.from_matrix(matrix[:3,:3]).as_rotvec(),matrix[:3,3]]
        solution=least_squares(residual,x0,loss='soft_l1',f_scale=.004,max_nfev=150)
        if not solution.success:raise ValueError('Target-plane optimization did not converge')
        matrix[:3,:3]=Rotation.from_rotvec(solution.x[:3]).as_matrix();matrix[:3,3]=solution.x[3:]
    errors=np.array([np.sqrt(np.mean((transform(pts,matrix)@p['n']-p['d'])**2)) for p,pts in segments])
    sv=np.linalg.svd(solution.jac,compute_uv=False)
    if sv[-1]/sv[0]<.005:raise ValueError('Target registration is poorly constrained')
    if np.median(errors)>.025:raise ValueError('Target-plane residual is too high')
    return matrix,dict(boards_used=len(segments),points_used=sum(len(v) for _,v in segments),
        median_plane_rms_m=float(np.median(errors)),max_plane_rms_m=float(errors.max()),
        jacobian_singular_ratio=float(sv[-1]/sv[0]),normal_singular_ratio=float(singular[-1]/singular[0])),segments


def transform_record(matrix,parent,child):
    return dict(parent_frame=parent,child_frame=child,convention='p_parent = T_parent_child @ p_child',
        translation=dict(zip(('x','y','z'),matrix[:3,3].tolist())),
        rotation=dict(zip(('x','y','z','w'),Rotation.from_matrix(matrix[:3,:3]).as_quat().tolist())),
        rpy_rad=Rotation.from_matrix(matrix[:3,:3]).as_euler('xyz').tolist(),matrix=matrix.tolist())
