#!/usr/bin/env python3
"""Calibrate camera intrinsics, camera/LiDAR, and LiDAR/ground from one ROS 2 bag."""
import argparse
import csv
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from tf2_msgs.msg import TFMessage
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml

# Allow both ros2-run installation and direct invocation.
sys.path.insert(0,str(Path(__file__).resolve().parent))
from calibration_math import (board_points, fit_ground, intrinsics, make_planes, reprojection_errors,
                              solve_target, transform, transform_record, robot_transforms)


def log(message):
    print(message,flush=True)


def default_output(bag):
    # Find the workspace from the source checkout, including symlink installs,
    # so invoking the CLI from another directory keeps results in memo's data/.
    workspace=next((p for p in Path(__file__).resolve().parents
                    if (p/'src/cam_lidar_calibration').is_dir()),Path.cwd())
    return workspace/'data/calibration/results'/bag.resolve().name


def stamp(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


def cloud_xyz(message):
    fields={f.name:f for f in message.fields}
    if not {'x','y','z'}<=set(fields):raise ValueError('PointCloud2 needs x/y/z fields')
    formats={7:'f4',8:'f8'};byteorder='>' if message.is_bigendian else '<'
    dtype=np.dtype({'names':['x','y','z'],'formats':[byteorder+formats[fields[k].datatype] for k in ('x','y','z')],
                    'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':message.point_step})
    rows=np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                    strides=(message.row_step,message.point_step))
    result=np.column_stack([rows[k].ravel() for k in ('x','y','z')]).astype(np.float64)
    return result[np.isfinite(result).all(axis=1)&(np.linalg.norm(result,axis=1)>.3)]


def decode(message):
    if isinstance(message,CompressedImage):
        result=cv2.imdecode(np.frombuffer(message.data,np.uint8),cv2.IMREAD_COLOR)
        if result is None:raise ValueError('Could not decode JPEG/PNG')
        return result
    channels={'bgr8':3,'rgb8':3,'mono8':1,'bgra8':4,'rgba8':4}
    if message.encoding not in channels:raise ValueError(f'Unsupported image encoding {message.encoding}')
    count=channels[message.encoding]
    array=np.ndarray((message.height,message.width,count),np.uint8,buffer=message.data,
                     strides=(message.step,count,1)).copy()
    conversions={'rgb8':cv2.COLOR_RGB2BGR,'mono8':cv2.COLOR_GRAY2BGR,
                 'bgra8':cv2.COLOR_BGRA2BGR,'rgba8':cv2.COLOR_RGBA2BGR}
    return cv2.cvtColor(array.squeeze() if count==1 else array,conversions[message.encoding]) if message.encoding!='bgr8' else array


def reader(bag,topics):
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=metadata['storage_identifier']),
           rosbag2_py.ConverterOptions('cdr','cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r,metadata


def lookup_transform(edges,destination,source):
    queue=[(source,np.eye(4))];seen=set()
    while queue:
        frame,chain=queue.pop(0)
        if frame==destination:return chain
        if frame in seen:continue
        seen.add(frame)
        for parent,child,matrix in edges:
            if child==frame:queue.append((parent,matrix@chain))
            if parent==frame:queue.append((child,np.linalg.inv(matrix)@chain))
    return None


def read_data(args):
    r,meta=reader(args.bag,[args.image_topic,args.info_topic,'/tf_static'])
    topic_types={v['topic_metadata']['name']:v['topic_metadata']['type'] for v in meta['topics_with_message_count']}
    for topic in (args.image_topic,args.points_topic):
        if topic not in topic_types:raise ValueError(f'Topic absent from bag: {topic}')
    if topic_types[args.image_topic] not in ('sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage'):
        raise ValueError('Image topic must contain Image or CompressedImage')
    typ=CompressedImage if topic_types[args.image_topic].endswith('/CompressedImage') else Image
    start=meta['starting_time']['nanoseconds_since_epoch']*1e-9
    duration=meta['duration']['nanoseconds']*1e-9
    detections=[];info=None;size=None;last=-float('inf');edges=[];camera_frame=None;sampled=0
    obj=board_points(args.columns-1,args.rows-1,args.square_mm/1000)
    while r.has_next():
        topic,raw,recorded=r.read_next()
        if topic=='/tf_static':
            for tf in deserialize_message(raw,TFMessage).transforms:
                q=tf.transform.rotation;t=tf.transform.translation;m=np.eye(4)
                m[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();m[:3,3]=[t.x,t.y,t.z]
                edges.append((tf.header.frame_id,tf.child_frame_id,m))
            continue
        if topic==args.info_topic:
            if info is None:info=deserialize_message(raw,CameraInfo)
            continue
        elapsed=recorded*1e-9-start
        if elapsed-last<args.sample_seconds:continue
        last=elapsed;sampled+=1
        message=deserialize_message(raw,typ);im=decode(message)
        if size is None:size=(im.shape[1],im.shape[0]);camera_frame=message.header.frame_id
        if size!=(im.shape[1],im.shape[0]) or camera_frame!=message.header.frame_id:
            raise ValueError('Image resolution/frame changed within recording')
        found,corners=cv2.findChessboardCornersSB(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),
                        (args.columns-1,args.rows-1),cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:detections.append(dict(t=stamp(message),elapsed=elapsed,corners=corners.reshape(-1,2),image=im))
        if sampled%25==0:log(f'  sampled {sampled} images; detected {len(detections)} boards')
    if info is not None and size!=(info.width,info.height):raise ValueError('CameraInfo/image resolutions differ')
    if len(detections)<15:raise ValueError(f'Only {len(detections)} board detections; need at least 15')
    r,_=reader(args.bag,[args.points_topic]);scans=[];ground_scans=[];background=None;lidar_frame=None;last_ground=-99
    background_time=args.background_start if args.background_start is not None else max(0,duration-3)
    targets=np.array([d['t'] for d in detections])
    # Decode only scans near selected images, spread ground scans, and background.
    while r.has_next():
        _,raw,recorded=r.read_next();message=deserialize_message(raw,PointCloud2)
        elapsed=recorded*1e-9-start;t=stamp(message)
        closest=float(np.min(abs(targets-t)))
        for_ground=elapsed-last_ground>=max(duration/25,1)
        for_background=background is None and elapsed>=background_time
        if closest>args.max_pair_dt and not for_ground and not for_background:continue
        p=cloud_xyz(message)
        if lidar_frame is None:lidar_frame=message.header.frame_id
        if lidar_frame!=message.header.frame_id:raise ValueError('LiDAR frame changed within bag')
        if for_ground:ground_scans.append(p);last_ground=elapsed
        if for_background:background=p
        if closest<=args.max_pair_dt:scans.append((t,p))
    if not scans:raise ValueError('No images have LiDAR scans within timestamp tolerance')
    scan_t=np.array([v[0] for v in scans])
    for det in detections:
        j=int(np.argmin(abs(scan_t-det['t'])))
        if abs(scan_t[j]-det['t'])<=args.max_pair_dt:
            det['cloud']=scans[j][1];det['pair_dt']=float(scan_t[j]-det['t'])
    prior=lookup_transform(edges,camera_frame,lidar_frame)
    return obj,detections,info,size,ground_scans,background,prior,dict(
        duration_s=duration,sampled_images=sampled,board_detections=len(detections),
        paired_detections=sum('cloud'in d for d in detections),background_start_s=background_time,
        camera_frame=camera_frame,lidar_frame=lidar_frame)


def save_yaml(path,data):
    path.write_text(yaml.safe_dump(data,sort_keys=False))


def camera_yaml(k,d,size,frame,model):
    return dict(image_width=size[0],image_height=size[1],camera_name=frame,distortion_model=model,
        camera_matrix=dict(rows=3,cols=3,data=k.ravel().tolist()),
        distortion_coefficients=dict(rows=1,cols=len(d),data=np.asarray(d).ravel().tolist()),
        rectification_matrix=dict(rows=3,cols=3,data=np.eye(3).ravel().tolist()),
        projection_matrix=dict(rows=3,cols=4,data=np.c_[k,np.zeros(3)].ravel().tolist()))


def select_intrinsics(obj,views,info,size,frame,*,recompute=False,max_error=1.,output=None):
    """Reuse recorded CameraInfo exactly unless refitting is requested or absent."""
    recorded=None
    if info is not None:
        k=np.array(info.k,dtype=float).reshape(3,3);d=np.array(info.d,dtype=float)
        if (min(k[0,0],k[1,1])<=0 or not np.isfinite(k).all() or not np.isfinite(d).all()
            or info.distortion_model not in ('plumb_bob','rational_polynomial')):
            raise ValueError('Recorded CameraInfo is invalid or uses an unsupported camera model')
        if info.header.frame_id and info.header.frame_id!=frame:
            raise ValueError('CameraInfo and image use different frames')
        recorded=camera_yaml(k,d,size,frame,info.distortion_model)
        recorded['rectification_matrix']['data']=[float(v)for v in info.r]
        recorded['projection_matrix']['data']=[float(v)for v in info.p]
        if output is not None:save_yaml(output/'camera_intrinsics_recorded.yaml',recorded)
    if recorded is not None and not recompute:
        errors=reprojection_errors(obj,views,k,d)
        validation=float(np.sqrt(np.mean(errors**2)))
        checks=dict(selected='recorded_CameraInfo',recomputed=False,views=len(views),
                    validation_rms_px=validation,heldout_rms_px=None)
        active=recorded
    else:
        k,d,checks,test=intrinsics(obj,views,size)
        validation=checks['heldout_rms_px']
        checks.update(selected='new_fit',recomputed=True,validation_rms_px=validation,
                      reason='explicit --recompute-intrinsics' if recompute else 'CameraInfo not recorded')
        active=camera_yaml(k,d,size,frame,'plumb_bob')
        if recorded is not None:
            errors=reprojection_errors(obj,test,np.array(info.k).reshape(3,3),np.array(info.d))
            checks['recorded_heldout_rms_px']=float(np.sqrt(np.mean(errors**2)))
        if output is not None:save_yaml(output/'camera_intrinsics_candidate.yaml',active)
    checks['status']='passed' if np.isfinite(validation) and validation<max_error else 'failed'
    return k,d,active,checks


def debug_intrinsics(out,obj,views,k,d,checks):
    (out/'debug/intrinsics').mkdir(parents=True,exist_ok=True)
    rms_values=[]
    with (out/'intrinsics_errors.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['elapsed_s','corner','observed_u','observed_v','predicted_u','predicted_v','error_px'])
        for index,v in enumerate(views):
            _,rv,tv=cv2.solvePnP(obj,v['corners'],k,d)
            predicted=cv2.projectPoints(obj,rv,tv,k,d)[0].reshape(-1,2)
            errors=np.linalg.norm(predicted-v['corners'],axis=1)
            rms_values.append(float(np.sqrt(np.mean(errors**2))))
            for j,(a,b,e) in enumerate(zip(v['corners'],predicted,errors)):
                writer.writerow([v['elapsed'],j,*a,*b,e])
            if index%max(1,len(views)//12):continue
            im=v['image'].copy()
            for a,b in zip(v['corners'],predicted):
                a=tuple(np.rint(a).astype(int));b=tuple(np.rint(b).astype(int))
                cv2.circle(im,a,3,(0,255,0),-1);cv2.circle(im,b,4,(0,0,255),1);cv2.line(im,a,b,(0,255,255),1)
            cv2.putText(im,f'{v["elapsed"]:.1f}s RMS {np.sqrt(np.mean(errors**2)):.3f}px',(15,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
            cv2.imwrite(str(out/f'debug/intrinsics/{v["elapsed"]:07.2f}s.jpg'),im)
    chart=np.full((480,1000,3),255,np.uint8)
    top=max(.5,max(rms_values)*1.15)
    for tick in np.linspace(0,top,6):
        y=int(410-tick/top*330)
        cv2.line(chart,(80,y),(960,y),(220,220,220),1)
        cv2.putText(chart,f'{tick:.2f}',(10,y+5),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),1)
    xy=np.array([[80+i/max(1,len(rms_values)-1)*880,410-e/top*330] for i,e in enumerate(rms_values)],np.int32)
    cv2.polylines(chart,[xy],False,(200,60,20),2)
    cv2.putText(chart,'Intrinsic reprojection RMS per sampled board view (pixels)',(70,35),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,0),2)
    cv2.putText(chart,f'Selected: {checks["selected"]}; validation RMS {checks["validation_rms_px"]:.3f}px',(70,65),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),1)
    cv2.putText(chart,'Sampled board view (chronological)',(300,455),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,0),1)
    cv2.imwrite(str(out/'debug/intrinsics_error.png'),chart)


def debug_projection(out,planes,matrix,k,d):
    (out/'debug/lidar_projection').mkdir(parents=True,exist_ok=True)
    for p in planes[::max(1,len(planes)//15)]:
        view=p['view'];source=view['cloud'];camera=transform(source,matrix)
        keep=(camera[:,2]>.3)&(np.linalg.norm(source,axis=1)<20)
        camera=camera[keep];depth=np.linalg.norm(source[keep],axis=1)
        uv=cv2.projectPoints(camera,np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2)
        im=view['image'].copy();h,w=im.shape[:2]
        mask=np.isfinite(uv).all(axis=1)&(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
        colors=cv2.applyColorMap(np.clip(depth[mask]/10*255,0,255).astype(np.uint8),cv2.COLORMAP_TURBO).reshape(-1,3)
        for point,color in zip(uv[mask],colors):cv2.circle(im,tuple(np.rint(point).astype(int)),1,tuple(int(v)for v in color),-1)
        cv2.putText(im,f'{view["elapsed"]:.1f}s lidar overlay dt={view["pair_dt"]*1000:.1f}ms',(15,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
        cv2.imwrite(str(out/f'debug/lidar_projection/{view["elapsed"]:07.2f}s.jpg'),im)


def run(args):
    args.bag=args.bag.resolve();out=args.output.resolve()
    if not (args.bag/'metadata.yaml').is_file():raise ValueError('Expected a ROS 2 bag directory containing metadata.yaml')
    if out.exists() and any(out.iterdir()):raise ValueError('Output directory is not empty; select a fresh --output')
    out.mkdir(parents=True,exist_ok=True)
    report=dict(bag=str(args.bag),board=dict(columns=args.columns,rows=args.rows,square_mm=args.square_mm),stages={})
    transforms=[];bundle=dict(schema_version=1,transforms=transforms)
    log('Reading calibration topics and detecting checkerboards...')
    obj,views,info,size,ground_scans,background,prior,metadata=read_data(args)
    report['input']=metadata
    log(f'Found {len(views)} boards; {metadata["paired_detections"]} paired with LiDAR')
    k,d,active,checks=select_intrinsics(obj,views,info,size,metadata['camera_frame'],
        recompute=args.recompute_intrinsics,max_error=args.max_intrinsics_error,output=out)
    report['stages']['intrinsics']=checks
    save_yaml(out/'camera_intrinsics.yaml',active);bundle['camera_info']=active;bundle['camera_frame']=metadata['camera_frame']
    debug_intrinsics(out,obj,views,k,d,checks)
    try:
        ground,ground_checks=fit_ground(ground_scans)
        record=transform_record(ground,args.ground_frame,metadata['lidar_frame'])
        save_yaml(out/'lidar_ground.yaml',dict(transform=record,checks=ground_checks));transforms.append(record)
        report['stages']['ground']=dict(status='passed',**ground_checks)
    except ValueError as error:report['stages']['ground']=dict(status='failed',reason=str(error))
    try:
        if args.initial_extrinsic:
            initial=yaml.safe_load(args.initial_extrinsic.read_text());initial=np.array(initial.get('matrix',initial) if isinstance(initial,dict) else initial).reshape(4,4)
            prior_source='CLI T_camera_lidar'
        elif prior is not None:initial=prior;prior_source='recorded /tf_static chain'
        else:
            # Conventional forward-facing optical camera, approximately 25 cm
            # below a roof-mounted lidar. This is an initializer, never a result.
            initial=np.array([[0.,-1.,0.,0.],[0.,0.,-1.,-.25],[1.,0.,0.,0.],[0.,0.,0.,1.]])
            prior_source='forward-facing optical-axis guess; t=[0,-0.25,0]m'
        if not np.isfinite(initial).all() or not np.allclose(initial[3],[0,0,0,1]):raise ValueError('Invalid initial extrinsic matrix')
        report['extrinsic_initialization']=dict(source=prior_source,T_camera_lidar=initial.tolist())
        planes=make_planes(obj,views,k,d,args.columns-1,args.rows-1,args.square_mm/1000)
        # Ignore the ground-board segment used for background; leave final 20%
        # of the remaining poses held out of the extrinsic fit.
        planes=[p for p in planes if p['view']['elapsed']<metadata['background_start_s']]
        training=[p for i,p in enumerate(planes) if i%5!=4];heldout=[p for i,p in enumerate(planes) if i%5==4]
        if background is not None:
            tree=cKDTree(background)
            for p in planes:
                cloud=p['view']['cloud'];distance,_=tree.query(cloud,workers=1)
                p['view']['full_cloud']=cloud;p['view']['cloud']=cloud[distance>.035]
        best=None;failures=[];solutions=[]
        for dz in (0.,-.15,.15):
            seed=initial.copy();seed[2,3]+=dz
            try:
                solved,quality,segments=solve_target(training,seed)
                solutions.append(solved)
                if best is None or quality['median_plane_rms_m']<best[1]['median_plane_rms_m']:best=(solved,quality,segments)
            except ValueError as error:failures.append(str(error))
        if best is None:raise ValueError('; '.join(failures))
        matrix,quality,segments=best
        quality['successful_restarts']=len(solutions)
        quality['restart_max_rotation_deg']=float(max(np.rad2deg(Rotation.from_matrix(v[:3,:3]@matrix[:3,:3].T).magnitude()) for v in solutions))
        quality['restart_max_translation_m']=float(max(np.linalg.norm(v[:3,3]-matrix[:3,3]) for v in solutions))
        if quality['restart_max_rotation_deg']>3 or quality['restart_max_translation_m']>.1:
            raise ValueError('Different initializations disagree; inspect target association or supply a better initial extrinsic')
        from calibration_math import select_board
        # Hold-out board planes are excluded from optimization; association still
        # uses the final estimate, so this is a consistency check, not ground truth.
        hold=[]
        for p in heldout:
            pts=select_board(p,matrix,.06)
            if len(pts)>=20:hold.append(float(np.sqrt(np.mean((transform(pts,matrix)@p['n']-p['d'])**2))))
        if len(hold)<3:raise ValueError('Fewer than three held-out LiDAR board planes have support')
        quality['heldout_boards']=len(hold);quality['heldout_median_plane_rms_m']=float(np.median(hold))
        if np.median(hold)>.03:raise ValueError('Held-out target-plane residual exceeds 30 mm')
        record=transform_record(np.linalg.inv(matrix),metadata['lidar_frame'],metadata['camera_frame'])
        save_yaml(out/'lidar_camera.yaml',dict(transform=record,T_camera_lidar=matrix.tolist(),checks=quality));transforms.append(record)
        report['stages']['lidar_camera']=dict(status='passed',**quality)
        for p in planes:
            if 'full_cloud'in p['view']:p['view']['cloud']=p['view']['full_cloud']
        debug_projection(out,planes,matrix,k,d)
    except ValueError as error:
        report['stages']['lidar_camera']=dict(status='failed',reason=str(error))
        # Explicitly labeled initializer overlays help diagnose failed alignment.
        if 'planes'in locals():debug_projection(out/'debug/initial_guess_only',planes,initial,k,d)
    report['status']='passed' if all(s['status']=='passed' for s in report['stages'].values()) else 'failed'
    if report['status']=='passed':
        payload=robot_transforms(ground,np.linalg.inv(matrix),args.body_height_m,
            body_frame=args.body_frame,lidar_frame=metadata['lidar_frame'],camera_frame=metadata['camera_frame'],
            ground_frame=args.ground_frame,footprint_frame=args.footprint_frame)
        save_yaml(out/'robot_transforms.yaml',payload)
    save_yaml(out/'calibration.yaml',bundle);save_yaml(out/'report.yaml',report)
    log('\n'+yaml.safe_dump(dict(status=report['status'],output=str(out),camera_intrinsics=active,transforms=transforms,checks=report['stages']),sort_keys=False))
    log(f'Calibration {report["status"]}. Results saved to: {out}')
    return 0 if report['status']=='passed' else 2


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag',type=Path);p.add_argument('--output',type=Path,
        help='Default: workspace/data/calibration/results/<bag-directory-name>')
    p.add_argument('--recompute-intrinsics','--recompute_intrinsics',action='store_true',
        help='Fit and use new intrinsics even when CameraInfo is recorded (default: reuse recorded CameraInfo)')
    p.add_argument('--body-height-m',type=float,default=.5123518455439847,help='Body-frame height during capture; used for the directly-above LiDAR mount')
    p.add_argument('--body-frame',default='body');p.add_argument('--footprint-frame',default='base_footprint')
    p.add_argument('--image-topic',default='/camera/rgb/image_raw/compressed');p.add_argument('--info-topic',default='/camera/rgb/camera_info')
    p.add_argument('--points-topic',default='/velodyne_points');p.add_argument('--columns',type=int,default=9);p.add_argument('--rows',type=int,default=6)
    p.add_argument('--square-mm',type=float,default=83.63);p.add_argument('--sample-seconds',type=float,default=2.)
    p.add_argument('--max-pair-dt',type=float,default=.06);p.add_argument('--background-start',type=float,help='Default: three seconds before end of bag')
    p.add_argument('--initial-extrinsic',type=Path,help='YAML 4x4 T_camera_lidar matrix for nonstandard mounts')
    p.add_argument('--ground-frame',default='calibration_ground');p.add_argument('--max-intrinsics-error',type=float,default=1.)
    args=p.parse_args()
    args.output=(args.output if args.output is not None else default_output(args.bag)).resolve()
    if min(args.rows,args.columns)<3 or args.square_mm<=0 or args.sample_seconds<=0 or args.max_pair_dt<=0:p.error('Invalid board/sampling parameters')
    if args.output.exists() and any(args.output.iterdir()):
        print('Calibration failed: output directory is not empty; select a fresh --output',file=sys.stderr)
        return 2
    try:return run(args)
    except (ValueError,RuntimeError,OSError,cv2.error) as error:
        args.output.mkdir(parents=True,exist_ok=True)
        # Do not overwrite a previous result when input validation rejects reuse.
        failure=args.output/'failure.yaml'
        if not failure.exists():save_yaml(failure,dict(status='failed',reason=str(error)))
        print(f'Calibration failed: {error}',file=sys.stderr);return 2


if __name__=='__main__':sys.exit(main())
