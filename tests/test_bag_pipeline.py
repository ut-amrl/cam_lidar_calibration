"""Synthetic known-truth and ROS message tests for the offline bag pipeline."""
import sys
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
import calibration_math as math


class SolversTest(unittest.TestCase):
    def test_ground_origin_and_plane(self):
        n=np.array([.04,-.03,1.]);n/=np.linalg.norm(n)
        matrix=math.ground_transform(n,.87)
        np.testing.assert_allclose(matrix[:3,3],[0,0,.87])
        points=np.array([[1.,2.,0.],[-2.,1.,0.],[3.,-2.,0.]])
        points[:,2]=(-.87-points[:,:2]@n[:2])/n[2]
        np.testing.assert_allclose(math.transform(points,matrix)[:,2],0,atol=1e-12)
        np.testing.assert_allclose(matrix[:3,:3]@n,[0,0,1],atol=1e-12)

    def test_ground_fit_with_clutter(self):
        rng=np.random.default_rng(8);scans=[]
        for _ in range(6):
            xy=rng.uniform([-6,-5],[6,5],(800,2))
            floor=np.c_[xy,np.full(800,-.87)]+rng.normal(0,.003,(800,3))
            clutter=rng.uniform([-6,-5,-.6],[6,5,1],(1600,3))
            scans.append(np.r_[floor,clutter])
        matrix,checks=math.fit_ground(scans)
        self.assertAlmostEqual(matrix[2,3],.87,delta=.008)
        self.assertGreaterEqual(checks['scans_used'],5)

    def test_target_transform_recovery(self):
        rng=np.random.default_rng(4)
        true=np.eye(4);true[:3,:3]=np.array([[0,-1,0],[0,0,-1],[1,0,0]])
        true[:3,3]=[.03,-.23,-.4]
        inverse=np.linalg.inv(true);obj=math.board_points(8,5,.08363)
        k=np.array([[610.,0,640],[0,610,360],[0,0,1]])
        views=[]
        for i in range(22):
            r=Rotation.from_euler('xyz',rng.uniform(-.5,.5,3)).as_matrix()
            t=np.array([rng.uniform(-.5,.2),rng.uniform(-.3,.1),rng.uniform(1.4,3)])
            corners=cv2.projectPoints(obj,cv2.Rodrigues(r)[0],t,k,np.zeros(5))[0].reshape(-1,2)
            xy=rng.uniform([-.06,-.06],[.62,.36],(180,2))
            camera=np.c_[xy,np.zeros(len(xy))]@r.T+t+rng.normal(0,.001,(len(xy),3))
            lidar=math.transform(camera,inverse)
            clutter=rng.uniform([-4,-4,-1],[6,4,2],(200,3))
            views.append(dict(corners=corners,cloud=np.r_[lidar,clutter]))
        planes=math.make_planes(obj,views,k,np.zeros(5),8,5,.08363)
        guess=true.copy();guess[:3,3]+=[.02,-.04,.06]
        solved,checks,_=math.solve_target(planes,guess)
        error=Rotation.from_matrix(solved[:3,:3]@true[:3,:3].T).magnitude()
        self.assertLess(error,np.deg2rad(.3))
        self.assertLess(np.linalg.norm(solved[:3,3]-true[:3,3]),.01)
        self.assertLess(checks['median_plane_rms_m'],.004)
        # Identical planes cannot constrain all six extrinsic DOFs.
        with self.assertRaisesRegex(ValueError,'diversity'):
            math.solve_target([planes[0]]*10,guess)

    def test_camera_intrinsics_and_heldout(self):
        rng=np.random.default_rng(42);obj=math.board_points(8,5,.08363)
        k=np.array([[610.,0,640],[0,605,360],[0,0,1]])
        views=[]
        for _ in range(25):
            rv=rng.uniform(-.5,.5,3);tv=np.array([rng.uniform(-.6,.1),rng.uniform(-.4,.1),rng.uniform(1,2.5)])
            corners=cv2.projectPoints(obj,rv,tv,k,np.zeros(5))[0].reshape(-1,2)
            views.append(dict(corners=corners))
        fitted,d,checks,test=math.intrinsics(obj,views,(1280,720))
        np.testing.assert_allclose(fitted,k,atol=.1)
        self.assertGreater(len(test),0)
        self.assertLess(checks['heldout_rms_px'],.01)

    def test_padded_cloud_rows(self):
        import calibrate_bag as bag
        from sensor_msgs.msg import PointCloud2,PointField
        cloud=PointCloud2(height=2,width=2,point_step=12,row_step=32,is_bigendian=False)
        cloud.fields=[PointField(name=name,offset=i*4,datatype=7,count=1)for i,name in enumerate(('x','y','z'))]
        import struct
        cloud.data=struct.pack('<ffffff8xffffff8x',1,2,3,4,5,6,7,8,9,10,11,12)
        np.testing.assert_allclose(bag.cloud_xyz(cloud),np.arange(1,13).reshape(4,3))


if __name__=='__main__':unittest.main()


class SavedOutputTest(unittest.TestCase):
    def test_publisher_rejects_failed_report(self):
        import publish_calibration as pub
        import yaml
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);(p/'report.yaml').write_text(yaml.safe_dump({'status':'failed'}))
            with self.assertRaisesRegex(ValueError,'did not pass'):pub.load(p)

    def test_camera_yaml_and_publisher_frame_conventions(self):
        import calibrate_bag as bag
        import publish_calibration as pub
        import yaml
        k=np.array([[610.,0,640],[0,605,360],[0,0,1]])
        camera=bag.camera_yaml(k,np.zeros(5),(1280,720),'camera_optical','plumb_bob')
        ground=math.ground_transform(np.array([0.,0.,1.]),.87)
        tf=math.transform_record(ground,'ground','lidar')
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);(p/'report.yaml').write_text(yaml.safe_dump({'status':'passed'}))
            document={'schema_version':1,'camera_info':camera,'camera_frame':'camera_optical','transforms':[tf]}
            (p/'calibration.yaml').write_text(yaml.safe_dump(document))
            bundle=pub.load(p)
            self.assertEqual(bundle['transforms'][0]['parent_frame'],'ground')
            self.assertEqual(bundle['transforms'][0]['translation'],{'x':0.,'y':0.,'z':.87})
            self.assertEqual(bundle['camera_info']['camera_matrix']['data'],k.ravel().tolist())


class OutputPathTest(unittest.TestCase):
    def test_default_output_uses_mission_name_in_workspace(self):
        import calibrate_bag as bag
        from unittest.mock import patch
        workspace=Path(__file__).resolve().parents[3]
        mission=Path('/data/arthur/bags/calibration/mission_example')
        expected=workspace/'data/calibration/results/mission_example'
        self.assertEqual(bag.default_output(mission),expected)
        with patch.object(sys,'argv',['calibrate_bag',str(mission)]), patch.object(bag,'run',return_value=0) as run:
            self.assertEqual(bag.main(),0)
            self.assertEqual(run.call_args.args[0].output,expected)

    def test_explicit_output_overrides_default(self):
        import calibrate_bag as bag
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)/'custom'
            with patch.object(sys,'argv',['calibrate_bag','/data/mission_example','--output',str(output)]), patch.object(bag,'run',return_value=0) as run:
                self.assertEqual(bag.main(),0)
                self.assertEqual(run.call_args.args[0].output,output)
