"""ROS-free ChArUco temporal-offset tests."""
import math
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from calibrate_time_offset import search_offset, visual_gyro_z, quaternion_matrix

class TimingTests(unittest.TestCase):
    def sequence(self, offset_ms):
        base=1_790_000_000_000_000_000
        terms=[(0.12,0.43),(0.065,1.1),(0.038,2.2)]
        def yaw(t):
            return sum(a*math.sin(2*math.pi*f*t) for a,f in terms)
        def gyro(t):
            return sum(a*2*math.pi*f*math.cos(2*math.pi*f*t) for a,f in terms)
        imu=[(base+round(j*1e7),gyro(j/100)) for j in range(5001)]
        poses=[]
        for j in range(1501):
            t=j/30
            a=-yaw(t)
            c,s=math.cos(a),math.sin(a)
            r=np.array([[c,-s,0],[s,c,0],[0,0,1]])
            poses.append((base+round(t*1e9)+offset_ms*1_000_000,r))
        return poses,imu

    def test_positive_offset(self):
        poses,imu=self.sequence(34)
        r=search_offset(poses,imu,np.eye(3))
        self.assertTrue(r['valid'],r)
        self.assertLessEqual(abs(r['lag_ms']-34),4)

    def test_negative_offset(self):
        poses,imu=self.sequence(-56)
        r=search_offset(poses,imu,np.eye(3))
        self.assertLessEqual(abs(r['lag_ms']+56),4)

    def test_stationary_rejected(self):
        poses,imu=self.sequence(34)
        result=search_offset([(t,np.eye(3)) for t,_ in poses],imu,np.eye(3))
        self.assertFalse(result['valid'])

    def test_quaternion(self):
        q=quaternion_matrix((0,0,math.sin(math.pi/4),math.cos(math.pi/4)))
        np.testing.assert_allclose(q@np.array([1.,0,0]),[0,1,0],atol=1e-12)

if __name__=='__main__':
    unittest.main()
