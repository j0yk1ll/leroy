#!/usr/bin/env python3
"""Leroy ChArUco-guided planar *temporal-only* camera/IMU calibration.

Default: record data while an operator moves the robot; no motion commands.
Armed: small, bounded mecanum movements through /cmd_vel with active ChArUco
tracking, strict visual/IMU freshness gates, and a stop on any anomaly.

This is NOT a navigation or collision avoidance system. Use only in a physically
cleared, supervised test area with an independent E-stop, after independently
commissioning the motor controller. It does not override the motor interlock.

Reads the existing printed ChArUco board via subpixel corners,
then transforms visual angular velocity into the measured IMU frame using TF.
Fits only the time offset; camera/IMU spatial extrinsics remain unchanged.
"""

from __future__ import annotations

import argparse
import gc
import json
import math
from pathlib import Path
import random
import time
import sys

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'charuco'))
from pose import CharucoPoseEstimator


def timestamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_matrix(q):
    x, y, z, w = map(float, q)
    length = x*x + y*y + z*z + w*w
    if not 0.99 < length < 1.01:
        raise ValueError('Invalid camera-to-IMU TF quaternion')
    s = 2.0 / length
    return np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)],
    ], dtype=float)


def visual_gyro_z(pose_rows, rot_imu_cam):
    """Return timestamp pairs and rotation-rate component in the IMU z axis."""
    series = []
    rows = sorted(pose_rows, key=lambda x: x[0])
    for (ta, ra), (tb, rb) in zip(rows, rows[1:]):
        dt = (tb - ta) / 1e9
        if dt <= 0.03 or dt > 0.25:
            continue
        # R_C,B is world(board) orientation expressed in a rotating camera.
        # Minus sign turns apparent stationary-board motion into camera motion.
        delta_r = rb @ ra.T
        vec, _ = cv2.Rodrigues(delta_r)
        omega_cam = -vec.reshape(3) / dt
        omega_imu = rot_imu_cam @ omega_cam
        series.append((ta, tb, float(omega_imu[2])))
    return series


def search_offset(pose_rows, imu_rows, rot_imu_cam):
    """Estimate camera-header minus IMU-header shift, with split validation."""
    visual = visual_gyro_z(pose_rows, rot_imu_cam)
    if len(visual) < 100 or len(imu_rows) < 500:
        return {'valid': False, 'reason': 'Not enough tracked ChArUco poses / IMU samples',
                'visual_pairs': len(visual)}
    origin = min(visual[0][0], imu_rows[0][0])
    time0 = np.array([(a-origin)/1e9 for a,b,v in visual])
    time1 = np.array([(b-origin)/1e9 for a,b,v in visual])
    vz = np.array([v for a,b,v in visual],dtype=float)
    imu = sorted(imu_rows)
    ti = np.array([(t-origin)/1e9 for t,z in imu])
    gi = np.array([z for t,z in imu])
    if np.std(vz) < 0.035 or np.std(gi) < 0.035:
        return {'valid': False, 'reason': 'Insufficient varying yaw signal',
                'visual_pairs': len(visual)}
    # Integrated gyro is correctly averaged over each observed camera interval.
    cumulative = np.r_[0.0,np.cumsum((gi[1:]+gi[:-1])*np.diff(ti)*0.5)]
    lag_ms = np.arange(-250,251,2,dtype=int)
    tmid = (time0+time1)/2

    def estimate(selection):
        scores=[]
        for m in lag_ms:
            shift=m/1000
            a=time0[selection]-shift
            b=time1[selection]-shift
            inside=(a>=ti[0])&(b<=ti[-1])
            if np.count_nonzero(inside)<60:
                scores.append(np.nan)
                continue
            reference=(np.interp(b[inside],ti,cumulative)-np.interp(a[inside],ti,cumulative))/(b[inside]-a[inside])
            seen=vz[selection][inside]
            if np.std(reference)<0.035 or np.std(seen)<0.035:
                scores.append(np.nan)
            else:
                scores.append(float(np.corrcoef(reference,seen)[0,1]))
        scores=np.array(scores)
        if not np.isfinite(scores).any():
            return None
        k=int(np.nanargmax(scores))
        excluded=np.abs(lag_ms-lag_ms[k])>=40
        competing=float(np.nanmax(scores[excluded])) if np.isfinite(scores[excluded]).any() else float('nan')
        return {'lag_ms':int(lag_ms[k]),'correlation':float(scores[k]),
                'peak_separation':float(scores[k]-competing),
                'zero_correlation':float(scores[np.flatnonzero(lag_ms==0)[0]])}

    all_res=estimate(np.ones(len(tmid),dtype=bool))
    median_t=(tmid[0]+tmid[-1])/2
    first=estimate(tmid<median_t)
    second=estimate(tmid>=median_t)
    if all_res is None:
        return {'valid':False,'reason':'Insufficient correlated signal','visual_pairs':len(visual)}
    all_res.update({'visual_pairs':len(visual), 'first_half':first,'second_half':second})
    valid=(all_res['correlation']>=0.65 and all_res['peak_separation']>=0.06
           and abs(all_res['lag_ms'])<240 and first is not None and second is not None
           and abs(first['lag_ms']-second['lag_ms'])<=15)
    all_res['valid']=bool(valid)
    if not valid:
        all_res['reason']='Weak/broad correlation or inconsistent split estimates'
    all_res['meaning']='camera header time minus IMU header time for corresponding physical rotation'
    return all_res


def charuco_pose_violation(p, min_range, max_range, max_displacement,
                           initial_camera_in_board=None):
    """Identify a genuine geometric safety violation from a valid ChArUco pose.

    A missing pose is handled separately by the controller's brief stop/hold
    gate: a single skipped detector frame is not a displacement violation.
    """
    if p is None:
        return 'ChArUco pose unavailable'
    cx, cy = float(p['cx']), float(p['cy'])
    if not (0.23 < cx < 0.77 and 0.20 < cy < 0.80):
        return f'ChArUco center outside image safety margins ({cx:.2f}, {cy:.2f})'
    rng = float(p['range_m'])
    if not min_range <= rng <= max_range:
        return f'ChArUco target range outside limits ({rng:.2f}m)'
    if initial_camera_in_board is not None:
        delta = np.asarray(p['location_b'], dtype=float) - initial_camera_in_board
        displacement = float(np.linalg.norm(delta))
        if not math.isfinite(displacement) or displacement > max_displacement:
            return (f'ChArUco camera displacement exceeds limit '
                    f'({displacement:.3f}m > {max_displacement:.3f}m)')
    return None


def args_parser():
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--target',help='Existing ChArUco board_config.json')
    p.add_argument('--output',default='/tmp/leroy_charuco_time_01',help='New ROS 2 bag directory')
    p.add_argument('--seconds',type=float,default=75,help='Capture duration after acquisition')
    p.add_argument('--wait',type=float,default=90,help='Max seconds waiting for ChArUco')
    p.add_argument('--min-corners',type=int,default=8)
    p.add_argument('--camera-frame',default='camera_optical_frame')
    p.add_argument('--max-rmse',type=float,default=2.5)
    p.add_argument('--max-range',type=float,default=2.5)
    p.add_argument('--min-range',type=float,default=0.65)
    p.add_argument('--max-displacement',type=float,default=0.18,
                   help='Max camera displacement in board coordinates from starting pose')
    p.add_argument('--arm',action='store_true',help='Enable limited /cmd_vel motions')
    p.add_argument('--confirm-estop',action='store_true',help='Independent hardware emergency stop tested')
    p.add_argument('--confirm-clear-area',action='store_true',
                   help='Explicit acknowledgement: cleared bounded area, supervised, independent E-stop ready')
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--self-test',action='store_true')
    a=p.parse_args()
    if a.self_test:
        return a
    if not a.target:
        p.error('--target required')
    if a.seconds<20 or a.wait<5 or a.min_corners<6 or a.min_corners>20:
        p.error('seconds >=20; wait>=5; min-corners in [6,20]')
    if a.arm and not (a.confirm_clear_area and a.confirm_estop):
        p.error('--arm requires --confirm-clear-area and --confirm-estop')
    if not (0.4<=a.min_range<a.max_range<=5):
        p.error('Invalid distance bounds')
    if not (0.05<=a.max_displacement<=0.25):
        p.error('max-displacement must be 0.05..0.25 m')
    if Path(a.output).exists():
        p.error('Bag output already exists: '+a.output)
    return a


def main():
    args=args_parser()
    if args.self_test:
        config = (Path(args.target) if args.target else
                  Path(__file__).resolve().parents[1] /
                  'charuco' / 'calibration_board' / 'board_config.json')
        print(f'OpenCV {cv2.__version__}: loading ChArUco board', flush=True)
        detector = CharucoPoseEstimator(config)
        print('ChArUco board created; rendering synthetic target', flush=True)
        # Exercise the full marker/corner detector, not only board construction.
        board = detector.board
        if hasattr(board, 'generateImage'):
            test_image = board.generateImage((700, 980), marginSize=35)
        else:
            test_image = board.draw((700, 980), marginSize=35)
        print('Synthetic target generated; detecting ChArUco corners', flush=True)
        from calibrate import detect_charuco
        corners, ids, _, _ = detect_charuco(
            test_image, detector.board, detector.dictionary, diagnostics=True
        )
        found = len(ids) if ids is not None else 0
        if found < detector.min_corners:
            raise RuntimeError(
                f'ChArUco synthetic detection failed: {found} corners; '
                f'expected at least {detector.min_corners}'
            )
        print(f'ChArUco SELF-TEST PASSED: {found}/{len(detector.xyz)} '
              'internal corners detected')
        return 0

    detector=CharucoPoseEstimator(Path(args.target),args.min_corners,args.max_rmse)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from rclpy.time import Time
    from rclpy.parameter_client import AsyncParameterClient
    import rosbag2_py
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import CameraInfo, CompressedImage, Imu
    from tf2_ros import Buffer, TransformListener

    class Controller(Node):
        def __init__(self):
            super().__init__('leroy_charuco_time_calibrate')
            self.info=None
            self.imu_frame=None
            self.image_frame=None
            self.last_image_at=None
            self.last_imu_at=None
            self.last_pose_at=None
            self.last_pose=None
            self.initial_camera_in_board=None
            self.good_since=None
            self.recording_started=None
            self.stop_reason=None
            self.pose_rows=[]
            self.imu_rows=[]
            self.latest_gyro_z=None
            self.writer=None
            self.counts={'images':0,'imu':0,'good_poses':0,'bad_poses':0,'bag_frames':0,'bag_imu':0}
            self.last_detection_at=-1e9
            self.r_imu_cam=None
            self.r_base_cam=None
            self.t_base_cam=None
            self.preflight_ok=False
            self.tf_buffer=Buffer()
            self.tf_listener=TransformListener(self.tf_buffer,self,spin_thread=False)
            self.param_client=AsyncParameterClient(self,'/leroy_bridge')
            self.pub=self.create_publisher(Twist,'/cmd_vel',10) if args.arm else None
            self.rng=random.Random(args.seed)
            self.stage='idle'
            self.stage_end=None
            self.segment=None
            self.motion_count=0
            self.create_subscription(CompressedImage,'/camera/image_raw/compressed',self.on_image,qos_profile_sensor_data)
            self.create_subscription(Imu,'/imu/data_raw',self.on_imu,qos_profile_sensor_data)
            self.create_subscription(CameraInfo,'/camera/camera_info',self.on_info,qos_profile_sensor_data)
            self.create_timer(0.05,self.control)
            self.create_timer(2.0,self.status)

        def preflight(self):
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                rclpy.spin_once(self,timeout_sec=0.05)
                if self.image_frame and self.imu_frame:
                    try:
                        tf=self.tf_buffer.lookup_transform(self.imu_frame,self.image_frame,Time())
                        q=tf.transform.rotation
                        self.r_imu_cam=quaternion_matrix((q.x,q.y,q.z,q.w))
                        if args.arm:
                            bc=self.tf_buffer.lookup_transform('base_link',self.image_frame,Time())
                            cq=bc.transform.rotation
                            self.r_base_cam=quaternion_matrix((cq.x,cq.y,cq.z,cq.w))
                            ct=bc.transform.translation
                            self.t_base_cam=np.array([ct.x,ct.y,ct.z],dtype=float)
                        break
                    except Exception:
                        pass
            if self.r_imu_cam is None:
                raise RuntimeError('Cannot resolve TF between actual IMU and camera frame IDs; check /tf_static.')
            if self.image_frame != args.camera_frame:
                raise RuntimeError(f'Camera frame {self.image_frame!r} does not match expected {args.camera_frame!r}')
            print('Loaded actual TF camera -> IMU rotation:\n'+str(self.r_imu_cam),flush=True)
            if args.arm:
                required=['wheel_velocity_to_pwm_gain',
                          'max_pwm','cmd_vel_timeout_s','watchdog_ms']
                fut=self.param_client.get_parameters(required)
                deadline=time.monotonic()+5
                while not fut.done() and time.monotonic()<deadline:
                    rclpy.spin_once(self,timeout_sec=0.05)
                if not fut.done():
                    raise RuntimeError('Cannot confirm Leroy bridge motor interlock; refusing to arm')
                vals=fut.result().values
                pwm_gain=float(vals[0].double_value)
                max_pwm=int(vals[1].integer_value)
                cmd_timeout=float(vals[2].double_value)
                watchdog_ms=int(vals[3].integer_value)
                if (not math.isfinite(pwm_gain) or pwm_gain<=0
                        or not 1<=max_pwm<=64 or not 0<cmd_timeout<=0.30
                        or not 1<=watchdog_ms<=500):
                    raise RuntimeError(
                        'Motor interlock or safety limits invalid; refusing to arm. '
                        f'wheel_velocity_to_pwm_gain={pwm_gain}, '
                        f'max_pwm={max_pwm}, cmd_vel_timeout_s={cmd_timeout}, '
                        f'watchdog_ms={watchdog_ms}')
                if self.count_publishers('/cmd_vel')!=1:
                    raise RuntimeError('Another /cmd_vel publisher exists; refusing competing motor commands')
                print(f'ARMED FOR SUPERVISED BOUNDED MOTION: PWM gain '
                      f'{pwm_gain:.2f} PWM/(rad/s); actual PWM capped at {max_pwm}/255; '
                      f'cmd timeout {cmd_timeout:.2f}s; MCU watchdog {watchdog_ms}ms. '
                      'Stop with Ctrl+C or physical E-stop.',flush=True)
            self.preflight_ok=True

        def on_info(self,msg):
            self.info=msg
            if self.writer is not None:
                self.write('/camera/camera_info',msg,timestamp_ns(msg.header.stamp))

        def on_imu(self,msg):
            self.last_imu_at=time.monotonic()
            self.imu_frame=msg.header.frame_id
            self.counts['imu']+=1
            z=float(msg.angular_velocity.z)
            if np.isfinite(z):
                if self.recording_started is not None:
                    self.imu_rows.append((timestamp_ns(msg.header.stamp),z))
                self.latest_gyro_z=z
            if self.writer is not None:
                self.write('/imu/data_raw',msg,timestamp_ns(msg.header.stamp))
                self.counts['bag_imu']+=1

        def on_image(self,msg):
            now=time.monotonic()
            self.counts['images']+=1
            self.last_image_at=now
            self.image_frame=msg.header.frame_id
            if self.writer is not None:
                self.write('/camera/image_raw/compressed',msg,timestamp_ns(msg.header.stamp))
                self.counts['bag_frames']+=1
            if now-self.last_detection_at<0.055:
                return
            self.last_detection_at=now
            gray=cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
            if gray is None:
                self.last_pose=None
                self.good_since=None
                return
            self.last_pose=detector.pose(gray,self.info,args.min_corners,args.max_rmse)
            if self.last_pose is None:
                self.counts['bad_poses']+=1
                self.good_since=None
                return
            self.counts['good_poses']+=1
            self.last_pose_at=now
            if self.good_since is None:
                self.good_since=now
            if self.recording_started is not None:
                self.pose_rows.append((timestamp_ns(msg.header.stamp),self.last_pose['rot'].copy()))

        def write(self,topic,msg,ts):
            self.writer.write(topic,serialize_message(msg),int(ts))

        def begin(self):
            self.initial_camera_in_board=self.last_pose['location_b'].copy()
            self.writer=rosbag2_py.SequentialWriter()
            self.writer.open(rosbag2_py.StorageOptions(uri=args.output,storage_id='sqlite3'),
                             rosbag2_py.ConverterOptions('cdr','cdr'))
            # Keep compressed images (the measured reliable stream) with original timestamps.
            for name,msg_type in [('/camera/image_raw/compressed','sensor_msgs/msg/CompressedImage'),
                                  ('/camera/camera_info','sensor_msgs/msg/CameraInfo'),
                                  ('/imu/data_raw','sensor_msgs/msg/Imu')]:
                # Jazzy's rosbag2_py pybind constructor requires id even
                # though rosbag2 assigns the persisted topic IDs itself.
                self.writer.create_topic(rosbag2_py.TopicMetadata(
                    id=0,name=name,type=msg_type,serialization_format='cdr'))
            self.recording_started=time.monotonic()
            print('ChArUco acquired; synchronized recording begun. '+
                  ('AUTONOMOUS BOUNDED MOTION STARTED.' if args.arm else 'MANUAL MOTION MODE.'),flush=True)

        def stop(self,reason):
            if self.stop_reason is None:
                self.stop_reason=reason
                print('STOP: '+reason,flush=True)
            self.send_stop()

        def send_stop(self):
            if self.pub is not None:
                self.pub.publish(Twist())

        def control(self):
            now=time.monotonic()
            if self.stop_reason is not None:
                self.send_stop()
                return
            if not self.preflight_ok:
                return
            if self.recording_started is None:
                self.send_stop()
                if self.last_pose is not None and self.r_imu_cam is not None and self.good_since is not None:
                    if now-self.good_since>=1.5 and now-self.last_pose_at<=0.25:
                        if self.safe_pose(self.last_pose,initial=False):
                            self.begin()
                return
            if now-self.recording_started>=args.seconds:
                self.stop('Recording completed')
                return
            if not args.arm:
                return
            if self.count_publishers('/cmd_vel')!=1:
                self.stop('Concurrent /cmd_vel publisher detected')
                return
            if self.last_image_at is None or now-self.last_image_at>0.25:
                self.stop('Camera stream stale')
                return
            if self.last_imu_at is None or now-self.last_imu_at>0.25:
                self.stop('IMU stream stale')
                return
            if self.latest_gyro_z is None or abs(self.latest_gyro_z)>0.8:
                self.stop('Unexpectedly high yaw rate or invalid gyro reading')
                return
            if self.last_pose_at is None or now-self.last_pose_at>0.30:
                self.stop('Lost ChArUco tracking')
                return
            if self.last_pose is None:
                # A single failed ChArUco detection used to trigger the
                # generic pose-limit STOP even when the target was centered.
                # Command zero while waiting briefly for reacquisition;
                # the 0.30s missing-pose timeout above remains a hard stop.
                self.send_stop()
                return
            violation = self.pose_violation(self.last_pose, initial=True)
            if violation is not None:
                self.stop(violation)
                return
            # Require several good detections before resuming motor output
            # following an intermittent dropout; preserve the hard limit.
            if self.good_since is None or now-self.good_since<0.20:
                self.send_stop()
                return
            if self.stage_end is None or now>=self.stage_end:
                if self.stage=='motion':
                    self.stage='pause'
                    self.stage_end=now+self.rng.uniform(0.4,1.0)
                    self.send_stop()
                    return
                self.stage='motion'
                self.stage_end=now+self.rng.uniform(0.5,1.0)
                kind=self.rng.choice(['yaw_left','yaw_right','front','back','left','right'])
                self.segment=kind
                self.motion_count+=1
                print(f'Excitation {self.motion_count}: {kind} for {self.stage_end-now:.2f}s',flush=True)
            if self.stage=='pause':
                self.send_stop()
                return
            # Use target direction transformed into imu_link (aligned with base_link
            # by the *configured* Leroy extrinsics), not image-coordinate heuristics.
            center_imu=self.r_base_cam@self.last_pose['center_c']+self.t_base_cam
            if center_imu[0]<=0:
                self.stop('Target not ahead of robot')
                return
            yaw_err=math.atan2(center_imu[1],center_imu[0])
            cmd=Twist()
            # Keep grid approximately centered. Servo dominates outside 9 deg.
            correction=max(-0.12,min(0.12,0.75*yaw_err))
            if abs(yaw_err)>0.16:
                cmd.angular.z=correction
            elif self.segment=='yaw_left':
                cmd.angular.z=0.16+correction
            elif self.segment=='yaw_right':
                cmd.angular.z=-0.16+correction
            elif self.segment=='front':
                cmd.linear.x=0.025
                cmd.angular.z=correction
            elif self.segment=='back':
                cmd.linear.x=-0.025
                cmd.angular.z=correction
            elif self.segment=='left':
                cmd.linear.y=0.025
                cmd.angular.z=correction
            elif self.segment=='right':
                cmd.linear.y=-0.025
                cmd.angular.z=correction
            cmd.angular.z=max(-0.22,min(0.22,cmd.angular.z))
            self.pub.publish(cmd)

        def pose_violation(self,p,initial):
            return charuco_pose_violation(
                p, args.min_range, args.max_range, args.max_displacement,
                self.initial_camera_in_board if initial else None)

        def safe_pose(self,p,initial):
            return self.pose_violation(p,initial) is None

        def status(self):
            p=self.last_pose
            desc=('tags=%d rmse=%.2f center=(%.2f,%.2f) range=%.2fm' %
                  (p['tags'],p['rmse'],p['cx'],p['cy'],p['range_m'])) if p else 'NO CHARUCO'
            mode=('REC' if self.recording_started is not None else 'SEARCH')
            print(f'{mode}: images={self.counts["images"]} imu={self.counts["imu"]} '
                  f'poses={self.counts["good_poses"]} {desc}',flush=True)

        def close_bag(self):
            writer,self.writer=self.writer,None
            if writer is not None:
                del writer
                gc.collect()

    rclpy.init()
    node=Controller()
    started=time.monotonic()
    result=None
    aborted=False
    try:
        node.preflight()
        print('Searching for ChArUco board. '+
              ('AUTO MOTION ARMED' if args.arm else 'DRY RUN / MANUAL MOVEMENT; no /cmd_vel publisher'),flush=True)
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=0.03)
            now=time.monotonic()
            if node.stop_reason is not None:
                break
            if node.recording_started is None and now-started>=args.wait:
                node.stop('Timed out waiting for visible ChArUco')
                break
    except KeyboardInterrupt:
        aborted=True
        node.stop('Operator interrupted')
    except Exception as exc:
        aborted=True
        node.stop('Exception: '+str(exc))
        raise
    finally:
        # Stop immediately and repeatedly before doing any analysis or disk work.
        for _ in range(5):
            node.send_stop()
            time.sleep(0.04)
        node.close_bag()
        if node.recording_started is not None:
            result=search_offset(node.pose_rows,node.imu_rows,node.r_imu_cam)
            print('\n=== TEMPORAL OFFSET ESTIMATE ===',flush=True)
            print(json.dumps(result,indent=2),flush=True)
        summary={
            'stop_reason':node.stop_reason,
            'recording_seconds':max(0,time.monotonic()-node.recording_started)
                                if node.recording_started is not None else 0,
            'bag':args.output if node.recording_started is not None else None,
            'motion_enabled':args.arm,
            'maneuvers_completed_or_started':node.motion_count,
            'camera_frame':node.image_frame,'imu_frame':node.imu_frame,
            'target_file':args.target,'counts':node.counts,
            'estimate':result,
        }
        out=Path(args.output+'_summary.json')
        out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
        print('\nSaved summary to '+str(out),flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.recording_started is not None and not aborted and result and result.get('valid') else 2


if __name__=='__main__':
    raise SystemExit(main())