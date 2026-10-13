#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

class Test(Node):
    def __init__(self):
        super().__init__("imu_axis_test")
        self.declare_parameter("topic", "/imu/data_raw")
        topic=self.get_parameter("topic").value
        qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lock=threading.Lock()
        self.reset()
        self.create_subscription(Imu,topic,self.cb,qos)
        print(f"Listening to {topic}")
        print("Press Enter to reset/start a test; perform ONE motion;")
        print("press Enter again to print peaks and start the next test.")
        print("Motions: LEFT side up | NOSE up | CCW yaw")
        print("Ctrl-C quits.\n")

    def reset(self):
        self.n=0
        self.lo=[float("inf")]*3
        self.hi=[float("-inf")]*3
        self.peak=[0.0]*3

    def cb(self,m):
        v=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]
        with self.lock:
            self.n+=1
            for i,x in enumerate(v):
                self.lo[i]=min(self.lo[i],x)
                self.hi[i]=max(self.hi[i],x)
                if abs(x)>abs(self.peak[i]): self.peak[i]=x

    def report(self):
        with self.lock:
            if self.n:
                print(f"\nSamples: {self.n}")
                for a,i in zip("XYZ",range(3)):
                    print(f"{a}: min={self.lo[i]:+.4f} max={self.hi[i]:+.4f} peak={self.peak[i]:+.4f} rad/s")
                i=max(range(3),key=lambda j:abs(self.peak[j]))
                print(f"Dominant: {'XYZ'[i]} {'+' if self.peak[i]>=0 else '-'} ({self.peak[i]:+.4f} rad/s)\n")
            else:
                print("\nNo samples yet.\n")
            self.reset()

def reader(n):
    while rclpy.ok():
        try: input()
        except EOFError: return
        n.report()

def main():
    rclpy.init()
    n=Test()
    threading.Thread(target=reader,args=(n,),daemon=True).start()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
