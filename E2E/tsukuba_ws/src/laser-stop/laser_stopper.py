import rospy
import numpy as np
from sensor_msgs.msg      import LaserScan
from std_msgs.msg      import Float32
import sensor_msgs.point_cloud2 as pc2
from laser_geometry import LaserProjection

class laser_stopper():
    def __init__(self):
	self.max_vel = 0.0
        self._sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)
        self._pub = rospy.Publisher('/max_speed', Float32, queue_size=1)
        self.laser_projector = LaserProjection()

    def callback_scan(self,scan_msg):
        vel = Float32()
        tmp = 100.0
        last_idx = 0
        idx = 0
        count = 0
        vel.data=10.0
        avg = 0
        prev = 0
        scancount = 0
        backcount = 0
        backing = 0
        for r in scan_msg.ranges:
            #if (backcount < 1 and backing):
            #    backcount = 0
            #    backing = 0
            #elif (backing and backcount > 1):
            #    backcount = backcount - 1
            #    continue
            if (backing == 0 and r < 0.8 and r > 0.1):
               if (last_idx == idx - 1):
                  last_idx = idx
                  count = count + 1
                  prev = avg
                  avg = prev + (r - avg)/count
                  if (count >= 5) :
                     #vel.data=0.0
                     #vel.data = avg * 0.1
                     if (avg < 0.3):
                        vel.data = 0
                        #scancount = scancount + 1
                        #if (scancount > 100 and backing == 0):
                        #   vel.data = -0.2
                        #   backcount = 100
                        #   backing = 1

               else:
                  last_idx = idx
                  count = 1
                  avg = 0
                  prev = 0
                  scancount = 0
                  backcount = 0
                  backing = 0

            idx = idx + 1
        #cloud = self.laser_projector.projectLaser(scan_msg)
        #for p in pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z")):
        #    if(p[1] > -0.3 and p[1] < 0.3):
        #       if (tmp>p[0]):
        #          tmp = p[0]
        #if(tmp==100.0):
        #    vel.data=10.0
        #    
        #else:
        #    if(tmp<=1):
        #        vel.data=0.0
        #    else:
        #        vel.data=(tmp-1)*0.1
        self._pub.publish(vel)
        print vel.data

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('laser_stopper')
    laser_stopper_c = laser_stopper()
    laser_stopper_c.main()


