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
        self.scancount = 0
        self.backcount = 0
        self.backing = 0

    def callback_scan(self,scan_msg):
        vel = Float32()
        tmp = 100.0
        last_idx = 0
        idx = 0
        count = 0
        vel.data=10.0
        avg = 0
        prev = 0
        stoploop = 0
        if (self.backcount < 1 and self.backing):
            self.backcount = 0
            self.backing = 0
            self.scancount = 0
        elif (self.backing and self.backcount >= 1):
            self.backcount = self.backcount - 1
            vel.data = -0.2
            stoploop = 1

        if (stoploop == 0):
		     for r in scan_msg.ranges :
		        if (self.backing == 0 and r < 0.8 and r > 0.1):
		           if (last_idx == idx - 1):
		              last_idx = idx
		              count = count + 1
		              prev = avg
		              avg = prev + (r - avg)/count
		              if (count >= 5) :
		                 #vel.data=0.0
		                 #vel.data = avg * 0.1
		                 if (avg < 0.4):
		                    vel.data = 0
		                    self.scancount = self.scancount + 1
		                    if (self.scancount > 100 and self.backing == 0):
		                       vel.data = -0.2
		                       self.backcount = 100
		                       self.backing = 1
		                       self.scancount = 0
		                       count = 0
		                       avg = 0
		                       prev = 0

		           else:
		              last_idx = idx
		              count = 1
		              avg = 0
		              prev = 0
		              #self.scancount = 0
		              #self.backcount = 0
		              #self.backing = 0

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
        print vel.data, self.backing, self.backcount, self.scancount

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('laser_stopper')
    laser_stopper_c = laser_stopper()
    laser_stopper_c.main()


