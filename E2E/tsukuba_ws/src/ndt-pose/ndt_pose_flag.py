import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class ndt_pose_flag():
    def __init__(self):
	self.Autoware_flame = 0
        self._sub = rospy.Subscriber('/ndt_pose', PoseStamped, self.callback_pose)
        self._pub = rospy.Publisher('/ndt_pose_flag', Bool, queue_size=1)

    def callback_pose(self,pose_msg):
        if(pose_msg.pose.position.x < -40):
            self.Autoware_flame = self.Autoware_flame + 1

        true_false = Bool()
        if(self.Autoware_flame >= 10):
            true_false.data = True
            self._pub.publish(true_false)
        else:
            true_false.data = False
            self._pub.publish(true_false)
            


    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ndt_pose_flag')
    ndt_pose_flag_c = ndt_pose_flag()
    ndt_pose_flag_c.main()


