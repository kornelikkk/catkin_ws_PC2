import rospy

from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

class Laser2PC:
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher ("/laserPointCloud", pc2, queve_size=1)
        self.laserSub = rospy.Subscriber ("/scan", LaserScan, self.LaserCallback)

    def LaserCallback(self, data):
        cloud_out = self. laserProj.projectLaser(data)
        self .pcPub.publish (cloud_out)

if __name__ == 'main':
    rospy.init_node ("Laser2PointCloud")
    l2pe = Laser2PC()
    rospy.spin()
