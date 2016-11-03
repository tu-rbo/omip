#!/usr/bin/env python
# license removed for brevity
import rospy
from roslib import message
import sensor_msgs.point_cloud2
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
import geometry_msgs.msg
import math
import PyKDL

# Because of transformations
import tf
import tf2_ros

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x, 
                                    t.transform.translation.y, 
                                    t.transform.translation.z))

# PointStamped
def do_transform_cloud(cloud, transform):
    res = pc2.create_cloud(cloud.header, cloud.fields, pc2.read_points(cloud))
    t_kdl = transform_to_kdl(transform)
    for p_in, p_out in [ pc2.read_points(cloud), pc2.read_points(res) ]:
        p = t_kdl * PyKDL.Vector(p_in.x, p_in.y, p_in.z)
        p_out.x = p[0]
        p_out.y = p[1]
        p_out.z = p[2]
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(PointCloud2, do_transform_cloud)

def FeatureTrackerMockup():
  publisher = rospy.Publisher('/feat_tracker/state', PointCloud2, queue_size=10)
  rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(1) # 10hz
  fake_tracked_feats = PointCloud2()
  header = Header()
  point = Point()
  
  cloud = [[1,2,3],[4,5,6],[7,8,9]]
  
  fake_tracked_feats = pc2.create_cloud_xyz32(fake_tracked_feats.header, cloud)
  
  fake_tracked_feats.header.frame_id = 'camera_rgb_optical_frame'
  
  publisher.publish(fake_tracked_feats)
  rate.sleep()
  
  transform = geometry_msgs.msg.TransformStamped()
  transform.transform.translation.x = 0.1
  transform.transform.translation.y = 0.05
  transform.transform.translation.z = 0.1
  q = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
  transform.transform.rotation.x = q[0]
  transform.transform.rotation.y = q[1]
  transform.transform.rotation.z = q[2]
  transform.transform.rotation.w = q[3]
    
  while not rospy.is_shutdown():
    cloud_out = do_transform_cloud(fake_tracked_feats, transform.transform)
    fake_tracked_feats.header.stamp = rospy.Time.now()
    publisher.publish(fake_tracked_feats)
    rate.sleep()

if __name__ == '__main__':
    try:
        FeatureTrackerMockup()
    except rospy.ROSInterruptException:
        pass
