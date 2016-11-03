import rosbag
import rospy
import sys

from sensor_msgs.msg import CameraInfo, PointCloud2, Image

#import cv2
#from cv_bridge import CvBridge, CvBridgeError

#bridge = CvBridge()

#def apply_segmentation(topic_dct):
    #try:
      #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #except CvBridgeError, e:
      #print e  


if __name__ == "__main__":
  if len(sys.argv) < 3:
    print "Usage:  <original_bag> <result_bag>]"
    sys.exit()
    
  rospy.init_node("player")

  print "Opening original bag %s" % sys.argv[1]
  orig_bag = rosbag.Bag(sys.argv[1], 'r')

  # publish camera info
  
  info = None
  for topic, msg, t in orig_bag.read_messages(topics=['camera/rgb/camera_info', '/camera/rgb/camera_info']):
    info = msg
  assert (info is not None)
  info_pub = rospy.Publisher('/camera/rgb/camera_info', type(info), queue_size=10)

  print "Opening result bag %s" % sys.argv[2]
  res_bag = rosbag.Bag(sys.argv[2], 'r')

  #result_topics = ['/camera/rgb/image_rect_color',]
  topic_buf = {}
  pub_buf = {}
  
  last_t = None
  for topic, msg, t in res_bag.read_messages():

    try:
      msg.header.frame_id = info.header.frame_id
      info.header.stamp = msg.header.stamp
      print "     resetting header of image %s" % topic
    except e:
      print e

    if last_t is not None and t > last_t:
      # publish
      info_pub.publish(info)
      for topic_, msg_ in topic_buf.items():
        print "  Publishing %s at %.3f" % (topic_, last_t.to_sec())
        pub_buf[topic_].publish(msg_)
        
      raw_input("Enter for next")
      topic_buf = {}

    topic_buf[topic] = msg

    if topic not in pub_buf:
      pub_buf[topic] = rospy.Publisher(topic, type(msg), queue_size=10)
      
    last_t = t

  orig_bag.close()
  res_bag.close()
