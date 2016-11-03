import rosbag
import rospy
import sys
import os

from generate_statistics import data_path, ground_truth_bags, infer_experiment_name

from sensor_msgs.msg import CameraInfo, PointCloud2, Image

import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

#def apply_segmentation(topic_dct):
    #try:
      #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #except CvBridgeError, e:
      #print e  

topics=['/camera/rgb/image_rect_color', 'camera/rgb/image_rect_color']

################

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print "Usage: extract_images <bag> [format=jpg] "
    sys.exit()
    
  rospy.init_node("generate_images")

  # Parse output format
  out_format = "ppm"
  if len(sys.argv) >= 3:
    out_format = sys.argv[2]
  print "Output format: %s" % out_format

  # Parse experiment name / input bag file
  bag_name = sys.argv[1]
  if not os.path.exists(bag_name):
    print "ERROR Bag does not exist: "  + bag_name

  experiment_name = infer_experiment_name(bag_name)
  if experiment_name is None:
    print "WARN: could not infer experiment name"
    experiment_name = os.path.basename(bag_name)
  print "Experiment: %s" % experiment_name

  print "Opening bag %s" % bag_name
  orig_bag = rosbag.Bag(bag_name, 'r')

  # Generate output dir and filename
  dirname = os.path.dirname(sys.argv[1])
  out_dir = os.path.join(dirname, out_format, experiment_name) #+"_"+out_format)
  if not os.path.exists(out_dir):
    print "Creating output directory %s" % out_dir
    os.makedirs(out_dir)

  img_files = []
  timestamps = []

  ##############################
  # Now iterate over the bag
  i = 1
  for topic, msg, t in orig_bag.read_messages(topics=topics):
    try:
      cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except CvBridgeError, e:
      print e
      continue

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(10)
    
    # write out image
    img_file = experiment_name+"_" + "%05d" % i + "."+out_format
    img_files.append(img_file)
    path = os.path.join(out_dir, img_file)
    print " writing %s" % path
    cv2.imwrite(path, cv_image)

    timestamps.append(t)

    i += 1

    #print str(msg)[:100]
  #info_pub = rospy.Publisher('/camera/rgb/camera_info', type(info), queue_size=10)

  orig_bag.close()

  print "Writing BMF file (required for Ochs segmentation)"
  with open(os.path.join(out_dir, experiment_name+".bmf"), "w") as f:
    f.write("%d 1\n" % len(img_files) )
    for imf in img_files:
      f.write(imf + "\n")
  

  print "Writing time file (required for evaluating segmentation)"
  with open(os.path.join(out_dir, experiment_name+".time.txt"), "w") as f:
    f.write("#time stamps\n")
    for t in timestamps:
      f.write(str(t)+"\n")
