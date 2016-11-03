#!/usr/bin/python

#######################################################
# Generate the statistics for further processing
# (required by compare_statistics)
# 
# It also invokes the plot_statistics script if you pass a single result file.
#
# If you put the experiment into the ground_truth_bags dictionary 
# the correct ground truth bag file is automatically inferred
# from the passed result bag name
#
# What you usually want:
#  for one result:
#    rosrun shape_reconstruction generate_statistics.py \
#        <path/to/result_bags_folder>/box_pc_imgs_8-13_11-27-22.bag
#
#  for one result with custom ground truth
#    rosrun shape_reconstruction generate_statistics.py \
#        <path/to/result_bags_folder>/box_pc_imgs_8-13_11-27-22.bag
#        <path/to/ground_truth_bags_folder>/box_pc_imgs_gt.bag
#
#  for an entire folder:
#    rosrun shape_reconstruction generate_statistics.py \
#         <path/to/result_bags_folder>


import rospy
import rospkg

import numpy as np
import matplotlib.pylab as plt
import matplotlib
matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

import sys
import os
import os.path
import shutil

import re
from plot_statistics import plot_statistics
from compare_statistics import compare_statistics

from subprocess import check_call, CalledProcessError

rp = rospkg.RosPack()
data_path = os.path.join(rp.get_path("shape_reconstruction"), "../../../data")

ground_truth_bags = {
  'bodies9': ["ground_truth/bodies9_gt_metalcase.bag", "ground_truth/bodies9_gt_statue.bag",],
  'box': "ground_truth/box_gt_2-17-14-13.bag",
  'doorOpenClose': "ground_truth/doorOpenClose_full_2-19_20-53-28_gt.bag",
  'drawer_pc': "ground_truth/drawer_pc_imgs_2-19_20-6-22_gt.bag",
  'globe_full': "ground_truth/globe_full_2-19_21-29-15_gt.bag",
  'robotDrawerOpen': "ground_truth/robotDrawerOpen_full_gt.bag",
  'head1_full': "ground_truth/head1_full_2015-08-10-11-57-55_gt.bag",
  'redthing': -1, # means there are different bags for with and without shape tracker (hacky)
  'ks_drawer': ["ground_truth/ks_drawer_full_gt_drawer.bag", "ground_truth/ks_drawer_full_gt_base.bag"],
  'ks_laptop': ["ground_truth/ks_laptop_full_gt_lid.bag", "ground_truth/ks_laptop_full_gt_base.bag"],
}

ground_truth_bags_different_for_w_wo_tracker = {
  'redthing': { "with": "ground_truth/redthing5_full_gt_shapetracker.bag", 
                "wo": "ground_truth/redthing5_full_gt_WOshapetracker.bag"},
}

ochs_result_folders = {}
ochs_result_root = os.path.join(data_path, "ochs_results")
for k,v in ground_truth_bags.items():
  ochs_result_folders[k] = os.path.join(ochs_result_root, k, "DenseSegmentation")
  
print ochs_result_folders

def infer_ground_truth_bag(result_bag):
  for k, v in ground_truth_bags.iteritems():
    if result_bag.find(k) != -1:
      print ("Infering ground truth bag: %s" % v)
      ground_truth_bag = v
      break

  return ground_truth_bag

def infer_experiment_name(bag_name):
  for k in ground_truth_bags.keys():
    if bag_name.find(k) != -1:
      print ("Inferred experiment name: %s" % k)
      return k

  return None
  


def create_statistic_files_and_plot(result_bag, ground_truth_bag, ochs_folder=""):
  cmd = "rosrun shape_reconstruction shape_reconstruction_statistics  %s %s %s" % \
    (result_bag, os.path.join(data_path, ground_truth_bag), ochs_folder)
  print (cmd)
  check_call(cmd, shell=True)

  print "----------------------"
  print ("Starting plot statistics")

  folders = result_bag.split("/")

  folder_path = os.path.join("/".join(folders[:-1]), folders[-1].split(".")[0])
  print ("Looking for stats in folder %s" % folder_path)

  if not os.path.isdir(folder_path):
    print "ERROR: Stats dir was not generated"
    return

  plot_statistics(folder_path)  

  return folder_path

def keep_best_rb_stat(stats_folder):
  rbs = {}
  for f in os.listdir(stats_folder):
    res = re.match("segment_ext_d_and_c_rb([0-9]+)\.txt", f)
    if res is not None:
      data = np.genfromtxt(os.path.join(stats_folder, f), dtype=float, delimiter=' ', names=True)
      seg_acc = data['tp'] / (data['tp']+data['fp']+data['fn'])

      # weigh acc by time
      seg_acc_mean = np.arange(1., seg_acc.shape[0]+1).dot(seg_acc)
      rb = int(res.group(1))
      print "  rigid body %d has %.5f" % (rb, seg_acc_mean)
      rbs[rb] = (f, seg_acc_mean)
      continue

  rb = max(rbs.items(), key=lambda x: x[1][1])
  print rb[0]
  
  print "Best matching rigid body: %d" % rb[0]

  moveto = os.path.join(stats_folder, "not_matching")
  try:
    os.makedirs(moveto)
    print "Creating %s" % moveto
  except:
    print "Directory %s exists" % moveto

  ptn = re.compile("[a-zA-Z_]+_rb%d\.[a-zA-Z]{3}" % rb[0])
  ptn_ochs = re.compile("ochs")
  
  for f in os.listdir(stats_folder):
    if ptn.match(f) is None and ptn_ochs.match(f) is None and f != "not_matching":
      print "Moving %s" % f
      fp = os.path.join(stats_folder, f)
      
      fdes = os.path.join(moveto, f)
      if os.path.exists(fdes):
        try: 
          os.remove(fdes)
        except:
          print "Cannot move %s because file exists and cannot be deleted" % fdes
          continue
      
      try:
        shutil.move(fp , moveto)
      except Exception, e:
        print e
        print "Unknown error moving file %s" % fp
        continue

  return rbs
  
def generate_statistics(result_bag, sys):
  print ("Starting statistics generation for %s " % result_bag)
  
  ground_truth_bag = None
  if len(sys.argv) == 3:
    ground_truth_bag = sys.argv[2]
    
  if ground_truth_bag is None:
    ground_truth_bag = infer_ground_truth_bag(result_bag)
    if ground_truth_bag is None:
      print ("Unable to infer ground truth bag - aborting")
      import sys
      sys.exit()

  if type(ground_truth_bag) == list:
    print "   Several ground truth bags available:\n%s " % "\n".join(map (lambda x: "  %d: %s" % (x[0], x[1]), enumerate(ground_truth_bag)))
  elif ground_truth_bag == -1:
    experiment_name = infer_experiment_name(result_bag)
    assert ( experiment_name in ground_truth_bags_different_for_w_wo_tracker)
    print "   %s gets special treatment - we have different ground truths for with and w/o tracker" % experiment_name
    elems = os.path.split(result_bag)
    elems2 = os.path.split(elems[0])
    if "wo" in ("%s/%s" % (elems2[-1], elems[-1])).lower():
      ground_truth_bag = ground_truth_bags_different_for_w_wo_tracker[experiment_name]["wo"]
    else:
      ground_truth_bag = ground_truth_bags_different_for_w_wo_tracker[experiment_name]["with"]
    print "   Selected %s" % ground_truth_bag
    ground_truth_bag = [ground_truth_bag]
    
  else:
    ground_truth_bag = [ground_truth_bag]

  for gt_bag in ground_truth_bag:
    experiment_name = infer_experiment_name(gt_bag)
    print "Bag: %s" % gt_bag
    print "Experiment name: %s" % experiment_name

    ochs_folder = ""
    try:
      ochs_folder = ochs_result_folders[experiment_name]
    except:
      print "ERROR could not find ochs folder for experiment %s" % experiment_name
      raw_input("(hit enter to ignore)")
    
    stats_folder = create_statistic_files_and_plot(result_bag, gt_bag, ochs_folder)
      
    if len(ground_truth_bag) > 1:
      keep_best_rb_stat(stats_folder)
      
      import re
      new_folder_name = stats_folder + "_" + re.split("\.|/", gt_bag)[-2]
      os.rename(stats_folder, new_folder_name)  

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print "Usage: generate_statistics.py  <result bag file/folder containing bag files>[ <ground_truth>]"
    sys.exit()

  path = sys.argv[1]
  
  if os.path.isdir(path):
    print "Processing all bags in directory"
    
    if len(sys.argv) > 2:
      print "You cannot pass ground truth if you process an entire directory"
      sys.exit()
    
    for f in os.listdir(path):
      if f[-4:] != "yaml":
        continue
        
      bag_file = f[:-4]+"bag"
      generate_statistics(os.path.join(path, bag_file), sys)
      
    #compare_statistics(path)
      
  else:
    generate_statistics(path, sys)
    plt.show()




