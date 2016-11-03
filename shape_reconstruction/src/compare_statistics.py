#!/usr/bin/python
#################################
# Plot the results from different experiments in one coherent plot.
#
# What you will usually do:
#  rosrun shape_reconstruction compare_statistics.py  \
#       <path/to/data>/results_centroid_and_neighbors' '<path/to/data>/results_centroid_and_neighbors_wo_tracker'  <path/to/data>/ochs_results
# Where results_centroid_and_neighbors is the result of the pipeline
# and results_centroid_and_neighbors_wo_tracker contains results with
# the shape-tracker being off (optional), and ochs_results contains
# results from the Ochs baseline (one subfolder per experiment, each
# containing a folder "DenseSegmentation" with the *_dense.ppm result files.
#
# WARN: this does not actually *generate* the statistics! You have to
# run "generate_statistics" script to do this first.

import numpy as np
import matplotlib.pylab as plt
import matplotlib
matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

import sys
import os
import os.path

import re
import shutil

## Title and settings

fscore_beta = 0.5

full_pattern = '(segment_ext_d_and_c_rb)(([0-9]+))'
wo_st_pattern= '(segment_no_st_ext_d_and_c_rb)(([0-9]+))'

# used for name replacement
wo_st_template = 'segment_no_st_ext_d_and_c_rb%d.txt'

variant2labelAndStyle = {
          # (regex, plotting params, legend order)
          full_pattern: ({'label': "Full", 'lw': 3.0, 'ls': "-", 'color': 'k'}, 0),
          wo_st_pattern: ({'label': "Full w/o Shape Tracking", 'lw': 3.0, 'ls': "--", 'color': 'k', 'alpha': 0.5}, 1),
          '(segment_naive_depth_rb)([0-9]+)': ({'label': 'Shape-based (depth only)', 'lw': 1.0, 'ls': "--", 'color': 'b'}, 4),
          '(segment_naive_color_rb)([0-9]+)': ({'label': 'Shape-based (color only)', 'lw': 1.0, 'ls': "--", 'color': 'r'}, 5),
          '(segment_depth_rb)([0-9]+)': ({'label': 'Motion (depth only)', 'lw': 1.5, 'ls': "-", 'color': 'b'}, 2),
          '(segment_color_rb)([0-9]+)': ({'label': 'Motion (color only)', 'lw': 1.5, 'ls': "-", 'color': 'r'}, 3),
          '(ochs)': ({'label': 'Ochs et al.', 'lw': 3.0, 'ls': ":", 'color': 'g'}, 6), 
          #'segment_ext_c_rb([0-9]+)': (''),
          #'segment_ext_d_rb([0-9]+)': (''),
         }

folderElems2Title = {
  ('box'): 'Box',
  ('globe'): 'Globe',
  ('metalcase'): '2 Bodies: Metal case',
  ('statue'): '2 Bodies: Statue',
  ('head1'): 'Head',
  ('robotDrawer'): 'Drawer',
  ('redthing'): 'Red shape',
  ('ks_drawer_full_gt_drawer'): 'Cabinet: Drawer',
  ('ks_drawer_full_gt_base'): 'Cabinet: Frame',
  ('ks_laptop_full_gt_lid'): 'Laptop: Lid',
  ('ks_laptop_full_gt_base'): 'Laptop: Base',
}

folderElems2Order = [
  ('robotDrawer'),
  ('box'),
  ('globe'),
  ('head1'),
  ('metalcase'),
  ('statue'),
  ('redthing'),
  ('ks_drawer_full_gt_base'),
  ('ks_drawer_full_gt_drawer'),
  ('ks_laptop_full_gt_base'),
  ('ks_laptop_full_gt_lid'),
]
#########################

def compare_statistics(folder, show_fscore=True, multiple_figures=True):
  stats_map = collect_stats(folder)
    
  no_exp = len(stats_map)

  figures = []
  if multiple_figures:
    for i in range(3*no_exp):
      figures.append(plt.figure(figsize=(2,2)))
  else:
    #adjust figure size to #experiments
    fig = plt.figure(figsize=(10,3*no_exp))
    figures.append(fig)
  
  #print stats_map.keys()
  #if no_exp == len(folderElems2Order):
  stats_map_sorted = []
  for i, name in enumerate(folderElems2Order):
    for k,v in stats_map.items():
      if name in k:
        stats_map_sorted.append ( (k,v) )
        break
  print [ k for k,v in stats_map_sorted ]
  #else:
    #stats_map_sorted = sorted(stats_map.items(), key=lambda x: x[0])
  
  cols = 2
  if show_fscore:
    cols = 3
  
  #print stats_map_sorted
  figure_names = []
  figure_rel_idx = []
  for i, (exp_name, variant_stats) in enumerate( stats_map_sorted ):
    if multiple_figures:
      fig = figures[cols*i]
      ax_prec = figures[cols*i].add_subplot(1,1,1)
      ax_rec =  figures[cols*i+1].add_subplot(1,1,1)
      ax_fscore = None
      if show_fscore:
        ax_fscore = figures[cols*i+2].add_subplot(1,1,1)
    else:
      ax_prec = plt.subplot(no_exp, cols, cols*i+1)
      ax_rec = plt.subplot(no_exp, cols, cols*i+2)
      ax_fscore = None
      if show_fscore:
        ax_fscore = plt.subplot(no_exp, cols, cols*i+3)
          
    plot_legend = i==0 and not multiple_figures
    plot_exp_name = not multiple_figures
    plot_stats_for_exp(exp_name, variant_stats, fig, ax_prec, ax_rec, ax_fscore, plot_legend, plot_exp_name)

    for c in range(cols):
      figure_names.append(exp_name)
      figure_rel_idx.append(c)

    if i == 0: # or multiple_figures:
      ax_prec.set_title("\\textrm{Precision}")
      ax_rec.set_title("\\textrm{Recall}")
      if show_fscore:
        ax_fscore.set_title("\\textrm{F$_{%.1f}$ score}" % fscore_beta)

    if i == len(stats_map_sorted)-1: # or multiple_figures:
      time_label = "\\textrm{time (sec)}"
      ax_prec.set_xlabel(time_label)
      ax_rec.set_xlabel(time_label)
      if show_fscore:
        ax_fscore.set_xlabel(time_label)

  for i, (fig, fig_idx) in enumerate(zip(figures, figure_rel_idx)):
    if multiple_figures:
      img_path = os.path.join(folder, "compare_results_%s_%d.pdf" % (figure_names[i], fig_idx)) 
    else:
      img_path = os.path.join(folder, "compare_results.pdf")
    print ("Saving image at %s" % img_path)
    fig.savefig(img_path, bbox_inches="tight")
      
def collect_stats(folder):
  stats_map = {}
  
  for f in os.listdir(folder):
    fld = os.path.join(folder, f)
    real_fld = os.path.realpath(fld)
    print real_fld
    if not os.path.isdir(real_fld):
      #print "  skipping %s" % real_fld
      continue
    
    stats_map[f] = collect_stats_for_exp(real_fld)
  return stats_map
  
def collect_stats_for_exp(folder):
  
  variant_map = {}
  
  for f in os.listdir(folder):
    if f[-3:] != "txt":
      continue
    filename = os.path.join(folder, f)
      
    data = np.genfromtxt(filename, dtype=float, delimiter=' ', names=True)
  
    precision = data['tp'] / (data['tp']+data['fp'])
    recall = data['tp'] / (data['tp']+data['fn'])
    seg_acc = data['tp'] / (data['tp']+data['fp']+data['fn'])
    #fscore = 2* (precision*recall) / (precision + recall)
    beta = fscore_beta
    fscore = (1+beta**2)* (precision*recall) / (beta**2 * precision + recall)
    time = data['time'] - data['time'][0]
    
    freq = np.mean(data['time'][1:] - data['time'][:-1])
    
    #rel_time = (time-data['time'][0]) / time[-1]
  
    # add t=0 -> 0
    precision = np.hstack( [ [0.], precision ]  )
    recall = np.hstack( [ [0.], recall ] )
    fscore = np.hstack( [ [0.], fscore ] )
    time = np.hstack( [ time, [time[-1] + freq] ]  )
    #rel_time = np.hstack( [ [0., rel_time] ] )
  
    # clean up
    precision[np.where(np.isnan(precision))] = 0.
    recall[np.where(np.isnan(recall))] = 0.
    seg_acc[np.where(np.isnan(seg_acc))] = 0.
    fscore[np.where(np.isnan(fscore))] = 0.

    filetitle = f.split(".")[-2]
    
    variant_map[filetitle] = {}
    variant_map[filetitle]['precision'] = precision
    variant_map[filetitle]['recall'] = recall
    variant_map[filetitle]['seg_acc'] = seg_acc
    variant_map[filetitle]['fscore'] = fscore
    variant_map[filetitle]['time'] = time
    #variant_map[filetitle]['rel_time'] = rel_time
    
  return variant_map

def plot_stats_for_exp(exp_name, variant_stats, fig, ax_prec, ax_rec, ax_fscore=None, plot_legend=True, plot_exp_name=True):
  
  print exp_name
  
  processed_variants = []
  
  lines = []
  labels = []
  order = []
  
  for variant, stats in variant_stats.items():
    dct = None
    print " "+variant
    
    try:
      for var_re, (dct_, order_) in variant2labelAndStyle.items():
        res = re.match(var_re, variant)
        if res is not None:
          print res.group(1)
          if res.group(1) in processed_variants:
            print "ERROR: More than one rigid body ground truth found - currently not supported. Delete the _rb* files that do not correspond to the ground truth and re-run"
            print " experiment: %s" % exp_name
            print " variant: %s" % variant
            raise Exception()
          dct = dct_
          processed_variants.append (res.group(1) )
          order.append(order_)
    except Exception, e:
      #raise e
      pass
    
    if dct is None:
      print "   WARN: Variant unsupported %s " % variant
      continue
    
    for y in np.arange(0.0, 1.1, 0.2):  
      for ax in [ax_prec, ax_rec, ax_fscore]:
        if ax_fscore is None:
          continue
        ax.plot(stats['time'], [y] * len(stats['time']), "-.", lw=0.3, color="black", alpha=0.1)  
  
    #ax_segacc.plot(stats['rel_time'], seg_acc, "k", label="Segmentation Accuracy", lw=3.0)
    l1 = ax_prec.plot(stats['time'], stats['precision'], **dct)
    ax_rec.plot(stats['time'], stats['recall'], **dct)
    
    if ax_fscore is not None:
      ax_fscore.plot(stats['time'], stats['fscore'], **dct)
    
    #print l1
    lines.extend(l1)
    labels.append( "\\textrm{%s}" % dct['label'])
    
    if plot_exp_name:
      ax_prec.set_ylabel(exp_name[:15])
      for elem, title in folderElems2Title.items():
        if elem in exp_name:
          ax_prec.set_ylabel(title)
    
    for ax in [ax_prec, ax_rec, ax_fscore]:
      if ax is None:
        continue
      ax.set_xlim(0, stats['time'][-1])
      ax.set_ylim(-0.1, 1.1)
    
    # Put a legend to the right of the current axis
    
  if plot_legend:
    lines_sorted, labels_sorted, ordered = zip (* sorted( zip(lines, labels, order), key=lambda x: x[2]) )
    
    #box = ax.get_position()
    #ax_rec.set_position([box.x0, box.y0, box.width * 0.8, box.height])
    fig.legend(lines_sorted, labels_sorted, 'lower center', ncol=4,prop={'size':12}) #, bbox_to_anchor=(1, 0.5))
    #fig.legend(loc=4)
    
def cherry_pick_full_wo_shape_tracker(wo_src, with_dst):
  print "Cherry-picking FULL result from %s" % wo_src

  exp2res = {}

  # collect result txts
  for d in os.listdir(wo_src):
    exp_dir = os.path.join(wo_src, d)
    if not os.path.isdir(exp_dir):
      continue
    for f in os.listdir(exp_dir):
      res = re.match(full_pattern, f)
      if res is not None:
        exp_name = None
        for elem, _ in folderElems2Title.items():
          if elem in d:
            exp_name = elem
        if exp_name is None:
          print "WARNING: source experiment unknown: %s" % d
        else:
          exp2res[exp_name] = (os.path.join(exp_dir, f), res.group(2))
        
  #print exp2res
  
  # copy to destination
  for d in os.listdir(with_dst):
    exp_dir = os.path.join(with_dst, d)
    if not os.path.isdir(exp_dir):
      continue
      
    for elem, _ in folderElems2Title.items():
      if elem in d:
        exp_name = elem
    if exp_name is None:
      print "WARNING: destination experiment unknown: %s" % d
      continue
      
    if exp_name not in exp2res:
      print "ERROR: source %s does not contain experiment %s" % (with_dst, d)
      continue
    
    fres, fres_rb = exp2res[exp_name]
    fdes = os.path.join(exp_dir, wo_st_template % int(fres_rb))
    print " Copying %s to %s" % (fres, fdes)

    if os.path.exists(fdes):
      try:
        os.remove(fdes)
      except:
        print "Cannot move %s because file exists and cannot be deleted" % fdes
        continue
    
    try:
      shutil.copyfile(fres, fdes)
    except Exception, e:
      print e
      print "Unknown error copying file %s" % fres
      continue      
      
  

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print "Usage: compare_statistics.py <experiment meta folder [contains ONLY folders that include statistics text files (and result bags)]> [<experiment meta folder w/o shape tracking]"
    sys.exit()

  if len(sys.argv) > 2:
    wst_folder = sys.argv[2]
    cherry_pick_full_wo_shape_tracker(sys.argv[2], sys.argv[1])

  print ("Starting compare statistics")
  multiple_figures=True
  compare_statistics(sys.argv[1],multiple_figures=multiple_figures)

  if not multiple_figures:
    plt.show()

