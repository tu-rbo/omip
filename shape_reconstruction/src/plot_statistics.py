#!/usr/bin/python
##############
# View the statistics for a single experiment (bag file)
#
# Statistics are plotted separately, i.e. one window per object per variant,
# showing precision and recall.


import numpy as np
import matplotlib.pylab as plt
import matplotlib
matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

import sys
import os
import os.path


def plot_statistics(folder):
  for f in os.listdir(folder):
    if f[-3:] != "txt":
      continue
    filename = os.path.join(folder, f)
      
    data = np.genfromtxt(filename, dtype=float, delimiter=' ', names=True)

    precision = data['tp'] / (data['tp']+data['fp'])
    recall = data['tp'] / (data['tp']+data['fn'])
    seg_acc = data['tp'] / (data['tp']+data['fp']+data['fn'])
      
    time = data['time'] - data['time'][0]
    
    # clean up
    precision[np.where(np.isnan(precision))] = 0.
    recall[np.where(np.isnan(recall))] = 0.
    seg_acc[np.where(np.isnan(seg_acc))] = 0.

    plt.figure(figsize=(7.5,5))
    for y in np.arange(0.0, 1.1, 0.2):  
      plt.plot(time, [y] * len(time), "--", lw=0.5, color="black", alpha=0.3)  
  
    plt.plot(time, seg_acc, "k", label="Segmentation Accuracy", lw=3.0)
    plt.plot(time, precision, "b", label="Precision", lw=2.0, ls="--")
    plt.plot(time, recall, "r", label="Recall", lw=2.0, ls="--")
    
    plt.xlim(0, data['time'][-1] - data['time'][0])
    plt.ylim(-0.1, 1.1)
    plt.xlabel("time")
    
    plt.title(os.path.basename(filename[:-3]))
    plt.legend(loc=4)
    
    img_path = os.path.join(folder, f[:-3]+"pdf")
    print ("Saving image at %s" % img_path)
    plt.savefig(img_path, bbox_inches="tight")

if __name__ == "__main__":
  if len(sys.argv) != 2:
    print "Usage: plot_statistics.py <experiment folder>"
    sys.exit()

  print ("Starting plot statistics")
  plot_statistics(sys.argv[1])

  plt.show()

