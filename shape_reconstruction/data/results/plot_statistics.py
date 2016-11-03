import numpy as np
import matplotlib.pylab as plt

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
    plt.plot(time, seg_acc, "k", label="seg_acc", lw=3.0)
    plt.plot(time, precision, "b", label="precision", lw=2.0, ls="--")
    plt.plot(time, recall, "r", label="recall", lw=2.0, ls="--")
    
    plt.xlim(0, data['time'][-1] - data['time'][0])
    plt.ylim(-0.1, 1.1)
    plt.xlabel("time")
    
    plt.title(os.path.basename(filename[:-3]))
    plt.legend(loc=4)
  

if __name__ == "__main__":
  if len(sys.argv) != 2:
    print "Usage: plot_statistics.py <experiment folder>"
    sys.exit()

  plot_statistics(sys.argv[1])

  plt.show()

