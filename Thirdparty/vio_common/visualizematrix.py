import numpy as np
import matplotlib.pyplot as plt
import time
import os
import scipy.sparse as sps

def visualizematrix(filename):
  if not os.path.isfile(filename):
    print('Unable to open %s' % filename )
    filename = 'test/hessian_0.txt'
  A = np.loadtxt(filename)
  M = sps.csr_matrix(A)
  plt.spy(M)
  plt.show()
  time.sleep(1.0)
  plt.close()

def main():
  filename = input('Which matrix text file do you want to visualize?')
  visualizematrix(filename)

if __name__ == "__main__":
  main()
