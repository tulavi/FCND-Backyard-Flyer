import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd

df = pd.read_csv('Local Velocity.csv', sep=',', header=None)
df2 = pd.read_csv('Local Velocity err0.3.csv', sep=',', header=None)

mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')

x1 = df[0]
y1 = df[1]
z1 = -1.0*df[2]

x2 = df2[0]
y2 = df2[1]
z2 = -1.0*df2[2]

ax.plot(x1, y1, z1, label='drone trajectory err 0.1', color='blue')
#ax.plot(x2, y2, z2, label='drone trajectory err 0.3', color='red')
ax.legend()

plt.show()