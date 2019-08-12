#!/usr/bin/python

import dubins
import csv
import numpy as np
import matplotlib.pyplot as plt

csv_file = "set_dubins.csv"
q0 = (10., 10., 0) # Starting Pose
q1 = (20., 20., 3.14) # Finishing pose

turning_radius = 5.0

step_size = 2.0

path = dubins.shortest_path(q0, q1, turning_radius)

configurations, _ = path.sample_many(step_size)

configurations = np.array(configurations)

points = configurations[:, :2]
np.savetxt(csv_file, points, delimiter=',', fmt='%.2f')

plt.figure()
plt.plot(configurations[:,0], configurations[:,1])
plt.show()
