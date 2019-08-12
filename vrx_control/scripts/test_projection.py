import shapely
import shapely.geometry
import matplotlib.pyplot as plt
import numpy as np

w  = (40., 40.)
t = (10., 10.)

p = (50., 60.)

line = shapely.geometry.LineString([w,t])
tp = np.array(line.interpolate(line.project(shapely.geometry.Point(p))))
target_point = np.array(tp)
line_array = np.array(line)
plt.figure()
plt.plot(line_array[:,0], line_array[:,1], label='WP Path')
plt.scatter(p[0], p[1], label='Vessel')
plt.scatter(target_point[0], target_point[1], label='Target Point')
plt.show()
