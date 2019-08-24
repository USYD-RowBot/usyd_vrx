from __future__ import division
import shapely
import shapely.geometry
import numpy as np
#import matplotlib.pyplot as plt

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def to_tuple(self):
        return (self.x, self.y)

def get_point_on_line(m, p0, x):
    return Point(x, m*(x - p0.x)+p0.y)

def find_normal_line(w, t, dist=1):
    if w.y == t.y:  # w,t is horizontal makes normal vertical
        a = Point(t.x, t.y-dist)
        b = Point(t.x, t.y+dist)
        # print("Horizontal line")
    else:
        m = -(t.x - w.x)/(t.y - w.y)
        a = get_point_on_line(m, t, t.x-dist)
        b = get_point_on_line(m, t, t.x+dist)
        # print(a.to_tuple())
        # print(b.to_tuple())
    return (a, b)

def get_gradient(line):
    coords = np.array(line.coords)
    m = (coords[1, 1] - coords[0, 1])/(coords[1, 0] - coords[0, 0])
    return m

def waypoint_passed(w, t, p, dist=1000):
    w = Point(w[0], w[1])
    t = Point(t[0], t[1])
    p = Point(p[0], p[1])
    a,b = find_normal_line(w,t,dist=dist)
    lab = shapely.geometry.LineString([a.to_tuple(), b.to_tuple()])
    lwp = shapely.geometry.LineString([w.to_tuple(), p.to_tuple()])
    lwt = shapely.geometry.LineString([w.to_tuple(), t.to_tuple()])
    if lwp.crosses(lab):
        return True
    else:
        return False

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    w = Point(2., 0.)
    t = Point(-10., 10.)
    p = Point(5., 6.)
    a, b = find_normal_line(w, t)
    lab = shapely.geometry.LineString([a.to_tuple(), b.to_tuple()])
    lwp = shapely.geometry.LineString([w.to_tuple(), p.to_tuple()])
    lwt = shapely.geometry.LineString([w.to_tuple(), t.to_tuple()])
    lab = np.array(lab.coords)
    lwp = np.array(lwp.coords)
    lwt = np.array(lwt.coords)
    print("Normal Line", -1/get_gradient(lab))
    print("Waypoint Line", get_gradient(lwt))
    plt.figure()
    plt.plot(lab[:,0], lab[:,1], label='lAB')
    plt.plot(lwp[:,0], lwp[:,1], label='lWP')
    plt.plot(lwt[:,0], lwt[:,1], label='lWT')
    plt.xlim([0, 20])
    plt.ylim([0, 20])
    plt.show()
