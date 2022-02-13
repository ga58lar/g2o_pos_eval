#!/usr/bin/env python3

from cProfile import label
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from mpl_toolkits.mplot3d import Axes3D
import osr
from pyproj import Proj
import sys
import os

# navsat: lat, lon, alt
# g2o: x,y,z


def generate_g2o_xyz(filedir_g2o, filedir_utm):
    g2o_array = np.array([[0, 0, 0]])
    file_g2o = open(filedir_g2o, "r")
    file_utm = open(filedir_utm, "r")
    for line in file_utm:
        lat, lon, alt = line.split()
    file_utm.close()

    for line in file_g2o:
        try:
            data_type, id, x, y, z, qx, qy, qz, qw = line.split()
            if int(id) == 0:
                g2o_array = np.array(
                    [[float(x)+float(lat), float(y)+float(lon), float(z)+float(alt)]])
            else:
                g2o_array = np.append(
                    g2o_array, [[float(x)+float(lat), float(y)+float(lon), float(z)+float(alt)]], axis=0)
        except:
            try:
                data_type, id = line.split()
            except:
                break

    file_g2o.close()
    return g2o_array


def generate_GNSS_xyz(filedir):
    file = open(filedir, "r")
    csv = pd.read_csv(filedir)
    lla = csv[['field.latitude', 'field.longitude',
               'field.altitude']].to_numpy()

    def get_utm_zone(longitude):
        return (int(1+(longitude+180.0)/6.0))

    def is_northern(latitude):
        if latitude < 0.0:
            return 0
        else:
            return 1

    utm_xyz = np.array([[0, 0, 0]])
    utm_cos = osr.SpatialReference()
    utm_cos.SetWellKnownGeogCS("WGS84")
    for k in range(len(lla)):
        utm_cos.SetUTM(get_utm_zone(lla[k, 1]), is_northern(lla[k, 0]))

        wgs84_cos = utm_cos.CloneGeogCS()
        wgs84_utm_trans = osr.CoordinateTransformation(wgs84_cos, utm_cos)
        utm_trans = wgs84_utm_trans.TransformPoint(
            lla[k, 0], lla[k, 1], lla[k, 2])
        if k == 0:
            utm_xyz = [[utm_trans[0], utm_trans[1], utm_trans[2]]]
        else:
            utm_xyz = np.append(utm_xyz, np.array(
                [[utm_trans[0], utm_trans[1], utm_trans[2]]]), axis=0)

    # stated below is a secon approch with a pre-programmed function
    # the result is exactly the same, but it is not general applicable,
    # as I have to pre-define the zone and hemisphere

    # myProj = Proj(
    #    "+proj=utm +zone=16, +north, +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    # UTMx, UTMy = myProj(lon, lat)
    # UTM_XY = np.array([[0, 0]])
    # for k in range(len(lla)):
    #    UTMx, UTMy = myProj(lla[k, 1], lla[k, 0])
    #    if k == 0:
    #        UTM_XY = [[UTMx, UTMy]]
    #    else:
    #        UTM_XY = np.append(UTM_XY, [[UTMx, UTMy]], axis=0)
    # UTM_XYZ = np.hstack((UTM_XY, lla[:, 2].reshape(len(lla[:, 2]), 1)))

    file.close()
    gnss_array = utm_xyz
    return gnss_array


def calc_dist_xyz(g2o_array, gnss_array):
    # eucledian dist between g2o and GNSS
    dist_min = np.array([0])

    for k in range(len(g2o_array)):
        g2o_marker = g2o_array[k, :]
        dist = np.sqrt(np.sum((gnss_array-g2o_marker)**2, axis=1))
        dist_min = np.append(dist_min, min(dist))

    dist_min = np.delete(dist_min, 0, 0)

    print(max(dist_min))
    return dist_min


def plot_msgs(g2o_array, gnss_array, dist_min, dim):
    g2o_x = g2o_array[:, 0]
    g2o_y = g2o_array[:, 1]
    g2o_z = g2o_array[:, 2]

    gnss_x = gnss_array[:, 0]
    gnss_y = gnss_array[:, 1]
    gnss_z = gnss_array[:, 2]

    if dim == 2:
        fig, ax = plt.subplots(1, 2, sharex=True, sharey=True)
        #ax[0].plot(g2o_x, g2o_y, label="g2o")
        ax[0].plot(gnss_x, gnss_y, 'y', label="gnss", linewidth=1)
        ax[0].scatter(g2o_x, g2o_y, s=15, label="g2o")

        dg2o_x = 0.05*(g2o_x.max()-g2o_x.min())
        dg2o_y = 0.05*(g2o_y.max()-g2o_y.min())

        ax[0].set_xlim(g2o_x.min()-dg2o_x, g2o_x.max()+dg2o_x)
        ax[0].set_ylim(g2o_y.min()-dg2o_y, g2o_y.max()+dg2o_y)
        ax[0].set_xlabel('UTM easting')
        ax[0].set_ylabel('UTM northing')
        ax[0].legend(loc="upper left")
        ax[0].set_aspect('equal')

        points = np.array([g2o_x, g2o_y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        norm = plt.Normalize(dist_min.min(), dist_min.max())
        lc = LineCollection(segments, cmap='viridis', norm=norm)
        lc.set_array(dist_min)
        # lc.set_linewidth(2)
        line = ax[1].add_collection(lc)
        fig.colorbar(line, ax=ax[1])

        ax[1].set_xlim(g2o_x.min()-dg2o_x, g2o_x.max()+dg2o_x)
        ax[1].set_ylim(g2o_y.min()-dg2o_y, g2o_y.max()+dg2o_y)
        ax[1].set_xlabel('UTM easting')
        ax[1].set_ylabel('UTM northing')
        ax[1].set_aspect('equal')

    elif dim == 3:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(g2o_x, g2o_y, g2o_z)
        # ax.set_zlim3d([0, 10])
    else:
        print("Error: Wrong dimension! Value must be '2' or '3'")

    # plt.savefig('figure.png')
    plt.show()


if __name__ == "__main__":
    file_path = []
    for k in range(1, 4):
        if os.path.isabs(sys.argv[k]):
            file_path.append(str(sys.argv[k]))
        else:
            file_path.append(str(os.path.join(os.path.abspath(
                os.path.dirname(__file__)), sys.argv[k])))

    g2o_xyz = generate_g2o_xyz(str(file_path[0]), str(file_path[1]))
    gnss_xyz = generate_GNSS_xyz(str(file_path[2]))
    dist_min = calc_dist_xyz(g2o_xyz, gnss_xyz)
    plot_msgs(g2o_xyz, gnss_xyz, dist_min, int(sys.argv[4]))
