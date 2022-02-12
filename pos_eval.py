#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import osr
from pyproj import Proj

# navsat: lat, lon, alt
# g2o: x,y,z

# lidarslam_ros2: nav_msgs/Path
# hdl_graph_slam: /hdl_graph_slam/markers


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
                    [[float(x)+float(lat), float(y)+float(lon), float(z)]])

            else:
                g2o_array = np.append(
                    g2o_array, [[float(x)+float(lat), float(y)+float(lon), float(z)]], axis=0)

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


# def calc_dist_xyz():
    # eucledian dist between g2o and GNSS
    # ausgehend von g2o, da deutlich weniger Punkte


def plot_msgs(g2o_array, gnss_array, dim):
    g2o_x = g2o_array[:, 0]
    g2o_y = g2o_array[:, 1]
    g2o_z = g2o_array[:, 2]

    gnss_x = gnss_array[:, 0]
    gnss_y = gnss_array[:, 1]
    gnss_z = gnss_array[:, 2]

    if dim == 2:
        fig, ax = plt.subplots()
        ax.plot(g2o_x, g2o_y, label="g2o")
        ax.plot(gnss_x, gnss_y, label="gnss")
        ax.legend(loc="upper left")

        ax.set_aspect('equal')

    elif dim == 3:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(g2o_x, g2o_y, g2o_z)
        # ax.set_zlim3d([0, 10])

    plt.show()


if __name__ == "__main__":
    g2o_xyz = generate_g2o_xyz(
        "/home/kulmer/Documents/MasterThesis/OtherSoftware/g2o_pos_eval/data/graph.g2o", "/home/kulmer/Documents/MasterThesis/OtherSoftware/g2o_pos_eval/data/zero_utm")
    gnss_xyz = generate_GNSS_xyz(
        "/home/kulmer/Documents/MasterThesis/OtherSoftware/g2o_pos_eval/data/2022-02-03-16-41-53_gps_navsat.txt")
    plot_msgs(g2o_xyz, gnss_xyz, 2)

# todo: lla to utm scheint fehlerhaft:
#       - Vergleiche ersten Eintrag aus Transformation mit zero_utm Koordinaten
#       - stimmen die Formeln?
# todo: auf g2o muss auf jedes x,y,z die zero_utm addiert werden für die globale Referenz
# todo: delta distanz berechnen:
#       - 2D oder 3D (?)
#       - euclidean (?)
