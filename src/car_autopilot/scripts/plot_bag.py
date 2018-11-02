#!/usr/bin/env python
import rosbag
import rospy
from tqdm import tqdm
import sys
import numpy as np
import matplotlib.pyplot as plt
# import pyproj
#from pyproj import Proj, transform
# from pandas import DataFrame
import simplekml

def convert_NED_to_LLa(arr, origin_LL):
    #myProj = Proj("+proj=utm +zone=12T, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    # p = pyproj.Proj("+lat_0= {} lon_0={} +units=m").format(origin_LL[0],origin_LL[1])
    # print arr
    # #df = DataFrame(np.c_[arr[:,0], arr[:,1]], columns=['Meters East', 'Meters North'])
    # lon, lat = p(arr[1,0],arr[1,1], inverse=True)
    # arr2 = [lon, lat]

    #inProj = Proj(init='epsg:3857')
    #outProj = Proj(init='epsg:4326')
    #x1,y1 = -11705274.6374,4826473.6922
    #x2,y2 = transform(inProj,outProj,x1,y1)
    ##print x2,y2

    EARTH_RADIUS = 6371000.0
    lat = origin_LL[0] + arr[:,0]*180.0/np.pi/EARTH_RADIUS
    lon = origin_LL[1] + arr[:,1]*180.0/np.pi/EARTH_RADIUS/np.cos(lat*np.pi/180.0)

    return np.concatenate((np.atleast_2d(lon), np.atleast_2d(lat))).T

def convert_to_KML(arr):

    kml=simplekml.Kml()
    # for i in tqdm(range(len(arr)), total=len(arr)):
    #     if i%4000 == 0:
    #         kml.newpoint(name="path", coords=[(arr[i,0],arr[i,1])])

    kml.newlinestring(name="path",
                        coords=arr)
    kml.save('/home/gary/toBrigham.kml')
    return kml


position = []
inbag = rosbag.Bag('/home/gary/to_brigham_gps_image_depth_encoder_servo_20180605.bag')

for topic, msg, t in tqdm(inbag.read_messages(), total=inbag.get_message_count()):
    if topic == '/ins':
        position.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

position = np.array(position) ## NE position in meters
origin = [40.250507, -111.649276] # [40.020503, -111.649279] #

lla = convert_NED_to_LLa(position, origin)

kml = convert_to_KML(lla)
# print kml

plt.figure()
plt.plot(position[:,1], position[:,0])
plt.show()
