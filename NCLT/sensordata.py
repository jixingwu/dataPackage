#-*- coding: UTF-8 -*-
import rosbag, rospy
from std_msgs.msg import Float64, UInt16, Float64MultiArray, MultiArrayDimension, MultiArrayLayout, Header
from sensor_msgs.msg import NavSatStatus, NavSatFix, Imu, Image
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import sys
import numpy as np
import struct
import math
import os
from cv_bridge import CvBridge
import cv2

MAVIMAGE = 1000
MAVDIM = (720, 480) # 1616*1232


def main(argv):

    if len(sys.argv) < 2:
        print 'Please specify data directory file'
        return 1

    if len(sys.argv) < 3:
        print 'Please specify output rosbag file'
        return 1
    print("loading files...")
    bag = rosbag.Bag(sys.argv[2], 'w')
    gps = np.loadtxt(sys.argv[1] + "sensor_data/2012-01-08/gps.csv", delimiter = ",")
    imu_100hz = np.loadtxt(sys.argv[1] + "sensor_data/2012-01-08/imu_100hz.csv", delimiter=",")



    ral_seq = 0
    bap_seq = 0
    img_seq = 0
    imu_seq = 0
    cal = -1
    gps_seq = 0
    # IMAGE_COUNT = 81169
    STREET_VIEW = 113

    print("package gps and image...")
    print("Packaging GPS and image")
    for gps_data in gps:
        utime = int(gps_data[0])
        mode = int(gps_data[1])
        timestamp = rospy.Time.from_sec(utime / 1e6)

        lat = float(gps_data[3])
        lng = float(gps_data[4])
        alt = float(gps_data[5])

        status = NavSatStatus()
        if mode == 0 or mode == 1:
            status.status = NavSatStatus.STATUS_NO_FIX
        else:
            status.status = NavSatStatus.STATUS_FIX

        status.service = NavSatStatus.SERVICE_GPS

        num_sats = UInt16()
        num_sats.data = float(gps_data[2])

        fix = NavSatFix()
        fix.header.seq = gps_seq
        fix.status = status
        fix.latitude = np.rad2deg(lat)
        fix.longitude = np.rad2deg(lng)
        fix.altitude = np.rad2deg(alt)

        track = Float64()
        track.data = float(gps_data[6])

        speed = Float64()
        speed.data = float(gps_data[7])

        bag.write('/gps', fix, t=timestamp)
        bag.write('/gps_track', track, t=timestamp)
        bag.write('/gps_speed', speed, t=timestamp)

        # write aerial image
        if gps_seq <= MAVIMAGE:
            img_path = sys.argv[1] + 'images/2012-01-08/lb3/Cam5/'
            img_list = os.listdir(img_path)
            img_list.sort()
            img_cv = cv2.imread(os.path.join(img_path, img_list[gps_seq]), -1)
            img_cv = cv2.resize(img_cv, MAVDIM, interpolation=cv2.INTER_AREA)

            # 顺时针旋转90度
            trans_img = cv2.transpose(img_cv)
            img_cv = cv2.flip(trans_img, 1)

            br = CvBridge()
            Img = Image()
            Img = br.cv2_to_imgmsg(img_cv, "bgr8")
            Img.header.seq = int(gps_seq)
            print(gps_seq)
            Img.header.stamp = timestamp
            Img.header.frame_id = 'camera'
            bag.write('/image/cam5', Img, t=timestamp)

        gps_seq = gps_seq + 1

    print('packaging imu...')
    for imu_data in imu_100hz:
        imu_seq = imu_seq + 1
        utime = int(imu_data[0])
        timestamp = rospy.Time.from_sec(utime/1e6)

        imu = Imu()
        imu.header.seq = imu_seq
        imu.header.stamp = timestamp
        imu.header.frame_id = '/Imu'

        imu.linear_acceleration.x = float(imu_data[5])
        imu.linear_acceleration.y = float(imu_data[6])
        imu.linear_acceleration.z = float(imu_data[7])
        imu.linear_acceleration_covariance = np.zeros(9)

        imu.angular_velocity.x = float(imu_data[8])
        imu.angular_velocity.y = float(imu_data[9])
        imu.angular_velocity.z = float(imu_data[10])
        imu.angular_velocity_covariance = np.zeros(9)

        imu.orientation.w = float(imu_data[1])
        imu.orientation.x = float(imu_data[2])
        imu.orientation.y = float(imu_data[3])
        imu.orientation.z = float(imu_data[4])

        bag.write('/Imu', imu, t=timestamp)

    bag.close()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

