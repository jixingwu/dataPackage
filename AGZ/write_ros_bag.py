import rosbag
import rospy
import roslib
import tf
from std_msgs.msg import UInt32, String, Int32, Int8MultiArray, Header
from sensor_msgs.msg import NavSatFix, Image, CameraInfo, Imu
from rosflight_msgs.msg import Barometer
from geometry_msgs.msg import PoseStamped, AccelStamped
import csv
import itertools
import ImageFile
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


def main(argv):

    if len(sys.argv) < 2:
        print 'Please specify data directory file'
        return 1

    if len(sys.argv) < 3:
        print 'Please specify output rosbag file'
        return 1
    bag = rosbag.Bag(sys.argv[2], 'w')

    ral = csv.reader(open(sys.argv[1] + "/LogFiles_tmp/RawAccel.csv"))
    rgo = csv.reader(open(sys.argv[1] + "/LogFiles_tmp/RawGyro.csv"))
    gps = csv.reader(open(sys.argv[1] + "/LogFiles_tmp/OnboardGPS.csv"))
    gta = csv.reader(open(sys.argv[1] + "/LogFiles_tmp/GroundTruthAGM.csv"))
    gtl = csv.reader(open(sys.argv[1] + "/LogFiles_tmp/GroundTruthAGL.csv"))
    bap = csv.reader(open(sys.argv[1] + "/LogFiles_tmp/BarometricPressure.csv"))
    onp = csv.reader(open(sys.argv[1] + "/LogFiles_tmp/OnboardPose.csv"))
lll
lll
lll
    ral_seq = 0
    bap_seq = 0
    cal = -1
    gps_seq = 0
    # IMAGE_COUNT = 81169
    STREET_VIEW = 113

    print("Packaging Imu...")
    for ral_data, rgo_data in zip(ral, rgo):

        ral_seq = ral_seq + 1
        utime = int(ral_data[0])
        timestamp = rospy.Time.from_sec(utime/1e6)

        header = Header()
        header.seq = ral_seq
        header.stamp = timestamp
        header.frame_id = 'imu'

        imu = Imu()
        imu.header = header
        imu.linear_acceleration.x = float(ral_data[2])
        imu.linear_acceleration.y = float(ral_data[3])
        imu.linear_acceleration.z = float(ral_data[4])
        imu.linear_acceleration_covariance = np.zeros(9)

        imu.angular_velocity.x = float(rgo_data[2])
        imu.angular_velocity.y = float(rgo_data[3])
        imu.angular_velocity.z = float(rgo_data[4])
        imu.angular_velocity_covariance = np.zeros(9)

        bag.write('/Imu', imu, t=timestamp)



    print("Package groundtruthAGM...")
    for gta_data in gta:
        Igt = Int8MultiArray()
        Igt.data = [int(gta_data[2]), int(gta_data[3]), int(gta_data[4])]
        bag.write('/groundtruth/sv_id', Igt)


    print("Package groundtruthIMAGE...")
    for stv_seq in range(STREET_VIEW):
        # print(stv_seq)
        img_cv = cv2.imread(sys.argv[1] + "/Street View Images/left-" + '{0:03d}'.format(stv_seq+1) + ".jpg", 1)
        br = CvBridge()
        Img = Image()
        Img = br.cv2_to_imgmsg(img_cv, "bgr8")
        Img.header.seq = stv_seq
        # Img.header.stamp = timestamp
        Img.header.frame_id = 'streetview'
        bag.write('/groundtruth/image', Img)


    MAV_IMAGE = 27050
    print("Package MAVimages...")

    for bap_data in bap:
        bap_seq = bap_seq + 1
        bar = Barometer()
        bar.altitude = float(bap_data[2])
        bar.pressure = float(bap_data[1])
        bar.temperature = float(bap_data[3])
        bar.header.seq - int(bap_seq)
        bar.header.frame_id = 'BarometricPressure'
        bag.write('/barometric_pressure', bar)


    print("Packaging GPS, MAV images and Caminfo")
    for gps_data in gps:
        print(int(gps_data[1]))
        # On board GPS processing
        imgid = int(gps_data[1])
        # gps_seq = gps_seq + 1
        utime = int(gps_data[0])
        timestamp = rospy.Time.from_sec(utime / 1e6)

        header = Header()
        header.seq = imgid
        header.stamp = timestamp
        header.frame_id = 'gps'

        Gps = NavSatFix()
        Gps.header = header
        Gps.status.service = 1
        Gps.latitude = float(gps_data[2])
        Gps.longitude = float(gps_data[3])
        Gps.altitude = float(gps_data[4])
        bag.write('/gps', Gps, t=timestamp)
        # print("gps")
        # MAV image processing


        img_cv = cv2.imread(sys.argv[1] + "/MAV Images/" + '{0:05d}'.format(int(gps_data[1])) + ".jpg", 1)
        br = CvBridge()
        Img = Image()
        Img = br.cv2_to_imgmsg(img_cv, "bgr8")
        # print(type(Img))
        Img.header.seq = imgid
        Img.header.stamp = timestamp
        Img.header.frame_id = 'camera'
        bag.write('/camera/image', Img, t=timestamp)
        if cal < 0:
            Caminfo = CameraInfo()
            cam_data = np.load(sys.argv[1] + '/calibration_data.npz')
            Caminfo.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            Caminfo.P = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            Caminfo.D = np.asarray(cam_data['distCoeff']).reshape(-1)
            Caminfo.K = np.asarray(cam_data['intrinsic_matrix']).reshape(-1)
            Caminfo.binning_x = 1
            Caminfo.binning_y = 1
            img_cv_h, img_cv_w = img_cv.shape[:2]
            Caminfo.width = img_cv_w
            Caminfo.height = img_cv_h
            Caminfo.distortion_model = 'plumb_bob'
            bag.write('/camera/camera_info', Caminfo, t=timestamp)
            cal = 0
            # print("cal")
    bag.close()

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))

