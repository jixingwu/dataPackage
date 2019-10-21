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

MAVIMAGE = 2000
MAVDIM = (1280, 720)
# MAVDIM = (1920, 1080)
TIMESTAMP = []

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
    ori = csv.reader(open(sys.argv[1] + "/LogFiles_tmp/RawOrien.csv"))
    pos = csv.reader(open(sys.argv[1] + "/LogFiles_tmp/OnboardPose.csv"))


    ral_seq = 0
    bap_seq = 0
    img_seq = 0
    pos_seq = 0
    cal = -1
    gps_seq = 0
    # IMAGE_COUNT = 81169
    STREET_VIEW = 113

    print("packageing Imu for OnboardPose.csv ...")

    for pos_data in pos:
        pos_seq = pos_seq + 1
        utime = int(pos_data[0])
        timestamp = rospy.Time.from_sec(utime/1e6)

        imu = Imu()
        imu.header.seq = pos_seq
        imu.header.stamp = timestamp
        imu.header.frame_id = '/Imu'

        imu.linear_acceleration.x = float(pos_data[4])
        imu.linear_acceleration.y = float(pos_data[5])
        imu.linear_acceleration.z = float(pos_data[6])
        imu.linear_acceleration_covariance = np.zeros(9)

        imu.angular_velocity.x = float(pos_data[1])
        imu.angular_velocity.y = float(pos_data[2])
        imu.angular_velocity.z = float(pos_data[3])
        imu.angular_velocity_covariance = np.zeros(9)

        imu.orientation.w = float(pos_data[14])
        imu.orientation.x = float(pos_data[15])
        imu.orientation.y = float(pos_data[16])
        imu.orientation.z = float(pos_data[17])

        bag.write('/Imu', imu, t=timestamp)

        pos_seq = pos_seq + 1
        imu.header.seq = pos_seq
        bag.write('/Imu', imu, t=timestamp)



        # if cal < 0:
        #     Caminfo = CameraInfo()
        #     cam_data = np.load(sys.argv[1] + '/calibration_data.npz')
        #     Caminfo.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        #     Caminfo.P = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        #     Caminfo.D = np.asarray(cam_data['distCoeff']).reshape(-1)
        #     Caminfo.K = np.asarray(cam_data['intrinsic_matrix']).reshape(-1)
        #     Caminfo.binning_x = 1
        #     Caminfo.binning_y = 1
        #     img_cv_h, img_cv_w = img_cv.shape[:2]
        #     Caminfo.width = img_cv_w
        #     Caminfo.height = img_cv_h
        #     Caminfo.distortion_model = 'plumb_bob'
        #     bag.write('/camera/camera_info', Caminfo, t=timestamp)
        #     cal = 0

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

    for bap_data in bap:
        bap_seq = bap_seq + 1
        bar = Barometer()
        bar.altitude = float(bap_data[2])
        bar.pressure = float(bap_data[1])
        bar.temperature = float(bap_data[3])
        bar.header.seq - int(bap_seq)
        bar.header.frame_id = 'BarometricPressure'
        bag.write('/barometric_pressure', bar)

    print("Packaging GPS and cam_info")
    for gps_data in gps:

        imgid = int(gps_data[1])

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

        if imgid <= MAVIMAGE and imgid % 3 == 0:
            # write aerial image
            img_seq = img_seq+1
            img_cv = cv2.imread(sys.argv[1] + "/MAV Images/" + '{0:05d}'.format(int(imgid)) + ".jpg", 1)
            img_cv = cv2.resize(img_cv, MAVDIM, interpolation=cv2.INTER_AREA)
            cv2.imshow('1', img_cv)
            br = CvBridge()
            Img = Image()
            Img = br.cv2_to_imgmsg(img_cv, "bgr8")
            Img.header.seq = int(img_seq)
            print(imgid)
            Img.header.stamp = timestamp
            Img.header.frame_id = 'camera'
            bag.write('/camera/image', Img, t=timestamp)

    bag.close()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

