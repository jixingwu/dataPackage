# !/usr/bin/python
#
# Convert the sensor data files in the given directory to a single rosbag.
#
# To call:
#
#   python sensordata_to_rosbag.py 2012-01-08/ 2012-01-08.bag
#

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
import cv2
import ImageFile
from cv_bridge import CvBridge, CvBridgeError

def write_gps(gps, i, bag):

    utime = gps[i, 0]
    mode = gps[i, 1]

    lat = gps[i, 3]
    lng = gps[i, 4]
    alt = gps[i, 5]

    timestamp = rospy.Time.from_sec(utime/1e6)

    status = NavSatStatus()

    if mode==0 or mode==1:
        status.status = NavSatStatus.STATUS_NO_FIX
    else:
        status.status = NavSatStatus.STATUS_FIX

    status.service = NavSatStatus.SERVICE_GPS
    
    num_sats = UInt16()
    num_sats.data = gps[i, 2]

    fix = NavSatFix()
    fix.status = status

    fix.latitude = np.rad2deg(lat)
    fix.longitude = np.rad2deg(lng)
    fix.altitude = alt
    
    track = Float64()
    track.data = gps[i, 6]

    speed = Float64()
    speed.data = gps[i, 7]

    bag.write('gps_fix', fix, t=timestamp)
    bag.write('gps_track', track, t=timestamp)
    bag.write('gps_speed', speed, t=timestamp)

def write_gps_rtk(gps, i, bag):

    utime = gps[i, 0]
    mode = gps[i, 1]

    lat = gps[i, 3]
    lng = gps[i, 4]
    alt = gps[i, 5]

    timestamp = rospy.Time.from_sec(utime/1e6)

    status = NavSatStatus()

    if mode==0 or mode==1:
        status.status = NavSatStatus.STATUS_NO_FIX
    else:
        status.status = NavSatStatus.STATUS_FIX

    status.service = NavSatStatus.SERVICE_GPS

    num_sats = UInt16()
    num_sats.data = gps[i, 2]

    fix = NavSatFix()
    fix.status = status

    fix.latitude = np.rad2deg(lat)
    fix.longitude = np.rad2deg(lng)
    fix.altitude = alt

    track = Float64()
    track.data = gps[i, 6]

    speed = Float64()
    speed.data = gps[i, 7]

    bag.write('gps_rtk_fix', fix, t=timestamp)
    bag.write('gps_rtk_track', track, t=timestamp)
    bag.write('gps_rtk_speed', speed, t=timestamp)



def write_ms25(ms25, i, bag):

    utime = ms25[i, 0]

    mag_x = ms25[i, 1]
    mag_y = ms25[i, 2]
    mag_z = ms25[i, 3]

    accel_x = ms25[i, 4]
    accel_y = ms25[i, 5]
    accel_z = ms25[i, 6]

    rot_r = ms25[i, 7]
    rot_p = ms25[i, 8]
    rot_h = ms25[i, 9]

    timestamp = rospy.Time.from_sec(utime/1e6)

    layout = MultiArrayLayout()
    layout.dim = [MultiArrayDimension()]
    layout.dim[0].label = "xyz"
    layout.dim[0].size = 3
    layout.dim[0].stride = 1

    mag = Float64MultiArray()
    mag.data = [mag_x, mag_y, mag_z]
    mag.layout = layout

    accel = Float64MultiArray()
    accel.data = [accel_x, accel_y, accel_z]
    accel.layout = layout

    layout_rph = MultiArrayLayout()
    layout_rph.dim = [MultiArrayDimension()]
    layout_rph.dim[0].label = "rph"
    layout_rph.dim[0].size = 3
    layout_rph.dim[0].stride = 1

    rot = Float64MultiArray()
    rot.data = [rot_r, rot_p, rot_h]
    rot.layout = layout_rph

    bag.write('mag', mag, t=timestamp)
    bag.write('accel', accel, t=timestamp)
    bag.write('rot', rot, t=timestamp)

def write_ms25_euler(ms25_euler, i, bag):

    utime = ms25_euler[i, 0]

    r = ms25_euler[i, 1]
    p = ms25_euler[i, 2]
    h = ms25_euler[i, 3]

    timestamp = rospy.Time.from_sec(utime/1e6)

    layout_rph = MultiArrayLayout()
    layout_rph.dim = [MultiArrayDimension()]
    layout_rph.dim[0].label = "rph"
    layout_rph.dim[0].size = 3
    layout_rph.dim[0].stride = 1

    euler = Float64MultiArray()
    euler.data = [r, p, h]
    euler.layout = layout_rph

    bag.write('ms25_euler', euler, t=timestamp)

def convert_vel(x_s, y_s, z_s):

    scaling = 0.005 # 5 mm
    offset = -100.0

    x = x_s * scaling + offset
    y = y_s * scaling + offset
    z = z_s * scaling + offset

    return x, y, z

def verify_magic(s):

    magic = 44444

    m = struct.unpack('<HHHH', s)

    return len(m)>=3 and m[0] == magic and m[1] == magic and m[2] == magic and m[3] == magic

def read_next_vel_packet(f_vel):

    try:
        magic = f_vel.read(8)
        if magic == '': # eof
            return -1, None

        if not verify_magic(magic):
            print "Could not verify magic"
            return -1, None

        num_hits = struct.unpack('<I', f_vel.read(4))[0]
        utime = struct.unpack('<Q', f_vel.read(8))[0]

        f_vel.read(4) # padding

        # Read all hits
        data = []
        for i in range(num_hits):

            x = struct.unpack('<H', f_vel.read(2))[0]
            y = struct.unpack('<H', f_vel.read(2))[0]
            z = struct.unpack('<H', f_vel.read(2))[0]
            i = struct.unpack('B', f_vel.read(1))[0]
            l = struct.unpack('B', f_vel.read(1))[0]

            x, y, z = convert_vel(x, y, z)

            data += [x, y, z, float(i), float(l)]

        return utime, data
    except Exception:
        pass

    return -1, None

def write_vel(vel_data, utime, bag):

    timestamp = rospy.Time.from_sec(utime/1e6)

    layout = MultiArrayLayout()
    layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
    layout.dim[0].label = "hits"
    #layout.dim[0].size = num_hits
    layout.dim[0].stride = 5
    layout.dim[1].label = "xyzil"
    layout.dim[1].size = 5
    layout.dim[1].stride = 1

    vel = Float64MultiArray()
    vel.data = vel_data
    vel.layout = layout

    bag.write('velodyne_packet', vel, t=timestamp)

def convert_hokuyo(x_s):

    scaling = 0.005 # 5 mm
    offset = -100.0

    x = x_s * scaling + offset

    return x

def read_next_hokuyo_30m_packet(f_bin):

    try:

        # hokuyo_30m always has 1081 hits
        num_hits = 1081

        # Read timestamp
        utime = struct.unpack('<Q', f_bin.read(8))[0]

        r = np.zeros(num_hits)

        for i in range(num_hits):
            s = struct.unpack('<H', f_bin.read(2))[0]
            r[i] = convert_hokuyo(s)

        return utime, r

    except Exception:
        pass

    return -1, None

def write_hokuyo_30m_packet(hok_30m, utime, bag):

    # hokuyo_30m always has 1081 hits
    num_hits = 1081

    timestamp = rospy.Time.from_sec(utime/1e6)

    layout = MultiArrayLayout()
    layout.dim = [MultiArrayDimension()]
    layout.dim[0].label = "r"
    layout.dim[0].size = num_hits
    layout.dim[0].stride = 1

    hits = Float64MultiArray()
    hits.data = hok_30m
    hits.layout = layout

    bag.write('hokuyo_30m_packet', hits, t=timestamp)

def read_next_hokuyo_4m_packet(f_bin):

    try:

        # hokuyo_4m always has 726 hits
        num_hits = 726

        # Read timestamp
        utime_str = f_bin.read(8)
        if utime_str == '': #EOF
            return -1, None

        #utime = struct.unpack('<Q', f_bin.read(8))[0]
        utime = struct.unpack('<Q', utime_str)[0]

        r = np.zeros(num_hits)

        for i in range(num_hits):
            s = struct.unpack('<H', f_bin.read(2))[0]
            r[i] = convert_hokuyo(s)

        return utime, r

    except Exception as e:
        print utime_str
        print len(utime_str)
        print i
        print r
        raise e
        pass

    return -1, None

def write_hokuyo_4m_packet(hok_4m, utime, bag):

    # hokuyo_4m always has 726 hits
    num_hits = 726

    timestamp = rospy.Time.from_sec(utime/1e6)

    layout = MultiArrayLayout()
    layout.dim = [MultiArrayDimension()]
    layout.dim[0].label = "r"
    layout.dim[0].size = num_hits
    layout.dim[0].stride = 1

    hits = Float64MultiArray()
    hits.data = hok_4m
    hits.layout = layout

    bag.write('hokuyo_4m_packet', hits, t=timestamp)


#######################
def write_gps_rtk_err(gps, i, bag):

    utime = gps[i,0]
    timestamp = rospy.Time.from_sec(utime/1e6)
    error = Float64()
    error.data = np.float64(gps[i,1])
    bag.write('gps_rtk_err', error, t=timestamp)

def write_kvh(kvh, i, bag):
    utime = kvh[i,0]
    psi = float(kvh[i,1])
    timestamp = rospy.Time.from_sec(utime/1e6)
    kvh_Imu = Imu()
    kvh_Imu.header.seq = i
    kvh_Imu.header.stamp = timestamp
    kvh_Imu.header.frame_id = 'kvh_Imu'
    kvh_Imu.orientation.w = math.cos(psi/2)
    kvh_Imu.orientation.x = 0
    kvh_Imu.orientation.y = 0
    kvh_Imu.orientation.z = math.sin(psi/2)
    kvh_Imu.orientation_covariance = np.zeros(9)
    bag.write('kvh', kvh_Imu, t=timestamp)

def write_wheel(wheel, i, bag):
    utime = wheel[i,0]
    timestamp = rospy.Time.from_sec(utime/1e6)

    
    header = Header()
    header.seq = i
    header.stamp = timestamp
    header.frame_id = 'wheelSpeed'

    wheelLeft = TwistStamped()
    wheelLeft.header = header
    wheelLeft.twist.linear.z = float(wheel[i,1])
    wheelRight = TwistStamped()
    wheelRight.header = header
    wheelRight.twist.linear.z = float(wheel[i,2])
    bag.write('wheelLeft', wheelLeft, t=timestamp)
    bag.write('wheelRight', wheelRight, t=timestamp)

def write_odometry(odometry, odometry_cov, i, bag):
    utime = odometry[i,0]
    timestamp = rospy.Time.from_sec(utime/1e6)

    header = Header()
    header.seq = i
    header.stamp = timestamp
    header.frame_id = 'odometry'
    odo = Odometry()
    odo.header = header
    odo.pose.pose.position.x = float(odometry[i,1])
    odo.pose.pose.position.y = float(odometry[i,2])
    odo.pose.pose.position.z = float(odometry[i,3])
    psi = float(odometry[i,6])
    odo.pose.pose.orientation.w = math.cos(psi/2)
    odo.pose.pose.orientation.x = 0
    odo.pose.pose.orientation.y = 0
    odo.pose.pose.orientation.z = math.sin(psi/2)
    # don't have twist in the forms
    # odo.pose.covariance
    for index in range(len(odometry_cov[i])-1):
        odo.pose.covariance[index] = float(odometry_cov[i,index+1])
    bag.write('odometry', odo, t=timestamp)

def write_odometry_100hz(odometry_100hz, odometry_cov_100hz, i, bag):
    utime = odometry_100hz[i,0]
    timestamp = rospy.Time.from_sec(utime/1e6)

    header = Header()
    header.seq = i
    header.stamp = timestamp
    header.frame_id = 'odometry_100hz'
    odo = Odometry()
    odo.header = header
    odo.pose.pose.position.x = float(odometry_100hz[i,1])
    odo.pose.pose.position.y = float(odometry_100hz[i,2])
    odo.pose.pose.position.z = float(odometry_100hz[i,3])
    psi = float(odometry_100hz[i,6])
    odo.pose.pose.orientation.w = math.cos(psi/2)
    odo.pose.pose.orientation.x = 0
    odo.pose.pose.orientation.y = 0
    odo.pose.pose.orientation.z = math.sin(psi/2)
    # don't have twist in the forms
    # odo.pose.covariance
    for index in range(len(odometry_cov_100hz[i])-1):
        odo.pose.covariance[index] = float(odometry_cov_100hz[i,index+1])
    bag.write('odometry_100hz', odo, t=timestamp)

# def write_odometry(heading, odometry, odometry_cov, i, bag):
# def write_image():



def main(argv):

    if len(sys.argv) < 2:
        print 'Please specify sensor data directory file'
        return 1

    if len(sys.argv) < 3:
        print 'Please specify output rosbag file'
        return 1

    bag = rosbag.Bag(sys.argv[2], 'w')

    gps = np.loadtxt(sys.argv[1] + "2012-01-08/gps.csv", delimiter = ",")
    gps_rtk = np.loadtxt(sys.argv[1] + "2012-01-08/gps_rtk.csv", delimiter = ",")
    ms25 = np.loadtxt(sys.argv[1] + "2012-01-08/ms25.csv", delimiter = ",")
    ms25_euler = np.loadtxt(sys.argv[1] + "2012-01-08/ms25_euler.csv", delimiter = ",")
########################
    gps_rtk_err = np.loadtxt(sys.argv[1] + "2012-01-08/gps_rtk_err.csv", delimiter = ",")
    kvh = np.loadtxt(sys.argv[1] + "2012-01-08/kvh.csv", delimiter = ",")
    wheel = np.loadtxt(sys.argv[1] + "2012-01-08/wheels.csv", delimiter = ",")
    odometry = np.loadtxt(sys.argv[1] + "2012-01-08/odometry_mu.csv", delimiter = ",")
    odometry_cov = np.loadtxt(sys.argv[1] + "2012-01-08/odometry_cov.csv", delimiter =",")
    # odometry_100hz = np.loadtxt()
    odometry_100hz = np.loadtxt(sys.argv[1] + "2012-01-08/odometry_mu_100hz.csv", delimiter = ",")
    odometry_cov_100hz = np.loadtxt(sys.argv[1] + "2012-01-08/odometry_cov_100hz.csv", delimiter =",")


#######################
    i_gps = 0
    i_gps_rtk = 0  
    i_ms25 = 0
    i_ms25_euler = 0










###############
    i_gps_rtk_err = 0
    i_kvh = 0
    i_wheel = 0
    i_odometry = 0
    i_odometry_100hz = 0
#################
    # f_vel = open(sys.argv[1] + "velodyne_hits.bin", "r")
    f_hok_30 = open(sys.argv[1] + "2012-01-08/hokuyo_30m.bin", "r")
    f_hok_4 = open(sys.argv[1] + "2012-01-08/hokuyo_4m.bin", "r")
    
    # utime_vel, vel_data = read_next_vel_packet(f_vel)
    utime_hok30, hok30_data = read_next_hokuyo_30m_packet(f_hok_30)
    utime_hok4, hok4_data = read_next_hokuyo_4m_packet(f_hok_4)

    print 'Loaded data, writing ROSbag...'

    while True:

        # Figure out next packet in time
        next_packet = "done"
        next_utime = -1

        if i_gps<len(gps) and (gps[i_gps, 0]<next_utime or next_utime<0):
            next_packet = "gps"

        if i_gps_rtk<len(gps_rtk) and (gps_rtk[i_gps_rtk, 0]<next_utime or next_utime<0):
            next_packet = "gps_rtk"

        if i_ms25<len(ms25) and (ms25[i_ms25, 0]<next_utime or next_utime<0):
            next_packet = "ms25"

        if i_ms25_euler<len(ms25_euler) and (ms25_euler[i_ms25_euler, 0]<next_utime or next_utime<0):
            next_packet = "ms25_euler"

        # if utime_vel>0 and (utime_vel<next_utime or next_utime<0):
        #     next_packet = "vel"


        if utime_hok30>0 and (utime_hok30<next_utime or next_utime<0):
            next_packet = "hok30"

        if utime_hok4>0 and (utime_hok4<next_utime or next_utime<0):
            next_packet = "hok4"

#####################################
        if i_gps_rtk_err<len(gps_rtk_err) and (gps_rtk_err[i_gps_rtk_err,0]<next_utime or next_utime<0):
            next_packet = "gps_rtk_err"
        if i_kvh<len(kvh) and (kvh[i_kvh,0]<next_utime or next_utime<0):
            next_packet = "kvh"
        if i_wheel<len(wheel) and (wheel[i_wheel,0]<next_utime or next_utime<0):
            next_packet = "wheel"
        if i_odometry<len(odometry) and (odometry[i_odometry,0]<next_utime or next_utime<0):
            next_packet = "odometry"
            #_100hz
        if i_odometry_100hz<len(odometry_100hz) and (odometry_100hz[i_odometry_100hz,0]<next_utime or next_utime<0):
            next_packet = "odometry_100hz"
#####################################
        # Now deal with the next packet
        if next_packet == "done":
            break
        elif next_packet == "gps":
            write_gps(gps, i_gps, bag)
            i_gps = i_gps + 1

        elif next_packet == "gps_rtk":
            write_gps_rtk(gps_rtk, i_gps_rtk, bag)
            i_gps_rtk = i_gps_rtk + 1



        elif next_packet == "ms25":
            write_ms25(ms25, i_ms25, bag)
            i_ms25 = i_ms25 + 1

        elif next_packet == "ms25_euler":
            write_ms25_euler(ms25_euler, i_ms25_euler, bag)
            i_ms25_euler = i_ms25_euler + 1

        # elif next_packet == "vel":
        #     write_vel(vel_data, utime_vel, bag)
        #     utime_vel, vel_data = read_next_vel_packet(f_vel)
        elif next_packet == "hok30":
            write_hokuyo_30m_packet(hok30_data, utime_hok30, bag)
            utime_hok30, hok30_data = read_next_hokuyo_30m_packet(f_hok_30)

        elif next_packet == "hok4":
            write_hokuyo_4m_packet(hok4_data, utime_hok4, bag)
            utime_hok4, hok4_data = read_next_hokuyo_4m_packet(f_hok_4)

#############################

        elif next_packet =="gps_rtk_err":
            write_gps_rtk_err(gps_rtk_err, i_gps_rtk_err, bag)
            i_gps_rtk_err = i_gps_rtk_err + 1
        elif next_packet =="kvh":
            write_kvh(kvh, i_kvh, bag)
            i_kvh = i_kvh + 1
        elif next_packet =="wheel":
            write_wheel(wheel, i_wheel, bag)
            i_wheel = i_wheel + 1
        elif next_packet =="odometry":
            write_odometry(odometry, odometry_cov, i_odometry, bag)
            i_odometry = i_odometry + 1
        elif next_packet =="odometry_100hz":
            write_odometry_100hz(odometry_100hz, odometry_cov_100hz, i_odometry_100hz, bag)
            i_odometry_100hz = i_odometry_100hz + 1


#############################

        else:
            print "Unknown packet type"

    print("package image...")
    img_path = sys.argv[1] + 'images/2012-01-08/lb3/Cam0/'
    img_list = os.listdir(img_path)
    i_img = 0
    for img_name in img_list:
        i_img = i_img + 1
        print(i_img)
        img_cv = cv2.imread(os.path.join(img_path, img_name), -1)
        br = CvBridge()
        img = Image()
        img = br.cv2_to_imgmsg(img_cv, "bgr8")
        img.header.seq = i_img
        img.header.frame_id = 'camImage'
        bag.write('/camera/image', img)



    # f_vel.close()
    f_hok_30.close()
    f_hok_4.close()
    bag.close()

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))

