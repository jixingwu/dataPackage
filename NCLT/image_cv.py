import cv2
#-*- coding: UTF-8 -*-
# 顺时针旋转90度
def RotateClockWise90(img):
    trans_img = cv2.transpose( img )
    new_img = cv2.flip(trans_img, 1)
    return new_img


# 逆时针旋转90度
def RotateAntiClockWise90(img):
    trans_img = cv2.transpose( img )
    new_img = cv2.flip( trans_img, 0 )
    return new_img

def test_rot(img_path):
    img = cv2.imread(img_path)
    cv2.imshow('raw', img)

    trans_img = cv2.transpose(img)
    cv2.imshow( 'trans', trans_img )

    clock90_img = RotateClockWise90(img)
    cv2.imshow( 'clock90', clock90_img )

    ant_clock90_img = RotateAntiClockWise90(img)
    cv2.imshow('ant_clock90', ant_clock90_img)

    key_ret = cv2.waitKey( 0 )

if __name__ == '__main__':
    test_rot('/home/jixingwu/dataPackage/dataProcess_m/nclt_path.jpg')
