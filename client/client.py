#! /usr/bin/python
import cv2
import argparse
import numpy as np
from socketIO_client import SocketIO, LoggingNamespace, ConnectionError
import time
parser = argparse.ArgumentParser()
parser.add_argument("--tune", help="tune hsv values", action='store_true')
parser.add_argument('--noserve', help="don't publish player data", action='store_true')
parser.add_argument('-p', '--player', help='specify the player number', type=int, default=1)
parser.add_argument('--ip', help="the ip of the socket to publish too", default='localhost')
parser.add_argument('-c', '--camera', help='specify the camera number', type=int, default=1)
args = parser.parse_args()

####################################################################################################
## Detection Parameters
####################################################################################################

## HSV thresholding
if args.player == 1:
    H_LOW = 33
    S_LOW = 125
    V_LOW = 33
    H_HIG = 81
    S_HIG = 230
    V_HIG = 230
else:
    H_LOW = 29
    S_LOW = 59
    V_LOW = 86
    H_HIG = 55
    S_HIG = 255
    V_HIG = 255

## Blob detection
BLOB_DETECTOR_PARAMS = cv2.SimpleBlobDetector_Params()
BLOB_DETECTOR_PARAMS.blobColor = 255
BLOB_DETECTOR_PARAMS.filterByArea = True
BLOB_DETECTOR_PARAMS.minArea = 500
BLOB_DETECTOR_PARAMS.maxArea = 99999999
BLOB_DETECTOR_PARAMS.filterByCircularity = False
BLOB_DETECTOR_PARAMS.filterByConvexity = False
BLOB_DETECTOR_PARAMS.minConvexity = 0.87
BLOB_DETECTOR_PARAMS.filterByInertia = False
BLOB_DETECTOR_PARAMS.minInertiaRatio = 0.01

## Smoothing
MOVING_AVERAGE_SAMPLES = 2

####################################################################################################
## Image Processing
####################################################################################################
def process_image(img, detector, tuneing, debug=True):
    h_low, s_low, v_low, h_hig, s_hig, v_hig = tuneing
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_blur = cv2.GaussianBlur(img_hsv, (2**(2)+1, 2**(2) + 1), 2)
    img_filtered = cv2.inRange(img_blur, (h_low, s_low, v_low), (h_hig, s_hig, v_hig))
    img_medblur = cv2.medianBlur(img_filtered, 2**(2) + 1)
    img_seg = cv2.bitwise_and(img, img, mask=img_medblur) if debug else img
    keypoints = detector.detect(img_medblur)
    (x, y) = (None, None)
    if len(keypoints) > 0:
        (x, y, norm) = reduce(
                lambda accum, point: (accum[0] + point.pt[0] * point.size, accum[1] + point.pt[1]*point.size, accum[2] + point.size),
                keypoints,
                (0, 0, 0)
            )
        (x, y) = (float(x) / norm, float(y) / norm)
    return (img_seg, x, y)

####################################################################################################
## Utilities
####################################################################################################
def create_moving_ave(n):
    values = [0] * n
    def next_value(value):
        if value != None:
            values.insert(0, value)
            values.pop()
        return float(sum(values)) / len(values)
    return next_value

def get_tuneing():
    h_low = cv2.getTrackbarPos('h_low', 'Player Tracker')
    s_low = cv2.getTrackbarPos('s_low', 'Player Tracker')
    v_low = cv2.getTrackbarPos('v_low', 'Player Tracker')
    h_hig = cv2.getTrackbarPos('h_hig', 'Player Tracker')
    s_hig = cv2.getTrackbarPos('s_hig', 'Player Tracker')
    v_hig = cv2.getTrackbarPos('v_hig', 'Player Tracker')
    return (h_low, s_low, v_low, h_hig, s_hig, v_hig)

def pick_point(cam, detector, x_filter, y_filter):
    while True:
        tuneing = get_tuneing() if args.tune else (H_LOW, S_LOW, V_LOW, H_HIG, S_HIG, V_HIG)
        ret_val, img = cam.read()
        if img == None:
            print 'bruh, check the webcam'
            exit()
        (img_processed, x, y) = process_image(img, detector, tuneing=tuneing)
        cv2.circle(img_processed, (int(x_filter(x)), int(y_filter(y))), 20, (0,0,255), -1)
        cv2.imshow('Player Tracker', img_processed)
        key = cv2.waitKey(1)
        if key == 10:
            return (x_filter(None), y_filter(None))
        elif key == 27:
            cv2.destroyAllWindows()
            exit()

####################################################################################################
## Main program
####################################################################################################
def main():
    # create connection
    if not args.noserve:
        try:
            print 'Connecting to server...'
            socket = SocketIO(args.ip, 3000, wait_for_connection=False)
            print 'Connected!'
        except ConnectionError:
            print 'Failed to connect to server!'
            exit()
    cam = cv2.VideoCapture(args.camera)
    # cam.set(cv2.cv.CV_CAP_PROP_BRIGHTNESS, 0)
    cv2.namedWindow('Player Tracker')
    detector = cv2.SimpleBlobDetector(BLOB_DETECTOR_PARAMS)

    if args.tune:
        cv2.createTrackbar('h_low', 'Player Tracker', 0, 180, lambda x: None)
        cv2.createTrackbar('s_low', 'Player Tracker', 0, 255, lambda x: None)
        cv2.createTrackbar('v_low', 'Player Tracker', 0, 255, lambda x: None)
        cv2.createTrackbar('h_hig', 'Player Tracker', 0, 180, lambda x: None)
        cv2.createTrackbar('s_hig', 'Player Tracker', 0, 255, lambda x: None)
        cv2.createTrackbar('v_hig', 'Player Tracker', 0, 255, lambda x: None)

    x_filter = create_moving_ave(MOVING_AVERAGE_SAMPLES)
    y_filter = create_moving_ave(MOVING_AVERAGE_SAMPLES)
    tuneing = (H_LOW, S_LOW, V_LOW, H_HIG, S_HIG, V_HIG)

    print 'calibrating, move far left then press enter...'
    (x1, y1) = pick_point(cam, detector, x_filter, y_filter)

    print 'calibrating, move far right the press enter...'
    (x2, y2) = pick_point(cam, detector, x_filter, y_filter)
    axis = (x2 - x1, y2 - y1)
    print 'done calibrating: (%s, %s), (%s, %s)' % (x1, y1, x2, y2)

    while True:
        ret_val, img = cam.read()
        if img == None:
            print 'bruh, check the webcam'
            exit()

        tuneing = get_tuneing() if args.tune else (H_LOW, S_LOW, V_LOW, H_HIG, S_HIG, V_HIG)
        (img_processed, x, y) = process_image(img, detector, tuneing=tuneing, debug=False)
        x_coord = x_filter(x)
        y_coord = y_filter(y)
        cv2.circle(img_processed, (int(x_coord), int(y_coord)), 20, (0,0,255), -1)
        game_position = (axis[0] * (x_coord - x1) + axis[1] * (y_coord - y1)) / (axis[0]**2 + axis[1]**2)
        game_position = min(max(0, game_position), 1)
        if not args.noserve:
            socket.emit('data', args.player, game_position)

        # cv2.imshow('Player Tracker', img_processed)

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break  # esc to quit

if __name__ == '__main__':
    main()
