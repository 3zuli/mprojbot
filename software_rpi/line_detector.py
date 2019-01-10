# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np


# uncomment lines 11-14 if you want to use piCamera
# initialize the camera and grab a reference to the raw camera capture
# camera = PiCamera()
# camera.resolution = (320, 240)
# camera.framerate = 10
# rawCapture = PiRGBArray(camera, size=(320, 240))

# allow the camera to warmup
time.sleep(0.1)

show_size = True

# uncomment this, if you want to use video sample to test algorithm
cap = cv2.VideoCapture('line_test.webm')

# use only one of lines 25, 26. 25 is for live vid from piCamera, 26 for using pre-recorded video
#for frameRaw in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
while (cap.isOpened()):
    # Capture frame-by-frame
    # uncomment for use live piCamera
    #frame = frameRaw.array

    # uncomment if you want to use pre-recorded video
    ret, frame = cap.read()
    frame = cv2.resize(frame, (320, 240))
    # Just show image size - will be used later
    if show_size:
        height, width = frame.shape[:2]
        print(width, height)
        show_size = False

    # blank image used for drawing contours
    blank_image = np.zeros((height, width, 3), np.uint8)

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # thresholding for filtering out white color
    ret, thresh = cv2.threshold(gray, 130, 150, cv2.THRESH_BINARY_INV)

    # find all contours from thresholded image
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    dilated = cv2.dilate(thresh, kernel)
    _, contours, _ = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    filtered_contours = []

    # we are able to calculate contour area
    # according to fact, that many small blobs are detected
    # we filter out contours with small area - izi
    biggest_contour = []
    biggest_contour_area = 0
    center_points = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        # this number is random and is strongly dependent on your cam resolution
        # todo: calculate parameter for area treshold
        if area > 3000:
            filtered_contours.append(cnt)
            if area > biggest_contour_area:
                biggest_contour_area = area
                biggest_contour = cnt

                # calculate center of line at the center of a frame
                # todo: do this with centroid with moments (https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html)
                # cx = int(M['m10']/M['m00'])
                # cy = int(M['m01']/M['m00'])
                for point in biggest_contour:
                    if (point[0,1] >= (height/2 - 3) and (point[0,1] <= (height/2 + 3))):
                        center_points.append(point[0,0])

    center_of_line = np.average(center_points)
    center_of_line = round(center_of_line)
    # print(center_of_line)

    # draw big-ass contours on our blank image
    cv2.fillPoly(blank_image, pts=[biggest_contour], color=(255, 255, 255))
    cv2.drawContours(blank_image, biggest_contour, -1, (0, 255, 0), 3)

    # draw two circles, one is a center of a line at the center of an image
    # second circle represents frame center
    cv2.circle(frame,(int(center_of_line), int(round(height/2))),3 , (0, 255, 0), 5)
    cv2.circle(frame, (int(round(width / 2)), int(round(height / 2))), 4, (0, 0, 255), 5)

    # Display the results and the grayscale input from your camera
    cv2.namedWindow('detected line', cv2.WINDOW_NORMAL)
    cv2.namedWindow('input', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('detected line', 640, 480)
    cv2.resizeWindow('input', 640, 480)

    cv2.imshow('detected line',blank_image)
    cv2.imshow('input', frame)

    # clear the stream in preparation for the next frame
    # rawCapture.truncate(0)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
