import numpy as np
import cv2

# -1 -> first working camera on system
cap = cv2.VideoCapture(-1)

show_size = True

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

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
    ret, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

    # find all contours from thresholded image
    _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    filtered_contours = []

    # we are able to calculate contour area
    # according to fact, that many small blobs are detected
    # we filter out contours with small area - izi
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100 :
            filtered_contours.append(cnt)

    # draw big-ass contours on our blank image
    cv2.drawContours(blank_image, filtered_contours, -1, (0,255,0), 3)

    # Display the results and the grayscale input from your camer
    cv2.imshow('detected line',blank_image)
    cv2.imshow('input', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()