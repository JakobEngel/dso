import cv2
import numpy as np
import os
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", required=True, help="path to input video")
args = vars(ap.parse_args())

# Playing video from file:
cap = cv2.VideoCapture(args["video"])

try:
    if not os.path.exists('images'):
        os.makedirs('images')
except OSError:
    print ('Error: Creating directory of data')

currentFrame = 0
rightFrameName = '0000' + str(currentFrame)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    if frame is None:
        break
    grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Saves image of the current frame in jpg file
    if currentFrame < 10:
        rightFrameName = rightFrameName[:-1]
        rightFrameName = '0000' + str(currentFrame)
    if currentFrame >= 10 and currentFrame < 100 :
        rightFrameName = rightFrameName[:-2]
        rightFrameName = rightFrameName + str(currentFrame)
    if currentFrame >= 100 and currentFrame < 1000:
        rightFrameName = rightFrameName[:-3]
        rightFrameName = rightFrameName + str(currentFrame)
    if currentFrame >= 1000:
        rightFrameName = rightFrameName[:-4]
        rightFrameName = rightFrameName + str(currentFrame)
    name = './images/' + rightFrameName + '.jpg'
    print ('Creating...' + name,end="\r")
    cv2.imwrite(name, grayFrame)

    # To stop duplicate images
    currentFrame += 1

# When everything done, release the capture
print('\nDone!')
cap.release()
cv2.destroyAllWindows()