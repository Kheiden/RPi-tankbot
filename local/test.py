import cv2
import time

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080

left = cv2.VideoCapture(0)
left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
left.set(cv2.CAP_PROP_FPS,1)
left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

#right = cv2.VideoCapture(1)
#right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
#right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
#right.set(cv2.CAP_PROP_FPS,1)
#right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

while(True):
    if not (left.grab()):
        print("No more frames")
        break

    _, leftFrame = left.retrieve()
    #_, rightFrame = right.retrieve()

    cv2.imshow('left', leftFrame)
    #cv2.imshow('right', rightFrame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

left.release()
#right.release()
cv2.destroyAllWindows()
