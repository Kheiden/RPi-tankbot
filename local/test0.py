import cv2
import time

CAMERA_WIDTH = 1270
CAMERA_HEIGHT = 700

left = cv2.VideoCapture(0)
left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
left.set(cv2.cv.CV_CAP_PROP_FPS,60)
left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

right = cv2.VideoCapture(1)


left.grab()
_, leftFrame = left.retrieve()
print(leftFrame.any())
left.release()


right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

right.grab()
_, rightFrame = right.retrieve()


print("~~~~")
print(rightFrame.any())
right.release()


cv2.destroyAllWindows()
